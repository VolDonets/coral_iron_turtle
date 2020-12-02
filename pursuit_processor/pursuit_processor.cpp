//
// Created by biba_bo on 2020-10-05.
//

#include <cmath>
#include "pursuit_processor.h"

PursuitProcessor::PursuitProcessor() {
    // now iron_turtle can't pursuit a human
    _isCanMove = false;

    // activate the serial device wrapper and read/write wrapping functions
    _serialManager = std::make_shared<SerialManager>();

    // now we haven't connected clients, so we shouldn't send control commands
    _serverCounter = 0;

    // now create an object for HoverboardAPI for the communication with
    // a serial device (here connected the iron engines controller.
    _ironTurtleAPI = std::make_shared<HoverboardAPI>(write_serial_wrapper);

    // disable processing
    _isProcessThread = false;
}

PursuitProcessor::PursuitProcessor(std::shared_ptr<SerialManager> serialManager) {
    // now iron_turtle can't pursuit a human
    _isCanMove = false;

    // activate the serial device wrapper and read/write wrapping functions
    _serialManager = serialManager;

    // now we haven't connected clients, so we shouldn't send control commands
    _serverCounter = 0;

    // now create an object for HoverboardAPI for the communication with
    // a serial device (here connected the iron engines controller.
    _ironTurtleAPI = std::make_shared<HoverboardAPI>(write_serial_wrapper);

    // disable processing
    _isProcessThread = false;
}

PursuitProcessor::~PursuitProcessor() {
    //Nothing to do here
}

void PursuitProcessor::say_server_here() {
    _serverCounter = SERVER_WAIT_STEPS_PURSUIT;
}

void PursuitProcessor::say_server_leave() {
    stop_moving();
    _serverCounter = 20;
}

void PursuitProcessor::stop_moving() {
    _isCanMove = false;
}

void PursuitProcessor::resume_moving() {
    _isCanMove = true;
}

int PursuitProcessor::get_speed() {
    return 0;
}

int PursuitProcessor::get_battery_voltage() {
    return 0;
}

int PursuitProcessor::stop_processing_thread() {
    if (_isProcessThread) {
        _isProcessThread = false;
        std::this_thread::sleep_for(std::chrono::microseconds(SLEEP_THREAD_TIME_MS));
        _ironTurtleAPI->sendSpeedData(0.0, 0.0, 0, 0, PROTOCOL_SOM_NOACK);
        _mutexProc.unlock();
        _isProcessThread = false;
        stop_moving();
    }
    return SUCCESSFUL_OPERATION_PURSUIT;
}

int PursuitProcessor::restart_processing_thread() {
    if (_isProcessThread) {
        return SUCCESSFUL_OPERATION_PURSUIT;
    }
    if (_serialManager->isSerialOK()) {
        _isProcessThread = true;
        _mutexProc.lock();
        resume_moving();
        if (_movingProcessingThread.joinable())
            _movingProcessingThread.detach();
        process_pursuit_process();
        return SUCCESSFUL_OPERATION_PURSUIT;
    } else {
        return SERIAL_MANAGER_PROBLEM_PURSUIT;
    }
}

bool PursuitProcessor::is_process_moving() {
    return _isProcessThread;
}

void PursuitProcessor::add_aim_for_processing(std::pair<float, float> aim) {
    _mutexRes.lock();
    if (_aimsQueue.size() > MAX_COUNT_AIMS_IN_QUEUE)
        _aimsQueue.pop_front();

    _aimsQueue.push_front(aim);
    _mutexRes.unlock();
    _mutexProc.unlock();
}

void
PursuitProcessor::universal_shift_fix(float distShift, float leftWheelFactor, float rightWheelFactor, int direction) {
    // fix inaccuracy from speed-up process
    distShift += 0.1;

    int currentPowerValue = 0;
    float currentSpeedValue = 0;

    int accelSteps = 0;
    int maxSpeedSteps = 0;

    if (distShift <= DIST_ASSEL) {
        float movingTime = sqrtf(distShift / MAX_SPEED_UP);
        accelSteps = movingTime / UPDATE_SPEED_TIME_OUT_SECONDS;
    } else {
        accelSteps = MAX_COUNT_ASSEL_STEP;
        float maxSpeedDist = distShift - DIST_ASSEL;
        float movingTime = maxSpeedDist / MAX_SPEED;
        maxSpeedSteps = movingTime / UPDATE_SPEED_TIME_OUT_SECONDS;
    }

    for (int i = 0; i < accelSteps && _isCanMove; i++) {
        currentSpeedValue += SPEED_UP_STEP;
        currentPowerValue =
                (currentPowerValue <= POWER_MAX_VALUE) ? currentPowerValue + POWER_CHANGE_STEP : POWER_MAX_VALUE;
        _ironTurtleAPI->sendSpeedData(leftWheelFactor * currentSpeedValue * direction,
                                      rightWheelFactor * currentSpeedValue * direction,
                                      currentPowerValue, MIN_WHEELS_START_SPEED_VALUE, PROTOCOL_SOM_NOACK);
        std::this_thread::sleep_for(std::chrono::microseconds(SLEEP_THREAD_TIME_MS));
    }

    for (int i = 0; i < maxSpeedSteps && _isCanMove; i++) {
        _ironTurtleAPI->sendSpeedData(leftWheelFactor * currentSpeedValue * direction,
                                      rightWheelFactor * currentSpeedValue * direction,
                                      currentPowerValue, MIN_WHEELS_START_SPEED_VALUE, PROTOCOL_SOM_NOACK);
        std::this_thread::sleep_for(std::chrono::microseconds(SLEEP_THREAD_TIME_MS));
    }

    for (int i = 0; i < accelSteps && _isCanMove; i++) {
        currentSpeedValue -= SPEED_UP_STEP;
        currentPowerValue =
                (currentPowerValue > 0) ? currentPowerValue - POWER_CHANGE_STEP : POWER_MAX_VALUE;
        _ironTurtleAPI->sendSpeedData(leftWheelFactor * currentSpeedValue * direction,
                                      rightWheelFactor * currentSpeedValue * direction,
                                      currentPowerValue, MIN_WHEELS_START_SPEED_VALUE, PROTOCOL_SOM_NOACK);
        std::this_thread::sleep_for(std::chrono::microseconds(SLEEP_THREAD_TIME_MS));
    }

    for (int i = 0; i < 3; i++) {
        _ironTurtleAPI->sendSpeedData(0, 0, 0, 0, PROTOCOL_SOM_NOACK);
        std::this_thread::sleep_for(std::chrono::microseconds(SLEEP_THREAD_TIME_MS));
    }
}

void PursuitProcessor::fix_angle_shift(float angleShift) {
    // first calculate a distance which should rotate one wheel for moving
    // distance between wheels is 0.3m, so I can calculate rotation for one wheel
    float rotDist = 0.3 * sinf(fabsf(angleShift));

    // just for removing possible bugs
    if (rotDist < 0) return;

    float rightWheelFactor = (angleShift > 0) ? 1.0 : 0.0;
    float leftWheelFactor = (angleShift > 0) ? 0.0 : 1.0;

    universal_shift_fix(rotDist, leftWheelFactor, rightWheelFactor, 1);
}

void PursuitProcessor::fix_distance_shift(float distanceShift) {
    universal_shift_fix(fabs(distanceShift), 1, 1,
                        (distanceShift > 0) ? 1 : -1);
}

void PursuitProcessor::process_pursuit_process() {
    this->_movingProcessingThread = std::thread([this]() {
        while (_isProcessThread) {
            if (_aimsQueue.empty()) {
                std::this_thread::sleep_for(std::chrono::microseconds(SLEEP_THREAD_TIME_MS * 2));
                continue;
            }
            _mutexProc.lock();
            _mutexRes.lock();
            std::pair<float, float> currentAim = _aimsQueue.front();
            _aimsQueue.pop_front();
            _mutexRes.unlock();

            if ((fabsf(currentAim.first) > ANGLE_ALLOWABLE_SHIFT) && _isCanMove) {
                fix_angle_shift(currentAim.first);
            }
            if ((fabs(currentAim.second) > DISTANCE_ALLOWABLE_SHIFT) && _isCanMove) {
                fix_distance_shift(currentAim.second);
            }
            if (!_aimsQueue.empty())
                _mutexProc.unlock();
        }
    });
}
