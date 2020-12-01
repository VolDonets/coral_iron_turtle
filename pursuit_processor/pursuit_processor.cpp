//
// Created by biba_bo on 2020-10-05.
//

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
    _mutexProc.unlock();
    _mutexRes.unlock();

}

void PursuitProcessor::fix_angle_shift(float angleShift) {

}

void PursuitProcessor::fix_distance_shift(float distanceShift) {

}

void PursuitProcessor::process_pursuit_process() {

}
