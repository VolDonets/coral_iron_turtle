//
// Created by biba_bo on 2020-10-05.
//

#ifndef IRON_TURTLE_PURSUIT_PROCESSOR_H
#define IRON_TURTLE_PURSUIT_PROCESSOR_H

#include <memory>   // used for class shared_ptr<> and make_shared<>()
#include <thread>   // used for a thread creating
#include <atomic>   // used for making class fields as atomic variables for the threads
#include <list>     // used as a container for aims with angle and distance to the tracked body
#include <mutex>    // used as a waiting points for the different threads

#include "../turtle_manager/binary_com_manager/serial_manager.h"                  //this .h-file contains my wrapper for the serial device
#include "../turtle_manager/binary_com_manager/bipropellant-api/HoverboardAPI.h"  //this .h-file contains C++ class - wrapper for the C bipropellant API

/** @brief this constant means how many steps should to wait a server thread before it decides, that client leave a connection
 *         that means we should to stop sending a commands.*/
constexpr int SERVER_WAIT_STEPS_PURSUIT = 10;
/**
 * @brief this constant is a code of the successful operation */
constexpr int SUCCESSFUL_OPERATION_PURSUIT = 0;
/**
 * @brief this constant is s code of the unsuccessful operation - the reason is problems with connection to the serial devise. */
constexpr int SERIAL_MANAGER_PROBLEM_PURSUIT = -1;
/**
 * @brief this constant contains a max count aims in the aims queue
 */
constexpr int MAX_COUNT_AIMS_IN_QUEUE = 2;

/**
 * @brief this constant contains a minimum angle shift which program can use */
constexpr float ANGLE_ALLOWABLE_SHIFT = 0.15;
/**
 * @brief this constant constant contains a minimum distance shift which program can use */
constexpr float DISTANCE_ALLOWABLE_SHIFT = 0.1;
/// max wheel speed in m/s
constexpr float MAX_SPEED = 0.597;
/// max wheel speed-up in m/s2
constexpr float MAX_SPEED_UP = 3.32;
/// maximum distance which move wheel with current acceleration
constexpr float DIST_ASSEL = 1.0746;
/// maximum count steps for achieving maximum speed
constexpr int MAX_COUNT_ASSEL_STEP = 60;
/// time which we should wait before update speed data
constexpr float UPDATE_SPEED_TIME_OUT_SECONDS = 0.03;
/// time which we should wait before update speed data
constexpr float SPEED_UP_STEP = 0.005;
/// value for changing power
constexpr int POWER_CHANGE_STEP = 50;
/// the maximum power value
constexpr int POWER_MAX_VALUE = 300;
                           //0  1  2    3   4     5    6    7    8    9   10    11   12   13   14
/// this array contains a power values for smooth moving
static const int POWERS[] = {0, 50, 100, 150, 200, 250, 300};

/** @brief The class PursuitProcessor is controller for the turtle engines throw the turtle engine controller with using the binary protocol.
 *         It can manually move for the interested human.
 *  @warning In your program please use only one object of this class, cause in constructor creates wrapper for the serial interface and
 *          creates new thread for sending commands throw UART connection to the turtle engine controller.*/
class PursuitProcessor {
private:
    /** @brief this variable contains a server connection status (connection with client), if we have a client which is able to
     *         manage the iron turtle, so we can to send commands to the iron turtle controller.
     *         This variable decrease to 0, in the tread loop, and if a client says 'I'm here it will updates to SERVER_WAIT_STEPS,
     *         otherwise the processor won't able to send a control command and the iron turtle just stop moving.*/
    std::atomic<int> _serverCounter;
    /**
     *  @brief this is a smart pointer to the wrapper for the serial device driver */
    std::shared_ptr<SerialManager> _serialManager;
    /**
     * @brief this is a smart pointer to the bipropellant API (C++ class wrapper of the C API).
     */
    std::shared_ptr<HoverboardAPI> _ironTurtleAPI;
    /**
     * @brief this is a bool variable, which means is now something are processed or no.
     */
    std::atomic<bool> _isProcessThread;
    /**
     * @brief this variable is a flag, is should to process moving or no*/
    std::atomic<bool> _isCanMove;
    /**
     * @brief this field contains a queue of pairs of angle and delta moving */
    std::list<std::pair<float, float>> _aimsQueue;
    /** @brief This mutex synchronize a block code where we process a resources
     *         (here is - std::list<cv::Mat> queueFrames)*/
    std::mutex _mutexRes;
    /** @brief This mutex used for blocking processing, it blocks code when we trying to process image, but actually we don't have a Mat object for this
    *          (it means std::list<cv::Mat> queueFrames is empty)
    *          Firstly it blocks when we are creating a PoseDetectorWrapper
    *         class (calls in constructor)*/
    std::mutex _mutexProc;
    /**
     * @brief this thread used for processing new speed values for the right and the left wheels, and send it to the turtle engine controller */
    std::thread _movingProcessingThread;

    void universal_shift_fix(float distShift, float leftWheelFactor, float rightWheelFactor, int direction);

    /** @brief this function fix angle shift just move the iron turtle body righter or lefter
     *  @param angleShift - a shift angle for the current iron and human body position*/
    void fix_angle_shift(float angleShift);
    /** @brief this function fix distance to human body shift just move the iron turtle body forward or backward
     *  @param distanceShift - a shift distance for the current iron and human body position.*/
    void fix_distance_shift(float distanceShift);
    /** @brief this function starts new thread for the wheel's speed processing
     *         (here is a pursuit processing) and
     *         for the sending control commands to the turtle engine controller */
    void process_pursuit_process();

public:
    /** @brief this is default constructor, here inits class fields
    *  @warning this doesn't start a new thread for the processing (a bit changed logic).*/
    PursuitProcessor();
    /** @brief this is constructor with parameters
     *  @param std::shared_ptr<SerialManager> serialManager - a driver wrapper for the working with a serial  device throw simple interface.
     *  @warning this doesn't start a new thread for the processing (a bit changed logic).*/
    PursuitProcessor(std::shared_ptr<SerialManager> serialManager);
    ~PursuitProcessor();

    /**
     * @brief a server call this function, when it has active connection with an active client */
    void say_server_here();

    /**
     * @brief a server call this function, when it lost an active client */
    void say_server_leave();

    /** @brief this function sets _isCanMove as 'false'
     *         called when client send command to STOP moving*/
    void stop_moving();

    /** @brief this function sets _isCanMove as 'true'
     *         called when client send command to RESUME moving*/
    void resume_moving();

    /** @brief this function returns current speed
     *  @return returns current speed */
    int get_speed();

    /** @brief this function returns current battery voltage
     * @return returns current battery voltage of the iron turtle battery*/
    int get_battery_voltage();

    /** @brief This function returns code of operation for the STOPPING thread and it trying if possible to stop the processing thread.
     *  @return an operation result code */
    int stop_processing_thread();

    /** @brief This function returns code of operation for the RESTARTING thread and it trying if possible to stop the processing thread.
     *  @return an operation result code */
    int restart_processing_thread();

    /** @brief This function returns status of the processing thread. If a thread works - the function returns 'true', otherwise it returns - 'false'.
     *  @return an operation result code */
    bool is_process_moving();

    /** @brief this function adds an aim in the aimsQueue, where aim
     * @param aim is a pair<float, float>,
     *        where first is angle shift
     *        second is a distance shift
     */
    void add_aim_for_processing(std::pair<float, float> aim);
};


#endif //IRON_TURTLE_PURSUIT_PROCESSOR_H
