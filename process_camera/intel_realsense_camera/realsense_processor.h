//
// Created by biba_bo on 2020-12-07.
//

#ifndef CORAL_IRON_TURTLE_REALSENSE_PROCESSOR_H
#define CORAL_IRON_TURTLE_REALSENSE_PROCESSOR_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <thread>
#include <atomic>
#include <list>

constexpr int DEPTH_FRAME_WIDTH = 1280;
constexpr int DEPTH_FRAME_HEIGHT = 720;

constexpr int X_STEP_LENGTH = 1;

constexpr float MIN_POSSIBLE_DISTANCE = 0.0f;
constexpr float MAX_POSSIBLE_DISTANCE = 4.0f;

constexpr int MAX_DISTANCES_QUEUE_LENGTH = 2;


class RealsenseProcessor {
private:
    std::thread _realsenseCameraThread;
    std::atomic<bool> _isProcessedCalculation;
    std::list<float> _queueDistancesToCenter;
    std::pair<int, int> _interestPoint;

    void process_realsense_camera_stream();

public:
    RealsenseProcessor();
    ~RealsenseProcessor();

    int start_processing();
    int stop_processing();
    float get_last_distance();
};


#endif //CORAL_IRON_TURTLE_REALSENSE_PROCESSOR_H
