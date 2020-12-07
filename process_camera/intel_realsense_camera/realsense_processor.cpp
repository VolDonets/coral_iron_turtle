//
// Created by biba_bo on 2020-12-07.
//

#include "realsense_processor.h"

RealsenseProcessor::RealsenseProcessor() {
    _isProcessedCalculation = false;
    _interestPoint.first = DEPTH_FRAME_WIDTH / 2;
    _interestPoint.second = DEPTH_FRAME_WIDTH / 2;
}

RealsenseProcessor::~RealsenseProcessor() {
    // nothing to do here
}

int RealsenseProcessor::start_processing() {
    if (!_isProcessedCalculation) {
        _isProcessedCalculation = true;
        process_realsense_camera_stream();
    }
    return 0;
}

int RealsenseProcessor::stop_processing() {
    if (_isProcessedCalculation) {
        _isProcessedCalculation = false;
        if (_realsenseCameraThread.joinable())
            _realsenseCameraThread.detach();
    }
    return 0;
}

float RealsenseProcessor::get_last_distance() {
    if (_queueDistancesToCenter.empty())
        return 0.0;
    return _queueDistancesToCenter.front();
}

void RealsenseProcessor::process_realsense_camera_stream() {
    _realsenseCameraThread = std::thread([this]() {
        rs2::pipeline rsPipe;
        rs2::config rsConfig;
        rsConfig.disable_all_streams();
        rsConfig.enable_stream(RS2_STREAM_DEPTH, 1280, 720);
        rsConfig.enable_stream(RS2_STREAM_INFRARED, 1);
        //rsConfig.enable_stream(RS2_STREAM_COLOR, 640, 480);
        rsPipe.start(rsConfig);

        while (_isProcessedCalculation) {
            rs2::frameset data = rsPipe.wait_for_frames();
            rs2::depth_frame depthFrame = data.get_depth_frame();
            float currentDistance = depthFrame.get_distance(_interestPoint.first, _interestPoint.second);
            if (currentDistance < MIN_POSSIBLE_DISTANCE + 0.05 || currentDistance > MAX_POSSIBLE_DISTANCE) {
                currentDistance = 0;
            } else {
                currentDistance -= 1.0;
            }
            if (_queueDistancesToCenter.size() > MAX_DISTANCES_QUEUE_LENGTH)
                _queueDistancesToCenter.pop_front();
            _queueDistancesToCenter.push_back(currentDistance);
        }

        rsPipe.stop();
    });
}
