//
// Created by biba_bo on 2020-11-13.
//

#include "pose_detector_wrapper.h"

PoseDetectorWrapper::PoseDetectorWrapper() {
    // set status of thread working as disabled
    _isProcessThread = false;
    // lock mutexProc - it means, no frames to process
    _mutexProc.lock();
    // load model from file and prepare space for pose detection processing
    init_pose_detector(PATH_TO_POSENET_MODEL);
    _poseParamEngine = std::make_shared<PoseParamEngine>(true, 640, 480);

    _interestAreaCenterCoordinate.first = 640 / 2;
    _interestAreaCenterCoordinate.second = 480 / 2;
}

PoseDetectorWrapper::PoseDetectorWrapper(const std::string &pathToPoseNetModel) {
    // set status of thread working as disabled
    _isProcessThread = false;
    // lock mutexProc - it means, no frames to process
    _mutexProc.lock();
    // load model from file and prepare space for pose detection processing
    init_pose_detector(pathToPoseNetModel);

    _poseParamEngine = std::make_shared<PoseParamEngine>(true, 640, 480);
    _interestAreaCenterCoordinate.first = 640 / 2;
    _interestAreaCenterCoordinate.second = 480 / 2;
}

PoseDetectorWrapper::~PoseDetectorWrapper() {
    //Now here is nothing to do.
}

void PoseDetectorWrapper::init_pose_detector(const std::string &pathToModelFile) {
    _statusModelInterpreterActivation = CODE_STATUS_OK;
    _model = tflite::FlatBufferModel::BuildFromFile(pathToModelFile.c_str());

    if (_model == nullptr) {
        std::cerr << "FAIL to build FlatBufferModel from file!\n";
        _statusModelInterpreterActivation = CODE_FAILED_BUILD_MODEL_FROM_FILE;
        return;
    } else {
        std::cout << "Model from file SUCCESSFULLY built\n";
    }

    const auto &available_tpus = edgetpu::EdgeTpuManager::GetSingleton()->EnumerateEdgeTpu();
    _edgetpuContext =
            edgetpu::EdgeTpuManager::GetSingleton()->OpenDevice(
                    available_tpus[0].type, available_tpus[0].path);
    // here can be changed a value of _statusModelInterpreterActivation
    // so after it we should to check it
    _modelInterpreter = build_edge_tpu_interpreter(*_model, _edgetpuContext.get());
    if (_statusModelInterpreterActivation != CODE_STATUS_OK) {
        return;
    }

    const auto *dims = _modelInterpreter->tensor(_modelInterpreter->inputs()[0])->dims;
    std::cout << "Dims info: " << dims->data[0] << " " << dims->data[1] << " " << dims->data[2] << " " << dims->data[3]
              << "\n";

    _widthInputLayerPoseNetModel = dims->data[2];
    _heightInputLayerPoseNetModel = dims->data[1];

    // ////// init _outputModelShape
    const auto &out_tensor_indices = _modelInterpreter->outputs();
    _outputModelShape.resize(out_tensor_indices.size());
    //for debugging
    std::cout << "out_tensor_indices.size() : " << out_tensor_indices.size() << std::endl;
    for (size_t i = 0; i < out_tensor_indices.size(); i++) {
        const auto *tensor = _modelInterpreter->tensor(out_tensor_indices[i]);
        // We are assuming that outputs tensor are only of type float.
        _outputModelShape[i] = tensor->bytes / sizeof(float);
    }
    // ////// END of the initing _outputModelShape
}

void PoseDetectorWrapper::add_frame(cv::Mat frame) {
    _mutexRes.lock();
    if (_queueFrames.size() > MAX_COUNT_FRAMES_IN_QUEUE)
        _queueFrames.pop_front();

    _queueFrames.push_back(frame);
    _mutexProc.unlock();
    _mutexRes.unlock();
}

int PoseDetectorWrapper::start_pose_detection() {
    if (_isProcessThread)
        return CODE_STATUS_OK;
    else {
        if (_statusModelInterpreterActivation == CODE_STATUS_OK) {
            _isProcessThread = true;
            process_pose_detection();
            return CODE_STATUS_OK;
        } else
            return _statusModelInterpreterActivation;
    }
}

int PoseDetectorWrapper::stop_pose_detection() {
    _isProcessThread = false;
    _queueDetectedPoses.clear();
    return CODE_STATUS_OK;
}

std::vector<uint8_t> PoseDetectorWrapper::get_input_data_from_frame(cv::Mat &inputFrame) {
    cv::Mat resizedFrame;
    cv::resize(inputFrame, resizedFrame, cv::Size(_widthInputLayerPoseNetModel, _heightInputLayerPoseNetModel));
    cv::cvtColor(resizedFrame, resizedFrame, cv::COLOR_BGRA2RGB);
    cv::Mat flat = resizedFrame.reshape(1, resizedFrame.total() * resizedFrame.channels());
    std::vector<uint8_t> inputDataVector = resizedFrame.isContinuous() ? flat : flat.clone();
    return inputDataVector;
}

std::vector<float> PoseDetectorWrapper::get_raw_output_data_from_model(const std::vector<uint8_t> &inputData) {
    std::vector<float> outputData;
    auto *input = _modelInterpreter->typed_input_tensor<uint8_t>(0);
    std::memcpy(input, inputData.data(), inputData.size());
    _modelInterpreter->Invoke();


    const auto &output_indices = _modelInterpreter->outputs();
    const int num_outputs = output_indices.size();
    int out_idx = 0;
    for (int i = 0; i < num_outputs; ++i) {
        const auto *outTensor = _modelInterpreter->tensor(output_indices[i]);
        assert(outTensor != nullptr);
        if (outTensor->type == kTfLiteUInt8) {
            const int num_values = outTensor->bytes;
            outputData.resize(out_idx + num_values);
            const uint8_t *output = _modelInterpreter->typed_output_tensor<uint8_t>(i);
            for (int j = 0; j < num_values; ++j) {
                outputData[out_idx++] =
                        (output[j] - outTensor->params.zero_point) * outTensor->params.scale;
            }
        } else if (outTensor->type == kTfLiteFloat32) {
            const int num_values = outTensor->bytes / sizeof(float);
            outputData.resize(out_idx + num_values);
            const float *output = _modelInterpreter->typed_output_tensor<float>(i);
            for (int j = 0; j < num_values; ++j) {
                outputData[out_idx++] = output[j];
            }
        }
#ifdef MY_DEBUG_DEF
        else {
            std::cerr << "Tensor " << outTensor->name
                      << " has unsupported output type: " << outTensor->type << std::endl;
        }
#endif //MY_DEBUG_DEF
    }
    return outputData;
}

std::vector<DetectedPose>
PoseDetectorWrapper::get_pose_estimate_from_output_raw_data(const std::vector<float> &outputRawDataVector,
                                                            const float &threshold) {
    const auto *result_raw = outputRawDataVector.data();
    std::vector<std::vector<float>> results(_outputModelShape.size());
    int offset = 0;
    for (size_t i = 0; i < _outputModelShape.size(); ++i) {
        const size_t size_of_output_tensor_i = _outputModelShape[i];
        results[i].resize(size_of_output_tensor_i);
        std::memcpy(results[i].data(), result_raw + offset, sizeof(float) * size_of_output_tensor_i);
        offset += size_of_output_tensor_i;
    }
    std::vector<DetectedPose> inf_results;
    int n = lround(results[3][0]);
    for (int i = 0; i < n; i++) {
        float overall_score = results[2][i];
        if (overall_score > threshold) {
            DetectedPose result;
            std::copy(results[1].begin() + (17 * i), results[1].begin() + (17 * i) + 16,
                      std::back_inserter(result.keypointScores));
            std::copy(results[0].begin() + (17 * 2 * i), results[0].begin() + (17 * 2 * i) + 33,
                      std::back_inserter((result.keypointCoordinates)));
            inf_results.push_back(result);
        }
    }
    return inf_results;
}

void PoseDetectorWrapper::process_pose_detection() {
    _keyPointDetectionThread = std::thread([this]() {
        cv::Mat currentFrame;
        int inxInterestPose;
        while (_isProcessThread) {
            _mutexProc.lock();
            _mutexRes.lock();
            currentFrame = _queueFrames.front();
            _queueFrames.pop_front();
            _mutexRes.unlock();

            std::vector<uint8_t> inputData = get_input_data_from_frame(currentFrame);
            std::vector<float> rawOutputData = get_raw_output_data_from_model(inputData);
            std::vector<DetectedPose> detectedPoses = get_pose_estimate_from_output_raw_data(rawOutputData,
                                                                                             POSE_THRESHOLD);
            inxInterestPose = 0;
            find_the_most_suitable_pose(inxInterestPose, detectedPoses);
            if (_queueDetectedPoses.size() > MAX_COUNT_POSE_VECTORS_IN_QUEUE)
                _queueDetectedPoses.pop_front();
            _queueDetectedPoses.push_back(std::pair(inxInterestPose, detectedPoses));
            if (!_queueFrames.empty()) {
                _mutexProc.unlock();
            }
        }
    });
}

void PoseDetectorWrapper::draw_last_pose_on_image(cv::Mat &frame) {
    cv::circle(frame, cv::Point(static_cast<int>(_interestAreaCenterCoordinate.first),
                                static_cast<int>(_interestAreaCenterCoordinate.second)), 30,
               cv::Scalar(0, 255, 0), 2, cv::LINE_8, 0);
    if (_initWaitSteps < 75) {
        std::string strLine = "";
        if (_initWaitSteps < 15)
            strLine = "Wait for 5s...";
        else if (_initWaitSteps < 30)
            strLine = "Wait for 4s...";
        else if (_initWaitSteps < 45)
            strLine = "Wait for 3s...";
        else if (_initWaitSteps < 60)
            strLine = "Wait for 2s...";
        else
            strLine = "Wait for 1s...";
        cv::putText(frame, strLine, cv::Point(50, 200), cv::FONT_HERSHEY_COMPLEX_SMALL, 3.0,
                    cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        _initWaitSteps++;
        return;
    }
    float camera_width = frame.cols;
    float camera_height = frame.rows;
    if (!_queueDetectedPoses.empty()) {
        const auto &green = cv::Scalar(255, 0, 0);
        std::pair<int, std::vector<DetectedPose>> &interestPair = _queueDetectedPoses.front();
        DetectedPose &candidate = interestPair.second[interestPair.first];

        std::vector<int> k_x(17), k_y(17);
        for (int i = 0; i < 17; i++) {
            if (candidate.keypointScores[i] > POSE_THRESHOLD) {
                float x_coordinate =
                        candidate.keypointCoordinates[(2 * i) + 1] * (camera_width / _widthInputLayerPoseNetModel);
                float y_coordinate =
                        candidate.keypointCoordinates[2 * i] * (camera_height / _heightInputLayerPoseNetModel);
                k_x[i] = static_cast<int>(x_coordinate);
                k_y[i] = static_cast<int>(y_coordinate);
                cv::circle(frame, cv::Point(k_x[i], k_y[i]), 0, green, 6, 1, 0);
                std::string str = "-" + std::to_string(i);
                cv::putText(frame, str, cv::Point(k_x[i] + 5, k_y[i] + 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0,
                            cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            } else {
                k_x[i] = -1;
            }
        }

        float angle = 0.0;
//            _poseAngleEngine->get_angle(angle, k_x, k_y);
        _poseParamEngine->get_angle_no_dist(angle, k_x, k_y);
        angle = (180 * angle) / 3.14;

        std::string strLine = "Pose angle: " + std::to_string(angle);
        cv::putText(frame, strLine, cv::Point(10, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0,
                    cv::Scalar(46, 193, 24), 1, cv::LINE_AA);

//            strLine = "Eyes distance: " + std::to_string(_poseParamEngine->get_eyes_distance());
//            cv::putText(frame, strLine, cv::Point(10, 70), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0,
//                        cv::Scalar(46, 193, 24), 1, cv::LINE_AA);
//            strLine = "Shoulder distance: " + std::to_string(_poseParamEngine->get_shoulder_distance());
//            cv::putText(frame, strLine, cv::Point(10, 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0,
//                        cv::Scalar(46, 193, 24), 1, cv::LINE_AA);
        std::pair<float, float> poseParam;
        if (_poseParamEngine->get_xy_offset_no_dist(poseParam, k_x, k_y)) {
            strLine = "Angle offset: " + std::to_string((180 * poseParam.first) / 3.14) + "@";
            cv::putText(frame, strLine, cv::Point(10, 70), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0,
                        cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
            strLine = "Dist to object: " + std::to_string(poseParam.second) + "m";
            cv::putText(frame, strLine, cv::Point(10, 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0,
                        cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
            if (_pursuitProcessor != nullptr) {
                _pursuitProcessor->add_aim_for_processing(poseParam);
            }
        }
    }
}

std::unique_ptr<tflite::Interpreter>
PoseDetectorWrapper::build_edge_tpu_interpreter(const tflite::FlatBufferModel &model,
                                                edgetpu::EdgeTpuContext *edgetpu_context) {
    tflite::ops::builtin::BuiltinOpResolver resolver;
    resolver.AddCustom(coral::kPosenetDecoderOp, coral::RegisterPosenetDecoderOp());
    resolver.AddCustom(edgetpu::kCustomOp, edgetpu::RegisterCustomOp());
    std::unique_ptr<tflite::Interpreter> interpreter;
    if (tflite::InterpreterBuilder(model, resolver)(&interpreter) != kTfLiteOk) {
        std::cerr << "Failed to build interpreter." << std::endl;
        _statusModelInterpreterActivation = CODE_FAILED_BUILD_MODEL_FROM_FILE;
    }
    // Bind given context with interpreter.
    interpreter->SetExternalContext(kTfLiteEdgeTpuContext, edgetpu_context);
    interpreter->SetNumThreads(1);
    if (interpreter->AllocateTensors() != kTfLiteOk) {
        std::cerr << "Failed to allocate tensors." << std::endl;
        _statusModelInterpreterActivation = CODE_FAILED_BUILD_MODEL_FROM_FILE;
    }
    return interpreter;
}

void PoseDetectorWrapper::find_the_most_suitable_pose(int &inxInterestPose, std::vector<DetectedPose> &detectedPoses) {
    int inxTheMostInterested = 0;
    float distanceBetweenCenters = 7777777; // maximum value as possible
    float superMiddleX = _interestAreaCenterCoordinate.first,
            superMiddleY = _interestAreaCenterCoordinate.second;

    for (int inx = 0; inx < detectedPoses.size(); inx++) {
        float middleX = 0.0, middleY = 0.0;
        int countOKPoints = 0;
        for (int i = 0; i < 17; i++) {
            if (detectedPoses[inx].keypointScores[i] > POSE_THRESHOLD) {
                float x_coordinate =
                        detectedPoses[inx].keypointCoordinates[(2 * i) + 1] * (640.0 / _widthInputLayerPoseNetModel);
                float y_coordinate =
                        detectedPoses[inx].keypointCoordinates[2 * i] * (480.0 / _heightInputLayerPoseNetModel);
                middleX += static_cast<int>(x_coordinate);
                middleY += static_cast<int>(y_coordinate);
                countOKPoints++;
            }
        }
        if (countOKPoints > 0) {
            middleX /= countOKPoints;
            middleY /= countOKPoints;
            float currentDistanceBetweenCenter = sqrt(pow(_interestAreaCenterCoordinate.first - middleX, 2) +
                                                      pow(_interestAreaCenterCoordinate.second - middleY, 2));
            if (distanceBetweenCenters > currentDistanceBetweenCenter) {
                distanceBetweenCenters = currentDistanceBetweenCenter;
                inxTheMostInterested = inx;
                superMiddleX = middleX;
                superMiddleY = middleY;
            }
        }
    }

    inxInterestPose = inxTheMostInterested;
    _interestAreaCenterCoordinate.first = superMiddleX;
    _interestAreaCenterCoordinate.second = superMiddleY;
}

void PoseDetectorWrapper::set_pursuit_processor(std::shared_ptr<PursuitProcessor> pursuitProcessor) {
    _pursuitProcessor = pursuitProcessor;
}
