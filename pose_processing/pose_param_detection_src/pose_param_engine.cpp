//
// Created by biba_bo on 2020-11-17.
//

#include "pose_param_engine.h"

PoseParamEngine::PoseParamEngine(bool isModelHasFaceBasePoints, int widthImage, int heightImage) {
    _isModelHasFaceBasePoints = isModelHasFaceBasePoints;
    _widthImage = widthImage;
    _heightImage = heightImage;
    _countInitSteps = 0;
    _eyesDistanceOnImage = 0.0;
    _shoulderDistanceOnImage = 0.0;
    _lastDetectedAngle = 0.0;
    _currentBodyRatio = 0.0;

    _leftBodyDistanceOnImage = 0.0;
    _rightBodyDistanceOnImage = 0.0;
    _lastXYoffset.first = -1;
    _lastXYoffset.second = -1;
}

PoseParamEngine::~PoseParamEngine() {
    // nothing to do here
}

bool
PoseParamEngine::get_angle_no_dist(float &angle, const std::vector<int> &xCoords, const std::vector<int> &yCoords) {
    if (_countInitSteps != MAX_INIT_STEPS) {
        init_angle_detector_no_dist(xCoords, yCoords);
        return false;
    }

    // check is here shoulder points
    // if it doesn't, function return last detected angle;
    if (xCoords[5] != -1 && xCoords[6] != -1) {
        // here is a few ways to get current maximum shoulder distance and if it compare with
        // current shoulder distance we can get a body angle
        float currentMaxShoulderDistance = -1;
        if (xCoords[11] != -1 && xCoords[12] != -1) {
            // maximum precision
            currentMaxShoulderDistance = _currentBodyRatio
                                         * ((sqrt(pow((xCoords[5] - xCoords[11]), 2) +
                                                  pow((yCoords[5] - yCoords[11]), 2))
                                             + sqrt(pow((xCoords[6] - xCoords[12]), 2) +
                                                    pow((yCoords[6] - yCoords[12]), 2))) / 2);
        } else if (xCoords[11] != -1) {
            // less precision
            currentMaxShoulderDistance = _currentBodyRatio
                                         * sqrt(pow((xCoords[5] - xCoords[11]), 2) +
                                                pow((yCoords[5] - yCoords[11]), 2));
        } else if (xCoords[12] != -1) {
            // less precision
            currentMaxShoulderDistance = _currentBodyRatio
                                         * sqrt(pow((xCoords[6] - xCoords[12]), 2) +
                                                pow((yCoords[6] - yCoords[12]), 2));
        }

        if (currentMaxShoulderDistance > -1) {
            float currentShoulderDistanceOnImage = sqrt(
                    pow((xCoords[5] - xCoords[6]), 2) + pow((yCoords[5] - yCoords[6]), 2));
            if (currentShoulderDistanceOnImage >= currentMaxShoulderDistance) {
                // check is image has face elements line eyes or nose
                // it helps to detect a human body direction on camera
                if (xCoords[0] != -1)
                    _lastDetectedAngle = 0.0;
                else
                    _lastDetectedAngle = 3.14;
            } else {
                float middleFaceX = 0.0;
                float middleShoulderX = 0.0;
                int countMiddleFaceX = 0;
                for (int inx = 0; inx < 5; inx++) {
                    if (xCoords[inx] != -1) {
                        middleFaceX += xCoords[inx];
                        countMiddleFaceX++;
                    }
                }
                middleFaceX = middleFaceX / countMiddleFaceX;
                middleShoulderX = (xCoords[5] + xCoords[6]) / 2;

                // check is image has face elements line eyes or nose
                // it helps to detect a human body direction on camera
                if (xCoords[0] != -1) {
                    _lastDetectedAngle = (middleShoulderX > middleFaceX)
                                         ? (acos(currentShoulderDistanceOnImage / currentMaxShoulderDistance))
                                         : -(acos(currentShoulderDistanceOnImage / currentMaxShoulderDistance));
                } else {
                    _lastDetectedAngle = (middleShoulderX > middleFaceX)
                                         ? (3.14 - (acos(currentShoulderDistanceOnImage / currentMaxShoulderDistance)))
                                         : -(3.13 -
                                             (acos(currentShoulderDistanceOnImage / currentMaxShoulderDistance)));
                }
            }
        }
    }
    angle = _lastDetectedAngle;
    return true;
}

bool PoseParamEngine::get_angle(float &angle, const std::vector<int> &xCoords, const std::vector<int> &yCoords) {
    if (_countInitSteps != MAX_INIT_STEPS) {
        init_angle_detector(xCoords, yCoords);
        return false;
    }
    // check is image has face elements line eyes or nose
    // it helps to detect a human body direction on camera
    if (xCoords[0] != -1) {
        if (xCoords[5] != -1 && xCoords[6] != -1) {
            float currentShoulderDistance = sqrt(pow((xCoords[5] - xCoords[6]), 2) +
                                                 pow((yCoords[5] - yCoords[6]), 2));
            if (currentShoulderDistance >= _shoulderDistanceOnImage) {
                _lastDetectedAngle = 0;
            } else {
                float middleFaceX = 0.0;
                float middleShoulderX = 0.0;
                int countMiddleFaceX = 0;
                for (int inx = 0; inx < 5; inx++) {
                    if (xCoords[inx] != -1) {
                        middleFaceX += xCoords[inx];
                        countMiddleFaceX++;
                    }
                }
                middleFaceX = middleFaceX / countMiddleFaceX;
                middleShoulderX = (xCoords[5] + xCoords[6]) / 2;
                _lastDetectedAngle = (middleShoulderX > middleFaceX)
                                     ? (acos(currentShoulderDistance / _shoulderDistanceOnImage))
                                     : -(acos(currentShoulderDistance / _shoulderDistanceOnImage));
            }
        }
    } else {
        if (xCoords[5] != -1 && xCoords[6] != -1) {
            float currentShoulderDistance = sqrt(pow((xCoords[5] - xCoords[6]), 2) +
                                                 pow((yCoords[5] - yCoords[6]), 2));
            if (currentShoulderDistance >= _shoulderDistanceOnImage) {
                _lastDetectedAngle = 0;
            } else {
                float middleFaceX = 0.0;
                float middleShoulderX = 0.0;
                int countMiddleFaceX = 0;
                for (int inx = 0; inx < 5; inx++) {
                    if (xCoords[inx] != -1) {
                        middleFaceX += xCoords[inx];
                        countMiddleFaceX++;
                    }
                }
                middleFaceX = middleFaceX / countMiddleFaceX;
                middleShoulderX = (xCoords[5] + xCoords[6]) / 2;
                _lastDetectedAngle = (middleShoulderX > middleFaceX)
                                     ? (3.14 - (acos(currentShoulderDistance / _shoulderDistanceOnImage)))
                                     : -(3.13 - (acos(currentShoulderDistance / _shoulderDistanceOnImage)));
            }
        }
    }
    angle = _lastDetectedAngle;
    return true;
}

void PoseParamEngine::init_angle_detector(const std::vector<int> &xCoords, const std::vector<int> &yCoords) {
    if ((xCoords[1] != -1 && xCoords[2] != -1)
        && (xCoords[5] != -1 && xCoords[6] != -1)) {
        _eyesDistanceOnImage += sqrt(pow((xCoords[1] - xCoords[2]), 2) +
                                     pow((yCoords[1] - yCoords[2]), 2));
        _shoulderDistanceOnImage += sqrt(pow((xCoords[5] - xCoords[6]), 2) +
                                         pow((yCoords[5] - yCoords[6]), 2));
        _countInitSteps++;
        if (_countInitSteps == MAX_INIT_STEPS) {
            _eyesDistanceOnImage /= MAX_INIT_STEPS;
            _shoulderDistanceOnImage /= MAX_INIT_STEPS;
        }
    }
}

void PoseParamEngine::init_angle_detector_no_dist(const std::vector<int> &xCoords, const std::vector<int> &yCoords) {
    if (_countInitSteps == 0) {
        _shoulderDistanceOnImage = 0.0;
        _rightBodyDistanceOnImage = 0.0;
        _leftBodyDistanceOnImage = 0.0;
        _currentBodyRatio = 0.0;
    }
    if ((xCoords[1] != -1 && xCoords[2] != -1)
        && (xCoords[5] != -1 && xCoords[6] != -1)
        && (xCoords[11] != -1 && xCoords[12] != -1)) {
        _shoulderDistanceOnImage += sqrt(pow((xCoords[5] - xCoords[6]), 2) +
                                         pow((yCoords[5] - yCoords[6]), 2));
        _leftBodyDistanceOnImage += sqrt(pow((xCoords[5] - xCoords[11]), 2) +
                                         pow((yCoords[5] - yCoords[11]), 2));
        _rightBodyDistanceOnImage += sqrt(pow((xCoords[6] - xCoords[12]), 2) +
                                          pow((yCoords[6] - yCoords[12]), 2));
        _countInitSteps++;
        if (_countInitSteps == MAX_INIT_STEPS) {
            _shoulderDistanceOnImage = _shoulderDistanceOnImage / MAX_INIT_STEPS;
            _leftBodyDistanceOnImage = _leftBodyDistanceOnImage / MAX_INIT_STEPS;
            _rightBodyDistanceOnImage = _rightBodyDistanceOnImage / MAX_INIT_STEPS;
            _currentBodyRatio = _shoulderDistanceOnImage / ((_leftBodyDistanceOnImage + _rightBodyDistanceOnImage) / 2);
        }
    }
}

bool PoseParamEngine::get_xy_offset_no_dist(std::pair<float, float> &xy_offset, const std::vector<int> &xCoord,
                                            const std::vector<int> &yCoords) {
    if (_countInitSteps != MAX_INIT_STEPS) {
        init_angle_detector_no_dist(xCoord, yCoords);
        return false;
    }
    float bodyWidth = -1;
    if (xCoord[5] != -1 && xCoord[6] != -1 && xCoord[11] != -1 && xCoord[12] != -1) {
        bodyWidth = (sqrt(pow((xCoord[5] - xCoord[11]), 2) + pow((yCoords[5] - yCoords[11]), 2))
                     + sqrt(pow((xCoord[6] - xCoord[12]), 2) + pow((yCoords[6] - yCoords[12]), 2))) / 2;
    } else if (xCoord[5] != -1 && xCoord[11] != -1) {
        bodyWidth = sqrt(pow((xCoord[5] - xCoord[11]), 2) + pow((yCoords[5] - yCoords[11]), 2));
    } else if (xCoord[6] != -1 && xCoord[12] != -1) {
        bodyWidth = sqrt(pow((xCoord[6] - xCoord[12]), 2) + pow((yCoords[6] - yCoords[12]), 2));
    }

    if (bodyWidth < 0) {
        return false;
    }

    std::pair<int, int> bodyCenterCoordinate;
    int includedPointsCounter = 0;
    for (int inx = 0; inx < xCoord.size(); inx++) {
        if (xCoord[inx] != -1) {
            bodyCenterCoordinate.first += xCoord[inx];
            bodyCenterCoordinate.second += yCoords[inx];
            includedPointsCounter++;
        }
    }
    bodyCenterCoordinate.first /= includedPointsCounter;
    bodyCenterCoordinate.second /= includedPointsCounter;

    // I took a basic distance to human body as 1m
    // 2.907 - is scale parameter, which I have calculated for current camera, but it can be changed to other
    //TODO - !?possibly I should add kinda camera interface for making this interface universal
    _lastXYoffset.second = 2.907 * (1 - (bodyWidth / ((_rightBodyDistanceOnImage + _leftBodyDistanceOnImage) / 2)));
    // I have measured a camera angle as 150 degrees, so now I can use a proportion
    _lastXYoffset.first = (2.62 * (bodyCenterCoordinate.first - (_widthImage / 2))) / _widthImage;

    xy_offset.first = _lastXYoffset.first;
    xy_offset.second = _lastXYoffset.second;
    return true;
}

float PoseParamEngine::get_eyes_distance() {
    return _eyesDistanceOnImage;
}

float PoseParamEngine::get_shoulder_distance() {
    return _shoulderDistanceOnImage;
}

