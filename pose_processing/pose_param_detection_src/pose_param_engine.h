//
// Created by biba_bo on 2020-11-17.
//

#ifndef CORAL_POSE_DETECTION_CPP_POSE_ANGLE_ENGINE_H
#define CORAL_POSE_DETECTION_CPP_POSE_ANGLE_ENGINE_H


#include <cmath>
#include <vector>

/** @brief this constant contains a count of the initialization steps, it's used for increasing
 *   accuracy while init's constant body parameters.*/
const int MAX_INIT_STEPS = 30;

/** @brief class PoseParamEngine is used as a wrapper for calculating current pose paramenters, as an angle,
 *         an approximate distance to the body, offset by x and y axis in the device space.
 *  @warning for the correct usage, u should to init human pose, for correct detection distance to human*/
class PoseParamEngine {
private:
    /** @brief this bool variable shows is use face points for pose param calculating */
    bool _isModelHasFaceBasePoints;
    /** @brief this variable contains a width parameter of the current image (or frame) */
    int _widthImage;
    /** @brief this variable contains a height parameter of the current image (or frame) */
    int _heightImage;
    /** @brief this variable contains a Distance between eyes on the current image - used as a constant for distance detections */
    float _eyesDistanceOnImage;
    /** @brief this variable contains a distance between shoulders on the current image - used as a constant for calculating
     *         a body angle.     */
    float _shoulderDistanceOnImage;
    /** @brief this variable contains a distance between body point (left shoulder and left hip joint) - it's used for
     *         calculating a body angle independently from current distance*/
    float _leftBodyDistanceOnImage;
    /** @brief this variable contains a distance between body point (right shoulder and right hip joint) - it's used for
     *         calculating a body angle independently from current distance*/
    float _rightBodyDistanceOnImage;
    /** @brief this variable contains a body ration between maximum shoulder distance and distance between shoulder and hip points
     *         it's used for calculating */
    float _currentBodyRatio;
    /** @brief this variable contains a last detected pose angle - it's used for returning if pose points are not enough
     *         for correct angle detection*/
    float _lastDetectedAngle;

    /** @brief this variable contains a count of the current initialization steps - it's used as a break flag for
     *         stopping an init process*/
    int _countInitSteps;

    /** @brief this private function inits variables for detecting current pose angle
     *  @warning initing via this initializator doesn't give u possibility to detect a pose angle
     *           independently from distance to human body
     *  @param xCoords - const ref to vector of x coordinates of each pose points, if point isn't detected this one equals '-1'
     *  @param yCoords - const ref to vector of y coordinates of each pose points */
    void init_angle_detector(const std::vector<int> &xCoords, const std::vector<int> &yCoords);

    /** @brief this private function inits variables for detecting current pose angle, it inits independently from distance
     *         to the current human body
     *  @param xCoords - const ref to vector of x coordinates of each pose points, if point isn't detected this one equals '-1'
     *  @param yCoords - const ref to vector of y coordinates of each pose points */
    void init_angle_detector_no_dist(const std::vector<int> &xCoords, const std::vector<int> &yCoords);

public:
    /** @brief this is a standard constructor
     *  @param isModelHasFaceBasePoints - is a flag for using or no using face points
     *  @param heightImage - just a frame height
     *  @param widthImage - just a frame width*/
    PoseParamEngine(bool isModelHasFaceBasePoints, int widthImage, int heightImage);

    /** @brief this is a default destructor, now it's nothing to do*/
    ~PoseParamEngine();

    /** @brief this public function returns current pose angle
     *  @warning an angle value with will returned by this value are highly dependent from distance to the human body
     *  @param angle - this is returned angle
     *  @param xCoords - const ref to vector of x coordinates of each pose points, if point isn't detected this one equals '-1'
     *  @param yCoords - const ref to vector of y coordinates of each pose points
     *  @return status of the angle detection */
    bool get_angle(float &angle, const std::vector<int> &xCoords, const std::vector<int> &yCoords);

    /** @brief this public function returns current pose angle, independently from distance to the current human body
     *  @param angle - this is returned angle
     *  @param xCoords - const ref to vector of x coordinates of each pose points, if point isn't detected this one equals '-1'
     *  @param yCoords - const ref to vector of y coordinates of each pose points
     *  @return status of the angle detection */
    bool get_angle_no_dist(float &angle, const std::vector<int> &xCoords, const std::vector<int> &yCoords);

    /** @brief this public function returns current pose params (x and y offsets)
     *  @param xy_offset - a pair of offsets, the first - x_offset, the second one - y_offset
     *  @param xCoords - const ref to vector of x coordinates of each pose points, if point isn't detected this one equals '-1'
     *  @param yCoords - const ref to vector of y coordinates of each pose points
     *  @return status of pose params detection */
    bool get_xy_offset_no_dist(std::pair<double, double> &xy_offset, const std::vector<int> &xCoord,
                               const std::vector<int> &yCoords);

    /** @return - current distance between eyes*/
    float get_eyes_distance();

    /** @return - current distance between shoulders*/
    float get_shoulder_distance();
};


#endif //CORAL_POSE_DETECTION_CPP_POSE_ANGLE_ENGINE_H
