#ifndef FEATURE_MATCHING_H
#define FEATURE_MATCHING_H

#include "calibration.hpp"

class FeatureMatching {
    private:
        Calibration calibration;

    public:
        FeatureMatching(Calibration cameraCalibration);
        std::vector<cv::Mat> findMatches(cv::Mat img1, cv::Mat img2);
};

#endif
