#ifndef FEATURE_MATCHING_H
#define FEATURE_MATCHING_H

#include "calibration.hpp"

class FeatureMatching {
    private:
        Calibration calibration;
        cv::Mat preprocessImage(cv::Mat image);

    public:
        FeatureMatching(Calibration cameraCalibration);
        std::vector<cv::Mat> findMatches(cv::Mat img1, cv::Mat img2);
};

#endif
