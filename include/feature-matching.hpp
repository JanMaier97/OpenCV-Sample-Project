#ifndef FEATURE_MATCHING_H
#define FEATURE_MATCHING_H

#include "calibration.hpp"

class FeatureMatching {
    private:
        Calibration calibration;
        void preprocessImage(cv::Mat image, cv::OutputArray processedImage);
        void filterMatches(const std::vector<cv::KeyPoint> keypoints1, const std::vector<cv::KeyPoint> keypoints2, const std::vector<cv::DMatch> &matches, std::vector<cv::DMatch> &filteredMatches, cv::Size imageSize);

    public:
        FeatureMatching(Calibration cameraCalibration);
        std::vector<cv::Mat> findMatches(cv::Mat img1, cv::Mat img2);
};

#endif
