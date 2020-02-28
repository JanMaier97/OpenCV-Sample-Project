#ifndef ESSENTIAL_MATRIX_H
#define ESSENTIAL_MATRIX_H

#include "calibration.hpp"
#include "opencv2/core.hpp"

class EssentialMatrix {

    private:
        Calibration cameraCalibration;
        bool pointInfrontCamera(const cv::Point3f &worldPoint, 
                                const cv::Mat &cameraRotation,
                                const cv::Mat &cameraTranslation);


        cv::Mat computeCameraCenter(const cv::Mat &cameraRotation,
                                    const cv::Mat &cameraTranslation);
        

    public:
        EssentialMatrix(Calibration cameraCalibration);
        void determineEssentialMatrix(const std::vector<cv::Point2f> &pointsCamera1,
                                      const std::vector<cv::Point2f> &pointsCamera2,
                                      cv::OutputArray cameraRotation,
                                      cv::OutputArray cameraTranslation);


        int numberOfPointsInfront(const cv::Mat &worldPoints,
                                  const cv::Mat &cameraRotation,
                                  const cv::Mat &cameraTranslation);

};

#endif
