#include <filesystem>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>

class Calibration {

private:	
    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1);
    cv::Mat distortionCoefficients;
    std::vector<cv::Mat> rotationVectors;
    std::vector<cv::Mat> translationVectors;
    const std::string cameraMatrixSerName = "cameraMatrix";

public:
	Calibration(std::filesystem::path filepath);
	Calibration();

	void calibrate(std::vector<std::filesystem::path> imageFiles, cv::Size boardSize);

	void loadCalibration(std::filesystem::path filepath);

	void saveCalibration(std::filesystem::path filepath);

	void undistortImage(cv::Mat image);
	
};
