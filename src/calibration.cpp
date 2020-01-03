#include "calibration.hpp"
#include <iostream>
#include <exception>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;


Calibration::Calibration() {
    cameraMatrix.ptr<float>(0)[0] = 1;
    cameraMatrix.ptr<float>(1)[1] = 1;
}

Calibration::Calibration(filesystem::path filepath) {

}



void Calibration::saveCalibration(filesystem::path filepath) {

}

void Calibration::undistortImage(Mat image) {

}

void Calibration::loadCalibration(filesystem::path filepath) {
	
}

void Calibration::calibrate(vector<filesystem::path> imageFiles, Size boardSize) {
	

    vector<vector<Point3f>> objectPoints;
    vector<vector<Point2f>> imagePoints;
    vector<Point2f> corners;
    vector<Point3f> object;

    for (int tileCornerCounter = 0; tileCornerCounter < boardSize.area(); tileCornerCounter++) {
        object.push_back(Point3f(tileCornerCounter / boardSize.width, tileCornerCounter % boardSize.width, 0.0f));
    }

    Mat currentImage;
    for (auto const& imagePath: imageFiles) {
        currentImage = imread(imagePath.string(), IMREAD_GRAYSCALE);

        std::cout << "Trying to find corners in file " << imagePath.filename() << ": ";

        bool found = findChessboardCorners(currentImage, boardSize, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);

        if (found) {
            cout << "Found corners" << endl;

            cornerSubPix(currentImage, corners, Size(5, 5), Size(-1, -1), TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.001));

            imagePoints.push_back(corners);
            objectPoints.push_back(object);
        } else {
            cout << "Couldn't find corners." << endl;
        }
    }

    double calibrationOutput = calibrateCamera(objectPoints, imagePoints, currentImage.size(), cameraMatrix, distortionCoefficients, rotationVectors, translationVectors);

    cout << "Camera calibrated with output " << calibrationOutput << endl;
}
