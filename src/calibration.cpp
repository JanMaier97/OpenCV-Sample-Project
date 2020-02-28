#include "calibration.hpp"
#include <iostream>
#include <fstream>
#include <exception>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;


Calibration::Calibration() {
    cameraMatrix.ptr<float>(0)[0] = 1;
    cameraMatrix.ptr<float>(1)[1] = 1;
}

Calibration::Calibration(filesystem::path filepath) {
    this->loadCalibration(filepath);
}


void Calibration::saveCalibration(filesystem::path filepath) {
    cout << "Saving calibration to " << filepath.c_str() << endl;

    cv::FileStorage fs(filepath, cv::FileStorage::WRITE);
    fs << cameraMatrixSerName << cameraMatrix;
    fs.release();

    cout << "Successfully saved calibration!" << endl;
}

void Calibration::undistortImage(InputArray image, OutputArray undistortedImage) {
    // TODO: try out using getOptimalNewCameraMatrix as an additional paramter
    undistort(image, undistortedImage, cameraMatrix, distortionCoefficients, optimalCameraMatrix);
}

void Calibration::loadCalibration(filesystem::path filepath) {
    cout << "Loading calibration from file " << filepath.c_str() << endl;

    cv::FileStorage fs(filepath, cv::FileStorage::READ);
    cameraMatrix = fs[cameraMatrixSerName].mat();
    fs.release();

    cout << "Successfully loaded calibration!" << endl;
}

void Calibration::calibrate(vector <filesystem::path> imageFiles, Size boardSize) {
    vector <vector<Point3f>> objectPoints;
    vector <vector<Point2f>> imagePoints;
    vector <Point2f> corners;
    vector <Point3f> object;

    for (int tileCornerCounter = 0; tileCornerCounter < boardSize.area(); tileCornerCounter++) {
        object.push_back(Point3f(tileCornerCounter / boardSize.width, tileCornerCounter % boardSize.width, 0.0f));
    }

    Mat currentImage;
    for (auto const& imagePath: imageFiles) {
        currentImage = imread(imagePath.string());
        cvtColor(currentImage, currentImage, cv::COLOR_BGR2GRAY);

        std::cout << "Trying to find corners in file " << imagePath.filename() << ": ";

        bool found = findChessboardCorners(currentImage, boardSize, corners,
                                           CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);

        if (found) {
            cout << "Found corners" << endl;

            cornerSubPix(currentImage, corners, Size(5, 5), Size(-1, -1),
                         TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.001));

            imagePoints.push_back(corners);
            objectPoints.push_back(object);
        } else {
            cout << "Couldn't find corners." << endl;
        }
    }

    double calibrationOutput = calibrateCamera(objectPoints, imagePoints, currentImage.size(), cameraMatrix,
                                               distortionCoefficients, rotationVectors, translationVectors);

    cout << "Camera calibrated with output " << calibrationOutput << endl;

    // optimize cameraMatrix
    if (!currentImage.empty())
        optimalCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distortionCoefficients, currentImage.size(), 1, currentImage.size());
}
