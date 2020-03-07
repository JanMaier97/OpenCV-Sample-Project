#include "essentialMatrix.hpp"
#include <opencv2/calib3d.hpp>
#include "model-exporters.hpp"
#include "stdlib.h"

#include <iostream>

using namespace std;
using namespace cv;

EssentialMatrix::EssentialMatrix(Calibration cameraCalibration) {
    this->cameraCalibration = cameraCalibration;
}


Mat EssentialMatrix::normalizePoints(const vector<Point2f>& imagePoints) {
    Mat_<double> normalizedPoints(imagePoints.size(), 3);

    for (const auto &point: imagePoints) {
        Mat x = (Mat_<double>(1,3) << point.x, point.y, 1.0);
        normalizedPoints.push_back(x);
    }
    return normalizedPoints;
}

template<class T>
vector<T> EssentialMatrix::applyMask(const vector<T>& input,
                                     const vector<uchar>& mask) {
    vector<T> output;
    for(size_t i = 0; i < mask.size(); i++) {
        if (mask[i] == 1)
            output.push_back(input[i]);
    }
    return output;
}

void EssentialMatrix::determineEssentialMatrix(const vector<Point2f> &pointsCamera1,
                                               const vector<Point2f> &pointsCamera2, 
                                               Mat &cameraRotation,
                                               Mat &cameraTranslation) {

    cout << "Determining essentialMatrix" << endl;

    /* Mat fundamentalMask(pointsCamera1.size(), 1, CV_8UC1); */
    vector<uchar> fundamentalMask;
    Mat fundametalMatrix = findFundamentalMat(pointsCamera1, pointsCamera2, FM_RANSAC, 0.1, 0.99, fundamentalMask);

    /* cout << " fundametalMatrix:" << endl; */
    /* cout << fundametalMatrix << endl; */

    Mat inliners1 = normalizePoints(applyMask<Point2f>(pointsCamera1, fundamentalMask));
    Mat inliners2 = normalizePoints(applyMask<Point2f>(pointsCamera2, fundamentalMask));

    Mat essentialMatrix = cameraCalibration.getInstrincs().t() * fundametalMatrix * cameraCalibration.getInstrincs();

    /* cout << "Essential matrix: " << endl; */
    /* cout << essentialMatrix << endl; */

    // decomposes the essential matrix into 2 possible rotation matrices and 1 translation vector
    Mat rotation, translation;
    decomposeEssentialMatrix(essentialMatrix, inliners1, inliners2,  cameraRotation,  cameraTranslation);

    cout << "found rotation and camera rotation" << endl;
}


void EssentialMatrix::decomposeEssentialMatrix(const Mat &essentialMatrix,
                                               const Mat &inliners1,
                                               const Mat &inliners2,
                                               Mat &rotation,
                                               Mat &translation) {

    assert(essentialMatrix.rows == 3 && essentialMatrix.cols == 3);
    assert(inliners1.cols == 3);
    assert(inliners2.cols == 3);
    
    Mat rotation1, rotation2, trans;
    decomposeEssentialMat(essentialMatrix, rotation1, rotation2, trans);

    int pointsInfront = 0;
    int currentPointsInfront = 0;
    vector<Mat> rotations = {rotation1, rotation1, rotation2, rotation2};
    vector<Mat> translations = {trans, -trans, trans, -trans};

    for (size_t i = 0; i < rotations.size(); i++) {
        currentPointsInfront = pointsInfrontCamera(inliners1, inliners2, rotations[i], translations[i]);
        cout << "found " << currentPointsInfront << " points in front of camera." << endl;

        if (currentPointsInfront > pointsInfront) {
            pointsInfront = currentPointsInfront;
            rotation = rotations[i];
            translation = translations[i];
        }
    }

    cout << "Counted " << pointsInfront << " points" << endl;
    cout << "Seleced rotation and translation: " << endl;
    cout << rotation << endl;
    cout << translation << endl;
}


int EssentialMatrix::pointsInfrontCamera(const cv::Mat inliners1,
                                          const cv::Mat inliners2,
                                          const cv::Mat &cameraRotation,
                                          const cv::Mat &cameraTranslation) {
    /* siehe https://answers.opencv.org/question/27155/from-fundamental-matrix-to-rectified-images/ */
    /* und */ 
    /* https://en.wikipedia.org/wiki/Essential_matrix */
    assert(inliners1.cols == 3);
    assert(inliners2.cols == 3);
    assert(inliners1.rows == inliners2.rows);
    assert(cameraRotation.cols == 3 && cameraRotation.rows == 3);
    assert(cameraTranslation.rows == 3 && cameraTranslation.cols == 1);
    
    int count = 0;
    Mat worldPoints1;
    Mat worldPoints2;
    for (size_t index = 0; index < inliners1.rows; index++) {
        double z1 = calculateDepth(cameraRotation.row(0), cameraRotation.row(2), cameraTranslation, inliners2.at<double>(index, 0), inliners1.row(index).t());

        double z2 = calculateDepth(cameraRotation.row(1), cameraRotation.row(2), cameraTranslation, inliners2.at<double>(index, 1), inliners1.row(index).t());

        double x1 = inliners1.at<double>(index, 0) * z1;
        double y1 = inliners1.at<double>(index, 1) * z1;

        double x2 = inliners1.at<double>(index, 0) * z2;
        double y2 = inliners1.at<double>(index, 1) * z2;

        worldPoints1.push_back(Mat(1, 3, CV_32FC1, {x1, y1, z1}));
        worldPoints2.push_back(Mat(1, 3, CV_32FC1, {x2, y2, z2}));

        if (z1 > 0 && z2 > 0) { 
            count++;
        }
    }

    return count;
}


double EssentialMatrix::calculateDepth(const Mat &rotationRow1,
                                       const Mat &rotationRow2,
                                       const Mat &translation,
                                       const double respectivePoint,
                                       const Mat &imagePoint1) {
    assert(rotationRow1.rows == 1 && rotationRow1.cols == 3);
    assert(rotationRow2.rows == 1 && rotationRow2.cols == 3);
    assert(translation.rows == 3 && translation.cols == 1);
    assert(imagePoint1.rows == 3 && imagePoint1.cols == 1);
    
    Mat point = (rotationRow1 - respectivePoint * rotationRow2).t();

    return point.dot(translation) / point.dot(imagePoint1);
}
