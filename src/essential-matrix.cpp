#include "essentialMatrix.hpp"
#include <opencv2/calib3d.hpp>

#include <iostream>

using namespace std;
using namespace cv;

EssentialMatrix::EssentialMatrix(Calibration cameraCalibration) {
    this->cameraCalibration = cameraCalibration;
}

void EssentialMatrix::determineEssentialMatrix(const vector<Point2f> &pointsCamera1, const vector<Point2f> &pointsCamera2, OutputArray &cameraRotation, OutputArray &cameraTranslation) {
    Mat pointMask;
    Mat essentialMatrix = findEssentialMat(pointsCamera1, pointsCamera2, cameraCalibration.getCameraMatrix(), RANSAC, 0.999, 1, pointMask);


    // decomposes the essential matrix into 2 possible rotation matrices and 1 translation vector
    Mat rotation1, rotation2, translation;
    decomposeEssentialMat(essentialMatrix, rotation1, rotation2, translation);


    /* cout << essentialMatrix <<endl; */
    /* cout << rotation1 <<endl; */
    /* cout << rotation2 <<endl; */
    /* cout << translation <<endl; */

    Mat I = (Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    Mat t = (Mat_<double>(3,1) << 0, 0, 0);
    Mat extrincts_p1;

    hconcat(I, t, extrincts_p1);
    Mat p1 = cameraCalibration.getInstrincs() * extrincts_p1;

    Mat extrincts_p2_1;
    Mat extrincts_p2_2;
    Mat extrincts_p2_3;
    Mat extrincts_p2_4;

    hconcat(rotation1, translation, extrincts_p2_1);
    hconcat(rotation1, -translation, extrincts_p2_2);
    hconcat(rotation2, translation, extrincts_p2_3);
    hconcat(rotation2, -translation, extrincts_p2_4);
    
    Mat p2_1 = cameraCalibration.getInstrincs() * extrincts_p2_1;
    Mat p2_2 = cameraCalibration.getInstrincs() * extrincts_p2_2;
    Mat p2_3 = cameraCalibration.getInstrincs() * extrincts_p2_3;
    Mat p2_4 = cameraCalibration.getInstrincs() * extrincts_p2_4;

    /* projectionCamera1 = cameraRotation.getInstrincs() * */ 

    cv::Mat worldPoints1(1, pointsCamera1.size(), CV_64FC4);
    cv::Mat worldPoints2(1, pointsCamera1.size(), CV_64FC4);
    cv::Mat worldPoints3(1, pointsCamera1.size(), CV_64FC4);
    cv::Mat worldPoints4(1, pointsCamera1.size(), CV_64FC4);

    cv::triangulatePoints(p1, p2_1, pointsCamera1, pointsCamera2, worldPoints1);
    cv::triangulatePoints(p1, p2_2, pointsCamera1, pointsCamera2, worldPoints2);
    cv::triangulatePoints(p1, p2_3, pointsCamera1, pointsCamera2, worldPoints3);
    cv::triangulatePoints(p1, p2_4, pointsCamera1, pointsCamera2, worldPoints4);
    
    cout << worldPoints1 << endl;
    cout << worldPoints2 << endl;
    cout << worldPoints3 << endl;
    cout << worldPoints4 << endl;

    cout << " counting ..." << endl;
    numberOfPointsInfront(worldPoints1, rotation1, translation);

}


Mat EssentialMatrix::computeCameraCenter(const Mat &cameraRotation,
                                         const Mat &cameraTranslation) {
    return -1 * cameraRotation.t() * cameraTranslation;
}


int EssentialMatrix::numberOfPointsInfront(const Mat &worldPoints,
                                           const Mat &cameraRotation,
                                           const Mat &cameraTranslation) {

    Mat center = - cameraRotation.t() * cameraTranslation;
    Mat viewDirection = cameraRotation.col(2).t();

    unsigned int pointsInfront = 0;
   
    for (size_t column=0; column < worldPoints.cols; column++) {
        double data[3];
        cout << "data" << endl;
        for (int i=0; i < 3; i++) {
            data[i] = worldPoints.at<double>(i, column) / worldPoints.at<double>(3, column);
            cout << data[i] << endl;
        }

        cout << data << endl;
        cout << worldPoints.col(column) << endl;

        Mat normalizedPoint (3, 1, center.type(), data);
        cout << "normalized" << endl;
        cout << normalizedPoint  << endl;
        cout << viewDirection  << endl;
        cout << center << endl;
        

        if ((normalizedPoint - center).dot(viewDirection) > 0) {
            pointsInfront++;
            
        }
    }


    return 0;
}
