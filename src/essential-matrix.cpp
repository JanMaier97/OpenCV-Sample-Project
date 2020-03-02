#include "essentialMatrix.hpp"
#include <opencv2/calib3d.hpp>

#include <iostream>

using namespace std;
using namespace cv;

EssentialMatrix::EssentialMatrix(Calibration cameraCalibration) {
    this->cameraCalibration = cameraCalibration;
}


Mat EssentialMatrix::normalizePoints(const Mat& K_inverse,
                                     const vector<Point2f>& imagePoints) {
    Mat normalizedPoints(imagePoints.size(), 3, K_inverse.type());

    for (const auto &point: imagePoints) {
        Mat x = (Mat_<double>(1,3) << point.x, point.y, 1.0);
        cout << x.size() << " vs " << normalizedPoints.cols << endl;
        normalizedPoints.push_back(
                /* (K_inverse * (Mat_<double>(3,1) << point.x, point.y, 1.0)).t() */
            /* (Mat_<double>(1,3) << point.x, point.y, 1.0) */
            /* vector<double>{point.x, point.y, 1.0} */
            x
        );
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

void EssentialMatrix::determineEssentialMatrix(
                            const vector<Point2f> &pointsCamera1,
                            const vector<Point2f> &pointsCamera2, 
                            OutputArray &cameraRotation,
                            OutputArray &cameraTranslation) {

    cout << "Determining essentialMatrix" << endl;

    /* Mat fundamentalMask(pointsCamera1.size(), 1, CV_8UC1); */
    vector<uchar> fundamentalMask;
    Mat fundametalMatrix = findFundamentalMat(pointsCamera1, pointsCamera2, FM_RANSAC, 0.1, 0.99, fundamentalMask);

    cout << " fundametalMatrix:" << endl;
    cout << fundametalMatrix << endl;
    /* cout << "mask:" << endl; */
    /* cout << fundamentalMask << endl; */

    Mat K_inverse = cameraCalibration.getInstrincs().inv();
    Mat inliers1 = normalizePoints(K_inverse, applyMask<Point2f>(pointsCamera1, fundamentalMask));
    Mat inliers2 = normalizePoints(K_inverse, applyMask<Point2f>(pointsCamera2, fundamentalMask));

    Mat essentialMatrix = cameraCalibration.getInstrincs().t() * fundametalMatrix * cameraCalibration.getInstrincs();

    cout << "Essential matrix: " << endl;
    cout << essentialMatrix << endl;

    // decomposes the essential matrix into 2 possible rotation matrices and 1 translation vector
    Mat rotation, translation;
    decomposeEssentialMatrix(essentialMatrix, inliers1, inliers2,  rotation,  translation);

    /* Mat I = (Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1); */
    /* Mat t = (Mat_<double>(3,1) << 0, 0, 0); */
    /* Mat extrincts_p1; */

    /* cout << I.type() << endl; */

    /* hconcat(I, t, extrincts_p1); */
    /* Mat p1 = cameraCalibration.getInstrincs() * extrincts_p1; */

    /* Mat extrincts_p2_1; */
    /* Mat extrincts_p2_2; */
    /* Mat extrincts_p2_3; */
    /* Mat extrincts_p2_4; */

    /* hconcat(rotation1, translation, extrincts_p2_1); */
    /* hconcat(rotation1, -translation, extrincts_p2_2); */
    /* hconcat(rotation2, translation, extrincts_p2_3); */
    /* hconcat(rotation2, -translation, extrincts_p2_4); */
    
    /* Mat p2_1 = cameraCalibration.getInstrincs() * extrincts_p2_1; */
    /* Mat p2_2 = cameraCalibration.getInstrincs() * extrincts_p2_2; */
    /* Mat p2_3 = cameraCalibration.getInstrincs() * extrincts_p2_3; */
    /* Mat p2_4 = cameraCalibration.getInstrincs() * extrincts_p2_4; */
    
    /* projectionCamera1 = cameraRotation.getInstrincs() * */ 

    /* cv::Mat worldPoints1(1, pointsCamera1.size(), CV_64FC4); */
    /* cv::Mat worldPoints2(1, pointsCamera1.size(), CV_64FC4); */
    /* cv::Mat worldPoints3(1, pointsCamera1.size(), CV_64FC4); */
    /* cv::Mat worldPoints4(1, pointsCamera1.size(), CV_64FC4); */

    /* cv::triangulatePoints(p1, p2_1, pointsCamera1, pointsCamera2, worldPoints1); */
    /* cv::triangulatePoints(p1, p2_2, pointsCamera1, pointsCamera2, worldPoints2); */
    /* cv::triangulatePoints(p1, p2_3, pointsCamera1, pointsCamera2, worldPoints3); */
    /* cv::triangulatePoints(p1, p2_4, pointsCamera1, pointsCamera2, worldPoints4); */
    
    /* cout << worldPoints1 << endl; */
    /* cout << worldPoints2 << endl; */
    /* cout << worldPoints3 << endl; */
    /* cout << worldPoints4 << endl; */

    /* cout << " counting ..." << endl; */
    /* numberOfPointsInfront(worldPoints1, rotation1, translation); */
}


Mat EssentialMatrix::computeCameraCenter(const Mat &cameraRotation,
                                         const Mat &cameraTranslation) {
    return -1 * cameraRotation.t() * cameraTranslation;
}



void EssentialMatrix::decomposeEssentialMatrix(const Mat &essentialMatrix,
                                               const Mat &inliers1,
                                               const Mat &inliers2,
                                               Mat &rotation,
                                               Mat &translation) {

    assert(essentialMatrix.rows == 3 && essentialMatrix.cols == 3);
    assert(inliers1.cols == 3);
    assert(inliers2.cols == 3);
    
    Mat rotation1, rotation2, trans;
    decomposeEssentialMat(essentialMatrix, rotation1, rotation2, trans);

    int pointsInfront = 0;
    int currentPointsInfront = 0;
    vector<Mat> rotations = {rotation1, rotation1, rotation2, rotation2};
    vector<Mat> translations = {trans, -trans, trans, -trans};

    for (size_t i = 0; i < rotations.size(); i++) {
        currentPointsInfront = pointsInfrontCamera(inliers1, inliers2, rotations[i], translations[i]);

        if (currentPointsInfront > pointsInfront) {
            pointsInfront = currentPointsInfront;
            rotation = rotations[i];
            translation = translations[i];
        }
        if (currentPointsInfront == pointsInfront) {
            cout << "found rotation and translation with the same count of " << pointsInfront << endl;
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

    cout << "checking points in front of camera." << endl;
    
    int count = 0;
    for (size_t index = 0; index < inliners1.rows; index++) {

        double first_z = calculateDepth(cameraRotation.row(0), cameraRotation.row(2), cameraTranslation, inliners2.at<double>(index, 0), inliners1.row(index).t());

        double second_z = calculateDepth(cameraRotation.row(1), cameraRotation.row(2), cameraTranslation, inliners2.at<double>(index, 1), inliners1.row(index).t());

        cout << first_z << " && " << second_z << endl;
        if (first_z > 0 && second_z > 0) { 
            count++;
        }

        /* if (first_z < 0 || second_z << 0) */
        /*     return false; */

    }
    /* cout << "Counted " << count << " points from " << inliners1.size() << " in camera" << endl; */
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

    /* cout << point.size() << endl; */
    /* cout << translation.size() << endl; */
    /* cout << imagePoint1.size() << endl; */

    return point.dot(translation) / point.dot(imagePoint1);
}


