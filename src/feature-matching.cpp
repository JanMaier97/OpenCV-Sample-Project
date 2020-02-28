#include "feature-matching.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

using namespace std;
using namespace cv;

FeatureMatching::FeatureMatching(Calibration cameraCalibration) {
    calibration = cameraCalibration;
}

void FeatureMatching::findMatches(Mat image1, Mat image2, vector<Point2f> &imagePoints1, vector<Point2f> &imagePoints2) {
    Mat processedImage1; 
    Mat processedImage2;

    // apply filters and undistortion
    preprocessImage(image1, processedImage1);
    preprocessImage(image2, processedImage2);

    vector<KeyPoint> keyPoints1;
    vector<KeyPoint> keyPoints2;
    Mat descriptors1;
    Mat descriptors2;
    // parameters: nfeatures=500, scaleFactor=1.2f, nlevels=8, edgeThreshold=31, firstLevel=0, WTA_K=2, scoreType=ORB::HARRIS_SCORE, patchSize=31, fastThreshold=20
    Ptr<ORB> orb = ORB::create(2000, 1.2f, 8, 31, 0, 4, ORB::HARRIS_SCORE, 31, 30);

    // compute keypoints and descriptors using ORB
    orb->detectAndCompute(processedImage1, noArray(), keyPoints1, descriptors1);
    orb->detectAndCompute(processedImage2, noArray(), keyPoints2, descriptors2);

    // use NORM_HAMMING2 when WTA_K = 3 or 4 for ORB
    Ptr<BFMatcher> matcher =  BFMatcher::create(NORM_HAMMING2, true);

    // find matches using bruteforce
    vector<DMatch> matches;
    matcher->match(descriptors1, descriptors2, matches);

    // filter matches
    vector<DMatch> filteredMatches;
    filterMatches(keyPoints1, keyPoints2, matches, filteredMatches, image1.size());

    /* // show matches in a new window */
    /* Mat drawnMatches; */
    /* drawMatches(processedImage1, keyPoints1, processedImage2, keyPoints2, filteredMatches, drawnMatches);; */
    /* namedWindow("Matches", WINDOW_FREERATIO); */
    /* imshow("Matches", drawnMatches); */
    /* waitKey(0); */

    // crate index mask for keypoints from the filtered matches
    vector<int> keypointIndexes1;
    vector<int> keypointIndexes2;
    
    for (size_t i=0; i < filteredMatches.size(); i++) {
        keypointIndexes1.push_back(filteredMatches[i].trainIdx);;
        keypointIndexes2.push_back(filteredMatches[i].queryIdx);
    }

    // return masked keypoints as points2f
    KeyPoint::convert(keyPoints1, imagePoints1, keypointIndexes1);
    KeyPoint::convert(keyPoints2, imagePoints2, keypointIndexes2);

    /* cout << key.size() << endl; */
    /* cout << KeyPoint.size() << endl; */

    /* cout << "finding fundamental" << endl; */
    /* Mat fundamentalMask; */
    /* Mat fundamentalCameraMat = cv::findFundamentalMat(imagePoints1, imagePoints2, FM_RANSAC, 3, 0.99, fundamentalMask); */
    /* Mat essentialCameraMat = cv::findEssentialMat(imagePoints1, imagePoints2, calibration.getCameraMatrix()); */
    /* Mat essentialCameraMat2 = calibration.getInstrincs().t() * fundamentalCameraMat * calibration.getInstrincs(); */

    /* vector<uchar> indexes; */
    /* if (fundamentalMask.isContinuous()) { */
    /*     indexes.assign(fundamentalMask.data, fundamentalMask.data + fundamentalMask.total()); */
    /* } */
    /* cout << indexes << endl; */

    /* vector<Point2f> filteredImagePoints1; */
    /* vector<Point2f> filteredImagePoints2; */

    /* for (size_t i = 0; i < indexes.size(); i++) { */
    /*     if (indexes[i] == 1) { */
    /*         filteredImagePoints1.push_back(imagePoints1[i]); */
    /*         filteredImagePoints2.push_back(imagePoints2[i]); */
    /*     } */
    /* } */

    /* vector<cv::Vec3f> lines1; */
    /* vector<cv::Vec3f> lines2; */
    /* computeCorrespondEpilines(filteredImagePoints2, 2, fundamentalCameraMat, lines1); */
    /* computeCorrespondEpilines(filteredImagePoints1, 1, fundamentalCameraMat, lines2); */

    /* drawEpipolarLines(processedImage1, processedImage1, processedImage2, filteredImagePoints1, filteredImagePoints2, lines1); */

    /* drawEpipolarLines(processedImage2, processedImage2, processedImage1, filteredImagePoints2, filteredImagePoints1, lines2); */

    /* namedWindow("Image1", WINDOW_FREERATIO); */
    /* imshow("Image1", processedImage1); */
    /* namedWindow("Image2", WINDOW_FREERATIO); */
    /* imshow("Image2", processedImage2); */
    /* waitKey(0); */
}



void FeatureMatching::preprocessImage(Mat image, OutputArray processedImage) {

    // remove distortion from image so that straight lines are straigt
    calibration.undistortImage(image, processedImage);

    // transform to grey color, with only one channel (required by histogramm)
    cvtColor(processedImage, processedImage, COLOR_BGR2GRAY);   
    
    /* Mat convertedImage; */
    /* image.convertTo(convertedImage, CV_8UC1, 1.0/255); */

    //TODO: test out filters (Heurisitc, FFT)
    equalizeHist(processedImage, processedImage);
}


void FeatureMatching::filterMatches(const vector<KeyPoint> keypoints1, const vector<KeyPoint> keypoints2, const vector<DMatch> &matches, vector<DMatch> &filteredMatches, Size imageSize){
    
    // max distance for matches is 25% of the max distance / image diagonal
    double distanceLimit = 0.25f * sqrt(pow(imageSize.height, 2) + pow(imageSize.width, 2));
    double maxHeightDifference = 0.05f * imageSize.height;
    double minHeightDifference = 100;


    int filterCounter = 0;
    for (size_t index = 0; index < matches.size(); index++) {
        DMatch match = matches[index];
        KeyPoint queryKeyPoint = keypoints1[match.queryIdx];
        KeyPoint trainKeyPoint = keypoints2[match.trainIdx];

        double currentDistance = sqrt(pow(queryKeyPoint.pt.x - trainKeyPoint.pt.x, 2) + pow(queryKeyPoint.pt.y - trainKeyPoint.pt.y, 2));
        double currentHeightDifference = abs(queryKeyPoint.pt.y - trainKeyPoint.pt.y);

        if ( currentDistance < distanceLimit && currentHeightDifference < maxHeightDifference && queryKeyPoint.pt.y - trainKeyPoint.pt.y > minHeightDifference) {
            filteredMatches.push_back(match);
            /* cout << queryKeyPoint.pt.y - trainKeyPoint.pt.y << endl; */
        } else {
            /* cout << "Current Height diff: "<< currentHeightDifference << " vs " << maxHeightDifference << endl; */
            /* filteredMatches.push_back(match); */
            filterCounter++;
        }
    }
    cout << "Filtered matches: " << filterCounter << endl;
}

/* void FeatureMatching::drawEpipolarLines(cv::Mat& image_out, */
/* cv::Mat& image1, */
/* cv::Mat& image2, */
/* std::vector<cv::Point2f>& points1, // keypoints 1 */
/* std::vector<cv::Point2f>& points2, // keypoints 2 */
/* vector<Vec3f> lines) // image to compute epipolar lines in */
/* { */
/*     // Compute F matrix from 7 matches */ 
/*     /1* cv::Mat F = cv::findFundamentalMat(cv::Mat(points1), // points in object image *1/ */
/*     /1*                                    cv::Mat(points2), // points in scene image *1/ */
/*     /1*                                    cv::FM_7POINT); // 7-point method *1/ */
/*     /1* std::vector<cv::Vec3f> lines1; *1/ */

/*     /1* // Compute corresponding epipolar lines *1/ */
/*     /1* cv::computeCorrespondEpilines(cv::Mat(points1), // image points *1/ */
/*     /1*                               whichImage, // in image 1 (can also be 2) *1/ */
/*     /1*                               F, // F matrix *1/ */
/*     /1*                               lines); // vector of epipolar lines *1/ */
/*     // for all epipolar lines */
/*     for (std::vector<cv::Vec3f>::const_iterator it = lines.begin(); it!=lines.end(); ++it) */
/*     { */
/*         // Draw the line between first and last column */
/*         cv::line(image_out, */
/*                  cv::Point(0,-(*it)[2]/(*it)[1]), */
/*                  cv::Point(image2.cols, */
/*                            -((*it)[2] + (*it)[0] * image2.cols) / (*it)[1]), */
/*                  cv::Scalar(255,255,255)); */
/*     } */
/* } */
