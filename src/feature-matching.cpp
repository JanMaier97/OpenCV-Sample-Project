#include "feature-matching.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>

#include <iostream>

using namespace std;
using namespace cv;

FeatureMatching::FeatureMatching(Calibration cameraCalibration) {
    calibration = cameraCalibration;
}

vector<Mat> FeatureMatching::findMatches(Mat image1, Mat image2) {
    Mat processedImage1; 
    Mat processedImage2;

    preprocessImage(image1, processedImage1);
    preprocessImage(image2, processedImage2);

    vector<KeyPoint> keyPoints1;
    vector<KeyPoint> keyPoints2;
    Mat descriptors1;
    Mat descriptors2;
    // parameters: nfeatures=500, scaleFactor=1.2f, nlevels=8, edgeThreshold=31, firstLevel=0, WTA_K=2, scoreType=ORB::HARRIS_SCORE, patchSize=31, fastThreshold=20
    Ptr<ORB> orb = ORB::create(2000, 1.2f, 8, 31, 0, 4, ORB::HARRIS_SCORE, 31, 30);

    orb->detectAndCompute(image1, noArray(), keyPoints1, descriptors1);
    orb->detectAndCompute(image2, noArray(), keyPoints2, descriptors2);

    // use NORM_HAMMING2 when WTA_K = 3 or 4 for ORB
    Ptr<BFMatcher> matcher =  BFMatcher::create(NORM_HAMMING2, true);
    vector<DMatch> matches;

    matcher->match(descriptors1, descriptors2, matches);

    vector<DMatch> filteredMatches;
    filterMatches(keyPoints1, keyPoints2, matches, filteredMatches, image1.size());

    Mat drawnMatches;
    drawMatches(image1, keyPoints1, image2, keyPoints2, filteredMatches, drawnMatches);;

    namedWindow("Matches", WINDOW_FREERATIO);
    imshow("Matches", drawnMatches);
    waitKey(0);

    /* int win_size = 5; */
    /* Ptr<StereoSGBM> stereo = StereoSGBM::create(1, 32, 5, 8*3*pow(win_size,2), 32*3*pow(win_size,2), 1, 0, 5, 5, 5); */

    /* Mat disparityMap; */
    /* stereo->compute(processedImage1, processedImage2, disparityMap); */

    /* namedWindow("Image1", WINDOW_FREERATIO); */
    /* imshow("Image1", disparityMap); */
    /* waitKey(0); */

    /* cout << "computed disparityMap" << endl; */

    return vector<Mat>();
}


void FeatureMatching::preprocessImage(InputArray image, OutputArray processedImage) {
    calibration.undistortImage(image, processedImage);
    //TODO: test out filters (Heurisitc, FFT)
}


void FeatureMatching::filterMatches(const vector<KeyPoint> keypoints1, const vector<KeyPoint> keypoints2, const vector<DMatch> &matches, vector<DMatch> &filteredMatches, Size imageSize){
    
    // max distance for matches is 25% of the max distance / image diagonal
    double distanceLimit = 0.25f * sqrt(pow(imageSize.height, 2) + pow(imageSize.width, 2));
    double maxHeightDifference = 0.1f * imageSize.height;


    int filterCounter = 0;
    for (size_t index = 0; index < matches.size(); index++) {
        DMatch match = matches[index];
        KeyPoint queryKeyPoint = keypoints1[match.queryIdx];
        KeyPoint trainKeyPoint = keypoints2[match.trainIdx];

        double currentDistance = sqrt(pow(queryKeyPoint.pt.x - trainKeyPoint.pt.x, 2) + pow(queryKeyPoint.pt.y - trainKeyPoint.pt.y, 2));
        double currentHeightDifference = abs(queryKeyPoint.pt.y - trainKeyPoint.pt.y);



        if (currentDistance < distanceLimit && currentHeightDifference < maxHeightDifference) {
            filteredMatches.push_back(match);
        } else {
            cout << "Current Height diff: "<< currentHeightDifference << " vs " << maxHeightDifference << endl;
            /* filteredMatches.push_back(match); */
            filterCounter++;
        }
    }
    cout << "Filtered matches: " << filterCounter << endl;
}
