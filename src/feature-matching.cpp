#include "feature-matching.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>

using namespace std;
using namespace cv;

FeatureMatching::FeatureMatching(Calibration cameraCalibration) {
    calibration = cameraCalibration;
}

vector<Mat> FeatureMatching::findMatches(Mat image1, Mat image2) {
    Mat processedImage1 = preprocessImage(image1);
    Mat processedImage2 = preprocessImage(image2);

    int win_size = 5;
    Ptr<StereoSGBM> stereo = StereoSGBM::create(-1, 64, 5, 8*3*pow(win_size,2), 32*3*pow(win_size,2), 1, 0, 5, 5, 5);

    Mat disparityMap;
    stereo->compute(processedImage1, processedImage2, disparityMap);

    namedWindow("Image1", WINDOW_FREERATIO);
    imshow("Image1", disparityMap);
    waitKey(0);

    cout << "computed disparityMap" << endl;
    return vector<Mat>();
}


Mat FeatureMatching::preprocessImage(Mat image) {
    Mat processedImage = calibration.undistortImage(image);
    /* processedImage = */ 
    /* namedWindow("Image1", WINDOW_FREERATIO); */
    /* namedWindow("Image2", WINDOW_FREERATIO); */

    /* imshow("Image1", image); */
    /* imshow("Image2", processedImage); */

    /* cout << "processing done" << endl; */
    /* waitKey(0); */
    
    return processedImage;
}
