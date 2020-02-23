#include "feature-matching.hpp"
#include "calibration.hpp"
#include <string>
#include <filesystem>

#include <iostream>


using namespace std;

int main(int argc, char *argv[]) {
    Calibration calb = Calibration();

    filesystem::path path("./resources/chessboard_images");

    vector<filesystem::path> filepaths;
    for (const auto& entry : filesystem::directory_iterator(path)) {
        filepaths.push_back(entry.path());
    }

    // number of inner corners per chessboard row and column
    unsigned int cornersRow = 6;
    unsigned int cornersColumn = 9;


    calb.calibrate(filepaths, cv::Size(cornersColumn, cornersRow));

    cout << "calibration successful" << endl;

   
    cv::Mat img1 = cv::imread("./resources/guitar_images/20200223_125417880_iOS.jpg");
    /* cv::Mat img1 = cv::imread("./resources/guitar_images/20200223_125427418_iOS.jpg", cv::IMREAD_GRAYSCALE); */
    cv::Mat img2 = cv::imread("./resources/guitar_images/20200223_125447874_iOS.jpg");

    /* cv::namedWindow("Matches", cv::WINDOW_FREERATIO); */
    /* cv::imshow("Matches", img1); */
    /* cv::waitKey(0); */
    /* cv::imshow("Matches", img2); */
    /* cv::waitKey(0); */

    FeatureMatching featureMatching = FeatureMatching(calb);
    featureMatching.findMatches(img1, img2);
    
    cout << "matching successful" << endl;
}
