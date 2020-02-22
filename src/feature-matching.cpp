#include "feature-matching.hpp"

using namespace std;
using namespace cv;

FeatureMatching::FeatureMatching(Calibration cameraCalibration) {
    calibration = cameraCalibration;
}
