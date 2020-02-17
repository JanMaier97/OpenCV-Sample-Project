#include <calibration.hpp>
#include <string>
#include <filesystem>


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

    printf("Execution successful");
}
