#include "model-exporters.hpp"

#include <exception>
#include <fstream>
#include <iostream>


using namespace std;
using namespace cv;


void PlyModelExporter::exportPointCloud(const filesystem::path& filepath,
                                        const Mat& worldPoints) {
    assert(worldPoints.rows == 3);

    if (filepath.has_parent_path() && !filesystem::exists(filepath.parent_path()))
        throw runtime_error("The directory path for the file " + filepath.string() + " does not exist.");

    if (filepath.extension() != ".ply")
        throw runtime_error("Invalid file extension " + filepath.extension().string() + ". Exptected .ply");

    ofstream outputfile(filepath);

    if (outputfile.is_open()) {
        writeHeader(outputfile, worldPoints);
        writeVertexList(outputfile, worldPoints);
        outputfile.close();
    } else {
        throw runtime_error("Failed to open file " + filepath.string());
    }
}

void PlyModelExporter::writeHeader(ofstream& outputfile, const Mat& vertices) {
    outputfile << "ply" << endl;
    outputfile << "format ascii 1.0" << endl;
    outputfile << "element vertex " << vertices.cols << endl;
    outputfile << "property float x" << endl;
    outputfile << "property float y" << endl;
    outputfile << "property float z" << endl;
    /* outputfile << "property uchar red" << endl; */
    /* outputfile << "property uchar green" << endl; */
    /* outputfile << "property uchar blue" << endl; */
    /* outputfile << "element face 7" << endl; */
    /* outputfile << "property list uchar int vertex_index" << endl; */
    /* outputfile << "element edge 5" << endl; */
    /* outputfile << "property int vertex1" << endl; */
    /* outputfile << "property int vertex2" << endl; */
    /* outputfile << "property uchar red" << endl; */
    /* outputfile << "property uchar green" << endl; */
    /* outputfile << "property uchar blue" << endl; */
    outputfile << "end_header" << endl;
}

void PlyModelExporter::writeVertexList(ofstream& outputfile, const Mat& vertices) {
    cout << vertices << endl;

    for (size_t column = 0; column < vertices.cols; column++) {
        outputfile << vertices.at<float>(0, column) << " ";
        outputfile << vertices.at<float>(1, column) << " ";
        outputfile << vertices.at<float>(2, column) << endl;
    }

}