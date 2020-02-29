#include "model-exporter.hpp"

class PlyModelExporter : ModelExporter {
    
    public:
        void exportPointCloud(const std::filesystem::path& filepath,
                              const cv::Mat& worldPoints);

    private:
        cv::Mat worldPoints;

        void writeHeader();
        void writeVertexList();

};
