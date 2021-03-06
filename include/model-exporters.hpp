#ifndef MODEL_EXPORTERS_H
#define MODEL_EXPORTERS_H

#include "model-exporter.hpp"

class PlyModelExporter : ModelExporter {
    
    public:
        void exportPointCloud(const std::filesystem::path& filepath,
                              const cv::Mat& worldPoints) override;

    private:
        cv::Mat worldPoints;

        void writeHeader(std::ofstream& outputfile,
                         const cv::Mat& vertices);
        void writeVertexList(std::ofstream& outputfile,
                             const cv::Mat& vertices);

};

#endif
