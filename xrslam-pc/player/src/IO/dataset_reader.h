#ifndef XRSLAM_PC_DATASET_READER_H
#define XRSLAM_PC_DATASET_READER_H

#include <memory>
#include <utility>
#include <vector>
#include <algorithm>

#include <opencv2/opencv.hpp>

#include "XRSLAM.h"

class DatasetReader {
  public:
    enum NextDataType { AGAIN, CAMERA, GYROSCOPE, ACCELEROMETER, END };
    virtual ~DatasetReader() = default;
    virtual NextDataType next() = 0;
    virtual void get_image_resolution(int &width, int &height) = 0;
    virtual std::pair<double, cv::Mat> read_image() = 0;
    virtual std::pair<double, XRSLAMGyroscope> read_gyroscope() = 0;
    virtual std::pair<double, XRSLAMAcceleration> read_accelerometer() = 0;

    static std::unique_ptr<DatasetReader>
    create_reader(const std::string &filename, bool async = true);
};

#endif // XRSLAM_PC_DATASET_READER_H