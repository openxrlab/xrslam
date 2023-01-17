#ifndef XRSLAM_DATASET_READER_H
#define XRSLAM_DATASET_READER_H

#include <xrslam/bridge.h> 

class DatasetReader {
  public:
    enum NextDataType { AGAIN, CAMERA, GYROSCOPE, ACCELEROMETER, END };
    virtual ~DatasetReader() = default;
    virtual NextDataType next() = 0;
    virtual std::shared_ptr<xrslam::Image> read_image() = 0;
    virtual std::pair<double, xrslam::vector<3>> read_gyroscope() = 0;
    virtual std::pair<double, xrslam::vector<3>> read_accelerometer() = 0;

    static std::unique_ptr<DatasetReader>
    create_reader(const std::string &filename, bool async = true);
};

#endif // XRSLAM_DATASET_READER_H
