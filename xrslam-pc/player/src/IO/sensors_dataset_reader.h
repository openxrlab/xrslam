#ifndef XRSLAM_PC_SENSORS_DATASET_READER_H
#define XRSLAM_PC_SENSORS_DATASET_READER_H

#include <deque>
#include <fstream>
#include <dataset_reader.h>
#include <xrslam/extra/opencv_image.h>

namespace libsensors {
class Sensors;
}

class SensorsDataParser;

class SensorsDatasetReader : public DatasetReader {
  public:
    SensorsDatasetReader(const std::string &filename, void *yaml_config);
    ~SensorsDatasetReader();
    NextDataType next() override;

    void get_image_resolution(int &width, int &height) override;
    std::pair<double, cv::Mat> read_image() override;
    std::pair<double, XRSLAMGyroscope> read_gyroscope() override;
    std::pair<double, XRSLAMAcceleration> read_accelerometer() override;

  private:
    friend class SensorsDataParser;
    std::shared_ptr<xrslam::extra::YamlConfig> config;
    std::ifstream datafile;
    std::unique_ptr<libsensors::Sensors> sensors;
    std::deque<std::shared_ptr<xrslam::extra::OpenCvImage>> pending_images;
    std::deque<std::pair<double, XRSLAMGyroscope>> pending_gyroscopes;
    std::deque<std::pair<double, XRSLAMAcceleration>> pending_accelerometers;
    int image_width;
    int image_height;
};

#endif
