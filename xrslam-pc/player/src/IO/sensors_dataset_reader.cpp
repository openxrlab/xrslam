#include <sensors_dataset_reader.h>
#include <array>
#include <iostream>
#include <xrslam/extra/opencv_image.h>
#include <libsensors.h>

class SensorsDataParser : public libsensors::Sensors {
  public:
    SensorsDataParser(SensorsDatasetReader *reader) :
        reader(reader) {
    }

  protected:
    void on_image(double t, int width, int height, const unsigned char *bytes) override {
        std::shared_ptr<xrslam::extra::OpenCvImage> image = std::make_shared<xrslam::extra::OpenCvImage>();
        image->t = t;
        image->image = cv::Mat(cv::Size(width, height), CV_8UC4, const_cast<unsigned char *>(bytes)).clone();
        // image->image = cv::Mat(cv::Size(width, height), CV_8UC1, const_cast<unsigned char *>(bytes)).clone();
        reader->pending_images.push_back(image);
    }

    void on_accelerometer(double t, double x, double y, double z) override {
        XRSLAMAcceleration acc = {{x, y, z}, t};
        reader->pending_accelerometers.emplace_back(t, acc);
    }

    void on_gyroscope(double t, double x, double y, double z) override {
        XRSLAMGyroscope gyr = {{x, y, z}, t};
        reader->pending_gyroscopes.emplace_back(t, gyr);
    }

  private:
    SensorsDatasetReader *reader;
};

SensorsDatasetReader::SensorsDatasetReader(const std::string &filename, void *yaml_config) :
    datafile(filename.c_str(), std::ifstream::in | std::ifstream::binary) {
    if (!datafile) {
        std::cout << "Cannot open file " << filename << std::endl;
        exit(EXIT_FAILURE);
    }
    sensors = std::make_unique<SensorsDataParser>(this);
    config = std::shared_ptr<xrslam::extra::YamlConfig>(
        reinterpret_cast<xrslam::extra::YamlConfig *>(yaml_config));
}

SensorsDatasetReader::~SensorsDatasetReader() = default;

DatasetReader::NextDataType SensorsDatasetReader::next() {
    bool data_available = false;
    double image_time = std::numeric_limits<double>::max();
    double gyroscope_time = std::numeric_limits<double>::max();
    double accelerometer_time = std::numeric_limits<double>::max();
    if (pending_images.size() > 0) {
        image_time = pending_images.front()->t;
        data_available = true;
    }
    if (pending_gyroscopes.size() > 0) {
        gyroscope_time = pending_gyroscopes.front().first;
        data_available = true;
    }
    if (pending_accelerometers.size() > 0) {
        accelerometer_time = pending_accelerometers.front().first;
        data_available = true;
    }
    if (data_available) {
        if (accelerometer_time <= image_time && accelerometer_time <= gyroscope_time) {
            return DatasetReader::ACCELEROMETER;
        } else if (gyroscope_time <= image_time && gyroscope_time < accelerometer_time) {
            return DatasetReader::GYROSCOPE;
        } else {
            return DatasetReader::CAMERA;
        }
    } else {
        std::array<char, 8192> buffer;
        if (!datafile.read(buffer.data(), 8192)) {
            return DatasetReader::END;
        }
        size_t len = datafile.gcount();
        if (len == 0) {
            return DatasetReader::END;
        }
        sensors->parse_data(buffer.data(), len);
        return DatasetReader::AGAIN;
    }
}

void SensorsDatasetReader::get_image_resolution(int &width, int &height) {
    height = image_height;
    width = image_width;
}

std::pair<double, cv::Mat> SensorsDatasetReader::read_image() {
    std::shared_ptr<xrslam::extra::OpenCvImage> image = pending_images.front();
    pending_images.pop_front();

    double t = image->t;

    cv::Mat img_distorted = image->image;

    // std::string name = "./logs/" + std::to_string(t) + ".png";
    // cv::imwrite(name, img_distorted);
    cv::Mat img;
    if (config->camera_distortion_flag()) {
        xrslam::vector<4> D = config->camera_distortion();
        xrslam::matrix<3> K = config->camera_intrinsic();
        cv::Mat distortion = (cv::Mat_<float>(4, 1) << D[0], D[1], D[2], D[3]);
        cv::Mat intrinsic =
            (cv::Mat_<float>(3, 3) << K(0, 0), K(0, 1), K(0, 2), K(1, 0),
             K(1, 1), K(1, 2), K(2, 0), K(2, 1), K(2, 2));
        cv::undistort(img_distorted, img, intrinsic, distortion);
    } else {
        img = img_distorted.clone();
    }

    image_height = img.rows;
    image_width = img.cols;
    return std::make_pair(t, img);
}

std::pair<double, XRSLAMGyroscope> SensorsDatasetReader::read_gyroscope() {
    std::pair<double, XRSLAMGyroscope> gyroscope = pending_gyroscopes.front();
    pending_gyroscopes.pop_front();
    return gyroscope;
}

std::pair<double, XRSLAMAcceleration> SensorsDatasetReader::read_accelerometer() {
    std::pair<double, XRSLAMAcceleration> accelerometer = pending_accelerometers.front();
    pending_accelerometers.pop_front();
    return accelerometer;
}