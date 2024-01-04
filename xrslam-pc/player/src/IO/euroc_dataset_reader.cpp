#include <euroc_dataset_reader.h>

EurocDatasetReader::EurocDatasetReader(const std::string &euroc_path,
                                       void *yaml_config) {

    config = std::shared_ptr<xrslam::extra::YamlConfig>(
        reinterpret_cast<xrslam::extra::YamlConfig *>(yaml_config));

    CameraCsv camera_csv;
    ImuCsv imu_csv;

    camera_csv.load(euroc_path + "/cam0/data.csv");
    imu_csv.load(euroc_path + "/imu0/data.csv");

    for (auto &item : camera_csv.items) {
        image_data.emplace_back(item.t + config->camera_time_offset(),
                                euroc_path + "/cam0/data/" + item.filename);
        all_data.emplace_back(item.t + config->camera_time_offset(),
                              NextDataType::CAMERA);
    }

    for (auto &item : imu_csv.items) {
        XRSLAMGyroscope gyr = {{item.w.x, item.w.y, item.w.z}, item.t};
        gyroscope_data.emplace_back(item.t, gyr);
        all_data.emplace_back(item.t, NextDataType::GYROSCOPE);
        XRSLAMAcceleration acc = {{item.a.x, item.a.y, item.a.z}, item.t};
        accelerometer_data.emplace_back(item.t, acc);
        all_data.emplace_back(item.t, NextDataType::ACCELEROMETER);
    }

    std::stable_sort(all_data.begin(), all_data.end(),
                     [](auto &a, auto &b) { return a.first < b.first; });
    std::stable_sort(image_data.begin(), image_data.end(),
                     [](auto &a, auto &b) { return a.first < b.first; });
    std::stable_sort(gyroscope_data.begin(), gyroscope_data.end(),
                     [](auto &a, auto &b) { return a.first < b.first; });
    std::stable_sort(accelerometer_data.begin(), accelerometer_data.end(),
                     [](auto &a, auto &b) { return a.first < b.first; });
}

DatasetReader::NextDataType EurocDatasetReader::next() {
    if (all_data.empty()) {
        return NextDataType::END;
    }
    auto [t, type] = all_data.front();
    return type;
}

void EurocDatasetReader::get_image_resolution(int &width, int &height) {
    height = image_height;
    width = image_width;
}

std::pair<double, cv::Mat> EurocDatasetReader::read_image() {
    if (image_data.empty()) {
        return {};
    }
    auto [t, filename] = image_data.front();
    cv::Mat img_distorted = cv::imread(filename, cv::IMREAD_UNCHANGED);

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

    if (img.channels() != 1)
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);

    image_height = img.rows;
    image_width = img.cols;

    all_data.pop_front();
    image_data.pop_front();
    return std::make_pair(t, img);
}

std::pair<double, XRSLAMGyroscope> EurocDatasetReader::read_gyroscope() {
    if (gyroscope_data.empty()) {
        return {};
    }
    auto item = gyroscope_data.front();

    all_data.pop_front();
    gyroscope_data.pop_front();
    return item;
}

std::pair<double, XRSLAMAcceleration> EurocDatasetReader::read_accelerometer() {
    if (accelerometer_data.empty()) {
        return {};
    }
    auto item = accelerometer_data.front();

    all_data.pop_front();
    accelerometer_data.pop_front();
    return item;
}
