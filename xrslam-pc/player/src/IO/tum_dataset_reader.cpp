#include <tum_dataset_reader.h>

TUMDatasetReader::TUMDatasetReader(const std::string &tum_path,
                                   void *yaml_config) {
    config = std::shared_ptr<xrslam::extra::YamlConfig>(
        reinterpret_cast<xrslam::extra::YamlConfig *>(yaml_config));

    TUMCameraCsv camera_csv;
    TUMImuCsv imu_csv;

    camera_csv.load(tum_path + "/cam0/data.csv");
    imu_csv.load(tum_path + "/imu0/data.csv");

    for (auto &item : camera_csv.items) {
        // printf("read image %lf\n", item.t);
        image_data.emplace_back(item.t + config->camera_time_offset(),
                                tum_path + "/cam0/data/" + item.filename);
        all_data.emplace_back(item.t + config->camera_time_offset(),
                              NextDataType::CAMERA);
    }
    num_images = image_data.size();

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

DatasetReader::NextDataType TUMDatasetReader::next() {
    if (all_data.empty()) {
        return NextDataType::END;
    }
    auto [t, type] = all_data.front();
    return type;
}

void TUMDatasetReader::get_image_resolution(int &width, int &height) {
    height = image_height;
    width = image_width;
}

std::pair<double, cv::Mat> TUMDatasetReader::read_image() {
    if (image_data.empty()) {
        return {};
    }
    auto [t, filename] = image_data.front();
    cv::Mat img_distorted = cv::imread(filename, cv::IMREAD_GRAYSCALE);

    cv::Mat img;
    if (!image_undistorter) {
        xrslam::matrix<3> intrinsic = config->camera_intrinsic();
        xrslam::vector<4> D = config->camera_distortion();
        std::vector<double> distortion = {D[0], D[1], D[2], D[3]};
        image_undistorter = std::make_unique<xrslam::extra::ImageUndistorter>(
            img_distorted.cols, img_distorted.rows, intrinsic, distortion,
            "equidistant");
    }
    if (config->camera_distortion_flag()) {
        image_undistorter->undistort_image(img_distorted, img);
    } else {
        img = img_distorted.clone();
    }

    image_height = img.rows;
    image_width = img.cols;

    all_data.pop_front();
    image_data.pop_front();
    return std::make_pair(t, img);
}

std::pair<double, XRSLAMGyroscope> TUMDatasetReader::read_gyroscope() {
    if (gyroscope_data.empty()) {
        return {};
    }
    auto item = gyroscope_data.front();

    all_data.pop_front();
    gyroscope_data.pop_front();
    return item;
}

std::pair<double, XRSLAMAcceleration> TUMDatasetReader::read_accelerometer() {
    if (accelerometer_data.empty()) {
        return {};
    }
    auto item = accelerometer_data.front();

    all_data.pop_front();
    accelerometer_data.pop_front();
    return item;
}
