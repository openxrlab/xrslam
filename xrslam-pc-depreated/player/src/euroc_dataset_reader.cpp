#include <euroc_dataset_reader.h>
#include <xrslam/extra/opencv_image.h>

EurocDatasetReader::EurocDatasetReader(const std::string &euroc_path) {
    CameraCsv camera_csv;
    ImuCsv imu_csv;

    camera_csv.load(euroc_path + "/cam0/data.csv");
    imu_csv.load(euroc_path + "/imu0/data.csv");

    for (auto &item : camera_csv.items) {
        image_data.emplace_back(item.t,
                                euroc_path + "/cam0/data/" + item.filename);
        all_data.emplace_back(item.t, NextDataType::CAMERA);
    }

    for (auto &item : imu_csv.items) {
        xrslam::vector<3> gyr = {item.w.x, item.w.y, item.w.z};
        gyroscope_data.emplace_back(item.t, gyr);
        all_data.emplace_back(item.t, NextDataType::GYROSCOPE);

        xrslam::vector<3> acc = {item.a.x, item.a.y, item.a.z};
        accelerometer_data.emplace_back(item.t, acc);
        all_data.emplace_back(item.t, NextDataType::ACCELEROMETER);
    }

    std::sort(all_data.begin(), all_data.end(),
              [](auto &a, auto &b) { return a.first < b.first; });
    std::sort(image_data.begin(), image_data.end(),
              [](auto &a, auto &b) { return a.first < b.first; });
    std::sort(gyroscope_data.begin(), gyroscope_data.end(),
              [](auto &a, auto &b) { return a.first < b.first; });
    std::sort(accelerometer_data.begin(), accelerometer_data.end(),
              [](auto &a, auto &b) { return a.first < b.first; });
}

DatasetReader::NextDataType EurocDatasetReader::next() {
    if (all_data.empty()) {
        return NextDataType::END;
    }
    auto [t, type] = all_data.front();
    return type;
}

std::shared_ptr<xrslam::Image> EurocDatasetReader::read_image() {
    if (image_data.empty()) {
        return nullptr;
    }
    auto [t, filename] = image_data.front();
    cv::Mat img_distorted = cv::imread(filename, cv::IMREAD_GRAYSCALE);
    cv::Mat img;
    cv::Mat dist_coeffs = (cv::Mat_<float>(4, 1) << -0.28340811, 0.07395907,
                           0.00019359, 1.76187114e-05);
    cv::Mat K = (cv::Mat_<float>(3, 3) << 458.654, 0, 367.215, 0, 457.296,
                 248.375, 0, 0, 1);
    cv::undistort(img_distorted, img, K, dist_coeffs);
    std::shared_ptr<xrslam::extra::OpenCvImage> opencv_image =
        std::make_shared<xrslam::extra::OpenCvImage>();
    opencv_image->t = t;
    opencv_image->image = img;
    opencv_image->raw = img.clone();

    all_data.pop_front();
    image_data.pop_front();
    return opencv_image;
}

std::pair<double, xrslam::vector<3>> EurocDatasetReader::read_gyroscope() {
    if (gyroscope_data.empty()) {
        return {};
    }
    auto item = gyroscope_data.front();

    all_data.pop_front();
    gyroscope_data.pop_front();
    return item;
}

std::pair<double, xrslam::vector<3>> EurocDatasetReader::read_accelerometer() {
    if (accelerometer_data.empty()) {
        return {};
    }
    auto item = accelerometer_data.front();

    all_data.pop_front();
    accelerometer_data.pop_front();
    return item;
}
