#include <argparse.hpp>
#include <iostream>
#include <thread>
#include <mutex>

#include <dataset_reader.h>
#include <trajectory_writer.h>

#include <opencv2/opencv.hpp>

#include "XRSLAM.h"

#ifndef XRSLAM_PC_HEADLESS_ONLY
#include <unistd.h>
#include <thread>
#include "visualizer.h"

std::thread visualizer_thread_;
VizElements cur_frame_viz_;

void GetShowElements(VizElements &curframe) {
    // clear
    curframe.landmarks.clear();
    curframe.landmarks_color.clear();
    // get pose
    XRSLAMPose pose_b;
    XRSLAMGetResult(XRSLAM_RESULT_BODY_POSE, &pose_b);
    curframe.camera_q =
        Eigen::Quaterniond(pose_b.quaternion[3], pose_b.quaternion[0],
                           pose_b.quaternion[1], pose_b.quaternion[2]);
    curframe.camera_p = Eigen::Vector3d(
        pose_b.translation[0], pose_b.translation[1], pose_b.translation[2]);
    // get landmarks
    XRSLAMLandmarks landmarks;
    XRSLAMGetResult(XRSLAM_RESULT_LANDMARKS, &landmarks);
    for (int i = 0; i < landmarks.num_landmarks; ++i) {
        curframe.landmarks.push_back(Eigen::Vector3f(landmarks.landmarks[i].x,
                                                     landmarks.landmarks[i].y,
                                                     landmarks.landmarks[i].z));
        // triangulated
        curframe.landmarks_color.emplace_back(0.0, 1.0, 0.0, 0.5);
    }
    // draw 2d features
    // XRSLAMFeatures features;
    // XRSLAMGetResult(XRSLAM_RESULT_FEATURES, &features);
    // for (int i = 0; i < features.num_features; ++i) {
    //     cv::Point center(features.features[i].x, features.features[i].y);
    //     cv::circle(curframe.show_frame, center, 2, cv::Scalar(0, 255, 0), 5);
    // }
    // get imu bias
    XRSLAMIMUBias imu_bias;
    XRSLAMGetResult(XRSLAM_RESULT_BIAS, &imu_bias);
    curframe.acc_bias =
        Eigen::Vector3d(imu_bias.acc_bias.data[0], imu_bias.acc_bias.data[1],
                        imu_bias.acc_bias.data[2]);
    curframe.gyro_bias =
        Eigen::Vector3d(imu_bias.gyr_bias.data[0], imu_bias.gyr_bias.data[1],
                        imu_bias.gyr_bias.data[2]);
}
#endif

typedef std::tuple<double, XRSLAMAcceleration, XRSLAMGyroscope> IMUData;

int main(int argc, char *argv[]) {
    argparse::ArgumentParser program("XRSLAM Player");
    program.add_argument("-sc", "--slamconfig")
        .help("SLAM configuration YAML file.")
        .nargs(1);
    program.add_argument("-dc", "--deviceconfig")
        .help("Device configuration YAML file.")
        .nargs(1);
    program.add_argument("-lc", "--license").help("License file.").nargs(1);
    program.add_argument("--csv").help("Save CSV-format trajectory.").nargs(1);
    program.add_argument("--tum").help("Save TUM-format trajectory.").nargs(1);
    program.add_argument("-p", "--play")
        .help("Start playing immediately.")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("input").help("input file");
    program.parse_args(argc, argv);
    std::string data_path = program.get<std::string>("input");
    std::string slam_config_path = program.get<std::string>("-sc");
    std::string device_config_path = program.get<std::string>("-dc");
    std::string license_path = program.get<std::string>("-lc");
    std::string csv_output = program.get<std::string>("--csv");
    std::string tum_output = program.get<std::string>("--tum");
    bool is_play = program.get<bool>("-p");

    // create slam with configuration files
    void *yaml_config = nullptr;
    int create_succ =
        XRSLAMCreate(slam_config_path.c_str(), device_config_path.c_str(),
                     license_path.c_str(), "XRSLAM PC", &yaml_config);
    std::cout << "create SLAM success: " << create_succ << std::endl;
#ifndef XRSLAM_PC_HEADLESS_ONLY
    Visualizer visualizer(is_play, device_config_path);
    visualizer.show();
    visualizer_thread_ = std::thread(&Visualizer::main, &visualizer);
    std::cout << "create visualizer thread ..." << std::endl;
#endif

    std::vector<std::unique_ptr<TrajectoryWriter>> outputs;
    if (csv_output.length() > 0) {
        outputs.emplace_back(std::make_unique<CsvTrajectoryWriter>(csv_output));
    }
    if (tum_output.length() > 0) {
        outputs.emplace_back(std::make_unique<TumTrajectoryWriter>(tum_output));
    }

    std::unique_ptr<DatasetReader> reader =
        DatasetReader::create_reader(data_path, yaml_config);
    if (!reader) {
        fprintf(stderr, "Cannot open \"%s\"\n", data_path.c_str());
        return EXIT_FAILURE;
    }

    bool has_gyroscope = false, has_accelerometer = false;
    outputs.emplace_back(std::make_unique<ConsoleTrajectoryWriter>());
    DatasetReader::NextDataType next_type;
    while ((next_type = reader->next()) != DatasetReader::END) {
#ifndef XRSLAM_PC_HEADLESS_ONLY
        if (!visualizer.is_running()) {
            break;
        }
#endif
        switch (next_type) {
        case DatasetReader::AGAIN:
            continue;
        case DatasetReader::GYROSCOPE: {
            has_gyroscope = true;
            auto [t, gyro] = reader->read_gyroscope();
            XRSLAMPushSensorData(XRSLAM_SENSOR_GYROSCOPE, &gyro);
        } break;
        case DatasetReader::ACCELEROMETER: {
            has_accelerometer = true;
            auto [t, acc] = reader->read_accelerometer();
            XRSLAMPushSensorData(XRSLAM_SENSOR_ACCELERATION, &acc);
        } break;
        case DatasetReader::CAMERA: {
            auto [t, img] = reader->read_image();
            XRSLAMImage image;
            image.camera_id = 0;
            image.timeStamp = t;
            image.ext = nullptr;
            image.data = img.data;
            image.stride = img.step[0];
            XRSLAMPushSensorData(XRSLAM_SENSOR_CAMERA, &image);
            if (has_accelerometer && has_gyroscope) {
                XRSLAMRunOneFrame();
                XRSLAMState state;
                XRSLAMGetResult(XRSLAM_RESULT_STATE, &state);
                if (state == XRSLAM_STATE_TRACKING_SUCCESS) {
#ifndef XRSLAM_PC_HEADLESS_ONLY
                    int cols = 0, rows = 0;
                    reader->get_image_resolution(cols, rows);
                    cv::Mat img = cv::Mat(rows, cols, CV_8UC1, image.data);
                    cv::cvtColor(img, cur_frame_viz_.show_frame,
                                 cv::COLOR_GRAY2BGR);
                    GetShowElements(cur_frame_viz_);
                    visualizer.update_frame(
                        std::make_shared<VizElements>(cur_frame_viz_));
#endif
                    XRSLAMPose pose_b;
                    XRSLAMGetResult(XRSLAM_RESULT_BODY_POSE, &pose_b);
                    if (pose_b.timestamp > 0) {
                        for (auto &output : outputs) {
                            output->write_pose(pose_b.timestamp, pose_b);
                        }
                    }
                }
            }
        } break;
        default: {
        } break;
        }
    }
#ifndef XRSLAM_PC_HEADLESS_ONLY
    if (visualizer_thread_.joinable()) {
        visualizer_thread_.join();
        std::cout << "\ndestory visualizer thread ..." << std::endl;
    }
#endif
    XRSLAMDestroy();

    return EXIT_SUCCESS;
}
