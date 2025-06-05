#include <argparse.hpp>
#include <iostream>
#include <thread>
#include <mutex>

#include <dataset_reader.h>
#include <trajectory_writer.h>

#include <opencv2/opencv.hpp>

#include "XRSLAM.h"

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
    XRSLAMDestroy();

    return EXIT_SUCCESS;
}
