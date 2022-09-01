#include <xrslam/common.h>

namespace xrslam {

Config::~Config() = default;

quaternion Config::output_to_body_rotation() const {
    return quaternion::Identity();
}
vector<3> Config::output_to_body_translation() const {
    return vector<3>::Zero();
}

size_t Config::sliding_window_size() const { return 10; }

size_t Config::sliding_window_force_keyframe_landmarks() const { return 35; }

double Config::feature_tracker_min_keypoint_distance() const { return 20.0; }

size_t Config::feature_tracker_max_keypoint_detection() const { return 150; }

size_t Config::feature_tracker_max_init_frames() const { return 60; }

size_t Config::feature_tracker_max_frames() const { return 200; }

bool Config::feature_tracker_predict_keypoints() const { return true; }

size_t Config::initializer_keyframe_num() const { return 8; }

size_t Config::initializer_keyframe_gap() const { return 5; }

size_t Config::initializer_min_matches() const { return 50; }

double Config::initializer_min_parallax() const { return 10; }

size_t Config::initializer_min_triangulation() const {
    return 50; // 50
}

size_t Config::initializer_min_landmarks() const { return 30; }

bool Config::initializer_refine_imu() const { return true; }

bool Config::visual_localization_enable() const { return false; }

std::string Config::visual_localization_config_ip() const { return "0.0.0.0"; }

size_t Config::visual_localization_config_port() const { return 0; }

size_t Config::solver_iteration_limit() const { return 10; }

double Config::solver_time_limit() const { return 1.0e6; }

int Config::random() const { return 648; }

size_t Config::sliding_window_tracker_frequent() const { return 1; }

void Config::log_config() const {
    std::stringstream ss;
    ss << std::scientific << std::boolalpha << std::setprecision(5);

    ss << "Config::camera_intrinsic:\n"
       << camera_intrinsic() << "\n"
       << std::endl;

    ss << "Config::camera_to_body_rotation:\n"
       << camera_to_body_rotation().coeffs().transpose() << "\n"
       << std::endl;

    ss << "Config::camera_to_body_translation:\n"
       << camera_to_body_translation().transpose() << "\n"
       << std::endl;

    ss << "Config::imu_to_body_rotation:\n"
       << imu_to_body_rotation().coeffs().transpose() << "\n"
       << std::endl;

    ss << "Config::imu_to_body_translation:\n"
       << imu_to_body_translation().transpose() << "\n"
       << std::endl;

    ss << "Config::keypoint_noise_cov:\n"
       << keypoint_noise_cov() << "\n"
       << std::endl;

    ss << "Config::gyroscope_noise_cov:\n"
       << gyroscope_noise_cov() << "\n"
       << std::endl;

    ss << "Config::accelerometer_noise_cov:\n"
       << accelerometer_noise_cov() << "\n"
       << std::endl;

    ss << "Config::gyroscope_bias_noise_cov:\n"
       << gyroscope_bias_noise_cov() << "\n"
       << std::endl;

    ss << "Config::accelerometer_bias_noise_cov:\n"
       << accelerometer_bias_noise_cov() << "\n"
       << std::endl;

    ss << "Config::sliding_window_size: " << sliding_window_size() << std::endl;

    ss << "Config::sliding_window_force_keyframe_landmarks: "
       << sliding_window_force_keyframe_landmarks() << std::endl;

    ss << "Config::sliding_window_tracker_frequent: "
       << sliding_window_tracker_frequent() << std::endl;

    ss << "Config::feature_tracker_min_keypoint_distance: "
       << feature_tracker_min_keypoint_distance() << std::endl;

    ss << "Config::feature_tracker_max_keypoint_detection: "
       << feature_tracker_max_keypoint_detection() << std::endl;

    ss << "Config::feature_tracker_max_init_frames: "
       << feature_tracker_max_init_frames() << std::endl;

    ss << "Config::feature_tracker_max_frames: " << feature_tracker_max_frames()
       << std::endl;

    ss << "Config::feature_tracker_predict_keypoints: "
       << feature_tracker_predict_keypoints() << std::endl;

    ss << "Config::initializer_keyframe_gap: " << initializer_keyframe_gap()
       << std::endl;

    ss << "Config::initializer_min_matches: " << initializer_min_matches()
       << std::endl;

    ss << "Config::initializer_min_parallax: " << initializer_min_parallax()
       << std::endl;

    ss << "Config::initializer_min_triangulation: "
       << initializer_min_triangulation() << std::endl;

    ss << "Config::initializer_min_landmarks: " << initializer_min_landmarks()
       << std::endl;

    ss << "Config::initializer_refine_imu: " << initializer_refine_imu()
       << std::endl;

    ss << "Config::visual_localization_enable: " << visual_localization_enable()
       << std::endl;

    ss << "Config::visual_localization_config_ip: "
       << visual_localization_config_ip() << std::endl;

    ss << "Config::visual_localization_config_port: "
       << visual_localization_config_port() << std::endl;

    ss << "Config::solver_iteration_limit: " << solver_iteration_limit()
       << std::endl;

    ss << "Config::solver_time_limit: " << solver_time_limit() << std::endl;

#if defined(XRSLAM_ENABLE_THREADING)
    ss << std::endl;
    ss << "THREADING ENABLE" << std::endl;
#else
    ss << std::endl;
    ss << "THREADING DISABLE" << std::endl;
#endif

    log_info("Configurations: \n%s", ss.str().c_str());
}

} // namespace xrslam
