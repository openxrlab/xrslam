#ifndef XRSLAM_IOS_XRSLAMIOSCONFIG_H
#define XRSLAM_IOS_XRSLAMIOSCONFIG_H

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#import <xrslam/xrslam.h>
#pragma clang diagnostic pop

namespace xrslam {

class CommonIosConfig : public Config {
  public:
    struct Exception : public std::runtime_error {
        Exception(const std::string &what) : std::runtime_error(what) {}
    };
    struct LoadException : public Exception {
        LoadException(const std::string &filename)
            : Exception("cannot load config " + filename) {}
    };
    struct ParseException : public Exception {
        ParseException(const std::string &message) : Exception(message) {}
    };
    struct ConfigMissingException : public Exception {
        ConfigMissingException(const std::string &config_path)
            : Exception("config \"" + config_path + "\" is mandatory") {}
    };
    struct TypeErrorException : public Exception {
        TypeErrorException(const std::string &config_path)
            : Exception("config \"" + config_path + "\" has wrong type") {}
    };

    CommonIosConfig(const std::string &fileContent);
    ~CommonIosConfig();

    matrix<3> camera_intrinsic() const override;
    quaternion camera_to_body_rotation() const override;
    vector<3> camera_to_body_translation() const override;
    quaternion imu_to_body_rotation() const override;
    vector<3> imu_to_body_translation() const override;

    matrix<2> keypoint_noise_cov() const override;
    matrix<3> gyroscope_noise_cov() const override;
    matrix<3> accelerometer_noise_cov() const override;
    matrix<3> gyroscope_bias_noise_cov() const override;
    matrix<3> accelerometer_bias_noise_cov() const override;

    double feature_tracker_min_keypoint_distance() const override;
    size_t feature_tracker_max_keypoint_detection() const override;
    size_t feature_tracker_max_frames() const override;
    size_t sliding_window_size() const override;
    size_t sliding_window_tracker_frequent() const override;
    size_t solver_iteration_limit() const override;
    double solver_time_limit() const override;

    bool visual_localization_enable() const override;
    size_t visual_localization_config_port() const override;
    std::string visual_localization_config_ip() const override;

  private:
    double m_min_keypoint_distance;
    size_t m_max_keypoint_detection;
    double m_solver_time_limit;
    size_t m_solver_iteration_limit;
    size_t m_sliding_window_size;
    size_t m_sliding_window_tracker_frequent;
    size_t m_max_frame;

    bool m_visual_localization;
    std::string m_visual_localization_config_ip;
    size_t m_visual_localization_config_port;
};

class XRSLAMIosConfig : public Config {
  public:
    struct Exception : public std::runtime_error {
        Exception(const std::string &what) : std::runtime_error(what) {}
    };
    struct LoadException : public Exception {
        LoadException(const std::string &filename)
            : Exception("cannot load config " + filename) {}
    };
    struct ParseException : public Exception {
        ParseException(const std::string &message) : Exception(message) {}
    };
    struct ConfigMissingException : public Exception {
        ConfigMissingException(const std::string &config_path)
            : Exception("config \"" + config_path + "\" is mandatory") {}
    };
    struct TypeErrorException : public Exception {
        TypeErrorException(const std::string &config_path)
            : Exception("config \"" + config_path + "\" has wrong type") {}
    };

    XRSLAMIosConfig(const std::string &fileContent,
                    std::shared_ptr<Config> parent_config);
    ~XRSLAMIosConfig();

    quaternion output_to_body_rotation() const override;
    vector<3> output_to_body_translation() const override;

    matrix<3> camera_intrinsic() const override;

    quaternion camera_to_body_rotation() const override;
    vector<3> camera_to_body_translation() const override;

    quaternion imu_to_body_rotation() const override;
    vector<3> imu_to_body_translation() const override;

    matrix<2> keypoint_noise_cov() const override;
    matrix<3> gyroscope_noise_cov() const override;
    matrix<3> accelerometer_noise_cov() const override;
    matrix<3> gyroscope_bias_noise_cov() const override;
    matrix<3> accelerometer_bias_noise_cov() const override;

    double feature_tracker_min_keypoint_distance() const override;
    size_t feature_tracker_max_keypoint_detection() const override;
    size_t feature_tracker_max_frames() const override;
    size_t sliding_window_size() const override;
    size_t sliding_window_tracker_frequent() const override;
    size_t solver_iteration_limit() const override;
    double solver_time_limit() const override;

    bool visual_localization_enable() const override;
    size_t visual_localization_config_port() const override;
    std::string visual_localization_config_ip() const override;

  private:
    std::shared_ptr<Config> parent_config;
    matrix<3> K;
    quaternion q_bo;
    vector<3> p_bo;
    quaternion q_bc;
    vector<3> p_bc;
    quaternion q_bi;
    vector<3> p_bi;
    matrix<2> sigma_u;
    matrix<3> sigma_w;
    matrix<3> sigma_a;
    matrix<3> sigma_bg;
    matrix<3> sigma_ba;
};

} // namespace xrslam

#endif // XRSLAM_IOS_XRSLAMIOSCONFIG_H
