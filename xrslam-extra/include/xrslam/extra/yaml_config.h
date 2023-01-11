#ifndef XRSLAM_EXTRA_YAML_CONFIG_H
#define XRSLAM_EXTRA_YAML_CONFIG_H

#include <stdexcept>
#include <xrslam/xrslam.h>

namespace xrslam::extra {

class YamlConfig : public Config {
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

    YamlConfig(const std::string &slam_config_filename,
               const std::string &device_config_filename);
    ~YamlConfig();

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

    quaternion output_to_body_rotation() const override;
    vector<3> output_to_body_translation() const override;

    size_t sliding_window_size() const override;
    size_t sliding_window_force_keyframe_landmarks() const override;

    double feature_tracker_min_keypoint_distance() const override;
    size_t feature_tracker_max_keypoint_detection() const override;
    size_t feature_tracker_max_init_frames() const override;
    size_t feature_tracker_max_frames() const override;
    bool feature_tracker_predict_keypoints() const override;

    size_t initializer_keyframe_num() const override;
    size_t initializer_keyframe_gap() const override;
    size_t initializer_min_matches() const override;
    double initializer_min_parallax() const override;
    size_t initializer_min_triangulation() const override;
    size_t initializer_min_landmarks() const override;
    bool initializer_refine_imu() const override;

    bool visual_localization_enable() const override;

    size_t solver_iteration_limit() const override;
    double solver_time_limit() const override;

  private:
    matrix<3> m_camera_intrinsic;
    quaternion m_camera_to_body_rotation;
    vector<3> m_camera_to_body_translation;
    quaternion m_imu_to_body_rotation;
    vector<3> m_imu_to_body_translation;
    matrix<2> m_keypoint_noise_cov;
    matrix<3> m_gyroscope_noise_cov;
    matrix<3> m_accelerometer_noise_cov;
    matrix<3> m_gyroscope_bias_noise_cov;
    matrix<3> m_accelerometer_bias_noise_cov;

    quaternion m_output_to_body_rotation;
    vector<3> m_output_to_body_translation;

    size_t m_sliding_window_size;
    size_t m_sliding_window_force_keyframe_landmarks;

    double m_feature_tracker_min_keypoint_distance;
    size_t m_feature_tracker_max_keypoint_detection;
    size_t m_feature_tracker_max_init_frames;
    size_t m_feature_tracker_max_frames;
    bool m_feature_tracker_predict_keypoints;

    size_t m_initializer_keyframe_num;
    size_t m_initializer_keyframe_gap;
    size_t m_initializer_min_matches;
    double m_initializer_min_parallax;
    size_t m_initializer_min_triangulation;
    size_t m_initializer_min_landmarks;
    bool m_initializer_refine_imu;

    bool m_visual_localization_enable;

    size_t m_solver_iteration_limit;
    double m_solver_time_limit;
};

} // namespace xrslam::extra

#endif // XRSLAM_EXTRA_YAML_CONFIG_H
