#include "XRSLAMIosConfig.h"
#include <yaml-cpp/yaml.h>
#include <xrslam/extra/yaml_config.h>

namespace xrslam {

static YAML::Node find_node(const YAML::Node &root, const std::string &path,
                            bool mandatory = false) {
    std::stringstream ss(path);
    std::string child;
    YAML::Node node = root;
    while (std::getline(ss, child, '.')) {
        node.reset(node[child]);
    }
    if (!node) {
        if (mandatory) {
            throw XRSLAMIosConfig::ConfigMissingException(path);
        }
    } else {
        node.SetTag(path);
    }
    return node;
}

static void require_vector(const YAML::Node &node, size_t n) {
    if (node.size() != n) {
        throw XRSLAMIosConfig::TypeErrorException(node.Tag());
    }
    for (size_t i = 0; i < n; ++i) {
        if (!node[i].IsScalar()) {
            throw XRSLAMIosConfig::TypeErrorException(node.Tag());
        }
    }
}

static void assign(bool &value, const YAML::Node &node) {
    if (!node.IsScalar()) {
        throw XRSLAMIosConfig::TypeErrorException(node.Tag());
    }
    value = node.as<bool>();
}

static void assign(size_t &value, const YAML::Node &node) {
    if (!node.IsScalar()) {
        throw XRSLAMIosConfig::TypeErrorException(node.Tag());
    }
    value = node.as<size_t>();
}

static void assign(std::string &value, const YAML::Node &node) {
    value = node.as<std::string>();
}

static void assign(double &value, const YAML::Node &node) {
    if (!node.IsScalar()) {
        throw XRSLAMIosConfig::TypeErrorException(node.Tag());
    }
    value = node.as<double>();
}

template <typename V>
static void assign_vector(V &vec, const YAML::Node &node) {
    require_vector(node, vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
        vec[i] = node[i].as<double>();
    }
}

template <typename M>
static void assign_matrix(M &mat, const YAML::Node &node) {
    require_vector(node, mat.rows() * mat.cols());
    for (size_t i = 0; i < mat.rows(); ++i) {
        for (size_t j = 0; j < mat.cols(); ++j) {
            mat(i, j) = node[i * mat.cols() + j].template as<double>();
        }
    }
}

CommonIosConfig::CommonIosConfig(const std::string &fileContent) {
    YAML::Node config;
    try {
        config = YAML::Load(fileContent);
    } catch (const YAML::ParserException &parse_error) {
        throw ParseException(parse_error.what());
    } catch (...) {
        throw LoadException(fileContent);
    }

    if (auto node = find_node(config, "min_keypoint_distance", true)) {
        assign(m_min_keypoint_distance, node);
    }

    if (auto node = find_node(config, "max_keypoint_detection", true)) {
        assign(m_max_keypoint_detection, node);
    }

    if (auto node = find_node(config, "max_frame", true)) {
        assign(m_max_frame, node);
    }

    if (auto node = find_node(config, "solver_time_limit", true)) {
        assign(m_solver_time_limit, node);
    }

    if (auto node = find_node(config, "solver_iteration_limit", true)) {
        assign(m_solver_iteration_limit, node);
    }

    if (auto node = find_node(config, "sliding_window_size", true)) {
        assign(m_sliding_window_size, node);
    }

    if (auto node =
            find_node(config, "sliding_window_tracker_frequent", true)) {
        assign(m_sliding_window_tracker_frequent, node);
    }

    if (auto node = find_node(config, "visual_localization", true)) {
        assign(m_visual_localization, node);
    }

    if (auto node = find_node(config, "visual_localization_ip", true)) {
        assign(m_visual_localization_config_ip, node);
    }

    if (auto node = find_node(config, "visual_localization_port", true)) {
        assign(m_visual_localization_config_port, node);
    }
}

CommonIosConfig::~CommonIosConfig() = default;

matrix<3> CommonIosConfig::camera_intrinsic() const {
    return matrix<3>::Identity();
}
quaternion CommonIosConfig::camera_to_body_rotation() const {
    return quaternion::Identity();
}
vector<3> CommonIosConfig::camera_to_body_translation() const {
    return vector<3>::Identity();
}
quaternion CommonIosConfig::imu_to_body_rotation() const {
    return quaternion::Identity();
}
vector<3> CommonIosConfig::imu_to_body_translation() const {
    return vector<3>::Identity();
}
matrix<2> CommonIosConfig::keypoint_noise_cov() const {
    return matrix<2>::Identity();
}
matrix<3> CommonIosConfig::gyroscope_noise_cov() const {
    return matrix<3>::Identity();
}
matrix<3> CommonIosConfig::accelerometer_noise_cov() const {
    return matrix<3>::Identity();
}
matrix<3> CommonIosConfig::gyroscope_bias_noise_cov() const {
    return matrix<3>::Identity();
}
matrix<3> CommonIosConfig::accelerometer_bias_noise_cov() const {
    return matrix<3>::Identity();
}

bool CommonIosConfig::visual_localization_enable() const {
    return m_visual_localization;
}

size_t CommonIosConfig::visual_localization_config_port() const {
    return m_visual_localization_config_port;
}

std::string CommonIosConfig::visual_localization_config_ip() const {
    return m_visual_localization_config_ip;
}

double CommonIosConfig::feature_tracker_min_keypoint_distance() const {
    return m_min_keypoint_distance;
}

size_t CommonIosConfig::feature_tracker_max_keypoint_detection() const {
    return m_max_keypoint_detection;
}

size_t CommonIosConfig::sliding_window_size() const {
    return m_sliding_window_size;
}

size_t CommonIosConfig::sliding_window_tracker_frequent() const {
    return m_sliding_window_tracker_frequent;
}

size_t CommonIosConfig::solver_iteration_limit() const {
    return m_solver_iteration_limit;
}

double CommonIosConfig::solver_time_limit() const {
    return m_solver_time_limit;
}

size_t CommonIosConfig::feature_tracker_max_frames() const {
    return m_max_frame;
}

XRSLAMIosConfig::XRSLAMIosConfig(const std::string &fileContent,
                                 std::shared_ptr<Config> parent_config)
    : parent_config(parent_config) {
    YAML::Node config;
    try {
        config = YAML::Load(fileContent);
    } catch (const YAML::ParserException &parse_error) {
        throw ParseException(parse_error.what());
    } catch (...) {
        throw LoadException(fileContent);
    }

    double value;

    if (auto intrinsic = find_node(config, "intrinsic", true)) {
        require_vector(intrinsic, 4);
        K.setIdentity();
        K(0, 0) = intrinsic[0].as<double>();
        K(1, 1) = intrinsic[1].as<double>();
        K(0, 2) = intrinsic[2].as<double>();
        K(1, 2) = intrinsic[3].as<double>();
    }

    if (auto node = find_node(config, "q_bc", true)) {
        assign_vector(q_bc.coeffs(), node);
    }

    if (auto node = find_node(config, "p_bc", true)) {
        assign_vector(p_bc, node);
    }

    if (auto node = find_node(config, "q_bo", true)) {
        assign_vector(q_bo.coeffs(), node);
    }

    if (auto node = find_node(config, "p_bo", true)) {
        assign_vector(p_bo, node);
    }

    if (auto node = find_node(config, "q_bi", true)) {
        assign_vector(q_bi.coeffs(), node);
    }

    if (auto node = find_node(config, "p_bi", true)) {
        assign_vector(p_bi, node);
    }

    if (auto node = find_node(config, "sigma_u", true)) {
        assign(value, node);
        sigma_u = matrix<2>::Identity() * value;
    }

    if (auto node = find_node(config, "sigma_w", true)) {
        assign(value, node);
        sigma_w = matrix<3>::Identity() * value;
    }

    if (auto node = find_node(config, "sigma_a", true)) {
        assign(value, node);
        sigma_a = matrix<3>::Identity() * value;
    }

    if (auto node = find_node(config, "sigma_bg", true)) {
        assign(value, node);
        sigma_bg = matrix<3>::Identity() * value;
    }

    if (auto node = find_node(config, "sigma_ba", true)) {
        assign(value, node);
        sigma_ba = matrix<3>::Identity() * value;
    }
}

XRSLAMIosConfig::~XRSLAMIosConfig() = default;

quaternion XRSLAMIosConfig::output_to_body_rotation() const { return q_bo; }

vector<3> XRSLAMIosConfig::output_to_body_translation() const { return p_bo; }

matrix<3> XRSLAMIosConfig::camera_intrinsic() const { return K; }

quaternion XRSLAMIosConfig::camera_to_body_rotation() const { return q_bc; }

vector<3> XRSLAMIosConfig::camera_to_body_translation() const { return p_bc; }

quaternion XRSLAMIosConfig::imu_to_body_rotation() const { return q_bi; }

vector<3> XRSLAMIosConfig::imu_to_body_translation() const { return p_bi; }

matrix<2> XRSLAMIosConfig::keypoint_noise_cov() const { return sigma_u; }

matrix<3> XRSLAMIosConfig::gyroscope_noise_cov() const { return sigma_w; }

matrix<3> XRSLAMIosConfig::accelerometer_noise_cov() const { return sigma_a; }

matrix<3> XRSLAMIosConfig::gyroscope_bias_noise_cov() const { return sigma_bg; }

matrix<3> XRSLAMIosConfig::accelerometer_bias_noise_cov() const {
    return sigma_ba;
}

double XRSLAMIosConfig::feature_tracker_min_keypoint_distance() const {
    return parent_config->feature_tracker_min_keypoint_distance();
}

size_t XRSLAMIosConfig::feature_tracker_max_keypoint_detection() const {
    return parent_config->feature_tracker_max_keypoint_detection();
}

size_t XRSLAMIosConfig::sliding_window_size() const {
    return parent_config->sliding_window_size();
}

size_t XRSLAMIosConfig::sliding_window_tracker_frequent() const {
    return parent_config->sliding_window_tracker_frequent();
}

size_t XRSLAMIosConfig::solver_iteration_limit() const {
    return parent_config->solver_iteration_limit();
}

double XRSLAMIosConfig::solver_time_limit() const {
    return parent_config->solver_time_limit();
}

size_t XRSLAMIosConfig::feature_tracker_max_frames() const {
    return parent_config->feature_tracker_max_frames();
}

bool XRSLAMIosConfig::visual_localization_enable() const {
    return parent_config->visual_localization_enable();
}

size_t XRSLAMIosConfig::visual_localization_config_port() const {
    return parent_config->visual_localization_config_port();
}

std::string XRSLAMIosConfig::visual_localization_config_ip() const {
    return parent_config->visual_localization_config_ip();
}

} // namespace xrslam
