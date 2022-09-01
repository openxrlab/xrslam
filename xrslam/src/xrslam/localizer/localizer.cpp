#include <xrslam/localizer/localizer.h>
#include <xrslam/utility/logger.h>

namespace xrslam {

Localizer::Localizer(std::shared_ptr<Config> config)
    : config(config), init_flag(false), slam_to_sfm(Pose()) {

    if (verbose)
        std::cout << "initialize a localizer" << std::endl;

    const matrix<3> intrinsic_param = config->camera_intrinsic();

    intrinsic = {(float)intrinsic_param(0, 0), (float)intrinsic_param(1, 1),
                 (float)intrinsic_param(0, 2), (float)intrinsic_param(1, 2)};
    distortion = {0.0, 0.0, 0.0, 0.0};

    url = config->visual_localization_config_ip();
    port = config->visual_localization_config_port();
}

Localizer::~Localizer() {}

void Localizer::test_connection() {
    try {
        httplib::Client cli(url, port);
        cli.set_timeout_sec(1e7);
        cli.set_read_timeout(1e7, 0);

        nlohmann::json j_msg;
        j_msg["is_grayscale"] = true;

        auto res = cli.Post("/loc", j_msg.dump(), "application/json");
        if (!res) {
            printf("[NAV-Debug] query loc: cannot post to target, response is "
                   "null.\n");
            return;
        }
        if (res->status != 200) {
            printf("[NAV-Debug] query loc: Server or json error\n");
            return;
        }
        nlohmann::json res_json = nlohmann::json::parse(res->body);
        bool is_success = res_json["success"];
        if (is_success) {
            printf("Success!\n");
        }
    } catch (std::exception e) {
        printf("[NAV-Debug] %s\n", e.what());
    }
}

void Localizer::query_loc(const cv::Mat cvimg, const Pose &T_slam_body,
                          ScreenState screenState) {
    try {
        httplib::Client cli(url, port);
        cli.set_timeout_sec(1e7);
        cli.set_read_timeout(1e7, 0);

        cv::Mat img_rgb;
        if (cvimg.channels() == 1) {
            cv::cvtColor(cvimg, img_rgb, cv::COLOR_GRAY2RGB);
        } else {
            img_rgb = cvimg.clone();
        }

        std::vector<float> params =
            rotate_intrinsic(screenState, img_rgb.cols, img_rgb.rows);
        cv::Mat resImg = get_image_by_screenstate(screenState, img_rgb);

        std::string message = encode_image_msg(resImg);
        params.insert(params.end(), distortion.begin(), distortion.end());

        nlohmann::json j_msg;
        j_msg["image"] = message;
        j_msg["intrinsic"] = {{"model", "OPENCV"},
                              {"width", int(resImg.cols)},
                              {"height", int(resImg.rows)},
                              {"params", params}};

        auto res = cli.Post("/loc", j_msg.dump(), "application/json");
        if (!res) {
            if (verbose)
                printf("[NAV-Debug] query loc: cannot post to target, response "
                       "is null.\n");
            Logger::instance().pushMessage("server is unavailable!");
            return;
        }
        if (res->status != 200) {
            if (verbose)
                printf("[NAV-Debug] query loc: Server or json error\n");
            Logger::instance().pushMessage("json format error!");
            return;
        }
        nlohmann::json res_json = nlohmann::json::parse(res->body);
        bool is_success = res_json["ninlier"] > inliers_th;
        if (is_success) {
            std::unique_lock lock(transform_mutex);
            if (verbose)
                printf("[NAV-Debug] Success!\n");
            Logger::instance().pushMessage("query success!");
            Logger::instance().pushMessage(
                "inliers num: " + std::to_string((size_t)res_json["ninlier"]));

            Eigen::Quaterniond q_sfm_cam1 = {res_json["qvec"][0].get<double>(),
                                             res_json["qvec"][1].get<double>(),
                                             res_json["qvec"][2].get<double>(),
                                             res_json["qvec"][3].get<double>()};
            Eigen::Vector3d t_sfm_cam1 = {res_json["tvec"][0].get<double>(),
                                          res_json["tvec"][1].get<double>(),
                                          res_json["tvec"][2].get<double>()};

            Pose T_cam1_slam;

            {
                // deal with rotate
                Pose T_slam_cam0, T_slam_cam1;

                T_slam_cam0.q =
                    T_slam_body.q * config->output_to_body_rotation();
                T_slam_cam0.p =
                    T_slam_body.p +
                    T_slam_body.q * config->output_to_body_translation();

                T_slam_cam1.p = T_slam_cam0.p;

                Eigen::Quaterniond q_cam1_cam0;
                Eigen::Vector3d z_in_slam =
                    T_slam_cam0.q * Eigen::Vector3d::UnitZ();
                switch (screenState) {
                case Portrait:
                    q_cam1_cam0 = Eigen::Quaterniond::Identity();
                    break;
                case Right:
                    q_cam1_cam0 = Eigen::AngleAxisd(-M_PI_2, z_in_slam);
                    break;
                case Left:
                    q_cam1_cam0 = Eigen::AngleAxisd(M_PI_2, z_in_slam);
                    break;
                case Down:
                    q_cam1_cam0 = Eigen::AngleAxisd(M_PI, z_in_slam);
                    break;
                default:
                    break;
                }
                T_slam_cam1.q = q_cam1_cam0 * T_slam_cam0.q;

                T_cam1_slam.q = T_slam_cam1.q.inverse();
                T_cam1_slam.p = -(T_cam1_slam.q * T_slam_cam1.p);
            }

            slam_to_sfm.q = q_sfm_cam1 * T_cam1_slam.q;
            slam_to_sfm.p = t_sfm_cam1 + q_sfm_cam1 * T_cam1_slam.p;

            init_flag = true;
        } else {
            if (verbose)
                printf("[NAV-Debug] Failed, maybe the number of inliers is "
                       "lower than the threshold!\n");
            Logger::instance().pushMessage("query failed!");
            Logger::instance().pushMessage(
                "inliers num: " + std::to_string((size_t)res_json["ninlier"]));
        }
    } catch (std::exception e) {
        printf("[NAV-Debug] %s\n", e.what());
        Logger::instance().pushMessage("query failed with unknown reason!");
    }
}

void Localizer::add_pose_message(const double timestamp, const Pose &pose) {
    std::unique_lock lock(send_pose_mutex);
    std::vector<float> message = {(float)pose.p.x(), (float)pose.p.y(),
                                  (float)pose.p.z(), (float)pose.q.x(),
                                  (float)pose.q.y(), (float)pose.q.z(),
                                  (float)pose.q.w()};

    poses.insert(poses.end(), message.begin(), message.end());
}

void Localizer::send_pose() {
    try {
        httplib::Client cli(url, port);
        cli.set_timeout_sec(1e7);
        cli.set_read_timeout(1e7, 0);
        nlohmann::json j_msg;

        j_msg["pose"] = poses;
        auto res = cli.Post("/pose", j_msg.dump(), "application/json");
        if (!res) {
            if (verbose)
                printf("[NAV-Debug] query loc: cannot post to target, response "
                       "is null.\n");
            Logger::instance().pushMessage("server is unavailable!");
            return;
        }
        if (res->status != 200) {
            if (verbose)
                printf("[NAV-Debug] query loc: Server or json error\n");
            Logger::instance().pushMessage("json format error!");
            return;
        }
        std::unique_lock lock(send_pose_mutex);
        poses.clear();
    } catch (std::exception e) {
        printf("[NAV-Debug] %s\n", e.what());
    }
}

Pose Localizer::transform(const Pose &pose) {
    std::unique_lock lock(transform_mutex);
    OutputPose output_pose;
    output_pose.q = slam_to_sfm.q * pose.q;
    output_pose.p = slam_to_sfm.p + slam_to_sfm.q * pose.p;
    return output_pose;
}

bool Localizer::is_initialized() const { return init_flag; }

void Localizer::query_localization(std::shared_ptr<xrslam::Image> img,
                                   Pose pose) {
    if (img->t - image_last_time > send_image_interval || query_frame_flag) {
        if (verbose)
            std::cout << "query localization" << std::endl;

        Eigen::Quaterniond qcw =
            (pose.q * config->output_to_body_rotation()).inverse();
        Eigen::Matrix3d r(qcw);
        ScreenState screenState = get_screenstate(r);

        const cv::Mat cvimg(img->height(), img->width(), CV_8UC1,
                            img->get_rawdata());

        std::thread th = std::thread(&Localizer::query_loc, this, cvimg.clone(),
                                     pose, screenState);
        th.detach();

        image_last_time = img->t;
        query_frame_flag = false;
    }
}

void Localizer::send_pose_message(double time) {
    if (time - poses_last_time > send_poses_interval) {
        // if (verbose)
        //     std::cout << "send pose" << std::endl;

        std::thread th = std::thread(&Localizer::send_pose, this);
        th.detach();
        poses_last_time = time;
    }
}

void Localizer::query_frame() {
    std::cout << "query frame" << std::endl;
    query_frame_flag = true;
}

ScreenState Localizer::get_screenstate(const Eigen::Matrix3d R) {
    double x, y, z;
    ScreenState screenState;
    Eigen::Vector3d xyz = R * Eigen::Vector3d(0, 0, 1);
    x = xyz(0);
    y = xyz(1);
    z = xyz(2);
    if (fabs(y) > fabs(x)) {
        screenState = y < 0 ? Portrait : Down;
    } else {
        screenState = x < 0 ? Right : Left;
    }
    return screenState;
}

cv::Mat Localizer::get_image_by_screenstate(const ScreenState &screenState,
                                            const cv::Mat &img) {
    cv::Mat image = img.clone();
    switch (screenState) {
    case Portrait:
        break;
    case Left:
        cv::transpose(image, image);
        cv::flip(image, image, 0);
        break;
    case Right:
        cv::transpose(image, image);
        cv::flip(image, image, 1);
        break;
    case Down:
        cv::flip(image, image, 0);
        cv::flip(image, image, 1);
        break;
    default:
        break;
    }
    return image;
}

std::vector<float> Localizer::rotate_intrinsic(const ScreenState screenState,
                                               size_t w, size_t h) {
    const float &fx = intrinsic[0];
    const float &fy = intrinsic[1];
    const float &cx = intrinsic[2];
    const float &cy = intrinsic[3];
    switch (screenState) {
    case Right: {
        return {fy, fx, h - cy, cx};
    } break;
    case Left: {
        return {fy, fx, cy, w - cx};
    } break;
    case Down: {
        return {fx, fy, cx, h - cy};
    } break;
    default:
        break;
    }
    return {fx, fy, cx, cy};
}

} // namespace xrslam
