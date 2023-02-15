#include "visualizer.h"
#ifndef XRSLAM_PC_HEADLESS_ONLY
Visualizer::Visualizer(bool play, const std::string &config_file)
    : LightVis("XRSLAM Interface Player", 1200, 720) {
    is_playing = (int)play;
    trajectory_color = {0.0, 1.0, 1.0, 1.0};
    camera_color = {1.0, 1.0, 0.0, 1.0};

    feature_tracker_painter =
        std::make_unique<OpenCvPainter>(feature_tracker_cvimage);
    inspect_debug(feature_tracker_painter, painter) {
        painter = feature_tracker_painter.get();
    }
    bg_x.resize(250, 0.5);
    bg_y.resize(250, 0.5);
    bg_z.resize(250, 0.5);
    ba_x.resize(250, 0.5);
    ba_y.resize(250, 0.5);
    ba_z.resize(250, 0.5);

    read_device_params(config_file);
}

Visualizer::~Visualizer() {
    con_var.notify_all();
    running = 0;
}
void Visualizer::load() {
    feature_tracker_image = std::make_unique<lightvis::Image>();

    add_trajectory(trajectory, trajectory_color);
    add_points(landmarks, landmarks_color);
    add_camera(positions, camera_color);

    add_image(feature_tracker_image.get());
    add_separator();
    add_graph(bg_x);
    add_separator();
    add_graph(bg_y);
    add_separator();
    add_graph(bg_z);
    add_separator();
    add_graph(ba_x);
    add_separator();
    add_graph(ba_y);
    add_separator();
    add_graph(ba_z);
}

void Visualizer::main() {
    running = 1;
    if (EXIT_SUCCESS == lightvis::main()) {
        con_var.notify_all();
        running = 0;
    }
}

int Visualizer::is_running() { return running; }

void Visualizer::unload() { feature_tracker_image.reset(); }

void Visualizer::gui(void *ctx, int w, int h) {
    auto lk = lock();
    feature_tracker_image->update_image(feature_tracker_cvimage);
    {
        auto xt = bg_x.rbegin(), yt = bg_y.rbegin(), zt = bg_z.rbegin();
        for (auto it = bg_list.rbegin(); it != bg_list.rend();
             it++, xt++, yt++, zt++) {
            (*xt) = it->x() / 0.05 + 0.5;
            (*yt) = it->y() / 0.05 + 0.5;
            (*zt) = it->z() / 0.05 + 0.5;
        }
    }
    {
        auto xt = ba_x.rbegin(), yt = ba_y.rbegin(), zt = ba_z.rbegin();
        for (auto it = ba_list.rbegin(); it != ba_list.rend();
             it++, xt++, yt++, zt++) {
            (*xt) = it->x() / 0.1 + 0.5;
            (*yt) = it->y() / 0.1 + 0.5;
            (*zt) = it->z() / 0.1 + 0.5;
        }
    }
    LightVis::gui(ctx, w, h);

    auto *context = (nk_context *)(ctx);

    if (nk_begin(context, "Controls", nk_rect(0, h - 40, 240, 40),
                 NK_WINDOW_NO_SCROLLBAR)) {
        nk_layout_row_static(context, 40, 80, 3);
        if (is_playing) {
            con_var.notify_one();
        }
        if (is_playing) {
            if (nk_button_label(context, "Playing")) {
                is_playing = false;
            }
        } else {
            if (nk_button_label(context, "Stopped")) {
                is_playing = true;
            }
            nk_button_push_behavior(context, NK_BUTTON_REPEATER);
            if (nk_button_label(context, "Forward")) {
                con_var.notify_one();
            }
            nk_button_pop_behavior(context);
            nk_button_push_behavior(context, NK_BUTTON_DEFAULT);
            if (nk_button_label(context, "Step")) {
                con_var.notify_one();
            }
            nk_button_pop_behavior(context);
        }
    }
    nk_end(context);
}

void Visualizer::draw(int w, int h) {
    auto err = gl::glGetError();
    if (err != gl::GL_NONE) {
        std::cout << err << std::endl;
        exit(0);
    }
}

void Visualizer::update_frame(const std::shared_ptr<VizElements> curframe) {
    auto lk = lock();
    // landmarks
    landmarks.clear();
    landmarks_color.clear();
    landmarks = curframe->landmarks;
    landmarks_color = curframe->landmarks_color;
    // 2D image
    // feature_tracker_cvimage = curframe->show_frame;
    // camera pose and trajectory
    if (!curframe->camera_q.coeffs().isZero()) {
        Eigen::Vector3f p = (curframe->camera_p -
                             curframe->camera_q * camera_to_body_translation)
                                .cast<float>();
        trajectory.push_back(p);
        location() = {p.x(), p.y(), p.z()};
        latest_camera_q =
            curframe->camera_q * camera_to_body_rotation.conjugate();
        latest_camera_p = curframe->camera_p -
                          curframe->camera_q * camera_to_body_translation;
        // camera show position
        set_camera_position(positions, latest_camera_p, latest_camera_q,
                            camera_intrinsic, camera_color);
    }
    // imu bias
    bg_list.emplace_back(curframe->gyro_bias);
    while (bg_list.size() > 250) {
        bg_list.pop_front();
    }
    ba_list.emplace_back(curframe->acc_bias);
    while (ba_list.size() > 250) {
        ba_list.pop_front();
    }
    con_var.wait(lk);
}

void Visualizer::read_device_params(const std::string &config_file) {
    // Read device params
    cv::FileStorage device_config(config_file.c_str(), cv::FileStorage::READ);
    if (!device_config.isOpened()) {
        std::cerr << "Failed to open settings file at: " << config_file
                  << std::endl;
        exit(-1);
    }
    const cv::FileNode cam_configs = device_config["cam0"];
    cv::FileNode intrinsic_node = cam_configs["intrinsics"];
    if (!intrinsic_node.isNone()) {
        camera_intrinsic << intrinsic_node[0], 0, intrinsic_node[2], 0,
            intrinsic_node[1], intrinsic_node[3], 0, 0, 1;
    } else {
        std::cerr << "The intrinsic configuration is missing" << std::endl;
        exit(-1);
    }
    cv::FileNode T_BS_node = cam_configs["T_BS"]["data"];
    if (!T_BS_node.isNone()) {
        Eigen::Matrix3d camera_to_body_R;
        camera_to_body_R << T_BS_node[0], T_BS_node[1], T_BS_node[2],
            T_BS_node[4], T_BS_node[5], T_BS_node[6], T_BS_node[8],
            T_BS_node[9], T_BS_node[10];
        camera_to_body_rotation = Eigen::Quaterniond(camera_to_body_R);
        camera_to_body_translation << T_BS_node[3], T_BS_node[7], T_BS_node[11];
    } else {
        std::cerr << "The TBS configuration is missing" << std::endl;
        exit(-1);
    }
    device_config.release();
}
#endif