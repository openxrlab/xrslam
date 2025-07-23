#include <filesystem>
#include <liteviz/core/detail.h>

struct VisConfig{
    
    Eigen::Vector4f trajColor = Eigen::Vector4f(0.7f, 0.2f, 0.2f, 1.0f);
    Eigen::Vector4f pointsColor = Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f);
    Eigen::Vector4f cameraColor = Eigen::Vector4f(0.8f, 0.2f, 0.2f, 1.0f);
    float trajScale = 1.0f;
    float pointsScale = 1.0f;
    float cameraScale = 1.0f;
    size_t panelWidth = 300;
};

struct VisData{ // for visualization

struct Frame {
    cv::Mat image;
    Eigen::Vector4f intrinsics;
    Eigen::Matrix4f pose;

    Frame(const cv::Mat &img,
          const Eigen::Vector4f &intrin,
          const Eigen::Matrix4f &p)
        : image(img), intrinsics(intrin), pose(p) {}
};

public:
    std::mutex frame_mutex;
    std::mutex point_mutex;
    std::mutex image_mutex;
    std::mutex poses_mutex;

    std::vector<Frame> frames;
    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Matrix4f> poses;
    cv::Mat feature_tracker_cvimage;

    void update_poses(const Eigen::Matrix4f _pose) {
        std::lock_guard<std::mutex> lock(poses_mutex);
        poses.push_back(_pose);
    }

    void update_frames(std::vector<Frame> _frames) {
        std::lock_guard<std::mutex> lock(frame_mutex);
        frames = _frames;
    }

    void update_points(const std::vector<Eigen::Vector3f> &_points) {
        std::lock_guard<std::mutex> lock(point_mutex);
        points = _points;
    }

    void update_image(const cv::Mat &image) {
        std::lock_guard<std::mutex> lock(image_mutex);
        feature_tracker_cvimage = image.clone();
    }

    std::vector<Eigen::Vector3f> get_points() {
        std::lock_guard<std::mutex> lock(point_mutex);
        return points;
    }
    std::vector<Frame> get_frames() {
        std::lock_guard<std::mutex> lock(frame_mutex);
        return frames;
    }

    std::vector<Eigen::Matrix4f> get_poses() {
        std::lock_guard<std::mutex> lock(poses_mutex);
        return poses;
    }

    cv::Mat get_image() {
        std::lock_guard<std::mutex> lock(image_mutex);
        return feature_tracker_cvimage.clone();
    }

};

class SLAMViewer: public ViewerDetail {

public:
    SLAMViewer(std::string title, int width, int height):
        ViewerDetail(title, width, height) {

        bgColor = Eigen::Vector4f(0.2f, 0.2f, 0.2f, 1.0f);

        config = std::make_shared<VisConfig>();
        data = std::make_shared<VisData>();
    }

    ~SLAMViewer() {
        viewer_thread.join();
    }

    void drawData() override {

        grid->draw(gridShader, viewport);

        auto frames = data->get_frames();
        glLineWidth(config->cameraScale);
        for(auto & frame : frames) {
            frustum->setup(frame.intrinsics, 0.001);
            frustum->setColor(config->cameraColor);
            frustum->transform(frame.pose);
            frustum->draw(pointShader, viewer->viewport);
        }
        glLineWidth(1.0f);

        auto points = data->get_points();
        pointCloud->setup(points, config->pointsColor);
        pointCloud->setPointSize(config->pointsScale);
        pointCloud->draw(pointShader, viewport);

        auto poses = data->get_poses();
        if(!poses.empty()) {
            std::vector<Eigen::Vector3f> traj_points;
            traj_points.reserve(poses.size() * 2);
            for(size_t i = 0; i < poses.size() - 1; ++i) {
                traj_points.push_back(poses[i + 0].block<3, 1>(0, 3));
                traj_points.push_back(poses[i + 1].block<3, 1>(0, 3));
            }
            line->setup(traj_points, config->trajColor);

            glLineWidth(config->trajScale);
            line->draw(pointShader, viewport);
            glLineWidth(1.0f);
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        any_window_active = ImGui::IsAnyItemActive() ;
        size_t widgetWidth = config->panelWidth - 10;

        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.3f));
        ImGui::Begin("Configuration", nullptr,  window_flags); 
        ImGui::SetWindowSize(ImVec2(widgetWidth, 0));
        

        if(notifier){
            std::lock_guard<std::mutex> lock(notifier->mtx);  
            const char* buttonText = isRunning ? "Running..." : "Paused";
            if (ImGui::Button(buttonText, ImVec2(widgetWidth-10, 0))){
                isRunning = !isRunning;
                notifier->ready = isRunning;
            }
            notifier->cv.notify_one();
        }

        auto image = data->get_image();
        if(!image.empty()) {
            size_t w = widgetWidth - 10;
            size_t h = w * image.rows / image.cols;
            colorTexture->setup(image.data, Eigen::Vector2i(image.cols, image.rows));
            ImGui::Image((void*)(intptr_t)colorTexture->getTextureID(), ImVec2(w, h));
        }

        // if (ImGui::CollapsingHeader("Debug Settings")) {
        //     ImGui::SliderFloat("Camera Scale", &config->cameraScale, 0.1f, 5.0f, "%.2f");
        //     ImGui::SliderFloat("Points Scale", &config->pointsScale, 0.1f, 5.0f, "%.2f");
        //     ImGui::SliderFloat("Trajectory Scale", &config->trajScale, 0.1f, 5.0f, "%.2f");
        //     ImGui::ColorEdit4("Camera Color", config->cameraColor.data());
        //     ImGui::ColorEdit4("Points Color", config->pointsColor.data());
        //     ImGui::ColorEdit4("Trajectory Color", config->trajColor.data());
        //     ImGui::ColorEdit4("Background Color", bgColor.data());
        // }

        ImGui::End();
        ImGui::PopStyleColor();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());   

    }

    void start(){
        viewer_thread = std::thread([this]() { this->run(); });
    }

    std::thread viewer_thread;
    std::shared_ptr<VisData> data;
    std::shared_ptr<VisConfig> config;
};
