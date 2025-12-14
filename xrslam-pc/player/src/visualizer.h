#ifndef __LITEVIZ_VISUALIZER_H__
#define __LITEVIZ_VISUALIZER_H__

#include <liteviz/core/detail.h>
#include <liteviz/core/common.h>
#include <liteviz/core/base_renderer.h>
#include <liteviz/core/base_config.h>
#include <liteviz/core/base_data.h>
#include <liteviz/core/shader.h>
#include <liteviz/core/viewport.h>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <thread>
#include <vector>

using namespace liteviz;

struct VisConfig: public BaseConfig {

    VisConfig() {}
    ~VisConfig() = default;

    Eigen::Vector4f trajColor = Eigen::Vector4f(0.7f, 0.2f, 0.2f, 1.0f);
    Eigen::Vector4f pointsColor = Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f);
    Eigen::Vector4f cameraColor = Eigen::Vector4f(0.8f, 0.2f, 0.2f, 1.0f);
    float trajScale = 1.0f;
    float pointsScale = 1.0f;
    float cameraScale = 1.0f;
    size_t x_size = 300;

    bool followCamera = false;
};

class VisData : public liteviz::BaseData {
public:
    struct Frame {
        cv::Mat image;
        Eigen::Vector4f intrinsics;
        Eigen::Matrix4f pose;

        Frame() = default;
        Frame(const cv::Mat &img, const Eigen::Vector4f &intrin, const Eigen::Matrix4f &p)
            : image(img), intrinsics(intrin), pose(p) {}
    };

    VisData() = default;
    ~VisData() = default;

    DataType getType() const override { return DataType::UNKNOWN; }

    void update_poses(const Eigen::Matrix4f &_pose) {
        std::lock_guard<std::mutex> lock(poses_mutex);
        poses.push_back(_pose);
    }

    void update_frames(const std::vector<Frame> &_frames) {
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

    std::vector<Eigen::Vector3f> get_points() const {
        std::lock_guard<std::mutex> lock(point_mutex);
        return points;
    }

    std::vector<Frame> get_frames() const {
        std::lock_guard<std::mutex> lock(frame_mutex);
        return frames;
    }

    std::vector<Eigen::Matrix4f> get_poses() const {
        std::lock_guard<std::mutex> lock(poses_mutex);
        return poses;
    }

    cv::Mat get_image() const {
        std::lock_guard<std::mutex> lock(image_mutex);
        return feature_tracker_cvimage.clone();
    }

private:
    mutable std::mutex frame_mutex;
    mutable std::mutex point_mutex;
    mutable std::mutex image_mutex;
    mutable std::mutex poses_mutex;

    std::vector<Frame> frames;
    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Matrix4f> poses;
    cv::Mat feature_tracker_cvimage;
};


class ConfigRenderer: public BaseRenderer {
public:
    ConfigRenderer(VisConfig* config): _config(config){
        colorTexture = std::make_shared<ImageTexture>(GL_RGB, GL_BGR, GL_UNSIGNED_BYTE);
    }

    void render(const Viewport& viewport) override {

        ImGui::Begin("System. Configuration", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::SetWindowSize(ImVec2(_config->x_size, 0));

        if(notifier){
            std::lock_guard<std::mutex> lock(notifier->mtx);  
            const char* buttonText = isRunning ? "Running..." : "Paused";
            if (ImGui::Button(buttonText, ImVec2(_config->x_size - 10, 0))){
                isRunning = !isRunning;
                notifier->ready = isRunning;
            }
            notifier->cv.notify_one();
        }

        auto image = _data->get_image();
        if(!image.empty() && colorTexture) {
            size_t w = _config->x_size - 10;
            size_t h = w * image.rows / image.cols;
            colorTexture->setup(image.data, Eigen::Vector2i(image.cols, image.rows));
            ImGui::Image((void*)(intptr_t)colorTexture->getTextureID(), ImVec2(w, h));
        }

        ImGui::Checkbox("Follow Camera", &_config->followCamera);

        ImGui::End();
    }

    void setNotifier(ViewerNotifier* vn) {
        notifier = vn;
    }

    void setData(VisData* data) {
        _data = data;
    }

private:
    VisConfig* _config;
    VisData* _data = nullptr;
    ViewerNotifier* notifier = nullptr;
    std::shared_ptr<ImageTexture> colorTexture  = nullptr;
    bool isRunning = true;
};


class SLAMRenderer : public BaseRenderer {
public:
    SLAMRenderer(const VisData* slamData, VisConfig* config): _data(slamData), _config(config) {
        
        _shader = std::make_shared<Shader>(
            (std::string(RESOURCE_DIR) + "/shaders/draw_point.vert").c_str(), 
            (std::string(RESOURCE_DIR) + "/shaders/draw_point.frag").c_str(),
            true
        );
        
        _grid_shader = std::make_shared<Shader>(
            (std::string(RESOURCE_DIR) + "/shaders/draw_grid.vert").c_str(), 
            (std::string(RESOURCE_DIR) + "/shaders/draw_grid.frag").c_str(),
            true
        );

        frustum = std::make_shared<Frustum>();
        pointCloud = std::make_shared<PointCloud>();
        line = std::make_shared<Line>();
        grid = std::make_shared<Grid>();
        grid->setup();
    }

    void render(const Viewport& viewport) override {
        if(!_shader) return;

        grid->draw(_grid_shader.get(), viewport);
        
        // only show current frame
        auto frames = _data->get_frames();
        if(!frames.empty()){
            auto curr_frame = frames.back();
            
            Viewport &vp = const_cast<Viewport&>(viewport);
            vp.follow(curr_frame.pose, _config->followCamera);
            
            frustum->setup(curr_frame.intrinsics, 0.001f);
            frustum->setColor(_config->cameraColor);
            frustum->transform(curr_frame.pose);
            frustum->draw(_shader.get(), viewport);
        }

        auto points = _data->get_points();
        pointCloud->setup(points, _config->pointsColor);
        pointCloud->setPointSize(_config->pointsScale);
        pointCloud->draw(_shader.get(), viewport);

        auto poses = _data->get_poses();
        if(!poses.empty()) {
            std::vector<Eigen::Vector3f> traj_points;
            traj_points.reserve((poses.size()>1? (poses.size()-1)*2:0));
            for(size_t i = 0; i + 1 < poses.size(); ++i) {
                traj_points.push_back(poses[i].block<3,1>(0,3));
                traj_points.push_back(poses[i+1].block<3,1>(0,3));
            }
            line->setup(traj_points, _config->trajColor);
            line->draw(_shader.get(), viewport);
        }
    }


private:
    std::shared_ptr<Frustum> frustum = nullptr;
    std::shared_ptr<PointCloud> pointCloud = nullptr;
    std::shared_ptr<Line> line = nullptr;
    std::shared_ptr<Grid> grid = nullptr;

    const VisData*  _data;
    std::shared_ptr<Shader> _shader;
    std::shared_ptr<Shader> _grid_shader;
    VisConfig* _config;
};

class Viewer: public ViewerDetail {

    std::thread renderThread;
    std::shared_ptr<VisConfig> visConfig;
    std::shared_ptr<VisData> visData; 

public:
    Viewer(std::string title, int width, int height):

        ViewerDetail(title, width, height) {

        visConfig = std::make_shared<VisConfig>();

        clearColor = vec4f(0.2f, 0.2f, 0.2f, 1.0f);

        show_default_configuration = false;

        std::cout << "LiteViz initialized." << std::endl;
    }

    ~Viewer() {
        if (renderThread.joinable()) {
            renderThread.join();
        }
    }

    bool initResources() override {
        
        visData = std::make_shared<VisData>();
        std::shared_ptr<SLAMRenderer> visRenderer = std::make_shared<SLAMRenderer>(
            visData.get(),
            visConfig.get()
        );
        _registeredRenderers.push_back(visRenderer);

        std::shared_ptr<ConfigRenderer> configRenderer = std::make_shared<ConfigRenderer>(
            visConfig.get()
        );
        _registeredGUIRenderers.push_back(configRenderer);
        configRenderer->setNotifier(_notifier.get());
        configRenderer->setData(visData.get());

        return true;
    }

    void start() {
        renderThread = std::thread([this]() {
            try {
                this->draw();
            } catch (const std::exception& e) {
                std::cerr << "Render thread exception: " << e.what() << std::endl;
            } catch (...) {
                std::cerr << "Render thread unknown exception" << std::endl;
            }
        });
    }

    VisData* data() {
        return visData.get();
    }
};

#endif // __LITEVIZ_VISUALIZER_H__