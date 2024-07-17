#ifndef __WINDOW_H__
#define __WINDOW_H__

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <viewer.h>
#include <xrslam/utility/message.h>

class Viewer: public ViewerDetail{
public:
    Viewer(std::string title, int width=1280, int height=720, bool isRunning=false):
        ViewerDetail(title, width, height){
            ViewerDetail::isRunning = isRunning;
            ViewerDetail::show_grid = true;
            ViewerDetail::global_map_color = colors_table[0];
    }
    ~Viewer(){}

    void draw() override{
        draw_frame();

        draw_trajectory();

        if(show_map_keyframe){
            draw_keyframe();
        }
        
        if(show_map_pointcloud){
            draw_mappoint();
        }
    }

    void render_canvas(double width) override{
        if(auto frame = get_latest_frame()){
            double ratio = width / (double)frame->image.cols;
            int height = int(frame->image.rows * ratio);
            image_texture->setup(frame->image, frame->intrinsics);
            ImGui::Image((void*)(intptr_t)image_texture->getTextureID(), ImVec2(width, height));
        }
    }

    void draw_frame(){
        
        std::shared_ptr<VizFrame> frame = get_latest_frame();

        if(!frame) return;

        auto c = global_map_color;
        Eigen::Vector4f color(c.x, c.y, c.z, c.w);
        Eigen::Matrix4f model_matrix = frame->getModelMatrix();
        Eigen::Matrix4f transform = viewport.getProjectionMatrix() * viewport.getViewMatrix();

        draw_frustum(model_matrix, frame->intrinsics, color, frame->image, show_map_image, false);
    }

    void draw_trajectory(){

        if(frames.size() < 2) return;

        auto c = global_map_color;
        Eigen::Vector4f color(c.x, c.y, c.z, c.w);

        Eigen::Matrix4f transform = viewport.getProjectionMatrix() * viewport.getViewMatrix();
        auto trajectory = std::make_unique<Line>();

        std::vector<Eigen::Vector3f> frames_pos;
        for(size_t i = 0; i < frames.size(); ++i){
            if(auto frame = get_frame(i)){
                frames_pos.push_back(frame->pose.p.cast<float>());
            }
        }

        std::vector<Eigen::Vector3f> _t(frames_pos.size() * 2 - 2);
        std::vector<Eigen::Vector4f> _c(frames_pos.size() * 2 - 2);
        for(size_t i = 0; i < frames_pos.size() - 1; ++i){
            _t[i * 2] = frames_pos[i];
            _t[i * 2 + 1] = frames_pos[i + 1];
            _c[i * 2] = color;
            _c[i * 2 + 1] = color;
        }
        trajectory->setup(_t, _c);
        trajectory->draw(position_shader, transform, viewport);
    }

    void draw_keyframe(){
        auto c = keyframe_color;
        Eigen::Vector4f color(c.x, c.y, c.z, c.w);

        auto keyframes = get_sw_keyframes();

        if(keyframes.empty()) return;

        for(auto& kf: keyframes){
            draw_frustum(kf->getModelMatrix(), kf->intrinsics, color, cv::Mat(), false, false, 0.5);
        }
    }

    void draw_mappoint(){
        auto c = point_cloud_color;
        Eigen::Vector4f color(c.x, c.y, c.z, c.w);

        Eigen::Matrix4f transform = viewport.getProjectionMatrix() * viewport.getViewMatrix();
        auto mps = get_mappoints();

        if(mappoints.empty()) return;

        auto point_cloud = std::make_unique<PointCloud>();
        point_cloud->setup(mps, color);
        point_cloud->setPointSize(pc_size);
        point_cloud->draw(points_shader, transform, viewport);

    }

    std::mutex frame_mutex;
    std::vector<std::shared_ptr<VizFrame>> frames;

    std::mutex sw_keyframe_mutex;
    std::vector<std::shared_ptr<VizFrame>> sw_keyframes;

    std::mutex mappoint_mutex;
    std::vector<Eigen::Vector3f> mappoints;

    void add_frame(std::shared_ptr<VizFrame> frame){
        std::unique_lock<std::mutex> lock(frame_mutex);
        frames.push_back(frame);
    }
    
    std::shared_ptr<VizFrame> get_frame(size_t id){
        std::unique_lock<std::mutex> lock(frame_mutex);
        if(frames.empty() || id >= frames.size()) return nullptr;
        return frames[id];
    }

    std::shared_ptr<VizFrame> get_latest_frame(){
        std::unique_lock<std::mutex> lock(frame_mutex);
        if(frames.empty()) return nullptr;
        return frames.back();
    }

    std::vector<std::shared_ptr<VizFrame>> get_sw_keyframes(){
        std::unique_lock<std::mutex> lock(sw_keyframe_mutex);
        return sw_keyframes;
    }

    void update_mappoints(std::vector<Eigen::Vector3d> vmp){
        std::unique_lock<std::mutex> lock(mappoint_mutex);
        mappoints.clear();
        mappoints.reserve(vmp.size());
        for(auto xyz: vmp){
            mappoints.push_back(xyz.cast<float>());
        }
    }

    void update_sliding_window(std::vector<std::shared_ptr<VizFrame>> sw_keyframes){
        std::unique_lock<std::mutex> lock(sw_keyframe_mutex);
        this->sw_keyframes = sw_keyframes;
    }

    std::vector<Eigen::Vector3f> get_mappoints(){
        std::unique_lock<std::mutex> lock(mappoint_mutex);
        return mappoints;
    }

    Eigen::Vector3f& location(){
        return viewport.world_xyz;
    }

    void render_window(){

        std::shared_ptr<VizFrame> frame = get_latest_frame();

        if(!frame) return;

        const size_t w = panel_size.x;
        const size_t h = frame->image.rows * panel_size.x / frame->image.cols;

        render_buffer = std::make_unique<FrameBuffer>(frame->image.cols, frame->image.rows);
        render_buffer->rescaleFrameBuffer(w, h);
        // render_buffer->setup(frame->image, 1);

        glViewport(0, 0, w, h);
        ImGui::Image((void*)(intptr_t)render_buffer->getFrameTexture(), ImVec2(w, h), ImVec2(0, 1), ImVec2(1, 0));
        // ImGui::Image((void*)(intptr_t)render_buffer->getFrameTexture(), ImVec2(w, h), ImVec2(0, 1), ImVec2(1, 0));

        Eigen::Matrix4f transform = compute_camera_transform(
            frame->pose.p.cast<float>(),
            frame->pose.q.cast<float>(),
            frame->intrinsics,
            w, h);
        
        render_buffer->bind();
        GLfloat clearColor[4] = {clear_color.x, clear_color.y, clear_color.z, clear_color.w};
        glClearBufferfv(GL_COLOR, 0, clearColor);

        grid->setup(viewport.world_xyz, viewport.scale);
        grid->draw(grid_shader, transform, viewport);

        // for(auto& obj: object_list){
        //     transform = transform * obj->getModelMatrix(viewport.scale);
        //     obj->draw(position_shader, transform, viewport);
        // }
        
        render_buffer->unbind();
    }


};

class SLAMWindow{
public:
    std::string title;
    int width;
    int height;
    bool isRunning;
    std::shared_ptr<Viewer> detail = nullptr;

    SLAMWindow(std::string title, int width, int height, bool isRunning=false):
        title(title), width(width), height(height), isRunning(isRunning){
        }


    void start_thread(){
        detail = std::make_shared<Viewer>("XRSLAM PC", this->width, this->height, this->isRunning);
        detail->run();
    }

    void run(){
        std::thread viewer_thread([this]() { this->start_thread(); });
        viewer_thread.detach();
    }
};



#endif