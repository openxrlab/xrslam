#ifndef __VIEWER_H__
#define __VIEWER_H__

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <GL/glew.h>  
#include <GLFW/glfw3.h>
#include <opencv2/opencv.hpp>
#include <condition_variable> 
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "framebuffer.h"
#include "shader.h"
#include "mesh.h"
#include "glshader.h"
#include "viewport.h"

const ImVec4 colors_table[] = {
    ImVec4(0.9059f, 0.2980f, 0.2353f, 1.0f),    // RGB: 231, 76, 60
    ImVec4(0.9451f, 0.7686f, 0.0588f, 1.0f),    // RGB: 241, 196, 15
    ImVec4(0.2196f, 0.6980f, 0.3058f, 1.0f),    // RGB: 56, 178, 78
    ImVec4(0.3647f, 0.6784f, 0.8863f, 1.0f),    // RGB: 93, 173, 226
    ImVec4(0.7900f, 0.6190f, 1.0000f, 1.0f),    // RGB: 74, 20, 140
    ImVec4(0.1137f, 0.5137f, 0.2824f, 1.0f),    // RGB: 29, 131, 72
};

struct ViewerNotifier {  
public:  
    bool ready = true;  
    std::mutex mtx;  
    std::condition_variable cv;  
};

struct MouseStates {
    bool mouse_left = false;
    bool mouse_middle = false; 
    bool mouse_right = false;
    Eigen::Vector2f mouse_normal_position;
    Eigen::Vector2f mouse_drag_position;
    Eigen::Vector2f scroll;
};


class ViewerDetail{
public:
    std::string title;
    Viewport viewport;
    MouseStates mouse_states;

    std::shared_ptr<Shader> grid_shader;
    std::shared_ptr<Shader> position_shader;
    std::shared_ptr<Shader> texture_shader;
    std::shared_ptr<Shader> points_shader;
    std::shared_ptr<Shader> lighting_shader;

    std::unique_ptr<Grid> grid;
    std::unique_ptr<Cube> cube;
    std::unique_ptr<Frustum> frustum;
    std::unique_ptr<FovPoly> fov_poly;
    std::unique_ptr<Plane> image_plane;
    std::unique_ptr<CoordinateFrame> world_frame;
    std::unique_ptr<ImageTex> image_texture;
    std::unique_ptr<PointCloud> point_cloud;
    std::unique_ptr<FrameBuffer> render_buffer;

    std::shared_ptr<ViewerNotifier> notifier;

    std::mutex object_mutex;
    std::shared_ptr<ObjectMesh> logo_mesh;
    std::vector<std::shared_ptr<Mesh>> obj_meshes;
    std::shared_ptr<Cube> cube_mesh;
    
    void setNotifier(std::shared_ptr<ViewerNotifier> notifier);

    void add_mesh(std::shared_ptr<Mesh> obj){
        std::unique_lock<std::mutex> lock(object_mutex);
        obj_meshes.push_back(obj);
    }

    std::vector<std::shared_ptr<Mesh>> get_mesh_list(){
        std::unique_lock<std::mutex> lock(object_mutex);
        return obj_meshes;
    }

    std::unique_ptr<ImageTexture> showed_image_raw;
    std::unique_ptr<ImageTexture> showed_image_undistort;

    std::shared_ptr<ImageTexture> image_window;

    float cube_scale = 0.25f;
    float frustum_scale = 0.35f;

    ViewerDetail(std::string title, int width=1280, int height=720);

    ~ViewerDetail();

    void load_shader();

    void unload();

    void draw_grid();
    
    void draw_frustum(Eigen::Matrix4f model_matrix, Eigen::Vector4f intrinsics,
        Eigen::Vector4f color, cv::Mat image, bool show_image=false, bool show_fov=false, const float scale=1.0);
    
    void draw_trajectory(std::string name);

    bool init();

    void run();

    void predraw();

    void gui();

    void configuration_panel();

    cv::Mat save_canvas();

    std::string get_timestamp();

    static void updateWindowSize();

    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);

    static void cursorPosCallback(GLFWwindow* window, double x, double y);

    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);

    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

    bool check_window_hovered();

    void destroy();

    Eigen::Matrix4f compute_camera_transform(
        const Eigen::Vector3f& cameraPosition, 
        const Eigen::Quaternionf& cameraOrientation,  
        const Eigen::Vector4f& intrinsics,
        const int viewportWidth, 
        const int viewportHeight,
        const float near=1.0e-2, 
        const float far=1.0e4);

protected:
    bool isRunning = true;

    ImVec4 clear_color = ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
    // ImVec4 logo_color = ImVec4(0.788f, 0.329f, 0.200f, 1.00f);
    ImVec4 point_cloud_color = ImVec4(0.85f, 0.76f, 0.14f, 1.00f);
    ImVec4 global_map_color = ImVec4(0.1f, 0.1f, 0.1f, 0.80f);
    // ImVec4 loop_color = ImVec4(0.5f, 0.5f, 0.5f, 0.80f);

    ImVec4 keyframe_color = ImVec4(0.8f, 0.8f, 0.8f, 0.20f);

    // Eigen::Vector3f logo_pos;
    // Eigen::Vector3f cube_pos;

    bool show_imgui_demo_window = false;
    bool show_grid = true;
    // bool show_logo = false;
    bool show_global_map = false;
    
    bool show_map_keyframe = true;
    bool show_map_graph = true;
    bool show_map_pointcloud = true;
    bool show_map_loop = true;
    bool show_map_image = false;

    int pc_size = 1;

    // std::map<std::string, std::shared_ptr<VizAgent>> agent_list;

    mutable std::mutex elements_mutex;

    ImGuiWindowFlags window_flags = 0;
    ImVec2 panel_size = ImVec2(300, 1000);

private:
    GLFWwindow* window;
    static ViewerDetail* viewer;

    bool window_hovered;
    bool any_window_active;

    // bool record = false;
    // cv::VideoWriter outputVideo;

    virtual void draw() = 0;

    virtual void render_canvas(double width) = 0;

    virtual void render_window() = 0;
};


#endif