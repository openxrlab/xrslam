#include <GL/glew.h>  
#include <GLFW/glfw3.h>
#include "viewer.h"

ViewerDetail* ViewerDetail::viewer = nullptr;

ViewerDetail::ViewerDetail(std::string title, int width, int height):title(title){
    Eigen::Vector3f eye = Eigen::Vector3f(1, 0.5, 1);
    Eigen::Vector3f center = Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f up = Eigen::Vector3f(0, 1, 0); 
    viewport = Viewport(eye, center, up);

    viewer = this;
    viewer->viewport.window_size = {width, height};

    grid = std::make_unique<Grid>();
    cube = std::make_unique<Cube>();
    frustum = std::make_unique<Frustum>();
    fov_poly = std::make_unique<FovPoly>();
    image_plane = std::make_unique<Plane>();
    world_frame = std::make_unique<CoordinateFrame>();
    point_cloud = std::make_unique<PointCloud>();
    point_cloud->setup();

    image_texture = std::make_unique<ImageTex>();
    cube_mesh = std::make_shared<Cube>();

    render_buffer = std::make_unique<FrameBuffer>();

    notifier = std::make_shared<ViewerNotifier>();

}

Eigen::Matrix4f ViewerDetail::compute_camera_transform(
    const Eigen::Vector3f& cameraPosition, 
    const Eigen::Quaternionf& cameraOrientation,  
    const Eigen::Vector4f& intrinsics,
    const int viewportWidth, 
    const int viewportHeight,
    const float near, 
    const float far) {  

    Eigen::Matrix4f projmatrix = Eigen::Matrix4f::Zero();
    projmatrix(0, 0) = 2.0 * viewportHeight / viewportWidth;
    projmatrix(1, 1) = -2;
    projmatrix(2, 2) = (far + near) / (far - near);
    projmatrix(2, 3) = 2 * far * near / (near - far);
    projmatrix(3, 2) = 1.0;

    Eigen::Matrix3f rotation = cameraOrientation.matrix();
    Eigen::Vector3f position = cameraPosition * viewport.scale;
    Eigen::Matrix4f viewmatrix = Eigen::Matrix4f::Identity();
    viewmatrix.block<3, 3>(0, 0) = rotation.transpose();
    viewmatrix.block<3, 1>(0, 3) = -rotation.transpose() * position;

    Eigen::Matrix4f transform = projmatrix * viewmatrix;

    return transform;  
}  

ViewerDetail::~ViewerDetail(){
    unload();
    destroy();
}

void ViewerDetail::load_shader(){
    grid_shader = std::make_shared<Shader>(grid_vshader, grid_fshader);
    position_shader = std::make_shared<Shader>(position_vshader, position_fshader);
    texture_shader = std::make_shared<Shader>(texture_vshader, texture_fshader);
    points_shader = std::make_shared<Shader>(points_vshader, points_fshader);
    lighting_shader = std::make_shared<Shader>(lighting_vshader, lighting_fshader);   
}


void ViewerDetail::unload() {
    position_shader.reset();
    grid_shader.reset();
}

void ViewerDetail::draw_grid() {
    Eigen::Matrix4f transform = viewport.getProjectionMatrix() * viewport.getViewMatrix();

    grid->setup(viewport.world_xyz, viewport.scale);
    grid->draw(grid_shader, transform, viewport);
}


void ViewerDetail::draw_frustum(
    Eigen::Matrix4f model_matrix,
    Eigen::Vector4f intrinsics,
    Eigen::Vector4f color,
    cv::Mat image,
    bool show_image,
    bool show_fov,
    const float scale){

    Eigen::Matrix4f transform = viewport.getProjectionMatrix() * viewport.getViewMatrix();

    if(show_image){
        image_texture->setup(image, intrinsics);
        image_texture->transform(model_matrix);
        image_texture->draw(texture_shader, transform, viewport);
    }else{
        glDepthMask(GL_FALSE);
        image_plane->setup(intrinsics, scale);
        image_plane->setColor(Eigen::Vector4f(color.x(), color.y(), color.z(), 0.2));
        image_plane->transform(model_matrix);
        image_plane->draw(position_shader, transform, viewport);
        glDepthMask(GL_TRUE);
    }

    if(show_fov){
        glDepthMask(GL_FALSE);
        fov_poly->setup(intrinsics, scale);
        fov_poly->setColor(Eigen::Vector4f(color.x(), color.y(), color.z(), 0.1));
        fov_poly->transform(model_matrix);
        fov_poly->draw(position_shader, transform, viewport);
        glDepthMask(GL_TRUE);
    }

    frustum->setup(intrinsics, scale);
    frustum->setColor(color);
    frustum->transform(model_matrix);
    frustum->draw(position_shader, transform, viewport);

    world_frame->setup(scale);
    world_frame->transform(model_matrix);
    world_frame->draw(position_shader, transform, viewport);  
}


bool ViewerDetail::init(){
    
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW!" << std::endl;
        return false;
    }

    glfwWindowHint(GLFW_SAMPLES, 8);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    window = glfwCreateWindow(viewer->viewport.window_size.x(), viewer->viewport.window_size.y(), viewer->title.c_str(), NULL, NULL);

    if (window == NULL)
    {
        std::cerr << "Failed to create GLFW window!" << std::endl;
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    GLenum err = glewInit();
    if (err != GLEW_OK)
    {
        std::cerr << "Failed to create GLFW window" << std::endl;
        return false;
    }

    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, cursorPosCallback);
    glfwSetScrollCallback(window, scrollCallback);
    glfwSetKeyCallback(window, keyCallback);

    glEnable(GL_LINE_SMOOTH);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBlendEquation(GL_FUNC_ADD);  
    glEnable(GL_PROGRAM_POINT_SIZE);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    io.ConfigWindowsMoveFromTitleBarOnly = true;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    const char* glsl_version = "#version 330";
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Set Fonts
    io.Fonts->AddFontFromFileTTF("./assets/fonts/JetBrainsMono-Regular.ttf", 14.0f);

    // Set Windows option
    window_flags |= ImGuiWindowFlags_NoScrollbar;
    window_flags |= ImGuiWindowFlags_NoResize;
    // window_flags |= ImGuiWindowFlags_NoBackground;

    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 2.0f);

    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowTitleAlign = ImVec2(0.5f, 0.5f);
    style.WindowPadding = ImVec2(0.0f, 6.0f);
    style.WindowRounding = 6.0f;
    style.WindowBorderSize = 0.0f;

    return true;
}

void ViewerDetail::run(){

    if(!init()){
        std::cerr << "Failed to init the SLAM Viewer window" << std::endl;
        return;
    }

    load_shader();

    image_window = std::make_shared<ImageTexture>();
    
    while (!glfwWindowShouldClose(window))
    {
        GLfloat clearColor[4] = {clear_color.x, clear_color.y, clear_color.z, clear_color.w};
        glClearBufferfv(GL_COLOR, 0, clearColor);
        glEnable(GL_MULTISAMPLE);
        glEnable(GL_DEPTH_TEST);
        glClear(GL_DEPTH_BUFFER_BIT);

        glfwPollEvents();

        updateWindowSize();

        predraw();

        draw();

        gui();

        viewport.reset();
        
        glfwSwapBuffers(window);
    }
}

void ViewerDetail::setNotifier(std::shared_ptr<ViewerNotifier> notifier){
    viewer->notifier = notifier;
}

void ViewerDetail::predraw(){
    
    if(show_grid)
        draw_grid();

    Eigen::Matrix4f transform = viewport.getProjectionMatrix() * viewport.getViewMatrix();
}

void ViewerDetail::gui(){
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    ImGui::SetNextWindowSize(ImVec2(panel_size.x, 0), ImGuiCond_Always);

    window_hovered = false;
    any_window_active = ImGui::IsAnyItemActive() ;

    if (show_imgui_demo_window)
        ImGui::ShowDemoWindow(&show_imgui_demo_window);  

    configuration_panel();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

}

void ViewerDetail::configuration_panel(){
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.3f));
    ImGui::SetNextWindowSize(ImVec2(panel_size.x, 0), ImGuiCond_Always);
    ImGui::Begin("Configuration", nullptr,  window_flags); 
    window_hovered |= check_window_hovered();

    ImGui::Checkbox("Show ImGui Demo Window", &show_imgui_demo_window);

    // static ImGuiSliderFlags flags = ImGuiSliderFlags_None;
    // ImGui::SliderInt("Progress", &selected_id, 0, latest_id, "%d", flags);

    ImVec2 windowSize = ImGui::GetWindowSize();
    ImVec2 size = ImGui::GetItemRectSize();
    size.x = windowSize.x;

    render_canvas(size.x);

    if (ImGui::Button("Save", size)){
        cv::Mat canvas = save_canvas();
        std::string filename = "snapshot-" + get_timestamp() + ".png";
        cv::imwrite(filename, canvas);
    }

    if(notifier){
        std::lock_guard<std::mutex> lock(notifier->mtx);  
        const char* buttonText = isRunning ? "Running..." : "Stopped";
        if (ImGui::Button(buttonText, size)){
            isRunning = !isRunning;
            notifier->ready = isRunning;
        }
        notifier->cv.notify_one();
    }

    static ImGuiSliderFlags flags1 = ImGuiSliderFlags_None;
    ImGui::SliderInt("Point Size", &pc_size, 1, 10, "%d", flags1);

    ImGui::Checkbox("Show Grid", &show_grid);

    if (ImGui::CollapsingHeader("Virtual World")){
        render_window();
    }

    if (ImGui::CollapsingHeader("Global Map")){
        ImGui::Checkbox("Show KeyFrame", &show_map_keyframe);
        ImGui::Checkbox("Show Image", &show_map_image);
        ImGui::Checkbox("Show PointCloud", &show_map_pointcloud);
    }

    if (ImGui::CollapsingHeader("Colors")){
        ImGui::ColorEdit4("Background", (float*)&clear_color);
        ImGui::ColorEdit4("Keyframe", (float*)&keyframe_color);
        ImGui::ColorEdit4("PointCloud", (float*)&point_cloud_color);
    }

    ImGui::Text("------------------- INFO -------------------");

    const Eigen::Vector3f p = viewport.getPosition() / viewport.scale;
    const Eigen::Vector3f ypr = viewport.viewport_ypr;
    ImGui::Text("Viewport xyz: x:%.3f y:%0.3f z:%0.3f", p.x(), p.y(), p.z());
    ImGui::Text("Viewport ypr: y:%.3f p:%0.3f r:%0.3f", ypr.x(), ypr.y(), ypr.z());
    ImGui::Text("Viewport scale: %0.3f", viewport.scale);

    ImGui::Text("Average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

    ImGui::End();
    ImGui::PopStyleColor();
}

cv::Mat ViewerDetail::save_canvas(){
    cv::Mat pixels(viewport.framebuffer_size.y(), viewport.framebuffer_size.x(), CV_8UC3);
    glReadPixels(0, 0, pixels.cols, pixels.rows, GL_BGR, GL_UNSIGNED_BYTE, pixels.data);
    cv::Mat flipped;
    cv::flip(pixels, flipped, 0);
    return flipped;
}

std::string ViewerDetail::get_timestamp(){
    auto now        = std::chrono::system_clock::now();
    auto now_ms     = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    auto sectime    = std::chrono::duration_cast<std::chrono::seconds>(now_ms);

    std::time_t timet = sectime.count();
    struct tm curtime;
    localtime_r(&timet, &curtime);

    std::stringstream ss;
    ss << std::put_time(&curtime, "%Y-%m-%d-%H-%M-%S");
    std::string buffer = ss.str();
    return std::string(buffer);
}

void ViewerDetail::updateWindowSize()
{
    glfwGetWindowSize(viewer->window, &viewer->viewport.window_size.x(), &viewer->viewport.window_size.y());
    glfwGetFramebufferSize(viewer->window, &viewer->viewport.framebuffer_size.x(), &viewer->viewport.framebuffer_size.y());
    glViewport(0, 0, viewer->viewport.framebuffer_size.x(), viewer->viewport.framebuffer_size.y());
}

void ViewerDetail::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    auto &state = viewer->mouse_states;
    if(viewer->any_window_active){
        state.mouse_left = state.mouse_middle = state.mouse_right = false;
        return;
    }
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        state.mouse_left = (action == GLFW_PRESS);
    } else if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
        state.mouse_middle = (action == GLFW_PRESS);
    } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        state.mouse_right = (action == GLFW_PRESS);
    }
}

void ViewerDetail::cursorPosCallback(GLFWwindow* window, double x, double y) {
    if(viewer->any_window_active)
        return;
    
    auto &state = viewer->mouse_states;
    auto& viewport = viewer->viewport;

    state.mouse_drag_position = {static_cast<float>(x), static_cast<float>(y)};

    if (!(state.mouse_left || state.mouse_middle || state.mouse_right  )) {
        state.mouse_normal_position = {static_cast<float>(x), static_cast<float>(y)};
        viewport.last_ypr = viewport.viewport_ypr;
        viewport.last_position = viewport.position;
    }

    Eigen::Vector2f drag = state.mouse_drag_position - state.mouse_normal_position;
    if(state.mouse_left) {
        viewport.viewport_ypr.x() = viewport.last_ypr.x() + drag.x() / 10;
        viewport.viewport_ypr.y() = viewport.last_ypr.y() - drag.y() / 10;
    }

    if(state.mouse_right){
        viewport.position = viewport.last_position + (-viewport.right * drag.x() + viewport.forward * drag.y()) / 600.0;
    }
}

void ViewerDetail::scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    if(viewer->any_window_active)
        return;

    auto &state = viewer->mouse_states;
    auto &viewport = viewer->viewport;
    state.scroll = {xoffset, yoffset};
    viewport.scale = std::clamp(viewport.scale * (1.0 + yoffset / 100.0), 1.0e-4, 1.0e4);
}

void ViewerDetail::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods){
    if(viewer->any_window_active)
        return;

    auto &viewport = viewer->viewport;

    if (glfwGetKey(viewer->window, GLFW_KEY_W) == GLFW_PRESS){
            viewport.viewport_ypr.y() -= 2;
    }

    if (glfwGetKey(viewer->window, GLFW_KEY_S) == GLFW_PRESS){
        viewport.viewport_ypr.y() += 2;
    }

    if (glfwGetKey(viewer->window, GLFW_KEY_A) == GLFW_PRESS){
        viewport.viewport_ypr.x() -= 2;
    }

    if (glfwGetKey(viewer->window, GLFW_KEY_D) == GLFW_PRESS){
        viewport.viewport_ypr.x() += 2;
    }
    if (glfwGetKey(viewer->window, GLFW_KEY_UP) == GLFW_PRESS){
        viewport.scale = std::clamp(viewport.scale * (1.0 + 2 / 100.0), 1.0e-4, 1.0e4);
    }
    if (glfwGetKey(viewer->window, GLFW_KEY_DOWN) == GLFW_PRESS){
        viewport.scale = std::clamp(viewport.scale * (1.0 - 2 / 100.0), 1.0e-4, 1.0e4);
    }
}

bool ViewerDetail::check_window_hovered(){
    return ImGui::IsItemHovered() || ImGui::IsAnyItemHovered()? true: false;
}

void ViewerDetail::destroy(){
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    
    glfwDestroyWindow(window);
    glfwTerminate();
}