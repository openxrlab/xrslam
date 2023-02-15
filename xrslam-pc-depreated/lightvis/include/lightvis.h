#ifndef LIGHTVIS_H
#define LIGHTVIS_H

#include <Eigen/Eigen>
#include <functional>
#include <image.h>
#include <memory>
#include <shader.h>
#include <string>

namespace lightvis {

template <int Rows = Eigen::Dynamic, int Cols = Rows, bool UseRowMajor = false,
          typename T = double>
using matrix = typename std::conditional<
    Rows != 1 && Cols != 1,
    Eigen::Matrix<T, Rows, Cols,
                  UseRowMajor ? Eigen::RowMajor : Eigen::ColMajor>,
    Eigen::Matrix<T, Rows, Cols>>::type;

template <int Dimension = Eigen::Dynamic, bool RowVector = false,
          typename T = double>
using vector =
    typename std::conditional<RowVector, matrix<1, Dimension, false, T>,
                              matrix<Dimension, 1, false, T>>::type;

using quaternion = Eigen::Quaternion<double>;

class LightVisDetail;

struct MouseStates {
    bool mouse_left;
    bool mouse_middle;
    bool mouse_right;
    bool mouse_double_click;
    bool control_left;
    bool control_right;
    bool shift_left;
    bool shift_right;
    Eigen::Vector2f mouse_normal_position;
    Eigen::Vector2f mouse_drag_position;
    Eigen::Vector2f scroll;
};

class LightVis {
    friend class LightVisDetail;

  public:
    LightVis(const std::string &title, int width, int height);
    virtual ~LightVis();

    void show();
    void hide();

    int width() const;
    int height() const;

    const Eigen::Vector3f &location() const;
    Eigen::Vector3f &location();
    const float &scale() const;
    float &scale();

    Eigen::Matrix4f projection_matrix(float f = 1.0, float near = 1.0e-2,
                                      float far = 1.0e4);
    Eigen::Matrix4f view_matrix();
    Eigen::Matrix4f model_matrix();
    Shader *shader();

    void add_points(std::vector<Eigen::Vector3f> &points,
                    Eigen::Vector4f &color);
    void add_points(std::vector<Eigen::Vector3f> &points,
                    std::vector<Eigen::Vector4f> &colors);

    void add_trajectory(std::vector<Eigen::Vector3f> &positions,
                        Eigen::Vector4f &color);
    void add_trajectory(std::vector<Eigen::Vector3f> &positions,
                        std::vector<Eigen::Vector4f> &colors);

    void add_camera(std::vector<Eigen::Vector3f> &positions, vector<3> &p,
                    quaternion &q, matrix<3> K, Eigen::Vector4f &color,
                    double size = 0.0005);

    void add_separator();
    void add_label(const std::string &label);
    void add_image(const Image *image);
    void add_graph(const std::vector<double> &values);
    void add_progress(const double &value);

  protected:
    virtual void load();
    virtual void unload();
    virtual void draw(int w, int h);
    virtual bool mouse(const MouseStates &states);
    // virtual bool keyboard()
    virtual void gui(void *ctx, int w, int h);

  private:
    std::unique_ptr<LightVisDetail> detail;
};

int main();

} // namespace lightvis

#endif // LIGHTVIS_H
