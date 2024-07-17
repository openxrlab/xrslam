#ifndef __VIEWPORT_H__
#define __VIEWPORT_H__

#include <Eigen/Eigen>
#include <Eigen/Dense>

struct Viewport {
    Eigen::Vector2i window_size;
    Eigen::Vector2i framebuffer_size;
    Eigen::Vector3f viewport_ypr;
    Eigen::Vector3f last_ypr;
    Eigen::Vector3f position = {1, 1, 1};
    Eigen::Vector3f last_position = position;
    Eigen::Vector3f world_xyz = {0, 0, 0};
    Eigen::Vector3f forward;
    Eigen::Vector3f up;
    Eigen::Vector3f right;
    
    float viewport_distance = 10;
    float scale = 1.0;
    float near = 1.0e-2;
    float far = 1.0e4;
    float focal = 1;

    bool follow_camera = false;
    Eigen::Matrix4f followed_projmatrix = Eigen::Matrix4f::Identity();

    Viewport(
        Eigen::Vector3f eye = Eigen::Vector3f(1, 1, 1), 
        Eigen::Vector3f center = Eigen::Vector3f(0, 0, 0), 
        Eigen::Vector3f up = Eigen::Vector3f(0, 1, 0)):
        position(eye), world_xyz(center), up(up){
        
        forward = (center - eye).normalized();
        right = up.cross(forward).normalized();

        viewport_ypr[0] = atan2(forward[0], forward[2]) * 180.0 / M_PI;     // yaw
        viewport_ypr[1] = asin(forward[1]) * 180.0 / M_PI;                  // pitch
        viewport_ypr[2] = 0;                                                // roll

        position = getPosition();
        last_ypr = viewport_ypr;
        last_position = position;
    }

    Eigen::Matrix3f getRotation() const{
        Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
        R = Eigen::AngleAxisf(viewport_ypr[2] * M_PI / 180.0f, Eigen::Vector3f::UnitZ()) * R; // r
        R = Eigen::AngleAxisf(viewport_ypr[1] * M_PI / 180.0f, Eigen::Vector3f::UnitX()) * R; // p
        R = Eigen::AngleAxisf(viewport_ypr[0] * M_PI / 180.0f, Eigen::Vector3f::UnitY()) * R; // y

        Eigen::Matrix3f view((Eigen::Matrix3f() << 1, 0, 0, 0, 0, 1, 0, -1, 0).finished());
        R = view * R;
        return R;
    }

    Eigen::Vector3f getPosition() const{
        Eigen::Matrix3f rotation = getRotation();
        Eigen::Vector3f position = - rotation.col(2) * viewport_distance;
        return position;
    }

    Eigen::Matrix4f getViewMatrix() const{
        if(follow_camera){
            return followed_projmatrix;
        }
        Eigen::Matrix3f rotation = getRotation();
        Eigen::Vector3f position = getPosition();
        Eigen::Matrix4f viewmatrix = Eigen::Matrix4f::Identity();
        viewmatrix.block<3, 3>(0, 0) = rotation.transpose();
        viewmatrix.block<3, 1>(0, 3) = -rotation.transpose() * position;
        return viewmatrix;
    }

    Eigen::Matrix4f getProjectionMatrix() const {
        Eigen::Matrix4f projmatrix = Eigen::Matrix4f::Zero();
        projmatrix(0, 0) = 2 * (focal * framebuffer_size.y()) / framebuffer_size.x();
        projmatrix(1, 1) = -2 * focal ;
        projmatrix(2, 2) = (far + near) / (far - near);
        projmatrix(2, 3) = 2 * far * near / (near - far);
        projmatrix(3, 2) = 1.0;
        return projmatrix;
    }

    void setViewMatrix(Eigen::Matrix4f transform){
        follow_camera = true;
        auto R = transform.block<3, 3>(0, 0);
        auto t = transform.block<3, 1>(0, 3);
        t = (t - 5 * R.col(2));

        followed_projmatrix.block<3, 3>(0, 0) = R.transpose();
        followed_projmatrix.block<3, 1>(0, 3) = -R.transpose() * t;
    }

    void reset(){
        follow_camera = false;
    }
};

#endif