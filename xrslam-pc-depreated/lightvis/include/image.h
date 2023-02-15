#ifndef LIGHTVIS_IMAGE_H
#define LIGHTVIS_IMAGE_H

#include <Eigen/Eigen>
#include <glbinding/gl/gl.h>
#include <nuklear.h>
#include <opencv2/opencv.hpp>

namespace lightvis {

struct Image {
    Image() {
        gl::glGenTextures(1, &texture_id);
        gl::glBindTexture(gl::GL_TEXTURE_2D, texture_id);
        gl::glTexParameteri(gl::GL_TEXTURE_2D, gl::GL_TEXTURE_MIN_FILTER,
                            gl::GL_LINEAR_MIPMAP_LINEAR);
        gl::glTexParameteri(gl::GL_TEXTURE_2D, gl::GL_TEXTURE_MAG_FILTER,
                            gl::GL_LINEAR);
        gl::glTexParameteri(gl::GL_TEXTURE_2D, gl::GL_TEXTURE_WRAP_S,
                            gl::GL_CLAMP_TO_EDGE);
        gl::glTexParameteri(gl::GL_TEXTURE_2D, gl::GL_TEXTURE_WRAP_T,
                            gl::GL_CLAMP_TO_EDGE);
        gl::glBindTexture(gl::GL_TEXTURE_2D, 0);
        nuklear_image = nk_image_id((int)texture_id);

        // update_image(image);
    }

    ~Image() { gl::glDeleteTextures(1, &texture_id); }

    bool empty() const { return (size.x() == 0 || size.y() == 0); }

    void update_image(const cv::Mat &image);

    struct nk_image nuklear_image;
    gl::GLuint texture_id;
    Eigen::Vector2i texture_size;
    Eigen::Vector2i size;
};

} // namespace lightvis

#endif // LIGHTVIS_IMAGE_H
