#include <image.h>

namespace lightvis {

static unsigned int msb(unsigned int x) {
    x |= (x >> 1);
    x |= (x >> 2);
    x |= (x >> 4);
    x |= (x >> 8);
    x |= (x >> 16);
    return (x & ~(x >> 1));
}

void Image::update_image(const cv::Mat &image) {
    size.x() = image.cols;
    size.y() = image.rows;

    if (empty())
        return;

    texture_size.x() = std::min((int)msb((unsigned int)size.x()), 2048);
    texture_size.y() = std::min((int)msb((unsigned int)size.y()), 2048);

    cv::Mat rgb;
    cv::resize(image, rgb, cv::Size(texture_size.x(), texture_size.y()));
    cv::cvtColor(rgb, rgb, cv::COLOR_BGR2RGB);
    gl::glBindTexture(gl::GL_TEXTURE_2D, texture_id);
    gl::glTexImage2D(gl::GL_TEXTURE_2D, 0, gl::GL_RGB, texture_size.x(),
                     texture_size.y(), 0, gl::GL_RGB, gl::GL_UNSIGNED_BYTE,
                     rgb.ptr());
    gl::glGenerateMipmap(gl::GL_TEXTURE_2D);
    gl::glBindTexture(gl::GL_TEXTURE_2D, 0);
}

} // namespace lightvis
