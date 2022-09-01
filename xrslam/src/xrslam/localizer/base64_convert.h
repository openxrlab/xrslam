#ifndef BASE64_CONVERTER_HPP
#define BASE64_CONVERTER_HPP

#include "base64.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <xrslam/xrslam.h>

inline std::string encode_image_msg(cv::Mat img, std::string ext = ".png") {
    std::string str_binary;
    if (ext.find("bin") != std::string::npos) {
        assert(img.isContinuous());
        str_binary.assign(img.data, img.data + img.total());
    } else {
        // encode to jpg or png
        std::vector<uchar> buff;
        cv::imencode(ext, img, buff);
        str_binary.assign(buff.begin(), buff.end());
    }
    // uchar* data = img->get_rawdata();
    // str_binary.assign(data, data + img->total());

    // convert to base64
    std::string str_base64 = macaron::Base64::Encode(str_binary);

    return str_base64;
}

inline cv::Mat decode_image_msg(const std::string &str_base64,
                                bool is_uncompressed_binary = false,
                                int width = 0, int height = 0,
                                int channel = 1) {
    // get base64 image data
    std::string str_binary;
    // decode base64
    auto err_info = macaron::Base64::Decode(str_base64, str_binary);
    if (!err_info.empty())
        std::cerr << err_info;
    std::vector<uchar> buff(str_binary.begin(), str_binary.end());
    cv::Mat img;
    if (is_uncompressed_binary) {
        switch (channel) {
        case 1:
            img.create(cv::Size(width, height), CV_8UC1);
            break;
        case 3:
            img.create(cv::Size(width, height), CV_8UC3);
        default:
            std::runtime_error("Unknown channel");
            break;
        }
        std::memcpy(img.data, buff.data(), buff.size());
    } else {
        img =
            cv::imdecode(buff, cv::IMREAD_UNCHANGED); //, cv::IMREAD_GRAYSCALE);
    }
    return img;
}

#endif /*BASE64_CONVERTER_HPP*/
