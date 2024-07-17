#ifndef __MSEEAGE_H__
#define __MSEEAGE_H__

#include <iostream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/set.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/polymorphic.hpp>
#include <xrslam/utility/serialize.h>
#include <xrslam/xrslam.h>

namespace xrslam {

#define DEG2RAD(x) (x * 0.01745329251994329576923690768489)
#define RAD2DEG(x) (x * 57.295779513082320876798154814105)

enum DataType{
    DT_FRAME    = 0x00,
    DT_OBJ      = 0x01,
    DT_UNKNOWN  = 0x02,
    DT_SWT_MAP  = 0x03,
    DT_KEYFRAME = 0x04,
    DT_NOTIFY   = 0x05,
    DT_MESSAGE  = 0x06,
    DT_ALIGN    = 0x07,
};

struct NotifyMsg{
    friend class cereal::access;  

    template<class Archive>
    void serialize(Archive & ar){
        ar(is_stop);
        ar(name);
        ar(message);
    }

    NotifyMsg(){}
    NotifyMsg(std::string name, bool is_stop):name(name), is_stop(is_stop){}

    std::string name;
    bool is_stop = false;
    std::string message = "hi";

    std::string serialized(std::string name=""){
        if(!name.empty()) this->name = name;
        std::ostringstream oss;
        {
            cereal::BinaryOutputArchive oa(oss);  
            oa(*this);
        }
        return oss.str();
    }

    static NotifyMsg deserialized(const std::string& data){
        NotifyMsg msg;
        std::istringstream iss(data);
        try {
            cereal::BinaryInputArchive ia(iss);
            ia(msg);
        } catch (const std::exception& e) {
            std::cerr << "Cereal archive exception: " << e.what() << std::endl;
        }
        return msg;
    }
};
struct Notifier{
public:
    Notifier(){
        msg = NotifyMsg();
    }
    NotifyMsg msg;
    std::condition_variable cv;

    std::mutex mtx;
    std::mutex msg_mtx;

    void update_status(NotifyMsg msg){
        std::unique_lock<std::mutex> lock(msg_mtx);
        this->msg = msg;
    }

    bool is_stop(){
        std::unique_lock<std::mutex> lock(msg_mtx);
        return msg.is_stop;
    }
};

struct FrameMsg{
    friend class cereal::access;  
    template<class Archive>
    void serialize(Archive & ar){
        ar(name);
        ar(id);
        ar(scale);
        ar(pose);
        ar(landmarks);
        ar(image);
        ar(intrinsics);
        ar(width);
        ar(height);
        ar(timestamp);
    }

    size_t id;
    double timestamp = 0;
    std::string name;
    float scale = 1.0f;
    std::vector<Eigen::Vector3f> landmarks;
    std::vector<Eigen::Vector2f> features;
    cv::Mat image;
    Pose pose;
    Eigen::Vector4f intrinsics;
    size_t width;
    size_t height;

    std::string serialized(std::string name=""){
        if(!name.empty()) this->name = name;
        width = image.cols;
        height = image.rows;
        size_t w_ = size_t(image.cols * scale);
        size_t h_ = size_t(image.rows * scale);
        cv::resize(image, image, cv::Size(w_, h_));
        std::ostringstream oss;
        {
            cereal::BinaryOutputArchive oa(oss);  
            oa(*this);
        }
        return oss.str();
    }

    static FrameMsg deserialized(const std::string& data){
        FrameMsg msg;
        std::istringstream iss(data);
        try {
            cereal::BinaryInputArchive ia(iss);
            ia(msg);
        } catch (const std::exception& e) {
            std::cerr << "Cereal archive exception: " << e.what() << std::endl;
        }
        return msg;
    }
};

struct VizFrame{
    VizFrame(){}
    VizFrame(FrameMsg msg){
        id = msg.id;
        scale = msg.scale;
        pose = msg.pose;
        landmarks = msg.landmarks;
        image = msg.image;
        intrinsics = msg.intrinsics;
        name = msg.name;
        cv::resize(image, image, cv::Size(msg.width, msg.height));
    }

    VizFrame(cv::Mat img, Pose pose, Eigen::Vector4f intrinsic){
        this->image = img;
        this->pose = pose;
        this->intrinsics = intrinsic;
    }

    // VizFrame(KeyFrame* keyframe){
    //     id = keyframe->frame_id;
    //     scale = 1;
    //     pose = keyframe->pose;
    //     image = keyframe->image;
    //     intrinsics = keyframe->intrinsics.cast<float>();
    // }

    size_t id;
    std::string name;
    float scale = 1.0f;
    std::vector<Eigen::Vector3f> landmarks;
    cv::Mat image;
    Pose pose;
    Eigen::Vector4f intrinsics;

    Eigen::Matrix4f getModelMatrix(){
        Eigen::Matrix3f R = Eigen::AngleAxisf(DEG2RAD(180.0f), Eigen::Vector3f::UnitZ()).matrix();
        Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
        model.block<3,3>(0,0) = pose.q.cast<float>().toRotationMatrix() * R;
        model.block<3,1>(0,3) = pose.p.cast<float>();
        return model;
    }

    std::mutex pose_mutex;
    void set_pose(Pose pose){
        std::unique_lock<std::mutex> lock(pose_mutex);
        this->pose = pose;
    }

    Pose get_pose(){
        std::unique_lock<std::mutex> lock(pose_mutex);
        return pose;
    }
};

struct ObjectMsg{
    friend class cereal::access;  

    template<class Archive>
    void serialize(Archive & ar){
        ar(pose);
        ar(name);
    }

    std::string name;
    Pose pose;

    std::string serialized(std::string name=""){
        if(!name.empty()) this->name = name;
        std::ostringstream oss;
        {
            cereal::BinaryOutputArchive oa(oss);  
            oa(*this);
        }
        return oss.str();
    }

    static ObjectMsg deserialized(const std::string& data){
        ObjectMsg msg;
        std::istringstream iss(data);  
        try {
            cereal::BinaryInputArchive ia(iss);
            ia(msg);
        } catch (const std::exception& e) {
            std::cerr << "Cereal archive exception: " << e.what() << std::endl;
        }
        return msg;
    }
};


struct VizObject{

    friend class cereal::access;  
    
    template<class Archive>
    void serialize(Archive & ar){
        ar(pose);
    }

    Pose pose;

    VizObject(){}
    
    VizObject(Eigen::Vector3f p, Eigen::Quaternionf q){
        pose.p = p.cast<double>();
        pose.q = q.cast<double>();
    }
};

struct KeyFrameMsg{
    friend class cereal::access;  

    template<class Archive>
    void serialize(Archive & ar){
        ar(name);  
        ar(frame_id);  
        ar(timestamp);  
        ar(cvimage); 
        ar(mDescriptors);
        ar(mvKeysUn);
        ar(camera_pose);  
        ar(camera);  
        ar(keypoints);  
        ar(intrinsics);
        ar(head_id);
        ar(scale);
        ar(width);
        ar(height);
        ar(landmarks);
    }

    std::string name;
    PoseState camera_pose;
    ExtrinsicParams camera;
    cv::Mat cvimage;
    size_t frame_id;
    double timestamp;
    vector<4> intrinsics;
    std::vector<vector<2>> keypoints;
    size_t head_id;
    cv::Mat mDescriptors;
    std::vector<cv::KeyPoint> mvKeysUn;
    size_t width;
    size_t height;
    float scale;
    std::vector<vector<3>> landmarks;

    std::string serialized(std::string name=""){
        if(!name.empty()) this->name = name;
        std::ostringstream oss;
        {
            cereal::BinaryOutputArchive oa(oss);  
            oa(*this);
        }
        return oss.str();
    }

    static KeyFrameMsg deserialized(const std::string& data){
        KeyFrameMsg msg;
        std::istringstream iss(data);
        try {
            cereal::BinaryInputArchive ia(iss);
            ia(msg);
        } catch (const std::exception& e) {
            std::cerr << "Cereal archive exception: " << e.what() << std::endl;
        }
        return msg;
    }
};


struct AlignMsg{
    friend class cereal::access;  

    template<class Archive>
    void serialize(Archive & ar){
        ar(T_wmatch_wquery);
    }

    AlignMsg(){}
    AlignMsg(std::string name, matrix<4> T):name(name), T_wmatch_wquery(T){}

    std::string name;
    matrix<4> T_wmatch_wquery;
    
    std::string serialized(std::string name=""){
        if(!name.empty()) this->name = name;
        std::ostringstream oss;  
        {
            cereal::BinaryOutputArchive oa(oss);  
            oa(*this);
        }
        return oss.str();
    }

    static AlignMsg deserialized(const std::string& data){
        AlignMsg msg;
        std::istringstream iss(data);
        try {
            cereal::BinaryInputArchive ia(iss);
            ia(msg);
        } catch (const std::exception& e) {
            std::cerr << "Cereal archive exception: " << e.what() << std::endl;
        }
        return msg;
    }
};


struct BackendMapPointMsg{
    friend class cereal::access;  

    template<class Archive>
    void serialize(Archive & ar){
        ar(id);
        ar(landmark);
        ar(observations);
    }
    size_t id;
    vector<3> landmark;
    std::vector<std::pair<size_t, size_t>> observations;

};

struct BackendKeyFrameMsg{
    friend class cereal::access;  

    template<class Archive>
    void serialize(Archive & ar){
        ar(id);
        ar(pose);
    }

    size_t id;
    Pose pose;
};

struct LocalMapMsg{
    friend class cereal::access;  

    template<class Archive>
    void serialize(Archive & ar){
        ar(mappoints);
        ar(keyframes);
    }

    std::string name;
    std::vector<BackendMapPointMsg> mappoints;
    std::vector<BackendKeyFrameMsg> keyframes;

    std::string serialized(std::string name=""){
        if(!name.empty()) this->name = name;
        std::ostringstream oss;  
        {
            cereal::BinaryOutputArchive oa(oss);  
            oa(*this);
        }
        return oss.str();
    }

    static LocalMapMsg deserialized(const std::string& data){
        LocalMapMsg msg;
        std::istringstream iss(data);
        try {
            cereal::BinaryInputArchive ia(iss);
            ia(msg);
        } catch (const std::exception& e) {
            std::cerr << "Cereal archive exception: " << e.what() << std::endl;
        }
        return msg;
    }
};

}

#endif
