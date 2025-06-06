#ifndef EIGEN_CEREAL_SERIALIZATION
#define EIGEN_CEREAL_SERIALIZATION

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <cereal/types/vector.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/utility.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <xrslam/xrslam.h>
#include <xrslam/estimation/state.h>

using namespace xrslam;

namespace cereal {

    template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    inline
    typename std::enable_if<traits::is_output_serializable<BinaryData<_Scalar>, Archive>::value, void>::type
    save(Archive& ar, const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix) {
        const std::int32_t rows = static_cast<std::int32_t>(matrix.rows());
        const std::int32_t cols = static_cast<std::int32_t>(matrix.cols());
        ar(rows);
        ar(cols);
        ar(binary_data(matrix.data(), rows * cols * sizeof(_Scalar)));
    }

    template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    inline
    typename std::enable_if<traits::is_input_serializable<BinaryData<_Scalar>, Archive>::value, void>::type
    load(Archive& ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix) {
        std::int32_t rows;
        std::int32_t cols;
        ar(rows);
        ar(cols);

        matrix.resize(rows, cols);

        ar(binary_data(matrix.data(), static_cast<std::size_t>(rows * cols * sizeof(_Scalar))));
    }

    template<class Archive>  
    inline void serialize(Archive & ar, cv::KeyPoint& kp) {  
        ar(kp.pt.x, kp.pt.y, kp.size, kp.angle, kp.response, kp.octave, kp.class_id);  
    }  

    template<class Archive>
    inline void serialize(Archive &ar, cv::Mat& mat)
    {
        int cols, rows, type;
        bool continuous;

        if (Archive::is_saving::value) {
            cols = mat.cols; rows = mat.rows; type = mat.type();
            continuous = mat.isContinuous();
        }

        ar(cols, rows, type, continuous);

        if (Archive::is_loading::value)
            mat.create(rows, cols, type);

        if (continuous) {
            const unsigned int data_size = rows * cols * mat.elemSize();
            ar(cereal::binary_data(mat.ptr(), data_size));
        }
        else {
            const unsigned int row_size = cols*mat.elemSize();
            for (int i = 0; i < rows; i++) {
                ar(cereal::binary_data(mat.ptr(i), row_size));
            }
        }
    }

    template<class Archive>  
    inline void serialize(Archive & ar, Eigen::Quaterniond & q) {  
        ar(q.w(), q.x(), q.y(), q.z());  
    }  

    template<class Archive>  
    inline void serialize(Archive & ar, Pose & pose) {  
        ar(pose.q, pose.p);  
    }  

    template<class Archive>  
    inline void serialize(Archive & ar, ExtrinsicParams & params) {  
        ar(params.q_cs, params.p_cs);  
    }  
}

#endif
