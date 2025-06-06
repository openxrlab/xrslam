#ifndef XRSLAM_FRAME_H
#define XRSLAM_FRAME_H

#include <xrslam/common.h>
#include <xrslam/estimation/preintegrator.h>
#include <xrslam/estimation/state.h>
#include <xrslam/utility/identifiable.h>
#include <xrslam/utility/tag.h>

namespace xrslam {

class Config;
class Track;
class Map;
class ReprojectionErrorFactor;
class TinyReprojectionErrorFactor; 

enum FrameTag {
    FT_KEYFRAME = 0,
    FT_NO_TRANSLATION,
    FT_FIX_POSE,
    FT_FIX_MOTION
};

class Frame : public Tagged<FrameTag>, public Identifiable<Frame> {
    friend class Track;
    friend class Map;
    struct construct_by_frame_t;
    Map *map;

  public:
    Frame();
    Frame(const Frame &frame, const construct_by_frame_t &construct);
    virtual ~Frame();

    std::unique_ptr<Frame> clone() const;

    size_t keypoint_num() const { return bearings.size(); }

    void append_keypoint(const vector<3> &keypoint);

    const vector<3> &get_keypoint(size_t keypoint_index) const {
        runtime_assert(abs(bearings[keypoint_index].norm() - 1.0) <= 0.001,
                       "bearing vector is not normalized");
        return bearings[keypoint_index];
    }

    Track *get_track(size_t keypoint_index) const {
        return tracks[keypoint_index];
    }

    Track *get_track(size_t keypoint_index, Map *allocation_map);

    void detect_keypoints(Config *config);
    void track_keypoints(Frame *next_frame, Config *config);

    PoseState get_pose(const ExtrinsicParams &sensor) const;
    void set_pose(const ExtrinsicParams &sensor, const PoseState &pose);

    bool has_map() const { return map != nullptr; }

    std::unique_lock<std::mutex> lock() const;

    matrix<3> K;
    matrix<2> sqrt_inv_cov;
    std::shared_ptr<Image> image;

    PoseState pose;
    MotionState motion;
    ExtrinsicParams camera;
    ExtrinsicParams imu;

    PreIntegrator preintegration;
    PreIntegrator keyframe_preintegration;

    std::vector<std::unique_ptr<Frame>> subframes;
    std::vector<std::unique_ptr<ReprojectionErrorFactor>> reprojection_error_factors;
    std::vector<std::unique_ptr<TinyReprojectionErrorFactor>>  tiny_reprojection_error_factors;


  private:
    std::vector<vector<3>> bearings;
    std::vector<Track *> tracks;
};

} // namespace xrslam

#endif // XRSLAM_FRAME_H
