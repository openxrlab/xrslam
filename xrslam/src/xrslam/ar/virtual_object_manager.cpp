#include <ceres/ceres.h>
#include <xrslam/ar/virtual_object_manager.h>
#include <xrslam/common.h>
#include <xrslam/geometry/stereo.h>
#include <xrslam/map/frame.h>
#include <xrslam/map/map.h>
#include <xrslam/map/track.h>

namespace xrslam {

VirtualObjectManager::VirtualObjectManager(Map *map)
    : map(map), keypoint_radius(0.1), track_num(15) {}

VirtualObjectManager::VirtualObjectManager(Map *map, Localizer *localizer)
    : map(map), localizer(localizer), keypoint_radius(0.1), track_num(15) {}

VirtualObjectManager::~VirtualObjectManager() = default;

size_t VirtualObjectManager::create_virtual_object() {
    Frame *frame = map->get_frame(map->frame_num() - 1);
    std::vector<vector<3>> near_landmarks;
    vector<3> group_origin = {0.0, 0.0, 0.0};
    for (size_t i = 0; i < frame->keypoint_num(); ++i) {
        Track *track = frame->get_track(i);
        if (!track || !track->tag(TT_TRIANGULATED))
            continue;
        vector<2> p = frame->get_keypoint(i).hnormalized();
        if (p.x() < -keypoint_radius || p.x() > keypoint_radius ||
            p.y() < -keypoint_radius || p.y() > keypoint_radius)
            continue;
        near_landmarks.push_back(track->get_landmark_point());
        group_origin += near_landmarks.back();
    }

    if (near_landmarks.size() < 5) {
        return -1;
    }

    group_origin /= near_landmarks.size();

    auto camera_pose = frame->get_pose(frame->camera);
    vector<3> central_ray = (camera_pose.p - group_origin).normalized();
    vector<3> up = {0.0, 0.0, 1.0};
    vector<3> right = up.cross(central_ray).normalized();
    vector<3> backward = right.cross(up).normalized();

    matrix<3> R;
    R.col(0) = -up;
    R.col(1) = right;
    R.col(2) = backward;

    auto new_object = std::make_unique<VirtualObject>();
    for (size_t i = 0; i < track_ids.size(); ++i) {
        if (Track *track = map->get_track_by_id(track_ids[i])) {
            new_object->local_landmarks.push_back(
                R.transpose() * (track->get_landmark_point() - group_origin));
        }
    }

    new_object->pose.q = R;
    new_object->pose.p = group_origin;

    if (localizer) {
        Pose object_tmp = localizer->transform(new_object->pose);
        new_object->pose.p = object_tmp.p;
        new_object->pose.q = object_tmp.q;
    }

    id_map[new_object->id()] = new_object.get();
    virtual_objects.emplace_back(std::move(new_object));

    return virtual_objects.back()->id();
}

void VirtualObjectManager::update_virtual_objects() {}

} // namespace xrslam
