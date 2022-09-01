#ifndef XRSLAM_VIRTUAL_OBJECT_H
#define XRSLAM_VIRTUAL_OBJECT_H
#include <xrslam/common.h>
#include <xrslam/estimation/state.h>
#include <xrslam/localizer/localizer.h>
#include <xrslam/utility/identifiable.h>

namespace xrslam {

class Map;
class Track;
class Config;
class Localizer;

enum class ObjectFlag { OF_ISOLATED = 0, FLAG_NUM };

class VirtualObjectManager {
    class VirtualObject : public Flagged<ObjectFlag>,
                          public Identifiable<VirtualObjectManager> {
      public:
        VirtualObject() = default;
        ~VirtualObject() = default;
        OutputObject get_pose() const {
            return {pose.q, pose.p, flag(ObjectFlag::OF_ISOLATED)};
        }
        std::vector<vector<3>> local_landmarks;
        PoseState pose;
    };

  public:
    VirtualObjectManager(Map *map);
    VirtualObjectManager(Map *map, Localizer *localizer);
    ~VirtualObjectManager();

    size_t create_virtual_object();
    OutputObject get_virtual_object_pose_by_id(size_t id) const {
        if (id_map.count(id) > 0) {
            return id_map.at(id)->get_pose();
        } else {
            return {{0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0}, 1};
        }
    }
    OutputObject get_virtual_object_pose(size_t index) const {
        return virtual_objects[index]->get_pose();
    }
    size_t virtual_object_num() const { return virtual_objects.size(); }

    void update_virtual_objects();

  private:
    Map *map;
    std::vector<size_t> track_ids;
    double keypoint_radius;
    size_t track_num;
    std::vector<std::unique_ptr<VirtualObject>> virtual_objects;
    std::map<size_t, VirtualObject *> id_map;
    Localizer *localizer = nullptr;
};

} // namespace xrslam

#endif
