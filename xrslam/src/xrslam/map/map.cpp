#include <xrslam/ar/virtual_object_manager.h>
#include <xrslam/estimation/marginalization_factor.h>
#include <xrslam/map/frame.h>
#include <xrslam/map/map.h>
#include <xrslam/map/track.h>
#include <xrslam/utility/debug.h>

namespace xrslam {

struct Map::construct_by_map_t {};

Map::Map() = default;

Map::~Map() = default;

void Map::clear() {
    frames.clear();
    tracks.clear();
}

void Map::attach_frame(std::unique_ptr<Frame> frame, size_t position) {
    frame->map = this;
    if (position == nil()) {
        frames.emplace_back(std::move(frame));
    } else {
        frames.emplace(frames.begin() + position, std::move(frame));
    }
}

std::unique_ptr<Frame> Map::detach_frame(size_t index) {
    std::unique_ptr<Frame> frame = std::move(frames[index]);
    frames.erase(frames.begin() + index);
    frame->map = nullptr;
    return frame;
}

void Map::untrack_frame(Frame *frame) {
    for (size_t i = 0; i < frame->keypoint_num(); ++i) {
        if (Track *track = frame->get_track(i)) {
            track->remove_keypoint(frame);
        }
    }
}

void Map::erase_frame(size_t index) {
    Frame *frame = frames[index].get();
    untrack_frame(frame);
    detach_frame(index);
}

void Map::marginalize_frame(size_t index) {
    runtime_assert(index == 0, "currently only index == 0 is allowed");
    runtime_assert(marginalization_factor,
                   "marginalization_factor is not initialized yet");
    marginalization_factor->marginalize(index);
    Frame *frame = frames[index].get();
    for (size_t i = 0; i < frame->keypoint_num(); ++i) {
        if (Track *track = frame->get_track(i)) {
            track->remove_keypoint(frame);
        }
    }
    frames.erase(frames.begin() + index);
}

size_t Map::frame_index_by_id(size_t id) const {
    struct FrameID {
        FrameID(const std::unique_ptr<Frame> &frame) : id(frame->id()) {}
        FrameID(size_t id) : id(id) {}
        bool operator<(const FrameID &fi) const { return id < fi.id; }
        size_t id;
    };
    auto it = std::lower_bound(frames.begin(), frames.end(), id,
                               std::less<FrameID>());
    if (it == frames.end())
        return nil();
    if (id < (*it)->id())
        return nil();
    return std::distance(frames.begin(), it);
}

Track *Map::create_track() {
    std::unique_ptr<Track> track =
        std::make_unique<Track>(construct_by_map_t());
    track->map_index = tracks.size();
    track->map = this;
    track_id_map[track->id()] = track.get();
    return tracks.emplace_back(std::move(track)).get();
}

void Map::erase_track(Track *track) {
    while (track->keypoint_num() > 0) {
        track->remove_keypoint(track->keypoint_map().begin()->first, false);
    }
    recycle_track(track);
}

void Map::prune_tracks(const std::function<bool(const Track *)> &condition) {
    std::vector<Track *> tracks_to_prune;
    for (size_t i = 0; i < track_num(); ++i) {
        if (Track *track = get_track(i); condition(track)) {
            tracks_to_prune.push_back(track);
        }
    }
    for (const auto &track : tracks_to_prune) {
        erase_track(track);
    }
}

Track *Map::get_track_by_id(size_t id) const {
    if (track_id_map.count(id)) {
        return track_id_map.at(id);
    } else {
        return nullptr;
    }
}

void Map::recycle_track(Track *track) {
    if (track->map_index != tracks.back()->map_index) {
        tracks[track->map_index].swap(tracks.back());
        tracks[track->map_index]->map_index = track->map_index;
    }
    track_id_map.erase(track->id());
    tracks.pop_back();
}

void Map::create_virtual_object_manager(Localizer *localizer) {
    virtual_object_manager =
        std::make_unique<VirtualObjectManager>(this, localizer);
}

void Map::create_virtual_object_manager() {
    virtual_object_manager = std::make_unique<VirtualObjectManager>(this);
}

size_t Map::create_virtual_object() {
    return virtual_object_manager->create_virtual_object();
}

void Map::update_virtual_objects() {
    virtual_object_manager->update_virtual_objects();
}

OutputObject Map::get_virtual_object_pose_by_id(size_t id) const {
    return virtual_object_manager->get_virtual_object_pose_by_id(id);
}

OutputObject Map::get_virtual_object_pose(size_t index) const {
    return virtual_object_manager->get_virtual_object_pose(index);
}

size_t Map::virtual_object_num() const {
    return virtual_object_manager->virtual_object_num();
}

} // namespace xrslam
