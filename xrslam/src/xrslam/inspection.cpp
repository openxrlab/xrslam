#include <xrslam/common.h>
#include <xrslam/inspection.h>

namespace xrslam {

struct InspectionSupport::VersionTag {
    int major;
    int minor;
    int patch;
};

InspectionSupport::InspectionSupport(const VersionTag &tag)
    : storage(ITEM_COUNT) {
    storage[RESERVED].first = tag;
}

InspectionSupport::~InspectionSupport() = default;

std::pair<std::any &, std::unique_lock<std::mutex>>
InspectionSupport::get(InspectItem item) {
    auto &si = support().storage[item];
    return {si.first, std::unique_lock(si.second)};
}

InspectionSupport &InspectionSupport::support() {
    static std::unique_ptr<InspectionSupport> s_support =
        std::make_unique<InspectionSupport>(VersionTag{
            XRSLAM_MAJOR_VERSION, XRSLAM_MINOR_VERSION, XRSLAM_PATCH_VERSION});
    return *s_support;
}

} // namespace xrslam
