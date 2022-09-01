#include <gtest/gtest.h>
#include <xrslam/inspection.h>

using namespace xrslam;

TEST(test_version, get_version) {

    int major = XRSLAM_MAJOR_VERSION;
    int minor = XRSLAM_MINOR_VERSION;
    int patch = XRSLAM_PATCH_VERSION;

    std::cout << "Major: " << major << std::endl;
    std::cout << "Minor: " << minor << std::endl;
    std::cout << "Patch: " << patch << std::endl;
}
