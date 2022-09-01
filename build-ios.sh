#!/bin/bash
cd "$(dirname "$0")"
if [ -f "build-ios.conf" ]; then
    source build-ios.conf
else
    echo "Modify 'build-ios.conf' before continue."
    cp build-ios.conf.defaults build-ios.conf
    exit 1
fi

##
# Explanation to mandatory parameters:
#   -G Xcode                        <- this is required for building Swift code
#   -D CMAKE_TOOLCHAIN_FILE=...     <- this file defines the iOS toolchain used for iOS build
#   -D IOS_PLATFORM=OS64            <- OpenCV does not support 32 bit architectures
#   -D IOS_ARCH=arm64               <- OpenCV does not support arm64e, try recompile OpenCV if required
#   -D IOS_DEPLOYMENT_TARGET=12.0   <- for some C++17 features
#   -D ENABLE_BITCODE=0             <- ceres-solver does not support bitcode generation
#   -D ENABLE_ARC=1                 <- required for Objective-C code
##
cmake -S . -B build/iOS                                                 \
    -G Xcode                                                            \
    -D CMAKE_TOOLCHAIN_FILE=cmake/Modules/Platform/ios.toolchain.cmake  \
    -D CMAKE_CONFIGURATION_TYPES=Release                                \
    -D IOS_PLATFORM=OS64                                                \
    -D IOS_ARCH=arm64                                                   \
    -D IOS_DEPLOYMENT_TARGET=12.0                                       \
    -D ENABLE_BITCODE=0                                                 \
    -D ENABLE_ARC=1                                                     \
    -D ENABLE_VISIBILITY=0                                              \
    -D APP_IDENTIFIER_PREFIX="${APP_IDENTIFIER_PREFIX}"                 \
    -D IOS_DEVELOPMENT_TEAM="${IOS_DEVELOPMENT_TEAM}"

[ $? -eq 0 ] && cmake --build build/iOS --config Release -- -allowProvisioningUpdates
[ $? -eq 0 ] && open build/iOS/xrslam-superbuild.xcodeproj
