cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-ios-visualizer LANGUAGES CXX Swift)

set(CODE_SIGN_IDENTITY "iPhone Developer")      # /usr/bin/env xcrun security find-identity -v -p codesigning
set(DEVELOPMENT_TEAM "${IOS_DEVELOPMENT_TEAM}") # XCode: Build Settings > Signing > Development Team

set(TARGETED_DEVICE_FAMILY "1") # 1: iphone; 2: ipad; 1,2: both

# Generate .xcodeproject for xrslam-ios-visualizer
set(EXECUTABLE_NAME xrslam-ios-visualizer)
set(BUNDLE_NAME "XRSLAM")
set(BUNDLE_IDENTIFIER "${APP_IDENTIFIER_PREFIX}.visualizer")
set(BUNDLE_VERSION ${PROJECT_VERSION})
set(LAUNCH_STORYBOARD LaunchScreen)
set(MAIN_STORYBOARD Main)
set(SWIFT_OBJC_BRIDGING_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/src/SwiftBridge.h)

set(INFO_PLIST_TEMPLATE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Info.plist.in)
include_directories(${CMAKE_SOURCE_DIR}/include/)
set(XRSLAM_IOS_VISUALIZER_SOURCES
    ${CMAKE_SOURCE_DIR}/include/XRSLAM.h
    ${CMAKE_SOURCE_DIR}/xrslam-localization/include/XRGlobalLocalizer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/XRSLAM_iOS.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/XRSLAM_iOS.mm
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Camera.swift
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Motion.swift
    ${CMAKE_CURRENT_SOURCE_DIR}/src/NodeUtils.swift
    ${CMAKE_CURRENT_SOURCE_DIR}/src/XRSLAMer.swift
    ${CMAKE_CURRENT_SOURCE_DIR}/src/UIDevice+Model.swift
    ${CMAKE_CURRENT_SOURCE_DIR}/src/VideoToolbox+extensions.swift
    ${CMAKE_CURRENT_SOURCE_DIR}/src/AppDelegate.swift
    ${CMAKE_CURRENT_SOURCE_DIR}/src/AppUtility.swift
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ViewController.swift
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Toast.swift
)
set(MACOSX_BUNDLE_ICON_FILE icon.png)

set(XRSLAM_IOS_VISUALIZER_STORYBOARDS
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/${LAUNCH_STORYBOARD}.storyboard
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/${MAIN_STORYBOARD}.storyboard
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/icon.png
)

set(XRSLAM_IOS_VISUALIZER_CONFIGS
    ${CMAKE_CURRENT_SOURCE_DIR}/configs)

set(XRSLAM_IOS_VISUALIZER_SCN
    ${CMAKE_CURRENT_SOURCE_DIR}/scn)

set(XRSLAM_IOS_VISUALIZER_RESOURCES
    ${XRSLAM_IOS_VISUALIZER_STORYBOARDS}
    ${XRSLAM_IOS_VISUALIZER_CONFIGS}
    ${XRSLAM_IOS_VISUALIZER_SCN}
)

add_executable(${EXECUTABLE_NAME}
    MACOSX_BUNDLE
    ${SWIFT_OBJC_BRIDGING_HEADER}
    ${XRSLAM_IOS_VISUALIZER_SOURCES}
    ${XRSLAM_IOS_VISUALIZER_STORYBOARDS}
    ${XRSLAM_IOS_VISUALIZER_CONFIGS}
    ${XRSLAM_IOS_VISUALIZER_SCN}
)

target_link_libraries(${EXECUTABLE_NAME}
    PRIVATE
        xrslam
        xrslam-localization
)

set_target_properties(${EXECUTABLE_NAME} PROPERTIES
    MACOSX_BUNDLE_INFO_PLIST ${INFO_PLIST_TEMPLATE}
    RESOURCE "${XRSLAM_IOS_VISUALIZER_RESOURCES}"
)

set_xcode_property(${EXECUTABLE_NAME} CODE_SIGN_IDENTITY ${CODE_SIGN_IDENTITY} "All")
set_xcode_property(${EXECUTABLE_NAME} DEVELOPMENT_TEAM ${DEVELOPMENT_TEAM} "All")
set_xcode_property(${EXECUTABLE_NAME} TARGETED_DEVICE_FAMILY ${TARGETED_DEVICE_FAMILY} "All")
set_xcode_property(${EXECUTABLE_NAME} IPHONEOS_DEPLOYMENT_TARGET ${IOS_DEPLOYMENT_TARGET} "All")
set_xcode_property(${EXECUTABLE_NAME} SWIFT_OBJC_BRIDGING_HEADER ${SWIFT_OBJC_BRIDGING_HEADER} "All")
set_xcode_property(${EXECUTABLE_NAME} SWIFT_VERSION "5.0" "All")
set_xcode_property(${EXECUTABLE_NAME} DEBUG_INFORMATION_FORMAT "dwarf-with-dsym" "All")
set_xcode_property(${EXECUTABLE_NAME} LD_RUNPATH_SEARCH_PATHS "@executable_path/Frameworks" "All")
if(ENABLE_BITCODE)
    set_xcode_property(${EXECUTABLE_NAME} ENABLE_BITCODE "YES" "All")
else()
    set_xcode_property(${EXECUTABLE_NAME} ENABLE_BITCODE "NO" "All")
endif()
if(ENABLE_ARC)
    set_xcode_property(${EXECUTABLE_NAME} CLANG_ENABLE_OBJC_ARC "YES" "All")
else()
    set_xcode_property(${EXECUTABLE_NAME} CLANG_ENABLE_OBJC_ARC "NO" "All")
endif()
