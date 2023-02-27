/**
 * @file XRGlobalLocalizer.h
 * @brief XR Global Localizer API
 */

#ifndef _XR_GLOBAL_LOCALIZER_H_
#define _XR_GLOBAL_LOCALIZER_H_
#include <stdint.h>
#include "XRSLAM.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************************
 *                                   XR Global Localizer function interface
 ******************************************************************************************/

/**
 * @brief create Global Localization system with configuration files.
 * @param[in] _config_path  configuration file path.
 * @param[in] device_config_path  device configuration file path, include camera
 */
void XRGlobalLocalizerCreate(const char *_config_path,
                             const char *device_config_path);

/**
 * @brief Set queryFrame flag
 */
void XRGlobalLocalizerQueryFrame();

/**
 * @brief Query Global Localization
 * @param[in] image query frame.
 * @param[in] pose frame pose at SLAM coordinate system.
 */
void XRGlobalLocalizerQueryLocalization(XRSLAMImage *image, XRSLAMPose *pose);

/**
 * @brief Check if the Global Localizer is initialized
 * @return 1 initialized, otherwise 0 uninitialized.
 */
int XRGlobalLocalizerIsInitialized();

/**
 * @brief Enable/Disable Global Localization
 * @param[in] state, 0 Disable, 1 Enable
 */
void XRGlobalLocalizerEnable(int state);

/**
 * @brief transform slam pose to sfm global world pose
 * @param[in] pose current slam pose
 * @return  pose at sfm global world coordinate system
 */
XRSLAMPose XRGlobalLocalizerTransformPose(const XRSLAMPose &pose);

#ifdef __cplusplus
}
#endif

#endif // _XR_GLOBAL_LOCALIZER_H_