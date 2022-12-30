# Device Support Description

XRSLAM allows real-time state estimation on iOS devices. But there is something you need to be aware of:

- Considering the difference in CPU's computing power, we can only guarantee the performance on recent iPhones.
- Due to the intrinsics and extrinsics parameters of sensors such as camera and IMU on different iPhones are various, we have calibrated these parameters in advance and provided configuration files for the App to read.
- If you want to add an iPhone that is not in the supported devices list, the only thing you need to do is to manually add the configuration file, then a new device can be immediately supported.
- It should be noted that the accuracy of parameters will directly affect the performance of SLAM.

### Supported Devices List

- iPhone X
- iPhone XR
- iPhone XS
- iPhone XS Max
- iPhone 11
- iPhone 11 Pro
- iPhone 11 Pro Max
- iPhone 12 Mini
- iPhone 12
- iPhone 12 Pro
- iPhone 12 Pro Max
- iPhone 13 Mini
- iPhone 13
- iPhone 13 Pro
- iPhone 13 Pro Max
- iPhone 14 Pro
- iPhone 14 Pro Max
