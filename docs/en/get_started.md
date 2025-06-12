# Getting Started

This page provides basic tutorials about the usage of XRSLAM.

## Data Preparation

Please refer to [data_preparation.md](./dataset_preparation.md) for data preparation.

## Installation

### Requirements

* C++17
* GCC9/Clang13
* CMake 3.15+

### Build and Run

- In XRSLAM, run `cmake -B build && cmake --build build -j8` to generate the project using cmake.
- Start the XRSLAM pc palyer  with command  `./build/xrslam-pc/player/xrslam-pc-player -sc configs/euroc_slam.yaml -dc configs/euroc_sensor.yaml --tum trajectory.tum euroc:///data/EuRoC/MH_01_easy/mav0`

#### iOS

- In XRSLAM, run `./build-ios.sh` to generate the XCode project using cmake.
- The target `xrslam-ios-visulaizer` is what you need to download to the iPhone, and an APP named `XRSLAM` will start automatically.

#### ROS
After the project is compiled successfully, you could use this shared library **lib/libxrslam.so** and **include/XRSLAM.h** to build XRSLAM on ROS.

For more information on installation, please refer to [installation.md](./installation.md).

## Evaluation

Please refer to [euroc_evaluation.md](./tutorials/euroc_evaluation.md) for evaluation.

## API introduction

The project folder structure  is as follows.

```
xrslam
├── xrslam
├── xrslam-extra
├── xrslam-interface
├── xrslam-localization
├── xrslam-pc
├── xrslam-ios
├── xrslam-ros
...
```

+ xrslam : slam core functions are defined
+ xrslam-extra :  yaml_config and opencv image reading functions are defined
+ xrslam-interface : API functions
+ xrslam-localization : sfm-based relocalization functions are defined
+ xrslam-pc : contain examples for PC
+ xrslam-ios : contain examples for iOS
+ xrslam-ros : contain examples for ROS

#### Linux/Mac

After the project is compiled successfully,  **lib/libxrslam.so**  will be generated. Using this shared library and **include/XRSLAM.h** , you can build your own examples.

In xrslam-pc, we provide a  example on PC. You can refer to **xrslam-pc/player/src/main.cpp**.

In addition to the slam function, this example also includes dataset reading (Euroc/TUM), trajectory file saving, and visualization functions.

In order to understand the API usage more quickly, you can turn off the visualization and read the dataset in Euroc format. This mode only depends on the **lib/libxrslam.so** and **include/XRSLAM.h**, and does not contain additional auxiliary functions.

#### iOS

After the project is compiled successfully,  **lib/iOS/Release/libxrslam.a**  will be generated. Using this static library and **include/XRSLAM.h** , you can build your own examples.

In xrslam-ios, we provide a example on iOS. You can refer to **xrslam-ios/visualizer/src/XRSLAM_iOS.mm**.

In addition to the slam function, this example also includes the creation of virtual objects and the sfm-based localization. The API of sfm-based localization can refer to **xrslam-localization/include/XRGlobalLocalizer.h**.

Please refer to [API doc](https://xrslam.readthedocs.io/en/latest/cpp_api/index.html) for XRSLAM's API documentation.

## More information

* [AR Demo](./tutorials/app_intro.md)
* [Benchmark](./benchmark.md)
* [Changelog](./changelog.md)
* [Configuration Parse](./config_parse.md)
* [FAQ](./faq.md)
