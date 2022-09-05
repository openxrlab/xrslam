# Getting Started

This page provides basic tutorials about the usage of XRSLAM.

## Data Preparation

Please refer to [data_preparation.md](./dataset_preparation.md) for data preparation.

## Installation

### Requirements

* C++17
* GCC9/Clang13
* CMake 3.15+
* [XRPrimer](https://github.com/openxrlab/xrprimer)

Clone XRPrimer to keep the same root directory as XRSLAM.

### Build and Run

Firstly, switch XRPrimer to the branch of the specified OpenCV version `git checkout xrslam-opencv3.4.7`

#### Linux/Mac

- In XRPrimer, run `cmake -S. -Bbuild -DBUILD_EXTERNAL=ON -DCMAKE_BUILD_TYPE=Release -DENABLE_PRECOMPILED_HEADERS=OFF && cmake --build build --target install -j8` to configure some common dependencies.
- In XRSLAM, run `cmake -B build && cmake --build build -j8` to generate the project using cmake.
- Start the XRSLAM pc palyer  with command  `./build/xrslam-pc/player/xrslam-pc-player -c configs/euroc.yaml --tum trajectory.tum euroc:///data/EuRoC/MH_01_easy/mav0`

#### iOS

- In XRPrimer, run `./build-ios.sh` to configure some common dependencies.
- In XRSLAM, run `./build-ios.sh` to generate the XCode project using cmake.
- The target `xrslam-ios-visulaizer` is what you need to download to the iPhone, and an APP named `XRSLAM` will start automatically.

For more information on installation, please refer to [installation.md](./installation.md).

## Evaluation

Please refer to [euroc_evaluation.md](./tutorials/euroc_evaluation.md) for evaluation.

## More information

* [AR Demo](./tutorials/app_intro.md)
* [Benchmark](./benchmark.md)
* [Changelog](./changelog.md)
* [Configuration Parse](./config_parse.md)
* [FAQ](./faq.md)
