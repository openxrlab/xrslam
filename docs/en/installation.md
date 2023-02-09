# Installation Tutorial for XRSLAM

## 1. Prerequisites

### Requirements

* C++17
* GCC9/Clang13
* CMake 3.15+
* [XRPrimer](https://github.com/openxrlab/xrprimer)

### Build XRPrimer

- XRPrimer is a fundational library for XR-related algorithms provided by our team. First clone the project and check out to a specified version.


  ```bash
  git clone https://github.com/openxrlab/xrprimer.git
  cd xrprimer
  git checkout xrslam-opencv3.4.7

  as below:

  xrprimer
  ├──
  ...
  xrslam
  ├── xrslam
  ├── xrslam-pc
  ├── xrslam-ios
  ...
  ```


- Then configure some common dependencies.

  ```bash
  cmake -S. -Bbuild -DBUILD_EXTERNAL=ON -DCMAKE_BUILD_TYPE=Release -DENABLE_PRECOMPILED_HEADERS=OFF && cmake --build build --target install -j8
  ```

## 2. Build XRSLAM

<details> <summary>Build XRSLAM on Linux/Mac</summary>

- Generate the project using cmake.

  ```bash
  cd /path/to/xrslam
  cmake -B build && cmake --build build -j8
  ```

- Start the XRSLAM pc player with the command

  ```bash
  ./build/xrslam-pc/player/xrslam-pc-player -c configs/euroc.yaml --tum trajectory.tum euroc:///data/EuRoC/MH_01_easy/mav0
  ```

  + Click the first button "Stopped" of the player to automatically execute the program on the whole sequence.
  + Long press the second button "Forward" to run the program continued until the mouse button is released.
  + Click the last button "Step" to run the program by inputting a single image of the sequence.
  + Click the left mouse button to rotate the viewing angle, and slide the mouse wheel to scale the viewing size.

</details>

<details> <summary>Build XRSLAM for iOS</summary>

- In XRPrimer, run `./build-ios.sh` to configure some common dependencies.
- In XRSLAM, run `./build-ios.sh` to generate the XCode project using cmake.
  + If it is the first time to run its iOS version, `build-ios.conf` will be automatically generated, you need to assign the `IOS_DEVELOPMENT_TEAM` with your own development id, then run `./build-ios.sh` again to generate the xcode project.
  + It will cost about several minutes (Apple M1 Pro). Then the XCode project `xrslam-superbuild` will be automatically open. if not, you can also open this Xcode project manually by the path `project_path/build/iOS/xrslam-superbuild.xcodeproj`
- The target `xrslam-ios-visulaizer` is what you need to download to the iPhone, and an APP named `XRSLAM` will start automatically.
  + Be sure that your iPhone is supported by checking the [supported devices list](./supported_devices.md)
  + If the project failed to build in Xcode, try to clean the build folder using `cmd+shift+k`

</details>

## 3. Docker Support

### Build an Image

We provide a [Dockerfile](../../Dockerfile) to build an image.

```bash
docker build -t xrslam .
```

### Create a Container

Create a container with command:

```bash
docker run -it xrslam /bin/bash
```

### Copy Dataset into Container

```bash
# b1e0d3f809f6 is container id, using 'docker ps -a' to find id
docker cp data/EuRoC/MH_01_easy b1e0d3f809f6:MH_01_easy
```

### Run in Container without Visualization

```
cd xrslam
`./build/xrslam-pc/player/xrslam-pc-player -H -c configs/euroc.yaml --tum trajectory.tum euroc:///MH_01_easy/mav0`
```
