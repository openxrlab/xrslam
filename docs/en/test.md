# Running Tests

The xrslam-test provides unit test samples for XRSLAM. In the future, we will provide more test samples.

## Data Preparation

Download data from the file server, and extract files to `$PROJECT/xrslam-test/data`. Here, we provide two images (from V1_01) to test feature extraction and tracking.

## Build

```bash
cd xrslam/
cmake -B build  -D XRSLAM_TEST=ON && cmake --build build -j8
```

## Run

```bash
./build/bin/test_version
./build/bin/test_se3_cost_function
./build/bin/test_feature_track
```
