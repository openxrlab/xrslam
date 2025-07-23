<br/>

<div align="center">
    <img src="resources/XRSLAM.png" width="600"/>
</div>
<br/>

<div align="left">
<div align="left">


[![Documentation](https://readthedocs.org/projects/xrslam/badge/?version=latest)](https://xrslam.readthedocs.io/en/latest/)[![actions](https://github.com/openxrlab/xrslam/workflows/build/badge.svg)](https://github.com/openxrlab/xrslam/actions)[![LICENSE](https://img.shields.io/github/license/openxrlab/xrslam)](https://github.com/openxrlab/xrslam/blob/main/LICENSE)

</div>

## Introduction

OpenXRLab Visual-inertial SLAM Toolbox and Benchmark. It is a part of the OpenXRLab project.

https://private-user-images.githubusercontent.com/44204704/452296299-89c6e1d5-7856-40e3-a247-78ee58c18922.mp4?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NDkyMTY1NjYsIm5iZiI6MTc0OTIxNjI2NiwicGF0aCI6Ii80NDIwNDcwNC80NTIyOTYyOTktODljNmUxZDUtNzg1Ni00MGUzLWEyNDctNzhlZTU4YzE4OTIyLm1wND9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNTA2MDYlMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjUwNjA2VDEzMjQyNlomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTE4MzU5MWQxY2M2NmExYWFkM2YwOTNkNzA5ZmUxM2Q0NDQ5OWNmZTk5NzAyNDdhOGNkZTJjN2M0OTk2NzBlMDMmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.pAt9h0lc-Xxc0um3iov_xI12jrexF5FozZqnySVRYQo

https://user-images.githubusercontent.com/44204704/187863786-efbc3804-a4d8-4727-a7e2-ea45744330e4.mp4

### **Major Features**

* Robust and Lightweight optimization-based Visual Inertial Odometry
* Both desktop and mobile platforms are supported
* An interactive and real-time [AR application](docs/en/tutorials/app_intro.md) on iPhone
* Visual localization module for running [XRARDemo](https://user-images.githubusercontent.com/44204704/187864126-e9cd7a43-a773-487d-ad01-4cc2988f3b5a.mp4) on prebuilt scenarios
* Compared to other state-of-the-art systems, XRSLAM achieves competitive accuracy

## Installation

We provide detailed [installation tutorial](./docs/en/installation.md) for XRSLAM, users can install from scratch or use provided [Dockerfile](./Dockerfile).

## Getting Started

Please refer to [quick start](docs/en/get_started.md) for the basic usage of XRSLAM.

## License

The license of our codebase is [Apache-2.0](LICENSE). Note that this license only applies to code in our library, the dependencies of which are separate and individually licensed. We would like to pay tribute to open-source implementations to which we rely on. Please be aware that using the content of dependencies may affect the license of our codebase. Some supported methods may carry [additional licenses](docs/en/additional_licenses.md).

## FAQ

Please refer to [FAQ](./docs/en/faq.md) for frequently asked questions.

## Citation

If you use this toolbox or benchmark in your research, please cite this project.

```bibtex
@misc{xrslam,
    title={OpenXRLab Visual-inertial SLAM Toolbox and Benchmark},
    author={XRSLAM Contributors},
    howpublished = {\url{https://github.com/openxrlab/xrslam}},
    year={2022}
}
```
Please refer to [[Porject Page](https://panxkun.github.io/RD-VIO-page/)] for more detail. And if you use the Robust Visual-Inertial Odometry in your research, please cite:
```bibtex
@article{li2024rd,
  title={RD-VIO: Robust visual-inertial odometry for mobile augmented reality in dynamic environments},
  author={Li, Jinyu and Pan, Xiaokun and Huang, Gan and Zhang, Ziyang and Wang, Nan and Bao, Hujun and Zhang, Guofeng},
  journal={IEEE transactions on visualization and computer graphics},
  volume={30},
  number={10},
  pages={6941--6955},
  year={2024},
  publisher={IEEE}
}
```
## Contributing

We appreciate all contributions to improve XRSLAM.
Please refer to [CONTRIBUTING.md](.github/CONTRIBUTING.md) for the contributing guideline.

## Acknowledgement

XRSLAM is an open source project that is contributed by researchers and
engineers from both the academia and the industry.
We appreciate all the contributors who implement their methods or add new features,
as well as users who give valuable feedbacks.
We wish that the toolbox and benchmark could serve the growing research community
by providing a flexible toolkit to reimplement existing methods and develop their
own new models.

## Projects in OpenXRLab

- [XRPrimer](https://github.com/openxrlab/xrprimer): OpenXRLab foundational library for XR-related algorithms.
- [XRSLAM](https://github.com/openxrlab/xrslam): OpenXRLab Visual-inertial SLAM Toolbox and Benchmark.
- [XRSfM](https://github.com/openxrlab/xrsfm): OpenXRLab Structure-from-Motion Toolbox and Benchmark.
- [XRLocalization](https://github.com/openxrlab/xrlocalization): OpenXRLab Visual Localization Toolbox and Server.
- [XRMoCap](https://github.com/openxrlab/xrmocap): OpenXRLab Multi-view Motion Capture Toolbox and Benchmark.
- [XRMoGen](https://github.com/openxrlab/xrmogen): OpenXRLab Human Motion Generation Toolbox and Benchmark.
- [XRNeRF](https://github.com/openxrlab/xrnerf): OpenXRLab Neural Radiance Field (NeRF) Toolbox and Benchmark.
