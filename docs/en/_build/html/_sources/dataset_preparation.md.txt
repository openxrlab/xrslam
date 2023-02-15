# Data Preparation
[EuRoC Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) contains stereo images, synchronized IMU measurements, and accurate motion and structure ground truth. In XRSLAM, we only use Monocular camera and IMU data. The bottom figures are the representative images of the Machine hall and Vicon room, where the EuRoC dataset was collected.

It is recommended to construct your folder structure like this:

```
data
├── EuRoC
│   ├── MH_01_easy
│           |── mav0
│                |──...
│   └── MH_02_easy
│   └── ...
```

 Take EuRoC as an example, download a sequence (ASL format)  and uncompress it. You can also download the compressed zip file through the link below.

| Dataset              | ASL Dataset Formatcol                                                                                                  | Comment                            |
| -------------------- | ---------------------------------------------------------------------------------------------------------------------- | ---------------------------------- |
| *Machine Hall 01*  | [download](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip)           | Dataset machine hall “easy”      |
| *Machine Hall 02*  | [download](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_02_easy/MH_02_easy.zip)           | Dataset machine hall “easy”      |
| *Machine Hall 03*  | [download](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_03_medium/MH_03_medium.zip)       | Dataset machine hall “medium”    |
| *Machine Hall 04*  | [download](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_04_difficult/MH_04_difficult.zip) | Dataset machine hall “difficult” |
| *Machine Hall 05*  | [download](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_05_difficult/MH_05_difficult.zip) | Dataset machine hall “difficult” |
| *Vicon Room 1 01*  | [download](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.zip)            | Dataset Vicon room 1 “easy”      |
| *Vicon Room 1 02*  | [download](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_02_medium/V1_02_medium.zip)        | Dataset Vicon room 1 “medium”    |
| *Vicon Room 1 03*  | [download](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_03_difficult/V1_03_difficult.zip)  | Dataset Vicon room 1 “difficult” |
| *Vicon Room 2 01*  | [download](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room2/V2_01_easy/V2_01_easy.zip)            | Dataset Vicon room 2 “easy”      |
| *Vicon Room 2 02*  | [download](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room2/V2_02_medium/V2_02_medium.zip)        | Dataset Vicon room 2 “medium”    |
| *Vicon Room 2 03* | [download](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room2/V2_03_difficult/V2_03_difficult.zip)  | Dataset Vicon room 2 “difficult” |


<div align='center'><img src="../images/Machine-hall.png" width="55%" height="100%"><img src="../images/Vicon-room.png" width="38.7%" height="100%"></div>
