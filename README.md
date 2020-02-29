Mobile Manipulator Blitz Driver & IO Wrapper
============================================

This is the repository for the driver ROS launch files, packages, and other IO wrapper programs of the mobile manipulator Blitz(working title), which is the custom mobile manipulator belongs to the [SNU Biointelligence Lab](https://bi.snu.ac.kr/)

## 1. Blitz Prerequisites
### 1.1. Hardware & Environments
We only experimented on the below environments. The whole system may not work on different situations.
#### 1.1.1. Robots
* Fetch Robotics : Freight (Ubuntu 18.04, ROS Melodic, works as ROS master)
* Universal Robots : UR5
* Robotiq : 2F-140 Adaptive Gripper
* Intel : RealSense D435
* ASUS : Xtion PRO LIVE

#### 1.1.2. Client Computer
* Ubuntu 14.04, ROS Indigo
* Intel i7-6700 @ 3.40GHz
* NVIDIA TITAN X Pascal, CUDA=8.0, cuDNN=5.0

  [ROS Installation Guide](http://wiki.ros.org/ROS/Installation)

### 1.2. Library Dependencies
#### 1.2.1. Robot Packages Setup
* [ur\_modern\_driver](https://github.com/ros-industrial/ur_modern_driver/tree/kinetic-devel) : We confirmed that kinetic-devel works in the Melodic environment.
* [robotiq\_2finger\_grippers](https://github.com/ros-industrial/ur_modern_driver/tree/kinetic-devel)
* [openni2\_camera](https://github.com/ros-drivers/openni2_camera)
* [realsense-ros](https://github.com/IntelRealSense/realsense-ros)
* [audio\_common](http://wiki.ros.org/audio_common)

#### 1.2.2. Client Computer Setup
* [fetch\_ros](https://github.com/fetchrobotics/fetch_ros)
* [Darknet](https://github.com/pjreddie/darknet) : Blitz uses yolo-v3 configurations.
* [reuleaux](http://wiki.ros.org/reuleaux)
* [openpose](https://github.com/CMU-Perceptual-Computing-Lab/openpose): You should install python wrapper for openpose.
* [opencv==2.4.8](https://opencv.org/opencv-2-4-8/)
* [audio\_common](http://wiki.ros.org/audio_common)
* [gTTS](https://pypi.org/project/gTTS/)
* [google-cloud-speech==0.26.0](https://pypi.org/project/google-cloud-speech/) : You should use your own Google Cloud API Key.
* [pydub](https://github.com/jiaaro/pydub)
## 2. Blitz Installation
1. Put **blitz\_io** folder in the client computer, and **blitz\_setup** in the Freight.   **my\_blitz** should be put in the ROS catkin workspaces of the both client & master.    **blitz\_robot\_moveit\_config** and **blitz\_robot\_ikfast\_ur5\_plugin** should be only put in the catkin workspace in the ROS master.
2. For visualization purpose, change **reuleaux** source file at `(catkin workspace directory)/src/reuleaux/map_creator/src/load_reachability_map.cpp`.    Change *ws.header.frame\_id* from *"base\_link"* to *"arm\_base\_link"*.
3. Catkin make both catkin workspaces.

   You can customize MoveIt! of the Blitz for your own purpose.    
   [MoveIt! Setup](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html)    
   [MoveIt! IKFast Kinematics Setup](http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html)

## 3. Blitz Execution
* Before you execute, you should check darknet weight, Google API Key file, IPs, file paths, etc to your own settings.
1. After turning on every robot, launch bringup file in the ROS Master.
```sh
$ roslaunch ~/blitz_setup/blitz_bringup.launch
```
2. Execute server python scripts in the ROS Master
```sh
$ python ~/blitz_setup/blitz_arm_server.py
$ python ~/blitz_setup/sound_play_server.py
```
3. Launch navigation & RVIZ in the client computer.
```sh
$ roslaunch my_blitz blitz_total.launch
```
4. Now you can use Blitz by importing scripts in **blitz\_io**.
```sh
$ python blitz_navigate_and_pick.py
```

