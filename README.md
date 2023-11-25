# CG3
This is the source code of the ICRA2023 paper [**The Human Gaze Helps Robots Run Bravely and Efficiently in Crowds**](https://ieeexplore.ieee.org/document/10161222), an approach to make the robot run more efficiently by utilizing human gaze. 

[![](https://res.cloudinary.com/marcomontalbano/image/upload/v1668650554/video_to_markdown/images/youtube--q60r3eQVfio-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=q60r3eQVfio "")

## Table of Contents
* [Installation](#1-Installation)
* [Quick Start](#2-Quick-Start)
* [Contributors](#3-Contributors)
* [Acknowledgement](#4-acknowledge)


## 1. Installation
The project has been tested on Ubuntu 18.04 (ROS Melodic). To install the repository, please install some dependence firstly: 
```
$ sudo apt install ros-melodic-navigation
```
Then please install this project and build it: 
```
$ mkdir -p CG3_ws/src
$ cd CG3_ws/src
$ https://github.com/Chris-Arvin/CG3_The-Human-Gaze-Helps-Robots-Run-Bravely-and-Efficiently-in-Crowds.git
$ cd ..
$ rosdep install –from-paths src –ignore-src –rosdistro-melodic -y
$ catkin_make
```
## 2. Quick Start
Please open a terminal to launch the pedestrian simulation: 
```
$ source CG3_ws/devel/setup.bash
$ roslaunch pedsim_simulator pedsim_simulator.launch
```
Open another terminal to launch the navigation simulation: 
```
$ source CG3_ws/devel/setup.bash
$ roslaunch move_base navigation.launch
```
Open another terminal to launch the limit cycle planner: 
```
$ source CG3_ws/devel/setup.bash
$ roslaunch limit_cycle limit_cycle.launch
```
## 3. Contributors
* Qianyi Zhang  arvin.nkzqy@gmail.com
* Zhengxi Hu
* Yinuo Song
* Jiayi Pei
* Jingtai Liu

## 4. Acknowledge
This work is inspired by the previous works of [Prof.Lounis](https://scholar.google.fr/citations?user=LWGCtScAAAAJ&hl=en). Thanks again for his introduction of limit cycle. 
