# Humanoid Navigation

## ROS Packages for Humanoid Navigation
|Version|Kinetic + Ubuntu Xenial|Melodic + Ubuntu Bionic|
|:---:|:---:|:---:|
|[![GitHub version](https://badge.fury.io/gh/ROBOTIS-GIT%2Fhumanoid_navigation.svg)](https://badge.fury.io/gh/ROBOTIS-GIT%2Fhumanoid_navigation)|[![Build Status](https://travis-ci.org/ROBOTIS-GIT/humanoid_navigation.svg?branch=kinetic-devel)](https://travis-ci.org/ROBOTIS-GIT/humanoid_navigation)|-|

## Wiki for humanoid_navigation Packages
- http://wiki.ros.org/humanoid_navigation (metapackage)
- http://wiki.ros.org/footstep_planner
- http://wiki.ros.org/gridmap_2d
- http://wiki.ros.org/humanoid_localization
- http://wiki.ros.org/humanoid_planner_2d

## Documents related to humanoid_navigation Packages
- [humanoid_navigation](http://wiki.ros.org/humanoid_navigation): ROS metapackages with footstep planning and localization for humanoid / biped robots. This metapackge contains subpackages like [footstep_planner](http://wiki.ros.org/footstep_planner), [gridmap_2d](http://wiki.ros.org/gridmap_2d), [humanoid_localization](http://wiki.ros.org/humanoid_localization), [humanoid_planner_2d](http://wiki.ros.org/humanoid_planner_2d).
  - Author: Armin Hornung, Johannes Garimort, Stefan Osswald, Daniel Maier
  - License: GPLv3, BSD
- [footstep_planner](http://wiki.ros.org/footstep_planner): This package a footstep planner for humanoid / biped robots. The planner builds on SBPL and has anytime as well as dynamic replanning capabilities. The supported planners are: ARA*, AD*, R*.
  - Author: Johannes Garimort, Armin Hornung
  - License: GPLv3
- [gridmap_2d](http://wiki.ros.org/gridmap_2d): This package is a simple 2D grid map structure, based on OpenCV's 'cv::Mat'.
  - Author: Armin Hornung
  - License: BSD
- [humanoid_localization](http://wiki.ros.org/humanoid_localization): 6D localization for humanoid robots based on depth data (laser, point clouds). Two observation models are currently available based on OctoMap as 3D map: Ray casting and an end point model (lookup in the distance map).
  - Author: Armin Hornung, Stefan Osswald, Daniel Maier
  - License: GPLv3
- [humanoid_planner_2d](http://wiki.ros.org/humanoid_planner_2d): Thi package provides a simple 2D path planner as wrapper around SBPL (ARA*, AD*, R*).
  - Author: Armin Hornung
  - License: BSD
- Papers related to these packages:
  ```
  "Humanoid robot localization in complex indoor environments",
  by A. Hornung, K. M. Wurm and M. Bennewitz,
  2010 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2010, pp. 1690-1695.
  doi: 10.1109/IROS.2010.5649751
  ```
  ```
  "Humanoid navigation with dynamic footstep plans",
  by J. Garimort, A. Hornung and M. Bennewitz,
  2011 IEEE International Conference on Robotics and Automation (ICRA), 2011, pp. 3982-3987.
  doi: 10.1109/ICRA.2011.5979656
  ```
  ```
  "Anytime search-based footstep planning with suboptimality bounds", 
  by A. Hornung, A. Dornbush, M. Likhachev and M. Bennewitz,
  2012 12th IEEE-RAS International Conference on Humanoid Robots (Humanoids 2012), 2012, pp. 674-679.
  doi: 10.1109/HUMANOIDS.2012.6651592
  ```

---

## Notice for Original Source Code and author, maintainer

This packages are a modified version by forking the following [humanoid_navigation](https://github.com/ahornung/humanoid_navigation) package by ROBOTIS.
Please refer to the following links for original information.

- Original Source Code by Armin Hornung (Electric ~ Hydro Version)
  - Repository: https://github.com/ahornung/humanoid_navigation 
- Source code for maintenance on ROS Indigo Version (by Pramuditha Aravinda)
  - Repository: https://github.com/AravindaDP/humanoid_navigation
  - Issue related: https://github.com/ahornung/humanoid_navigation/issues/14
- Source code for maintenance on ROS Kinetic Version (by Pyo)
  - Repository: https://github.com/ROBOTIS-GIT/humanoid_navigation
  - Issue related: https://github.com/AravindaDP/humanoid_navigation/issues/5


SBPL install

        SBPL uses git as its version control system. From the directory where
            you want the SBPL source to reside, clone the latest source from
            https://github.com/sbpl/sbpl:

            git clone https://github.com/sbpl/sbpl.git

            In the source directory, build the SBPL library using standard
            CMake build conventions:

            mkdir build
            cd build
            cmake ..
            make
  1.2 Install SBPL

            Install the built library and headers onto your local system
            (usually into /usr/local):

            sudo make install    
            
1000  mkdir -p ~/catkin_ws/src
 1001  echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
 1002  source ~/.bashrc
 1003  sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential git
 1004  rosrun turtlesim turtlesim_node
 1005  cd catkin_ws/src/
 1006  git clone https://github.com/xmojiao/humanoid_navigation.git
 1007  ls
 1008  cd ../
 1009  catkin_make
 1010  cd src/
 1011  git clone git@github.com:sbpl/sbpl.git
 1012  mkdir build
 1013  cd build
 1014  cmake ..
 1015  make
 1016  cd ../
 1017  make
 1018  cd sbpl/
 1019  ls
 1020  cd ../
 1021  ls
 1022  rm build/
 1023  rm -rf build/
 1024  cd sbpl/
 1025  ls
 1026  mkdir build
 1027  cd build
 1028  cmake ..
 1029  make
 1030  cd ../../
 1031  cd ../
 1032  catkin_make
 1033  cd src/sbpl/build/
 1034  sudo make install
 1035  cd ../../../
 1036  sudo make install
 1037  catkin_make
 1038  sudo apt-get install ros-kinetic-humanoid-nav-msgs
 1039  catkin_make
 1040  sudo apt-get install ros-kinetic-map-sever
 1041  sudo apt-get install ros-kinetic-navigation-msgs
 1042  cd src/
 1043  git clone https://github.com/ros-planning/navigation_msgs.git
 1044  cd ../
 1045  catkin_make
 1046  cd src/
 1047  ls
 1048  mv humanoid_navigation/ ../../
 1049  cd ../
 1050  catkin_make
 1051  pip install em
 1052  catkin_make
 1053  cd src/
 1054  mv sbpl ../../
 1055  cd ..
 1056  catkin_make
 1057  cd srcatkin_make
 1058  cd src/
 1059  ls
 1060  mv navigation_msgs/ ../../
 1061  cd ../
 1062  catkin_make
 1063  vim ~/.bashrc
 1064  source ~/.bashrc
 1065  mv ../sbpl ./src
 1066  catkin_make
 1067  ls src
 1068  mv ../navigation_msgs/ ./src
 1069  catkin_make
 1070  mv ../humanoid_navigation/ ./src
 1071  catkin_make
 1072  mv ./src/humanoid_navigation/ ../../
 1073  mv ./src/humanoid_navigation/ ../
 1074  cd src/
 1075  git clone git@github.com:ROBOTIS-GIT/humanoid_navigation.git
 1076  cd ../
 1077  catkin_make
 1078  sudo apt-get install ros-kinetic-map-server
 1079  mv ./src/humanoid_navigation/ ../Documents/
 1080  mv ../humanoid_navigation/ ./src
 1081  catkin_make
 1082  sudo apt-get install ros-kinetic-octomap-msgs
 1083  catkin_make
 1084  sudo apt-get install ros-kinetic-octomap-ros
 1085  catkin_make
 1086  roslaunch footstep_planner footstep_planner_complete.launch
 1087  history
