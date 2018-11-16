
# 1. Prepare

## ROS Packages for Humanoid Navigation
|Version|Kinetic + Ubuntu Xenial|Melodic + Ubuntu Bionic|
|:---:|:---:|:---:|
|[![GitHub version](https://badge.fury.io/gh/ROBOTIS-GIT%2Fhumanoid_navigation.svg)](https://badge.fury.io/gh/ROBOTIS-GIT%2Fhumanoid_navigation)|[![Build Status](https://travis-ci.org/ROBOTIS-GIT/humanoid_navigation.svg?branch=kinetic-devel)](https://travis-ci.org/ROBOTIS-GIT/humanoid_navigation)|-|

## Original repositories
-  https://github.com/ROBOTIS-GIT/humanoid_navigation.git 

## Wiki for humanoid_navigation Packages
- http://wiki.ros.org/humanoid_navigation (metapackage)
- http://wiki.ros.org/footstep_planner (import)


-----------------------------------------------------
# 2.Getting Started
## 2.1 create a ROS Workspace

在home目录创建一个ros工作空间，即文件夹Footstep_planner。在src文件夹下初始化工作空间，然后转到上一层文件夹进行catkin_make编译。

```
mkdir -p ~/Footstep_planner/src
cd ~/Footstep_planner/src
catkin_init_workspace
cd ../
catkin_make
```
如图：

![image](https://note.youdao.com/yws/api/personal/file/0E42BF1FCE6C4B38AF91EE4BED15F7B2?method=download&shareKey=acf124840f2f8114fde05d3cb28d7c16)

使用source将对应的工作空间的路径加入环境变量ROS_PACKAGE_PATH中,使用echo判断添加环境变量成功是否。为了每次打开新的终端仍可以直接使用ros包，将source命令加入到~/.bashrc文件中
```
source devel/setup.bash
echo $ROS_PACKAGE_PATH
vim ~/.bashrc
##add $source /opt/ros/kinetic/setup.bash $source ~/Footstep_planner/devel/setup.bash to ~/.bashrc
source ~/.bashrc
```

![image](https://note.youdao.com/yws/api/personal/file/3876183C39114F7E85717E8CD65E649F?method=download&shareKey=31b661f325f9c540318ee1f768384277)

添加路径如下图：

![image](https://note.youdao.com/yws/api/personal/file/CF79DC4D77D74F4EBD1A6EDABC223B17?method=download&shareKey=c9eb06e8910bb097944cce7bb4d61c0f)

## 2.2 clone the package 

克隆本github（humanoid_navigation）到src目录下，并到上一级目录下进行catkin_make.
```
cd ~/Footstep_planner/src
git clone https://github.com/xmojiao/humanoid_navigation.git
cd ../
catkin_make
```
## 2.3 run the project
`roslaunch footstep_planner footstep_planner_complete.launch`

启动落脚点规划的rviz

![image](https://note.youdao.com/yws/api/personal/file/E1512872F170485A9C2FE809809F7066?method=download&shareKey=718f350862b651606fbfaef1dda680a4)
