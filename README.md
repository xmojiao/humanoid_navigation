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



---


## 1.项目介绍

基于kinetic 版的ros系统，跟随[footstep planner wiki](http://wiki.ros.org/footstep_planner)，从GitHub下载并安装footstep planner(落脚点规划系统)，使其在给定开始和目标位置时，实现在2d map上规划机器人的落脚点序列。 


**原footstep planner github:  https://github.com/ROBOTIS-GIT/humanoid_navigation.git**

**本github(https://github.com/xmojiao/humanoid_navigation)使用贝塞尔曲线限制sbpl的搜索空间，损失路径的cost, 但减少了2/3的规划时间**。****

为了便于算法调试，我们整合footstep和sbpl，去掉ros，实现在win10_x64+visual studio 2017环境下编译调试优化，具体代码：https://github.com/xmojiao/footstep_sbpl_vs2017_cpp/tree/dev-jiao





## 2.项目安装
#### 本项目的附加依赖包
- sbbp
- navigation-msgs
- python empy包

#### 1.创建工作空间
在主目录下创建工作空间 catkin_ws,然后初始化 catkin_init_workspace。
执行完catkin_make时，发现工作空间catkin_ws中有三个目录： build  devel  src
其中，src是我们创建工作空间时创建的目录，另外两个是执行完 catkin_make 后生成的。

```

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```


#### 2. SBPL install
从github下载SBPL源程序，然后编译安装

SBPL uses git as its version control system. From the directory where
you want the SBPL source to reside, clone the latest source from
https://github.com/sbpl/sbpl:

``` git clone https://github.com/sbpl/sbpl.git ```

In the source directory, build the SBPL library using standard
CMake build conventions:
``` 
mkdir build
cd build
cmake ..
make
``` 
Install the built library and headers onto your local system
            (usually into /usr/local):

```sudo make install    ```


#### 3. 安装humanoid_navigation包

```
cd src/
git clone git@github.com:ROBOTIS-GIT/humanoid_navigation.git
cd ../
catkin_make
```


 
#### 4. 安装必要的navigation-msgs包
此处编译时也可能需要其他包，需要什么就使用ros-kinetic-安装。

```
sudo apt-get install ros-kinetic-humanoid-nav-msgs
sudo apt-get install ros-kinetic-map-sever
git clone https://github.com/ros-planning/navigation_msgs.git
../
catkin_make

```
也可能会提示no modeule em,此时使用以下命令安装

```
pip install em
```
#### 5. launch
catkin_make 编译成功，就可以进行roslaunch


```
catkin_make
roslaunch footstep_planner footstep_planner_complete.launch

```
分别使用2D Pose Estimate和2D Nav Goal 给定开始和结束的位置和方向，稍等片刻就可以看到落脚点规划器规划的足迹序列

![image](https://note.youdao.com/yws/api/personal/file/E22D500184A44DC5BB6DF58B0AD05FDA?method=download&shareKey=844ac104e466837217995a5521846315)
