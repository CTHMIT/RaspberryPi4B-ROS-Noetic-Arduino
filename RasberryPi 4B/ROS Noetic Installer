#!/bin/bash
echo "[------------------- ROS installer Start -------------------]"
echo "[------------------- Set Swap -------------------]"

cd /var
sudo swapoff /var/swap
sudo dd if=/dev/zero of=swap bs=1M count=4096 status=progress
sudo mkswap /var/swap
sudo swapon /var/swap


echo "[------------------- ROS add repository -------------------]"
sudo apt-get install dirmngr
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu buster main" > /etc/apt/sources.list.d/ros-noetic.list'


echo "[------------------- ROS keys -------------------]"
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654


echo "[Update the package lists]"
sudo apt update -y
sudo apt upgrade -y

echo "[------------------- Environment setup and rosinstall -------------------]"
source /opt/ros/noetic/setup.sh

# echo "[------------------- Environment all the required dependencies manually built -------------------]"
# mkdir -p ~/ros_catkin_ws/external_src
# cd ~/ros_catkin_ws/external_src
# wget http://sourceforge.net/projects/assimp/files/assimp-3.1/assimp-3.1.1_no_test_models.zip/download -O assimp-3.1.1_no_test_models.zip
# unzip assimp-3.1.1_no_test_models.zip -y
# cd assimp-3.1.1
# cmake .
# make
# sudo make install

sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
pip install cmake --upgrade


echo "[------------------- Initialize and Update -------------------]"
sudo rosdep init && rosdep update
sudo rosdep fix-permissions
rosdep update

echo "[------------------- Catkin workspace -------------------]"
sudo mkdir -p /home/pi/ros_catkin_ws
cd /home/pi/ros_catkin_ws
rosinstall_generator ros_comm --rosdistro noetic --deps --wet-only --tar > noetic-ros_comm-wet.rosinstall

echo "[------------------- Install ROS and Update -------------------]"
sudo apt-get install -y python3-catkin-pkg
wstool init src noetic-ros_comm-wet.rosinstall
wstool update -t src
rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r --os=debian:buster

sudo apt-get upgrade
echo "[------------------- Set the ROS evironment -------------------]"

cd ~/ros_catkin_ws
sudo src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j1 -DPYTHON_EXECUTABLE=/usr/bin/python3

source /opt/ros/noetic/setup.bash

echo "[------------------- Build ROS and Update -------------------]"
sudo chown $USER: -R /home/pi/ros_catkin_ws/
catkin_make

echo "[------------------- ROS installer Complete -------------------]"
exit 0
