#!/bin/bash

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full
sudo apt-get install python-rosinstall
sudo apt-get install cmake
sudo apt-get install ros-indigo-ecl-core ros-indigo-ecl-eigen ros-indigo-ecl-utilities ros-indigo-ecl-tools ros-indigo-ecl-geometry ros-indigo-ecl-mobile-robot 
sudo apt-get install ros-indigo-yocs-controllers
sudo apt-get install libftdi-dev python-ftdi ros-indigo-kobuki-ftdi 
sudo apt-get install ros-indigo-ecl-build 

echo "Done."

