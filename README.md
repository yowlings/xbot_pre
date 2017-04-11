# Overview
Xbot is a double wheeled mobile robot which is suitable for most of common sensors and hardware such as Microsoft's Kinect and Asus' Xtion Pro and RPlidar. Users can easily integrate their own customized hardware and applications to the development platform by using ROS and the related series tutorials. For more details about xbot robot, please visit [http://www.xbot.com](http://www.xbot.com)
![image](https://github.com/yowlings/xbot/blob/master/xbot.png)

# Packages
This package is a much simple version of xbot bringup package which can drive the xbot moving and publish some necessary messages. The xbot driver subscribes the cmd_vel message to know how to move, but it has not enough functions to promote xbot move safely. Just for your reference.

## xbot_driver

## xbot_core
Including the xbot_driver and xbot_ftdi, the second package is for creating the serial port in name xbot. The first package is the original driver of xbot, learn how to use and modify it by yourself.

## xbot_driver
The serial port communication driver in python.

## xbot_msgs
The necessary messages that xbot publishes and subscribes.

## xbot_node
Like kobuki_node

## xbot_safety_controller
Like kobuki_safety_controller

## xbot_bringup
Yes, as the name, launch the xbot_minimal.launch in your ROS.
