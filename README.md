# Morla

## Inspiration
Morla is an autonomous differential drive robot inspired from the popular robotics platform, turtlebot. While turtlebot being a fairly robust robotic platform, not everyone can afford it, hence morla is designed to be a plug, play & tweak platform to run most of the popular algorithms related to robot control, perception and navigation, while being cheaper than it's original inspiration.

Its main processing unit is a Raspberry Pi 4 B running Ubuntu 18.04 and the ROS 1 (ROS Melodic) middleware, while containing variety of sensors ranging from LIDAR to inertial and visual sensors, to serve as a true learning platform. This respository contains ROS driver packages, ROS Control Hardware Interface for the real robot and configurations for simulating Morla.
| Morla Isometric View | Morla Side View | 
|:-------:|:-----------------:|
|  <img src="https://user-images.githubusercontent.com/97186785/167469105-67300ebb-e358-42b8-afb9-97fa182423ff.JPG" width="700"> | <img src="https://user-images.githubusercontent.com/97186785/167471554-6f4d96a5-036d-4fa4-ae50-e7229ca264ae.JPG" width="700">|

| Running teleoperation with rosserial | SLAM using Google Cartographer(Partial) |
|:-------:|:-----------------:|
|<img src="media/morla_teleop.gif" width="750" height = "300"/> | <img src="https://user-images.githubusercontent.com/97186785/170633482-032d4c19-f23a-491f-96ae-c1397b1d0e2e.png" width="650" height = "300"/> |
