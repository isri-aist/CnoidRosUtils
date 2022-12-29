# [CnoidRosUtils](https://github.com/isri-aist/CnoidRosUtils)
ROS-based Choreonoid utilities

[![CI](https://github.com/isri-aist/CnoidRosUtils/actions/workflows/ci.yaml/badge.svg)](https://github.com/isri-aist/CnoidRosUtils/actions/workflows/ci.yaml)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](https://isri-aist.github.io/CnoidRosUtils/)
[![LICENSE](https://img.shields.io/github/license/isri-aist/CnoidRosUtils)](https://github.com/isri-aist/CnoidRosUtils/blob/master/LICENSE)

## Install

### Requirements
- Compiler supporting C++17
- Tested with `Ubuntu 20.04 / ROS Noetic` and `Ubuntu 18.04 / ROS Melodic`

### Dependencies
This package depends on
- [Choreonoid](https://github.com/choreonoid/choreonoid)

## Plugin list
#### [CnoidRosUtils::ClockPublisherItem](https://isri-aist.github.io/CnoidRosUtils/doxygen/classCnoidRosUtils_1_1ClockPublisherItem.html#details)
Plugin item to publish clock topic.

#### [CnoidRosUtils::PosePublisherItem](https://isri-aist.github.io/CnoidRosUtils/doxygen/classCnoidRosUtils_1_1PosePublisherItem.html#details)
Plugin item to publish topics and broadcast TF of pose and velocity of the model.
