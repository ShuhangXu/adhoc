# JHU Adhoc Project Repo

This project involves using various libraries and tools to work with ROS2, RealSense cameras, OpenCV, audio recording, and Pupil Labs gaze tracking.

## Prerequisites

To run the code successfully, you will need to install the following packages:

1. **rclpy**: The ROS2 client library for Python.
2. **cv_bridge**: A ROS package that converts between ROS image messages and OpenCV images.
3. **pyrealsense2**: The Python wrapper for Intel RealSense SDK.
4. **opencv-python**: The OpenCV package for Python.
5. **numpy**: A package for scientific computing with Python.
6. **pyzmq**: A Python binding for ØMQ (ZeroMQ).
7. **sounddevice**: A Python library for recording and playing audio.
8. **pupil-labs-realtime-api**: The Pupil Labs real-time API for eye-tracking.
9. **wave**: The Python standard library module for reading and writing WAV files.
10. **csv**: The Python standard library module for reading and writing CSV files.

## Installation

You can install the required packages using `pip`. Open your terminal and run the following commands:

```sh
pip install rclpy
pip install cv_bridge
pip install pyrealsense2
pip install opencv-python
pip install numpy
pip install pyzmq
pip install sounddevice
pip install pupil-labs-realtime-api
pip install wave
pip install csv
```

## Installing `cv_bridge`

`cv_bridge` is a ROS2 package that converts between ROS image messages and OpenCV images. It is recommended to install it from the ROS2 package manager instead of `pip`.

1. **Update the package list:**

    ```sh
    sudo apt-get update
    ```

2. **Install `cv_bridge`:**

    Replace `<your_ros2_distro>` with your ROS2 distribution, e.g., `foxy`, `galactic`, or `humble`.

    ```sh
    sudo apt-get install ros-<your_ros2_distro>-cv-bridge
    ```

## Setting Up ROS2

Ensure that you have the required ROS2 setup and the environment sourced correctly for `rclpy` and `cv_bridge` to work.

1. **Source the ROS2 setup file:**

    Replace `<your_ros2_distro>` with your ROS2 distribution.

    ```sh
    source /opt/ros/<your_ros2_distro>/setup.bash
    ```

2. **Create and build a ROS2 workspace:**

    If you don't already have a ROS2 workspace, you can create one:

    ```sh
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    colcon build
    ```

3. **Source the workspace setup file:**

    After building your workspace, source the setup file:

    ```sh
    source ~/ros2_ws/install/setup.bash
    ```
