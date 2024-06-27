# Introduction
This repository contains the code used to run the MultiCamera PointCloud Device developed by the AURA Project.
The devices utilized 3 Realsense D405 that are connected on a support frame. Their positioning is determined
by the d(x;y), a(x;y), b(x;y) coordinates with their orientation relative to eachother being set by alpha
(e.g. 120deg).

# Quick information

* In order for the software to work, you need to disconnect all the other cameras from the farmbot and connect the 3 realsense cameras.
* You cannot run the camera_handler package together with this package
* You can run the keyboard controller together with this package, but any other external node is recommended
* This package has its own launch file that runs all the farmbot nodes. You will then have to run the mcpc controller and server nodes alongside the launch.

# How to run everything

Make sure you correctly initialized the submodule in the ROS2 FarmBot repository. All commands must be done from the main project directory. Make sure to build and source the codebase.

Launch the farmbot nodes
```bash
ros2 launch multicam_pointcloud multicam.launch.py
```

Run the MCPC Controller node
```bash
ros2 run multicam_pointcloud mcpc_node.py
```


