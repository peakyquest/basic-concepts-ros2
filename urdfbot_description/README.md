
# urdfbot_description   

## Overview

`urdfbot_description` is a ROS 2 package designed for working with URDF (Unified Robot Description Format) files. This package leverages URDF and XACRO to describe robot models, and it includes necessary folders for launch files, RViz configurations, and URDF files.

## Prerequisites

This package has two primary dependencies:
- `urdf`
- `xacro`

## Modification in `CMakeList` 

```
cmake_minimum_required(VERSION 3.8)

#code

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

#code 

install(
  DIRECTORY
    urdf
    rviz
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
```


After configuring the package, build your ROS 2 workspace:

```
cd ~/ros2_ws
colcon build
```

Source the workspace to overlay this workspace on top of your current environment:

```
source install/setup.bash
```
## Visualization with RViz2

To visualize the URDF model using RViz2, follow these steps:

- Compile the Latest URDF File Version

- Ensure that you have the latest version of the URDF file compiled:


```
cd ~/ros2_ws
colcon build --packages-select urdfbot_description
source install/setup.bash
```

### Launch the URDF Visualization

Start the urdf_visualize.launch.py launch file:


```
cd ~/ros2_ws
source install/setup.bash
ros2 launch urdfbot_description urdf_visualize.launch.py
```


