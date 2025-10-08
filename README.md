## Totally detailed description
Individual ROS2 Architecture Design + Node Implementations for Mobile, Apple-picking robot


## Usage

```
colcon build && source install/setup.bash

ros2 launch launch/apple_picker_launch.py
```


## Dependencies
OpenCV C++ for Perception.

`sudo apt install ros-humble-vision-opencv ros-humble-cv-bridge libopencv-dev`


### Running user interface package

`. install/setup.bash && ros2 run user_interface supervisor_pub`

### Running control node (supervisor subscriber)

`. install/setup.bash && ros2 run control_node supervisor_sub`
