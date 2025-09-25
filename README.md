## Totally detailed description
Individual ROS2 Architecture Design + Node Implementations for Mobile, Apple-picking robot

## Building Workspace
`colcon build`

### Running user interface package

`. install/setup.bash && ros2 run user_interface supervisor_pub`

### Running control node (supervisor subscriber)

`. install/setup.bash && ros2 run control_node supervisor_sub`