## jetson_on_wheels

jetson_on_wheels is a ROS workspace responsible for controlling the scaled-down vehicle with a Jetson Nano.

### Installation
To get started, follow these steps:

1. Install ROS Melodic.
2. Initialize a ROS workspace.
3. Place the jetson_on_wheels folder in the src directory of the workspace.
4. Build the workspace.

### Operating Instructions

Follow these instructions to operate jetson_on_wheels:

1. Launch the RealSense camera by running the following command:
```
roslaunch jetson_on_wheels rs_t265.launch
```

2. Launch the relevant ROS nodes by running the following command:
```
roslaunch jetson_on_wheels run_jetson_on_wheels.launch
```
