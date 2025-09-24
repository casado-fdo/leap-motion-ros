# ROS Leap Motion Bridge

## Overview
This package provides a ROS Noetic driver for the Leap Motion controller, leveraging the latest SDK (v5.17, the only available version in 2025, supported only on Ubuntu 22.04 or later). To ensure compatibility and ease of deployment, both the SDK and the ROS package are fully containerized using Docker.

## Features
- Publishes Leap Motion hand tracking data to ROS topics using custom messages.
- Uses the latest official Leap Motion SDK together with ROS Noetic.
- Provides a Dockerized setup to simplify installation and execution.
- Supports visualization in Rviz.
- Compatible with Leap Motion Controller v1 and v2.


## Installation and Usage
### 1. Clone the Repository
```bash
git clone https://github.com/casado-fdo/leap-motion-ros.git
cd leapmotion-motion-ros
```

### 2. Build the Docker Container
Ensure Docker is installed on your system. Then, build the container:
```bash
make .build
```
This downloads and installs the Leap Motion SDK and sets up ROS Noetic inside the container.

### 3. Run
Start the container and launch the Leap Motion ROS nodes:
```bash
make start
```
To launch a visual demo with Rviz:
```bash
make start rviz:=true
```
This opens Rviz and visualizes the hand skeleton and TF transforms between bones.

![Demo in Rviz](assets/leap_demo_rviz.gif)


### ROS Topics
Available ROS topics (replace `{hand}` with `left` or `right`):

| Topic                              | Description                                                                                                                   |
| ---------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| `/leapmotion/hands/{hand}`         | Hand tracking data: includes palm center, fingers, and bone poses. See [`msg/Hand.msg`](leap_motion_controller/msg/Hand.msg). |
| `/leapmotion/hands/{hand}/grab`    | Grab state (0.0 = open, 1.0 = fully grabbing).                                                                                |
| `/leapmotion/hands/{hand}/pinch`   | Pinch state (0.0 = open, 1.0 = fully pinching).                                                                               |
| `/leapmotion/hands/{hand}/markers` | `MarkerArray` for visualizing hand skeleton in Rviz.                                                                          |



## License
This project is licensed under the MIT License. See `LICENSE` for details.


## Acknowledgments
Developed at the Personal Robotics Laboratory, Imperial College London (2025).