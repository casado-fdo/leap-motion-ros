# ROS LeapMotion Bridge

## Overview
This package provides a ROS Noetic driver for the LeapMotion controller, leveraging the latest SDK (v5.17, the only available version in 2025, supported only on Ubuntu 22.04 or later). To ensure compatibility and ease of deployment, both the SDK and the ROS package are fully containerized using Docker.

## Features
- Publishes LeapMotion hand tracking data to ROS topics using custom messages.
- Uses the latest official LeapMotion SDK together with ROS Noetic.
- Provides a Dockerized setup to simplify installation and execution.


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
This will download and install the LeapMotion SDK, and set up ROS Noetic inside the container.

### 3. Run
Once the container is built, simply run:
```bash
make start
```
This will start the container and launch the LeapMotion SDK together with the ROS nodes. To launch a visual demo with Rviz, run instead:
```bash
make start rviz:=true
```

### ROS Topics
The following ROS topics will be available (with `{hand}` replaced by `left` or `right`):

* `/leapmotion/hands/{hand}` – Hand tracking data. Includes hand poses (position and orientation) for the palm centre, as well as each finger and bone. See [`msg/Hand.msg`](leap_motion_controller/msg/Hand.msg) for further details.
* `/leapmotion/hands/{hand}/grab` – Value in the range \[0.0, 1.0] indicating the grab state of the hand (1.0 = fully grabbing, 0.0 = open).
* `/leapmotion/hands/{hand}/pinch` – Value in the range \[0.0, 1.0] indicating the pinch state of the hand (1.0 = fully pinching, 0.0 = open).
* `/leapmotion/hands/{hand}/markers` – `MarkerArray` for visualising the hand skeleton in Rviz.


## License
This project is licensed under the MIT License. See `LICENSE` for details.