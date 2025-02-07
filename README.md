# ROS LeapMotion Bridge (2025)

## Overview
This package provides a basic ROS Noetic driver for the LeapMotion controller, leveraging the latest SDK (the only available version in 2025, supported only on Ubuntu 22.04). To ensure compatibility and ease of deployment, both the SDK and the ROS package are fully containerized using Docker.

## Features
- Publishes LeapMotion hand tracking data to ROS topics (currently only hand pose and grabbing).
- Uses the latest official LeapMotion SDK together with ROS Noetic.
- Provides a Dockerized setup to simplify installation and execution.


## Installation and Usage
### 1. Clone the Repository
```bash
git clone https://github.com/casado-fdo/leap-motion-ros.git
cd leapmotion-docker
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
This will start the container and launch the LeapMotion SDK together with the ROS nodes.

### ROS Topics
The following ROS topics will be available:
- `/leapmotion/hands/right/pose` – Hand tracking data (position and orientation) for the right hand.
- `/leapmotion/hands/left/pose` – Hand tracking data (position and orientation) for the left hand.
- `/leapmotion/hands/right/grab` – Value in the range [0.0, 1.0] indicating whether the hand is open (0.0) or closed (1.0).
- `/leapmotion/hands/left/grab` – Value in the range [0.0, 1.0] indicating whether the hand is open (0.0) or closed (1.0).

## License
This project is licensed under the MIT License. See `LICENSE` for details.