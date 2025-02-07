#!/bin/bash
set -e  # Exit on any error

# Source ROS and Conda/Mamba
. /opt/miniforge3/etc/profile.d/conda.sh
. /opt/miniforge3/etc/profile.d/mamba.sh
mamba activate ros
source /catkin_ws/devel/setup.bash 

# Execute the command passed to the container
exec "$@"
