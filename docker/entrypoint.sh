#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash
echo "Sourced ROS 2 Humble"

# Source the base workspace, if built
if [ -f /ros2_ws/install/setup.bash ]
then
  echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
fi

# Source the overlay workspace, if built
if [ -f /develop_ws/install/setup.bash ]
then
  echo "source /develop_ws/install/setup.bash" >> ~/.bashrc
  source /develop_ws/install/setup.bash
  echo "Sourced ROS developer workspace"
fi

# Execute the command passed into this entrypoint
exec "$@"
