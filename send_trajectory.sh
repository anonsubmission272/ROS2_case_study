#!/bin/bash

echo "Launching trajectory send script"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ros2_control_demo_example_7 send_trajectory.launch.py

