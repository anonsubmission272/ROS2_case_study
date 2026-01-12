#!/bin/bash


echo "Starting r6bot_controller..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ros2_control_demo_example_7 r6bot_controller.launch.py
echo "Started r6bot_controller"
