#!/bin/bash

source /opt/ros/jazzy/setup.bash

echo "Installing Project Dependencies"
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy

echo "Building Project"
colcon build --symlink-install

echo ""
echo "Build Complete"
echo "To run in a workspace first run 'source ./source_workspace.sh'"
