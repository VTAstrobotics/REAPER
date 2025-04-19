#!/bin/bash

# Setup
echo "Setting up..."
cd /workspaces/REAPER/src/
source install/setup.bash

# Running
echo "Launching nodes..."
ros2 run teleop_control Distributor&
ros2 run dig DigActionServer&
ros2 run dump DumpActionServer&
ros2 run drivetrain DriveActionServer&
ros2 run camera_streamer usbCamStreamer --cam 0 &
ros2 run camera_streamer usbCamStreamer --cam 2 &
ros2 run joy joy_node
ros2 run joy joy_node --ros-args -r /joy:=/joy_operator -p dev:=/dev/input/js1
# Teardown
echo "Shutting down..."
pkill ros2
pkill Distributor
pkill DigActionServer
pkill DumpActionServer
pkill DriveActionServer
pkill usbCamStreamer
echo "Killed background processes"
