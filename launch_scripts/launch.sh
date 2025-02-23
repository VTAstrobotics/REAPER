#!/bin/bash

# Setup
echo "Setting up..."
#starting can

./launch_scripts/can_startup.sh
cd /workspaces/REAPER/src/
source install/setup.bash



# Running
echo "Launching nodes..."
ros2 run dump DumpActionServer&
# ros2 run drivetrain DriveActionServer&
ros2 run teleop_control Distributor&
ros2 run dig DigActionServer&
ros2 run joy joy_node

# Teardown
echo "Shutting down..."
pkill ros2
pkill DumpActionServer
# pkill DriveActionServer
pkill Distributor
pkill DigActionServer
echo "Killed background processes"
