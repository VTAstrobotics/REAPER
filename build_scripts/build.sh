#!/bin/bash

printf "\nbuild.sh:\n"
cd /workspaces/REAPER

echo "Building and sourcing project"
colcon build --packes-select action_interfaces
source install/setup.bash
colcon build --symlink-install
source install/setup.bash

printf "build.sh done.\n\n"
