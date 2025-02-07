#!/bin/bash

printf "\nbuild.sh:\n"
echo "Changing directory"
cd /workspaces/REAPER/src/

echo "Building project"
colcon build --symlink-install

echo "Sourcing"
source install/setup.bash

printf "build.sh done.\n\n"
