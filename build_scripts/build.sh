#!/bin/bash

printf "\nbuild.sh:\n"
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "build.sh: [ERROR]: This script must be sourced, not executed."
    echo "build.sh: [INFO]: Usage: source build.sh"
    exit 1
fi
echo "Changing directory"
cd /workspaces/REAPER/src/

echo "Building project"
colcon build --symlink-install

echo "Sourcing"
source install/setup.bash

echo build.sh: done
