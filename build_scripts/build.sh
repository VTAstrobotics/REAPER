#!/bin/bash

# Check if the script is being sourced or run directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "build.sh: [ERROR]: This script must be sourced, not executed."
    echo "build.sh: [INFO]: Usage: source build.sh"
    exit 1
fi

# cd /workspaces/REAPER

echo build.sh: building

colcon build --packages-select action_interfaces
source install/setup.bash

colcon build
source install/setup.bash

echo build.sh: done
