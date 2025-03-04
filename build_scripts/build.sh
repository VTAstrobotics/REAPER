#!/bin/bash

# Check if the script is being sourced or run directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "build.sh: [ERROR]: This script must be sourced, not executed."
    echo "build.sh: [INFO]: Usage: source build.sh"
    exit 1
fi

cd /workspaces/REAPER

echo build.sh: building

colcon build --packages-select action_interfaces state_messages
source install/setup.bash

colcon build --packages-select state_messages_utils
source install/setup.bash

colcon build --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON --packages-skip action_interfaces state_messages state_messages_utils
source install/setup.bash

echo build.sh: done
