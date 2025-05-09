#!/bin/bash

# Check if the script is being sourced or run directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "build.sh: [ERROR]: This script must be sourced, not executed."
    echo "build.sh: [INFO]: Usage: source build_scripts/build.sh"
    exit 1
fi

echo build.sh: building
colcon build --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON

echo build.sh: sourcing
source install/setup.bash

echo build.sh: done
