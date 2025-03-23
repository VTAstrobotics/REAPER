#!/bin/bash

BRANCH=${GITHUB_HEAD_REF} # this doesnt work on push, only on PR
echo "## Setting up clang-format on C/C++ source"
SRC=$(git ls-tree --full-tree -r HEAD | grep -e "\.\(c\|h\|hpp\|cpp\)\$" | cut -f 2)

# for clang-tidy
echo "## Building source code"
echo "### build action_interfaces"
$(colcon build --packages-select action_interfaces)

echo "### source reaper"
source install/setup.bash

echo "### build reaper"
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

echo "## Running clang-format on C/C++ src code"
clang-format -style=file -i $SRC

echo "## Running clang-tidy on C/C++ src code"
clang-tidy src/dump/src/dump_server.cpp --config-file=.clang-tidy -p /workspaces/REAPER/build/ --fix-errors

echo "## Commiting files..."
git commit -am "applied C/C++ auto formatting" || true

echo "## Pushing to $BRANCH"
git push -u origin $BRANCH
