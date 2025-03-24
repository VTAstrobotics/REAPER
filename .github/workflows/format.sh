#!/bin/bash

echo "### Getting branch"
# BRANCH=${GITHUB_REF#*refs/heads/} # this doesnt work on PR, only on push
BRANCH=${GITHUB_HEAD_REF} # this doesnt work on push, only on PR

echo "### Add safe directory"
git config --global --add safe.directory /home/astro-dev/src/ci

echo "### git fetch $BRANCH ..."
git fetch origin $BRANCH

echo "### Branch: $BRANCH (ref: $GITHUB_REF )"
git checkout $BRANCH

echo "## Configuring git author..."
git config --global user.email "clang-format@1337z.ninja"
git config --global user.name "Clang Format"

# Ignore workflow files (we may not touch them)
git update-index --assume-unchanged .github/workflows/*

echo "## Setting up clang-format on C/C++ source"
SRC=$(git ls-tree --full-tree -r HEAD | grep -e "\.\(c\|h\|hpp\|cpp\)\$" | cut -f 2)

# for clang-tidy
echo "## Building source code"
echo "### build action_interfaces state_messages"
colcon build --symlink-install --packages-select action_interfaces state_messages

echo "### source reaper"
source install/setup.bash

echo "### build reaper"
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

echo "## Running clang-format on C/C++ src code"
clang-format -style=file -i $SRC

echo "### build again"
colcon build --symlink-install --packages-select action_interfaces state_messages
source install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

echo "## Running clang-tidy on C/C++ src code"
clang-tidy --config-file=.clang-tidy -p build/ --fix $SRC

echo "## Commiting files if it builds"
source build_scripts/build.sh 2> >(tee err.log >&2)
if [ -s err.log ]; then
    echo Build failed due to output in stderr. Yes, this fails on warnings. Fix them.;
    exit 1;
else
    git commit -am "applied C/C++ auto formatting" || true
    echo "## Pushing to $BRANCH"
    git push -u origin $BRANCH
fi
