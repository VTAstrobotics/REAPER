#!/bin/bash

echo "HELLO I AM IN THE ENTRYPOINT SCRIPT OK"

sh -c "git config --global --add safe.directory $PWD"

set -eu

REPO_FULLNAME=$(jq -r ".repository.full_name" "$GITHUB_EVENT_PATH")
echo "repo fullname is $REPO_FULLNAME"

echo "## Initializing git repo..."
git init
echo "### Adding git remote..."
# git remote add origin https://x-access-token:$GITHUB_TOKEN@github.com/$REPO_FULLNAME.git
git remote set-url origin https://x-access-token:$GITHUB_TOKEN@github.com/$REPO_FULLNAME.git
echo "### Getting branch"
# BRANCH=${GITHUB_REF#*refs/heads/}
BRANCH=${GITHUB_HEAD_REF}
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
echo "## Setting up clang-tidy"
echo "### build action_interfaces"
colcon build --packages-select action_interfaces
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
