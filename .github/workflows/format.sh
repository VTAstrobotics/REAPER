#!/bin/bash

set -e

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
echo "## Clean build source code"
echo "### Clean"
rm -rf build/ install/ log/

echo "### Build"
colcon build --symlink-install --merge-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

echo "### Source"
source install/setup.bash

echo "## Run clang-format on C/C++ src code"
clang-format -style=file -i $SRC

echo "## Merge build, and source"
colcon build --symlink-install --merge-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

echo "### Combine compile_commands.json into a global one"
find build/ -name compile_commands.json -exec jq -s 'add' {} + > compile_commands.json
source install/setup.bash

echo "## Running clang-tidy on C/C++ src code"
clang-tidy --config-file=.clang-tidy -p . --fix $SRC

echo "## Commiting files if it builds after clean"
rm -rf build/ install/ log/
source build_scripts/build.sh 2> >(tee err.log >&2)

if [ -s err.log ]; then
    echo Build failed due to output in stderr. Yes, this fails on warnings. Fix them.;
    exit 1;
else
    # Notice this does not stage any newly created files, bc it creates some junk, so be careful if you choose to change this one day :)
    git commit -am "applied C/C++ auto formatting" || true
    echo "## Pushing to $BRANCH"
    git push -u origin $BRANCH
fi
