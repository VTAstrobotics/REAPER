#!/bin/bash

set -e

# echo "### Getting branch"
# # BRANCH=${GITHUB_REF#*refs/heads/} # this doesnt work on PR, only on push
# BRANCH=${GITHUB_HEAD_REF} # this doesnt work on push, only on PR

# echo "### Add safe directory"
# git config --global --add safe.directory /home/astro-dev/src/ci

# echo "### git fetch $BRANCH ..."
# git fetch origin $BRANCH

# echo "### Branch: $BRANCH (ref: $GITHUB_REF )"
# git checkout $BRANCH

# echo "## Configuring git author..."
# git config --global user.email "clang-format@1337z.ninja"
# git config --global user.name "Clang Format"

# # Ignore workflow files (we may not touch them)
# git update-index --assume-unchanged .github/workflows/*

echo "## Setting up clang-format on C/C++ source"
SRC_HEADERS=$(git ls-tree --full-tree -r HEAD | grep -e "\.\(h\|hpp\)\$" | cut -f 2)
SRC_IMPLEMENTATION=$(git ls-tree --full-tree -r HEAD | grep -e "\.\(c\|cpp\)\$" | cut -f 2)

# for clang-tidy
echo "## Clean build source code"
echo "### Clean (just in case; should already be clean)"
rm -rf build/ install/ log/ compile_commands.json

echo "### Build"
colcon build --symlink-install --merge-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

echo "### Combine compile_commands.json into a global one"
find build/ -name compile_commands.json -exec jq -s 'add' {} + > compile_commands.json

echo "### Source"
source install/setup.bash

echo "##########################################################################"
echo "CHECK VERSIONS"
echo "machine"
uname -a
echo "format"
clang-format --version
echo "tidy"
clang-tidy --version
echo "apply-replacements"
clang-apply-replacements --version
echo "##########################################################################"

echo "## Running clang-tidy on C/C++ src code"
# this script is better than running clang-tidy manually, but has similar effect

# it also runs clang-format after with the -format option, but incompatible version somehow so
# i just do it manually. feel free to figure out version issue i dont really care.
# it would probably work if we update the dev container to ubuntu 24, which you need to
# update ros from humble, then you can get the most updated version of the run-clang-tidy script
# and clang-format and clang-tools and it would be fine
python3 .github/workflows/run-clang-tidy.py \
    -extra-arg=-v \
    -fix \
    -header-filter="$SRC_HEADERS" \
    -p=. \
    -style=none \
    src/

# echo "## Run clang-format on C/C++ src code"
# clang-format -style=file -i $SRC_HEADERS $SRC_IMPLEMENTATION

# echo "## Commiting files if it still builds"
# echo "### Clean"
# rm -rf build/ install/ log/ compile_commands.json

# echo "### Build using build script"
# unset AMENT_PREFIX_PATH
# source /opt/ros/humble/setup.bash
# source build_scripts/build.sh 2> >(tee err.log >&2)

# if [ -s err.log ]; then
#     echo Build failed due to output in stderr. Yes, this fails on warnings. Fix them.;
#     exit 1;
# else
#     # Notice this does not stage any newly created files, bc it creates some junk, so be careful if you choose to change this one day :)
#     git commit -am "applied C/C++ auto formatting and linting" || true
#     echo "## Pushing to $BRANCH"
#     git push -u origin $BRANCH
# fi
