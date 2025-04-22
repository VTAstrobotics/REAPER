#!/bin/bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common -y
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

sudo apt upgrade -y
sudo apt install ros-humble-desktop

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc


sudo curl -s --compressed -o /usr/share/keyrings/ctr-pubkey.gpg "https://deb.ctr-electronics.com/ctr-pubkey.gpg"
sudo curl -s --compressed -o /etc/apt/sources.list.d/ctr2024.list "https://deb.ctr-electronics.com/ctr2024.list"
sudo apt update
sudo apt install -y phoenix6 can-utils

# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash


ENV PICO_SDK_PATH=$HOME/pico-sdk

sudo apt install -y \
    gcc-arm-none-eabi \
    doxygen \
    && git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git $HOME/pico-sdk


echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
echo "source /workspaces/REAPER/install/setup.bash" >> ~/.bashrc

sudo apt-get install -y lsb-release \
    wget \
    software-properties-common \
    gnupg \
    && sudo wget https://apt.llvm.org/llvm.sh \
    && sudo chmod +x llvm.sh \
    && sudo ./llvm.sh 16 \
    && sudo apt-get install -y clang-format-16 \
    && sudo ln -s $(which clang-format-16) /usr/local/bin/clang-format

pip install numpy opencv-python

sudo apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup