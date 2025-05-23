FROM ros:humble-ros-base-jammy

# Basic ROS2 setup
RUN apt update && apt install -y --no-install-recommends \
    ros-humble-desktop \
    python3 \
    python3-pip \
    && pip3 install setuptools==58.2.0 \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Add nonroot user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=astro-dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages (bloat, but dev container so we feel worth it. do not recommend for final container)
RUN sudo apt update && sudo apt upgrade -y

# install Phoenix for the motors
RUN sudo curl -s --compressed -o /usr/share/keyrings/ctr-pubkey.gpg "https://deb.ctr-electronics.com/ctr-pubkey.gpg"
RUN sudo curl -s --compressed -o /etc/apt/sources.list.d/ctr2024.list "https://deb.ctr-electronics.com/ctr2024.list"
RUN sudo apt update
RUN sudo apt install -y phoenix6 can-utils

#allows phoenix 6 to work
RUN echo "export LD_LIBRARY_PATH=/usr/lib/phoenix6:\$LD_LIBRARY_PATH" >> ~/.bashrc

# MicroROS pico env var
ENV PICO_SDK_PATH=$HOME/pico-sdk

# MicroROS dependencies
RUN sudo apt install -y \
    gcc-arm-none-eabi \
    doxygen

# Clone MicroROS Pi Pico
RUN git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git $HOME/pico-sdk

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /workspaces/REAPER/install/setup.bash" >> ~/.bashrc

###############################################################################
#                                                                             #
# Add dependencies below using RUN <command you used to install>              #
#                                                                             #
###############################################################################
# install clang-format dependencies (also add-apt-repository relies on software-properties-common)
RUN sudo apt install -y \
    lsb-release \
    wget \
    software-properties-common \
    gnupg

# install clang-format and clang-tidy
RUN sudo wget https://apt.llvm.org/llvm.sh \
    && sudo chmod +x llvm.sh \
    && sudo ./llvm.sh 21 \
    # NOTICE: i have set the version to 21. over time you may choose to migrate
    # to newer versions, but this may cause errors with the existing formatter
    # rules (.clang-format and .clang-tidy).
    && sudo apt update \
    && sudo apt install -y clang-format clang-tidy

# install sparkcan for sparkmax over can
RUN sudo add-apt-repository ppa:graysonarendt/sparkcan
RUN sudo apt update
RUN sudo apt install sparkcan

# Install dependencies for state messaging
RUN pip install numpy opencv-python

#### Begin Navigation Stack #####
RUN sudo apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup

# For simulation
RUN sudo apt install -y \
    gazebo \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro

# install turtlebot demo, including gz, unless on a Jetson
# i swear that it worked before but it doesnt anymore. So basically install this manually if you want nav stuff until i can figure this out.
# RUN if ! grep -q "Jetson" /proc/device-tree/model; then \
#         sudo apt install -y ros-humble-turtlebot3-gazebo; \
#     fi

# set environment variables for navigation
ENV TURTLEBOT3_MODEL=waffle \
    GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
# ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
#### End Navigation Stack #####
