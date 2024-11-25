FROM osrf/ros:humble-desktop-full

# Create overlay workspace
WORKDIR /workspaces/REAPER/src
COPY packages.repos .

# Rosdep update
RUN rosdep update

RUN vcs import < packages.repos; \
    cd .. \
    rosdep install --from-paths src --ignore-src -r -y \
    source /opt/ros/${ROS_DISTRO}/setup.bash

# Add sourcing ROS setup.bash to .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

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

# Set environment variable for Pico MicroROS
ENV PICO_SDK_PATH=$HOME/pico-sdk

# MicroROS dependencies
RUN sudo apt install -y \
    gcc-arm-none-eabi \
    doxygen \
    && git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git $HOME/pico-sdk

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /workspaces/PHOENIX/src/main_ws/install/setup.bash" >> ~/.bashrc

###############################################################################
#                                                                             #
# Add dependencies below using RUN <command you used to install>              #
#                                                                             #
###############################################################################
# install clang-format version 16
RUN sudo apt-get install -y lsb-release \
    wget \
    software-properties-common \
    gnupg \
    && sudo wget https://apt.llvm.org/llvm.sh \
    && sudo chmod +x llvm.sh \
    && sudo ./llvm.sh 16 \
    && sudo apt-get install -y clang-format-16 \
    && sudo ln -s $(which clang-format-16) /usr/local/bin/clang-format
