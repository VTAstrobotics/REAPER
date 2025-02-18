FROM ros:humble-ros-base-jammy
# FROM ubuntu:18.04

RUN apt-get update && apt-get install -y \
        git \
        jq \
        wget \
        xz-utils

# install clang-format version 16
RUN apt-get install -y lsb-release \
    wget \
    software-properties-common \
    gnupg \
    && wget https://apt.llvm.org/llvm.sh \
    && chmod +x llvm.sh \
    && ./llvm.sh 16 \
    && apt-get install -y clang-format-16 \
    && ln -s $(which clang-format-16) /usr/local/bin/clang-format

COPY .clang-format /.clang-format
COPY .clang-tidy /.clang-tidy

# Basic ROS2 setup
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop=0.10.0-1* \
    python3 \
    python3-pip \
    && pip3 install setuptools==58.2.0 \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# for clang-tidy
RUN colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
