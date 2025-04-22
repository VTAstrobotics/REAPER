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


PICO_SDK_PATH=$HOME/pico-sdk

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

# Setup
## Get initial state with non-root user
OLDUSR=$(logname)
OLDHME=$(getent passwd "$OLDUSR" | cut -d: -f6)

## Handle flags
y_flag=''
while getopts 'y' flag; do
        case "${flag}" in
                y) y_flag='true';;
                *) exit 1 ;;
        esac
done

# Set up Docker's apt repository
sudo apt update && sudo apt upgrade -y
sudo apt install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

## Add the repository to sources
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update

# Install Docker
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Verify installation
if [[ ! "${y_flag}" ]]; then
	read -ep "Verify Docker installation? [Y/n] " VRFY
fi
if [[ "${y_flag}" || $VRFY == "Y" || $VRFY == "y" || $VRFY == "" ]]; then
	OLDUSR=$(logname)
	echo "Verifying Docker installation..."
	sudo docker run hello-world
else
	echo "[WARNING]: Not verifying Docker installation."
fi

# Post-installation steps
if [[ ! "${y_flag}" ]]; then
	read -ep "Want to run docker without sudo? [Y/n] " PSTINSTL
fi
if [[ "${y_flag}" || $PSTINSTL == "Y" || $PSTINSTL == "y" || $PSTINSTL == "" ]]; then
	echo "Beginning post-installation steps"
	sudo groupadd docker
	sudo usermod -aG docker $OLDUSR
	# why not `newgrp docker`? can't `newgrp` in a script for security reasons
	if [[ $(sg docker -c "docker run hello-world") ]]; then
		echo "Post-installation steps were successful"
		echo "Reboot to run docker without sudo"
	else
		echo "Post-installation steps were unsuccessful"
	fi
else
	echo "Not performing post-installation steps"
fi

echo "Docker installed."