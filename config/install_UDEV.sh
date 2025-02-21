#!/bin/bash
echo "adding udev rules"
sudo cp 99-reaper-udev.rules /etc/udev/rules.d
sudo udevadm trigger # loads the new udev rules
echo "udev rules added and loaded"
