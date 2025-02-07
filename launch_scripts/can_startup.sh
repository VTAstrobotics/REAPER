#!/bin/bash
#from https://canable.io/getting-started.html bc we have a knock off canable
    sudo ifconfig can0 up
    sudo ifconfig can0 txqueuelen 1000
echo "CAN interface started, haul some rocks"
