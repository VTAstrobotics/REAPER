#!/bin/bash
#from https://canable.io/getting-started.html bc we have a knock off canable
    sudo slcand -o -c -s8 /dev/CAN-SERIAL-LINK can0
    sudo ifconfig can0 up
    sudo ifconfig can0 txqueuelen 1000
echo "CAN interface started, haul some rocks"