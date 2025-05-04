#!/bin/bash
#from https://canable.io/getting-started.html bc we have a knock off canable
    sudo slcand -ocs8 /dev/CAN-SERIAL-LINK can1
    sudo ifconfig can1 up
    sudo ifconfig can1 txqueuelen 1000
echo "CAN interface started, haul some rocks"
