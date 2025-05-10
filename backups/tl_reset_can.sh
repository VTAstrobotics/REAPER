#!/bin/bash

#just give it the usb port

port=$1 # as shown by lsusb -t: {bus}-{port}(.{subport})

bind_usb() {
    sudo  echo "$1" >/sys/bus/usb/drivers/usb/bind
}

unbind_usb() {
    sudo  echo "$1" >/sys/bus/usb/drivers/usb/unbind
}

unbind_usb "$port"
sleep 1 # enable delay here
#bind_usb "$port"
