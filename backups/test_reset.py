import re, subprocess, sys, fcntl, os
import pyudev

# … your find_canable_bus_dev() and usb_reset() here …

def wait_for_remove_and_add(bus, dev, timeout=5.0):
    context = pyudev.Context()
    monitor = pyudev.Monitor.from_netlink(context)
    monitor.filter_by(subsystem='usb')

    start = time.time()
    removed = False

    for action, device in monitor:
        # match bus & dev
        if device.get('BUSNUM') == f"{bus:03d}" and device.get('DEVNUM') == f"{dev:03d}":
            if action == 'remove':
                print("Received udev remove event")
                removed = True
            elif removed and action == 'add':
                print("Received udev add event")
                return True
        if time.time() - start > timeout:
            break
    return False

# In main():
usb_reset(bus, dev)
if wait_for_remove_and_add(bus, dev):
    print("Device disconnected and reconnected successfully")
else:
    print("Timeout waiting for udev events")
