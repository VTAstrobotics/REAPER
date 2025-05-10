#!/usr/bin/env python3
import re
import subprocess
import sys
import os
import fcntl
import pyudev
import time

# from <linux/usbdevice_fs.h>
USBDEVFS_RESET = ord('U') << (4*2) | 20

def find_canable_bus_dev():
    """
    Return (bus, dev) for the first lsusb line containing 'canable'.
    """
    pat = re.compile(r'^Bus\s+(\d{3})\s+Device\s+(\d{3}):', re.IGNORECASE)
    out = subprocess.check_output(['lsusb'], text=True)
    for line in out.splitlines():
        if 'canable' in line.lower():
            m = pat.match(line)
            if m:
                return int(m.group(1)), int(m.group(2))
    return None, None

def usb_reset(bus: int, dev: int):
    """
    Send USBDEVFS_RESET ioctl to /dev/bus/usb/<bus>/<dev>.
    """
    path = f"/dev/bus/usb/{bus:03d}/{dev:03d}"
    if not os.path.exists(path):
        print(f"Error: device node {path} does not exist!", file=sys.stderr)
        sys.exit(1)
    fd = os.open(path, os.O_WRONLY)
    try:
        fcntl.ioctl(fd, USBDEVFS_RESET, 0)
    finally:
        os.close(fd)
        

# … your find_canable_bus_dev() and usb_reset() here …
def wait_for_remove_and_add(bus, dev, timeout=5.0, poll_interval=0.1):
    context = pyudev.Context()
    monitor = pyudev.Monitor.from_netlink(context)
    monitor.filter_by(subsystem='usb')
    monitor.start()  # begin listening

    start = time.time()
    removed = False

    while True:
        # Compute how much time is left
        elapsed = time.time() - start
        if elapsed > timeout:
            return False

        # Wait up to poll_interval for a udev event
        device = monitor.poll(timeout=poll_interval)
        if device is None:
            # no event this tick, loop back to check timeout
            continue

        action = device.action
        # udev properties are strings like "001", so zero-pad
        if device.get('BUSNUM') == f"{bus:03d}" and device.get('DEVNUM') == f"{dev:03d}":
            if action == 'remove':
                print("[INFO] Received udev remove event")
                removed = True
            elif removed and action == 'add':
                print("[INFO] Received udev add event")
                return True

    # never reached


def wait_for_remove_and_add(bus, dev, timeout=5.0, poll_interval=0.1):
    context = pyudev.Context()
    monitor = pyudev.Monitor.from_netlink(context)
    monitor.filter_by(subsystem='usb')
    monitor.start()  # begin listening

    start = time.time()
    removed = False

    while True:
        # Compute how much time is left
        elapsed = time.time() - start
        if elapsed > timeout:
            return False

        # Wait up to poll_interval for a udev event
        device = monitor.poll(timeout=poll_interval)
        if device is None:
            # no event this tick, loop back to check timeout
            continue

        action = device.action
        # udev properties are strings like "001", so zero-pad
        if device.get('BUSNUM') == f"{bus:03d}" and device.get('DEVNUM') == f"{dev:03d}":
            if action == 'remove':
                print("[INFO] Received udev remove event")
                removed = True
            elif removed and action == 'add':
                print("[INFO] Received udev add event")
                return True

    # never reached



def main():
    bus, dev = find_canable_bus_dev()
    if bus is None:
        print("Error: could not find CANable in lsusb.", file=sys.stderr)
        sys.exit(1)
    print(f"Resetting USB device bus={bus:03d} dev={dev:03d}")
    usb_reset(bus, dev)
    if wait_for_remove_and_add(bus, dev):
        print("Device disconnected and reconnected successfully")
    else:
        print("Timeout waiting for udev events")

    print("Reset complete.")
    # In main():



if __name__ == "__main__":
    main()
