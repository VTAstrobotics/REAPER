#!/usr/bin/env python3
import re
import subprocess
import sys

def get_bus_dev_by_name(name_substr: str):
    """
    Return (bus, dev) for the first lsusb line whose description
    contains name_substr (case‐insensitive), e.g. "canable".
    """
    out = subprocess.check_output(['lsusb'], text=True)
    for line in out.splitlines():
        # Example line:
        # Bus 001 Device 007: ID 16d0:117e MCS CANable2 b158aa7 github.com/normaldotcom/canable2.git
        m = re.match(
            r'Bus\s+(\d{3})\s+Device\s+(\d{3}):\s+ID\s+[0-9a-f]{4}:[0-9a-f]{4}\s+(.+)$',
            line, re.IGNORECASE
        )
        if m:
            desc = m.group(3)
            if re.search(name_substr, desc, re.IGNORECASE):
                return int(m.group(1)), int(m.group(2))
    return None, None

def get_port_path(bus: int, dev: int):
    """
    Parse `lsusb -t` and return a port path like "1-1.1.3"
    for the given bus and device numbers.
    """
    tree = subprocess.check_output(['lsusb', '-t'], text=True)
    lines = tree.splitlines()

    # Match the Bus block header: "/:  Bus 01.Port 02: ..."`
    bus_pattern = re.compile(r'^/:\s+Bus\s+%02d\.Port\s+(\d+):' % bus)
    in_block = False
    port_map = {}

    for line in lines:
        # Enter our bus block
        m_bus = bus_pattern.match(line)
        if m_bus:
            root_port = int(m_bus.group(1))
            in_block = True
            port_map.clear()
            continue
        # Exit if another bus starts
        if in_block and line.startswith('/:') and not bus_pattern.match(line):
            break
        if not in_block:
            continue

        # Lines like "    |__ Port 1: Dev 007, If 0, ..."
        m = re.match(r'^(?P<indent>\s*)\|__\s+Port\s+(\d+):\s+Dev\s+(\d+)', line)
        if not m:
            continue

        indent = m.group('indent')
        depth = len(indent) // 4
        port_num = int(m.group(2))
        dev_num = int(m.group(3))

        port_map[depth] = port_num

        if dev_num == dev:
            # Build path: bus-rootPort.sub1.sub2...
            # First two parts: bus and root_port
            path_parts = [str(bus), str(root_port)]
            # Append each subport
            for d in range(1, depth + 1):
                path_parts[1] += f".{port_map[d]}"
            # Return "bus-root.sub1.sub2"
            return "-".join([path_parts[0], path_parts[1]])

    return None

def main():
    # find by name substring "canable"
    bus, dev = get_bus_dev_by_name("canable")
    if bus is None:
        print("Error: no CANable device found in lsusb.", file=sys.stderr)
        sys.exit(1)

    port_path = get_port_path(bus, dev)
    if port_path is None:
        print(f"Error: could not determine port path for Bus {bus} Dev {dev}.", file=sys.stderr)
        sys.exit(1)

    cmd = ['./tl_reset_can.sh', f'port="{port_path}"']
    print(f"Invoking: {' '.join(cmd)}")
    ret = subprocess.call(cmd)
    sys.exit(ret)

if __name__ == "__main__":
    main()
import os


os