#!/bin/bash

# Setup
echo "Setting up..."
cd /workspaces/REAPER/
sudo xhost + # Probably not needed but I'll keep it for now

# Running
echo "Launching GUI..."
/bin/python3 /workspaces/REAPER/src/gui.py
# Teardown
echo "Shutting down..." # I don't believe we need to do anything for the GUI shutdown, it's just a python file

