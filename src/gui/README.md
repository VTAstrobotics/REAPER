# Astrobotics GUI

## About

This is an python file desgined to display ros topics and camera streams in the context of a competition run. 

The basic features are working, but improvements and additions are ongoing. The "feature" branch contains the in-development version.

This GUI is currently maintained by Andrew Walls.

## Current Features

-  Can display multiple ros topics and camera feeds.
-  Displays when topic data times out or doesn't show
-  Individual topics can be rearraged and removed using mouse controls.
-  Subscribe to topics while running OR specify topic to initalize on startup.

## Installation & Running

The only requirement to run is having python3 installed.

Unlike ROS packages, these files do not need to be build within the workspace using colcon. Instead, they simply need to exist anywhere within the ROS workspace.

To run the GUI, call /bin/python3 then the path. For example...
```
/bin/python3 /workspaces/REAPER/src/gui.py
```
Alternately, you can run these via VSCode using the run button.

## Usage

You can subscribe to topics using the two bars at the topic. You do not need to include the "/".

To subscribe to topics on initization, edit the "initial topics" and "initial cameras" within the main loop inside the python file.

You can drag topic windows by clicking and dragging it to any location in the window you want.

You can remove any topic by right clicking within the window and clicking "remove".

## TODO

-  Camera latency values
-  Controller connectivity display
-  "Swapping" camera feeds in one window