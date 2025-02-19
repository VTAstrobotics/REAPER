#!/bin/bash

# Set up logging entrypoint stuff to build.log file in .devcontainer directory
rm /workspaces/REAPER/.devcontainer/build.log
sudo touch /workspaces/REAPER/.devcontainer/build.log
sudo chmod 777 /workspaces/REAPER/.devcontainer/build.log

{ printf "entrypoint.sh:\n"
###############################################################################
#                                                                             #
# Startup commands (do not put anything above this)                           #
#                                                                             #
###############################################################################

# ROS basic build and source
cd /workspaces/REAPER/
source build_scripts/build.sh

# Give permissions to input devices, like Xbox controller
printf "[INFO]: If you get a message like \"chmod: cannot access '/dev/input/js*': No such file or directory\" do not worry. If it ALSO says \"chmod: cannot access '/dev/input/event*': No such file or directory\" AND you have a controller plugged in, there may be an issue with seeing the controller. Note that the new Jetson Orin Nanos require a manual kernel patch to see Xbox controllers.\n"
sudo chmod +rx /dev/input/event*
sudo chmod +rx /dev/input/js*


###############################################################################
#                                                                             #
# Shutdown commands (do not put anything below this)                          #
#                                                                             #
###############################################################################
} &> /workspaces/REAPER/.devcontainer/build.log

exit 0
