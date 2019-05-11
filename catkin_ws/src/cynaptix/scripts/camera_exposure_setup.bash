#!/bin/bash

# This sets the camera to use manual exposure rather than automatic 
# exposure settings, as well as setting the exposure time
# This prevents over saturation from the bright LEDs

# Stop on any error
set -e

# "$1" is the camera name (e.g. "1" for "/dev/video1")
# "$2" is the exposure time (e.g. "10")
# Read the arguments, if there are not 2 print error message and return
if [ $2 = "" ]
then
    echo "Usage: $0 <camera_number> <camera_exposure_time>"
    exit 1
else
    # Set manual
    v4l2-ctl --set-ctrl=exposure_auto=1 -d "$1"
    # Set exposure time
    v4l2-ctl --set-ctrl=exposure_absolute="$2" -d "$1"
    # Turn off white automatic temperature balance
    v4l2-ctl --set-ctrl=white_balance_temperature_auto=0 -d "$1"
    # Nice message
    echo "Set /dev/video$1 to an absolute exposure of $2"
    exit 0
fi
