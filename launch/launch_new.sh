#!/bin/bash

# If no output URL is given, launch in fully autonomous mode
# Otherwise, communicate mavlink data to the ground control station at the given URL

if [ -z $1 ]
then
mavproxy.py --master=/dev/ttyTHS2,57600 --quadcopter --aircraft MySpiri
elif [ -n $1 ]
then
mavproxy.py --master=/dev/ttyTHS2,57600 --quadcopter --aircraft MySpiri --out=udp:$1:14550
fi
