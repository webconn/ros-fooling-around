#!/bin/bash

IMAGE=webconn/ros-fooling-around
PWD=`pwd`
X11_SOCKET=/tmp/.X11-unix

docker run --rm -ti \
    -e DISPLAY=$DISPLAY \
    -e DEV_UID=$UID \
    -e DEV_USER=$USER \
    -v $PWD:/userdata \
    -v $X11_SOCKET:$X11_SOCKET \
    -h ros-fooling-around \
    $IMAGE

