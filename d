#!/bin/bash

set -e

SHARED_DIR=~/ros_shared/
IMAGE=webconnru/ros-fooling-around
PWD=`pwd`
X11_SOCKET=/tmp/.X11-unix

EXEC=docker
USE_NVIDIA=false

if which nvidia-docker >& /dev/null; then
    EXEC=nvidia-docker
    USE_NVIDIA=true
    NVIDIA_FLAGS="-e QT_X11_NO_MITSHM=1"
    echo "Using nvidia-docker"
fi

if [[ $0 != './d' ]]; then
        echo "Must run ./d in repository root."
        exit 2
fi

if [[ -z $1 ]]; then
        echo "Usage: $0 [uasy], u for up, a for attach, s for stop, y for sync"
        exit 0
fi

if [[ ! -d $SHARED_DIR ]]; then
        mkdir -p $SHARED_DIR
fi

CONTAINER_NAME="$USER-rosfooling-$(pwd | sha256sum - | head -c 8)"
HOME=/home/$USER

DRI_DEV=
if ! $USE_NVIDIA && [[ -e /dev/dri ]]; then
        DRI_DEV="--device /dev/dri:/dev/dri"
fi

sync() {
        $EXEC pull $IMAGE
}

up() {
        $EXEC run --rm -d -ti \
            --tmpfs /tmp \
            -e USE_NVIDIA=$USE_NVIDIA \
            $NVIDIA_FLAGS \
            -v $SHARED_DIR:$HOME/ros_shared \
            -v $PWD:$HOME/workspace \
            -v $X11_SOCKET:$X11_SOCKET \
            $DRI_DEV \
            --name $CONTAINER_NAME \
            -h ros-fooling-around \
            $IMAGE
}

down() {
        $EXEC stop $CONTAINER_NAME
}

attach() {
        $EXEC exec -ti \
            -e DISPLAY=$DISPLAY \
            -e DEV_UID=$UID \
            -e DEV_USER=$USER \
            -e USE_NVIDIA=$USE_NVIDIA \
            -e TERM=$TERM \
            $NVIDIA_FLAGS \
            $CONTAINER_NAME \
            /docker/entrypoint.sh
}

if [[ $1 == *"y"* ]]; then
        sync
fi

if [[ $1 == *"u"* ]]; then
        up
elif [[ $1 == *"s"* ]]; then
        down
fi

if [[ $1 == *"a"* ]]; then
        attach
fi
