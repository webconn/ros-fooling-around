#!/bin/bash

set -e

export DEV_UID=${DEV_UID:-1000}
DEV_USER=${DEV_USER:-user}
DEV_GID=${DEV_GID:-$DEV_UID}
DEV_GROUP=${DEV_GROUP:-$DEV_USER}

# add group and user
if ! getent group "$DEV_GROUP" >& /dev/null; then
    addgroup --gid "$DEV_GID" "$DEV_GROUP" >& /dev/null
fi

if ! getent passwd "$DEV_USER" >& /dev/null; then
    adduser --uid "$DEV_UID" --gecos "" --gid "$DEV_GID" --disabled-password --home "/home/$DEV_USER" "$DEV_USER" >& /dev/null
    mkdir -p "/home/$DEV_USER" && chown "$DEV_USER:$DEV_GROUP" "/home/$DEV_USER"
fi

#configure sudo
SUDOERS_FILE="/etc/sudoers.d/${DEV_USER}_docker"
if [[ ! -f $SUDOERS_FILE ]]; then
    echo "$DEV_USER ALL=(ALL) NOPASSWD: ALL" > $SUDOERS_FILE
fi

SUDO_VARS=

if $USE_NVIDIA; then
    echo "Using nvidia backend"
    SUDO_VARS="USE_NVIDIA=true ${SUDO_VARS}"
    SUDO_VARS="PATH=/usr/local/nvidia/bin:${PATH} ${SUDO_VARS}"
    SUDO_VARS="LD_LIBRARY_PATH=/usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH} ${SUDO_VARS}"
fi

exec sudo -E -i -u $DEV_USER $SUDO_VARS /bin/bash --rcfile /docker/bashrc
