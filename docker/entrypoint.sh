#!/bin/bash

export DEV_UID=${DEV_UID:-1000}
DEV_USER=${DEV_USER:-user}
DEV_GID=${DEV_GID:-$DEV_UID}
DEV_GROUP=${DEV_GROUP:-$DEV_USER}

# add group and user
if ! getent group "$DEV_GROUP" >& /dev/null; then
    addgroup --gid "$DEV_GID" "$DEV_GROUP" >& /dev/null
fi

if ! getent passwd "$DEV_USER" >& /dev/null; then
    adduser --uid "$DEV_UID" --gecos "" --gid "$DEV_GID" --disabled-password --home "/userdata" "$DEV_USER" >& /dev/null
fi

#configure sudo
SUDOERS_FILE="/etc/sudoers.d/${DEV_USER}_docker"
if [[ ! -f $SUDOERS_FILE ]]; then
    echo "$DEV_USER ALL=(ALL) NOPASSWD: ALL" > $SUDOERS_FILE
fi

exec sudo -E -i -u $DEV_USER /bin/bash --rcfile /docker/bashrc
