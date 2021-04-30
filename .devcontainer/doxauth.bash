#!/bin/bash -u
XAUTH=/tmp/.docker.xauth
if [ -d "$XAUTH" ]; then
    echo "Authority file exists, delete it first\n"
    sudo rm -rf $XAUTH
fi
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -