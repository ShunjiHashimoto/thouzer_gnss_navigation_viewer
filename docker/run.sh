#!/bin/bash

DOCKER_IMAGE="gnss-docker:latest"
CONTAINER_NAME="gnss-docker"

docker run --rm -it \
    --env="DISPLAY=$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --volume="/home/ubuntu/ros2_ws:/root/ros2_ws" \
    --name=$CONTAINER_NAME \
    --privileged \
    --net=host \
    $DOCKER_IMAGE \
    bash
