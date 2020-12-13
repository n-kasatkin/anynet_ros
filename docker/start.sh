#!/bin/bash
docker run -itd --rm \
    --ipc host \
    --gpus all \
    -v $(pwd)/../:/home/anynet_docker/catkin_ws/src/anynet_ros:rw \
    --name anynet-ros \
    x64noetic/anynet-ros:latest

