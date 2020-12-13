#!/bin/bash
docker exec --user "anynet_docker" -it anynet-ros \
    /bin/bash -c "source /opt/ros/noetic/setup.bash; cd /home/anynet_docker; /bin/bash"
