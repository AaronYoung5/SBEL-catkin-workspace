#!/bin/bash

xhost +

docker run --env='DISPLAY' -v /home/aaron:/home/aaron -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /etc/group:/etc/group:ro -v /etc/passwd:/etc/passwd:ro -v /et    c/shadow:/etc/shadow:ro -v /usr/local/cuda:/usr/local/cuda --runtime=nvidia --network=host --rm --name ros-cuda-docker ros-cuda-docker &
