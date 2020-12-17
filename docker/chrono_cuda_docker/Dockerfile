FROM nvidia/cuda:10.1-cudnn7-devel-ubuntu18.04

RUN apt-get update; apt-get install -y  sudo vim build-essential xterm x11-xserver-utils mesa-utils \
                                        cmake-curses-gui ccache libirrlicht-dev git libeigen3-dev \
                                        libglfw3-dev libgles2-mesa-dev libglew-dev freeglut3-dev

WORKDIR /home/aaron
