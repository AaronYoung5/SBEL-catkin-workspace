FROM nvidia/cuda:10.1-cudnn7-devel-ubuntu18.04

# ROS melodic core
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
RUN rosdep init \
    && rosdep update
ENV ROS_DISTRO melodic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-get update && apt-get install -y \
    ros-melodic-ros-core=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

# ROS base
RUN apt-get update && apt-get install -y \
    ros-melodic-ros-base=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

# ROS robot
RUN apt-get update && apt-get install -y \
    ros-melodic-robot=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

# ROS desktop
RUN apt-get update && apt-get install -y \
    ros-melodic-desktop=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

# ROS desktop full
# RUN apt-get update && apt-get install -y \
#     ros-melodic-desktop-full=1.3.1-0* \
#     && rm -rf /var/lib/apt/lists/*

# RUN mkdir -p /home/catkin_ws/src
# RUN apt-get update && apt-get -y install build-essential \
    # git wget nano v4l-utils

RUN apt-get update; apt-get install -y sudo vim build-essential xterm x11-xserver-utils mesa-utils ros-melodic-pcl-ros cmake-curses-gui ccache python3-yaml python3-numpy python3-matplotlib python-rospkg ros-melodic-graph-msgs

RUN ln -sfv /home/aaron/flatbuffers/include/flatbuffers /usr/local/include/flatbuffers

# WORKDIR /home/aaron/Downloads/darknet/
# COPY ./Makefile ./
# RUN \
# 	sed -i 's/GPU=.*/GPU=1/' Makefile && \
# 	make
# RUN rm /home/aaron/ros/synchrono_ws/src/01_perception/yolo_v3/nodes/libdarknet.so
# RUN cp libdarknet.so /home/aaron/ros/synchrono_ws/src/01_perception/yolo_v3/nodes/

# RUN apt-get update; apt-get install -y python-pip python-dev; pip install --upgrade pip; pip install moviepy

RUN apt-get update; apt-get install -y ros-melodic-movie-publisher python-pip; pip install moviepy

WORKDIR /home/aaron

ENV LD_LIBRARY_PATH = $LD_LIBRARY_PATH:/usr/local/cuda/lib64

COPY ./ros_entrypoint.sh /
# ENTRYPOINT ["/ros_entrypoint.sh"]
CMD tail -f /dev/null
