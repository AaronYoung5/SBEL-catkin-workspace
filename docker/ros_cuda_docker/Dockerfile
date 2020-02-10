FROM nvidia/cuda:10.2-cudnn7-devel-ubuntu18.04

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

RUN ln -sfv /home/aaron/packages/flatbuffers/include/flatbuffers /usr/local/include/flatbuffers

RUN apt-get update; apt-get install -y jstest-gtk zsh wget libirrlicht-dev libopenmpi-dev openmpi-bin openmpi-common openmpi-doc libglfw3-dev libgles2-mesa-dev libglew-dev freeglut3-dev mesa-utils glibc-source

RUN wget https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | zsh || true

# RUN git clone https://github.com/tarjoilija/zgen.git "${HOME}/.zgen"

# RUN ln -f /bin/zsh /bin/bash

# RUN /bin/zsh -c  "source "${HOME}/.zgen/zgen.zsh""

RUN ln -s /usr/lib/x86_64-linux-gnu/libGLEW.so.2.0 /usr/lib/x86_64-linux-gnu/libGLEW.so.2.1

RUN ln -s /usr/lib/x86_64-linux-gnu/libmpi_cxx.so.20 /usr/lib/x86_64-linux-gnu/libmpi_cxx.so.40

RUN ln -s /usr/lib/x86_64-linux-gnu/libmpi.so.20 /usr/lib/x86_64-linux-gnu/libmpi.so.40

RUN apt-get update; apt-get install -y libeigen3-dev

# RUN apt-get -qq update && apt-get install -q -y software-properties-common
# RUN add-apt-repository ppa:ubuntu-toolchain-r/test -y
#
# # We pass the boost version argument as argument
# ARG BOOST_VERSION
# ARG BOOST_VERSION_
# ENV BOOST_VERSION=${BOOST_VERSION}
# ENV BOOST_VERSION_=${BOOST_VERSION_}
# ENV BOOST_ROOT=/usr/include/boost
#
# RUN wget --max-redirect 3 https://dl.bintray.com/boostorg/release/1.72.0/source/boost_1_72_0.tar.gz
#
# RUN mkdir -p /usr/include/boost && tar zxf boost_1_72_0.tar.gz -C /usr/include/boost --strip-components=1
#
# RUN echo ${BOOST_ROOT}

WORKDIR /home/aaron

ENV LD_LIBRARY_PATH = $LD_LIBRARY_PATH:/usr/local/cuda/lib64

COPY ./ros_entrypoint.sh /

CMD tail -f /dev/null
