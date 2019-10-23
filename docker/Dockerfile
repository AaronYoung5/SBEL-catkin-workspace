FROM osrf/ros:melodic-desktop

RUN apt-get update; apt-get install -y sudo vim build-essential xterm x11-xserver-utils mesa-utils ros-melodic-pcl-ros cmake-curses-gui ccache python3-yaml python3-numpy python3-matplotlib python-rospkg ros-melodic-graph-msgs 

RUN ln -sfv /home/aaron/flatbuffers/include/flatbuffers /usr/local/include/flatbuffers

WORKDIR /home/aaron

ENV PATH /usr/local/cuda/bin/:$PATH
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/local/cuda/lib64/stubs
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
LABEL com.nvidia.volumes.needed="nvidia_driver"

CMD tail -f /dev/null
