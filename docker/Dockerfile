FROM osrf/ros:melodic-desktop

RUN apt-get update; apt-get install -y sudo vim xterm x11-xserver-utils mesa-utils ros-melodic-pcl-ros cmake-curses-gui ccache python3-yaml python3-numpy python3-matplotlib python-rospkg ros-melodic-graph-msgs 

RUN ln -sfv /home/aaron/flatbuffers/include/flatbuffers /usr/local/include/flatbuffers

WORKDIR /home/aaron

EXPOSE 8080
EXPOSE 11311

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

CMD tail -f /dev/null
