FROM osrf/ros:melodic-desktop

RUN apt-get update; apt-get install -y sudo vim build-essential xterm x11-xserver-utils mesa-utils ros-melodic-pcl-ros cmake-curses-gui ccache python3-yaml python3-numpy python3-matplotlib python-rospkg ros-melodic-graph-msgs 

RUN ln -sfv /home/aaron/flatbuffers/include/flatbuffers /usr/local/include/flatbuffers

WORKDIR /home/aaron

CMD tail -f /dev/null
