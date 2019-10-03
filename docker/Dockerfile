FROM osrf/ros:melodic-desktop

RUN apt-get update; apt-get install -y sudo vim xterm x11-xserver-utils mesa-utils ros-melodic-serial ros-melodic-pcl-ros cmake-curses-gui ccache ros-melodic-eigen-stl-containers ros-melodic-graph-msgs ros-melodic-image-geometry ros-melodic-navigation python3-yaml python3-numpy python3-matplotlib mjpegtools ffmpeg mencoder ros-melodic-image-view

RUN apt-get update; apt-get install -y /home/aaron/Downloads/sdkmanager_0.9.14-4961_amd64.deb

#RUN useradd -G sudo -u 1001 -m -s /bin/bash aaron
#RUN echo "aaron:password" | chpasswd

RUN ln -sfv /home/aaron/flatbuffers/include/flatbuffers /usr/local/include/flatbuffers

#USER aaron
WORKDIR /home/aaron

EXPOSE 8080
EXPOSE 11311

CMD tail -f /dev/null
