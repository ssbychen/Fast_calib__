FROM ros:noetic-perception

RUN apt-get update && apt-get install -y \
    ros-noetic-rviz \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/catkin_ws/src

COPY . ./FAST-Calib

WORKDIR /root/catkin_ws

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
