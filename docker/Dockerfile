FROM ros:humble

ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ="UTC"
ENV ROS_DISTRO=humble
ENV DISPLAY=:0
ENV LIBGL_ALWAYS_INDIRECT=0

RUN apt-get update && apt-get install -yy \
    python3-pip \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
    terminator \
    git \
    gedit \
  && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip

RUN git clone https://github.com/lnotspotl/unitree_sdk2_python.git
WORKDIR /unitree_sdk2_python
RUN pip3 install -e .

RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
