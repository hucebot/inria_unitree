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
    ros-${ROS_DISTRO}-pcl-conversions \
    libpcl-dev \
    terminator \
    git \
    gedit \
  && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip

RUN git clone https://github.com/lnotspotl/unitree_sdk2_python.git
WORKDIR /unitree_sdk2_python
RUN pip3 install -e .

RUN mkdir -p /ros2_ws/src
RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git /ros2_ws/src/livox_ros_driver2

WORKDIR /
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && mkdir build && cd build && \
    cmake .. && make -j && make install

WORKDIR /ros2_ws/src/livox_ros_driver2
RUN rm -f package.xml && \
    cp package_ROS2.xml package.xml && \
    cp -rf launch_ROS2 launch

WORKDIR /ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble

RUN pip install pyqt5 pinocchio
RUN apt install ros-$ROS_DISTRO-pinocchio

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

RUN echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" >> /root/.bashrc

CMD ["bash"]
