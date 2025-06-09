# For graphics
xhost +local:docker

docker run \
        -it \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=0" \
        --net host \
        --privileged \
        -v /dev:/dev \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        -v `pwd`/../:/ros2_ws/src/inria_unitree \
        -v `pwd`/../../g1_description:/ros2_ws/src/g1_description \
        -w /ros2_ws \
        inria_unitree:latest