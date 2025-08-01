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
        -v `pwd`/../:/ros2_ws/src/g1pilot \
        -v `pwd`/../../g1_description:/ros2_ws/src/g1_description \
        -v `pwd`/../config/livox_mid.json:/ros2_ws/src/livox_ros_driver2/config/MID360_config.json  \
        -w /ros2_ws \
        g1pilot:latest