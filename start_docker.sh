WORK_WS=/root/catkin_ws/src/FAST-Calib
DOCKERIMAGE="fast_calib:latest"
xhost +
CURRENT_DIR=$(pwd)
docker run -it --rm --runtime=nvidia --gpus all  --net=host -v ${CURRENT_DIR}:${WORK_WS} \
    -v /dev/:/dev/ --privileged -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix  --name="fast_calib" ${DOCKERIMAGE} /bin/bash 