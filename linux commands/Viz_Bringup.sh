#!/bin/bash
xhost +

sudo docker run -it  \
-e DISPLAY=$DISPLAY \
--env="QT_X11_NO_MITSHM=1" \
--name=viz_image \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /dev/shm:/dev/shm \
--security-opt apparmor:unconfined \
-p 9090:9090 \
-p 8888:8888 \
raspberrycore:latest /bin/bash
