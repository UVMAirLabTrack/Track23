#!/bin/bash
xhost +
​
sudo docker run -it --rm \
--net=host \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-p 9090:9090 \
-p 8888:8888 \
coreraspberry:latest /bin/bash