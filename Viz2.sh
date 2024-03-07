xhost+

sudo docker run -it --rm \
--net=host \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
--device=/dev/dri:/dev/dri \
--security-opt apparmor:unconfined \
-p 9090:9090 \
-p 8888:8888 \
-p 11811:11811 \
-p 11812:11812 \
-p 7412:7412 \
-p 7413:7413 \
-p 5900:5900 \
-p 5901:5901 \
raspberrycore:latest /bin/bash