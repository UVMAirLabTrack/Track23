xhost+

docker run -it --rm \
--net=host \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix:rm \
--device=/dev/dri:/dev/dri \
--security-opt apparmor:unconfined \
raspberrycore:latest