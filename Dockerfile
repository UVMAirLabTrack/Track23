#Base image
#Hey, Got problems with not seeing the network and/or xcb plugins. TRY SUDO!!!
FROM osrf/ros:humble-desktop-full

EXPOSE 9090 
EXPOSE 11811
EXPOSE 11812
EXPOSE 7412
EXPOSE 7413
EXPOSE 5900
EXPOSE 5901
EXPOSE 8888
EXPOSE 6000

# working directory
ENV HOME /root
ENV DISPLAY=:0
ENV QT_QPA_PLATFORM=xcb
ENV ROS2_WS  /root/CoreRaspberry

WORKDIR $HOME

RUN mkdir -p ${ROS2_WS}

RUN apt-get update && apt-get install -q -y --no-install-recommends \
x11-apps \
xauth \
libxcb1 \
libxcb1-dev \
xvfb \
&& rm -rf /var/lib/apt/lists/*

#Could also utilize a Git Pull, but may need to separate the Repo for lightweight building, this seems better for now
#COPY CoreRaspberry $HOME/CoreRaspberry
#modded copy to bypass a built package
COPY CoreRaspberry/src $HOME/CoreRaspberry/src
COPY CoreRaspberry/worlds $HOME/CoreRaspberry/worlds
COPY CoreRaspberry/control $HOME/CoreRaspberry/control

#switch to workspace
WORKDIR $HOME/CoreRaspberry
#comment this out to copy from a built set


RUN echo ' \n\
echo "Sourcing ROS2 packages..." \n\
source $HOME/CoreRaspberry/install/local_setup.sh' >> $HOME/.bashrc


RUN cd ${ROS2_WS} \
    && . /opt/ros/humble/setup.sh \ 
    && colcon build --symlink-install

#comment out below to build in windows, still working on the networking side but gui apps are good.
CMD xhost + && source $HOME/CoreRaspberry/install/local_setup.sh
#CMD source $HOME/CoreRaspberry/install/local_setup.sh