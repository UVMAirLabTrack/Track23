#Base image
FROM osrf/ros:humble-desktop-full

# working directory
ENV HOME /root
ENV DISPLAY=:0
WORKDIR $HOME

RUN RUN apt-get update && apt-get install -y \
x11-apps \
xauth

#Could also utilize a Git Pull, but may need to separate the Repo for lightweight building, this seems better
COPY CoreRaspberry $HOME/CoreRaspberry

#switch to workspace
WORKDIR $HOME/CoreRaspberry

RUN colcon build --symlink-install

RUN echo ' \n\
echo "Sourcing ROS2 packages..." \n\
source $HOME/CoreRaspberry/install/local_setup.sh' >> $HOME/.bashrc
