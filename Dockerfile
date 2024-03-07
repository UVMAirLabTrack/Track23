#Base image
FROM osrf/ros:foxy-desktop

# working directory
ENV HOME /root
WORKDIR $HOME

#Could also utilize a Git Pull, but may need to separate the Repo for lightweight building, this seems better
COPY CoreRaspberry $HOME/CoreRaspberry

# general utilities
RUN apt-get update && apt-get install -y \
    wget \
    curl \
    git \
    vim \
    nano \
    python-dev \
    python3-pip \
    ipython

# pip
RUN pip3 install --upgrade pip

# Locale options
RUN apt-get install -y locales
RUN locale-gen en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

#switch to workspace
WORKDIR $HOME/CoreRaspberry

RUN colcon build --symlink-install

RUN echo ' \n\
echo "Sourcing ROS2 packages..." \n\
source $HOME/CoreRaspberry/install/local_setup.sh' >> $HOME/.bashrc
