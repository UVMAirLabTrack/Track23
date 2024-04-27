#!/bin/bash
set -x #echo on
sudo su
sudo apt-get -y update
sudo apt-get -y upgrade
sudo apt-get install -y net-tools
sudo apt-get install -y nmap
sudo apt-get install -y terminator
sudo apt-get install -y xfce4-goodies
sudo apt-get install -y openssh-server
sudo apt-get install -y git
sudo apt-get install -y python3-pip
echo installing Aptitude
sudo apt-get install aptitude -y
echo  installing Terminator
sudo apt-get install terminator -y
echo Installing Blender
sudo apt-get install blender -y
echo Installing notepadqq
sudo apt-get install notepadqq -y
echo installing nautilus
sudo apt-get install nautilus -y
echo installing openssh server
sudo apt-get install openssh-server -y	
echo installing timeshift
sudo add-apt-repository -y ppa:teejee2008/ppa
sudo apt-get install timeshift -y
echo installing x11VNC
sudo apt-get install x11vnc -y
echo installing python packages
sudo pip3 install pyserial
sudo pip3 install numpy
pip3 install pyserialtransfer