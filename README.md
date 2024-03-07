# UVM SEED 2023 Track
 
Repository for code and setup files associated with the 2023 UVM SEED Project for an urban track for autonomous cars

In this repo there are several different elements in place.
Under interactive elements are the code files to drive the ESP32 boards, and the PCB's used to make them.

CoreRaspbery is the ROS2 Workspace, at present the code is designed to build into a Humble distro, however in testing the code can alos be built for a Foxy distro.

Setup Files contains some basic files to aid in deploying the Raspberry Pi track computer

Car commands is a curated list of the terminal commands needed to operate specific functions of the Yahboom R2 car
Also contained within are some quality of life improvements to the deployment of the docker environment on the car, just some simple .sh files.

the Dockerfile in the main directory is intended to build a docker container to run all the ROS code, this is not intended for deployment on the PI as of this moment, instead it is intended as a complete package for the vizualizer nodes. At present, windows communication via WiFi is not functional.
