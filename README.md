# UVM SEED 2023 Track
 
## Repository for code and setup files associated with the 2023 UVM SEED Project for an urban track for autonomous cars

## File Structure

### A. Documentation
Look in here to find the detailed instructions on how to build and run the track system. As well as more in depth information as to the system's function

- 
- 
- 
- 

### 3D modelling

Under this folder are organized locations for all the .stl files used to create the various printed elements. 

### Interactive Elements

Here you will find the Arduino IDE code files used for the ESP32 interactive element network.
Additionally Kicad format PCB's and schematics are located here.
The receiver for each interactive element utilizes the same PCB, with 4 RJ45 connectors and a single ESP32 Dev Module with 38 pins (WROOM- D or E)
The only change is which code file is flashed. 

Some previous versions utilizing pure serial connections have been left for adapability. The latest track build utilizes:

1. Stoplight
    - ESPnow_4way_4int.ino  
    - ESP_now_3Way.ino
2. Train Crossing
    - ESPNowTrain_Complete.ino
3. Sender
    - ESPnowSender.ino

With the sender node being connected via serial to the computer controlling the track, In this case a Raspberry Pi 4B 8GB model.

### CoreRaspbery 
is the ROS2 Workspace,

at present the code is designed to build into a Humble distro, however in testing the code can also be built for a Foxy distro.

### Setup Files 
contains some basic files to aid in deploying the Raspberry Pi track computer

### Car commands 
is a curated list of the terminal commands needed to operate specific functions of the Yahboom R2 car
Also contained within are some quality of life improvements to the deployment of the docker environment on the car, just some simple .sh files.

### Other Code

A series of files not directly part of the track functionality. Mostly a series of HSV decoders and Webcam Capture files 

### files not in a folder

the Dockerfiles in the main directory are intended to build a docker container to run all the ROS code, this is not intended for deployment on the PI as of this moment, instead it is intended as a complete package for the vizualizer node(s). At present, windows communication via WiFi is not functional due to additional networking constraints.
