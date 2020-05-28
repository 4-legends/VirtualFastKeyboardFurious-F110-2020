# VirtualFastKeyboardFurious-F110-2020
Carla interfacing with F10 S2020 Class

Instructions:

Installation:

Carla Ros Bridge is built from source and is in the src so need to install it seprately.

1. [Install Carla Server](https://carla.readthedocs.io/en/latest/start_quickstart/) and [Client side](https://carla.readthedocs.io/en/latest/build_linux/)
2. Install  <br /> 
	sudo apt-get ros-melodic-pointcloud-to-laserscan

Execution:

1. Copy src into your workspace
2. Catkin_make
3. cd into your carla client side folder
4. #To run Server Side \
	SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl


Directory level Description:

1. Docs: Carla Integration Plan- Document describing work breakup
2. Docs: Videos - Carla1 and Carla2 are the video demonstrating vehicle pop up on the server side and open camera feed.
3. Docs: Carla Lab Submission: Report for carla lab which gives more information about getting carla setup and some pitfalls and how to avoid them, it also explains the carla_lane_finding package. 
4. src: ros-bridge: Ros bridge package from github edited some files to work with our carla plan. No need to install it for user as this is already cloned for source and this works.
5. carla_lane_finding package: This package takes camera input and does lane centering of the car.
