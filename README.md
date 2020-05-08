# VirtualFastKeyboardFurious-F110-2020
Carla interfacing with F10 S2020 Class

Instructions:

Installation:

1. Install Carla Server and Client side
2. Install sudo apt-get ros-melodic-pointcloud-to-laserscan

Execution:

1. Copy src into your workspace
2. Catkin_make
3. cd into your carla client side folder
4. Run SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl

For Wall Following\
5. Run roslaunch wall_following wall_following_sim.launch

Carla Ros Bridge is built from source and is in the src so need to install it seprately.
