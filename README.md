# VirtualFastKeyboardFurious-F110-2020
Carla interfacing with F10 S2020 Class

Instructions:

1. Install Carla Server and Client side and install sudo apt-get ros-melodic-pointcloud-to-laserscan
2. Copy src into your workspace or Catkin_make 
3. Run SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl   from your carla client side folder
4. Run roslaunch carla_ros_bridge pointcloud_converter.launch
5. Run roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch

Carla Ros Bridge is built from source and is in the src so need to install it seprately.
