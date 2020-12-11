
# Originally Forked from

https://github.com/akshaydr/ros-mobile-robot

# ros-mobile-robot-camera
Biulding on Mobile Robot code by adding a moving stereo camera and imu
 
## Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/kumarhans/ros-mobile-robot-camera.git
	cd ../
	catkin_make


## Usage

Run the main simulator node with

	roslaunch camera_gazebo moving_camera.launch
  
 

![Hallway Gif](/readme_images/hallway.gif)
 
