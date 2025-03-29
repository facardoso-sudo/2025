## Configuration of the system used:
Ubuntu 18.04.4 (http://releases.ubuntu.com/18.04.4/)
ROS Melodic (http://wiki.ros.org/melodic/Installation/Ubuntu)

## Step by step configuration:
```
<1 - Copy the following packages "ar_tag_toolbox", "rviz_textured_quads-image_topic", "rrt-master", "controle_posicao_3D", to your workspace, in catkin_ws/src>
<2 - Using the linux shell script navigate to /catkin_ws and compile the packages using the catkin_make command>
<3 - Go to the src folder and rename "stingelin_SBAI.cpp" or "stingelin_ROBOVIS.cpp" to "stingelin.cpp",>
```
## Step by step for execution
#### Terminal 1
```
<roslaunch ar_tag_toolbox usb_cam.launch>
```
#### Terminal 2
```
<roslaunch ar_tag_toolbox ar_track_usb_cam.launch>
```
#### Terminal 3
```
<roslaunch rviz_textured_quads demo.launch>
```
#### Terminal 4
```
<rosrun rrt rrt>
```
#### Terminal 5
```
<rosrun controle_posicao_3D stingelin>
```
