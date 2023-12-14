# ioc_ur_cb2_driver_ros

Repository for the UR CB2 Driver ROS layer. The communication with UR CB2 is mainly managed by the ur_driver class, which is wrapped by the ioc_ur_cb2_driver_ros that incorporates it to the ROS framework.

## Install and build the package
First create a workspace:
```
mkdir -p ws_arm/src
cd ws_arm/src
``` 
It is necessary to clone the following repositories:
```
git clone https://gitioc.upc.edu/robots/ioc_ur_cb2_driver_ros.git
git clone https://gitioc.upc.edu/robots/ioc_ur_cb2_driver.git -b devel
```
And have installed:
```
sudo apt install ros-humble-xacro
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-controller-manager
sudo apt install ros-humble-ur-description
```
Finally, at the root of your ROS 2 workspace, build using:
```
colcon build --symlink-install
``` 
