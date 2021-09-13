# T-Quad Installation Guide

## Configurations
* Ubuntu mate 16.04
* ros kinetic
* Python 2.7
## Installations

### Ubuntu mate 16.04
Download ubuntu mate 16.04 and install it on the memory card of the Tquad raspberry pi

Download link : [Ubuntu mate 16.04](https://releases.ubuntu-mate.org/archived/16.04/)

### ROS kinetic
Follow the following tutorial to install ROS.

Installation tutorial : [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

### ROS additional packages

Open a terminal and run the following commands to install the required ROS packages

    sudo apt-get install ros-kinetic-usb-cam
    sudo apt-get install ros-kinetic-rosbridge-server
    sudo apt-get install ros-kinetic-rosserial-arduino
    sudo apt-get install ros-kinetic-web-video-server
    sudo apt-get install ros-kinetic-teleop-twist-keyboard
    
### ROS T-Quad package
Before downloading the ros tquad package, you must create a ros workspace. Once done copy and paste the tquad package into the src folder of your workspace and build it again with catkin :

    catkin_make




