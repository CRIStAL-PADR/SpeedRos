# SpeedRos
An API to control speed devices using ROS

SpeedRos offers us the possibility to control miniature devices like Faller (c) Crane, DCC trains or Switches using ROS

Software requirements
---------------------
 * Ubuntu mate
 * Python >= 3.6
 * `bitstring` module [details](https://pypi.python.org/pypi/bitstring/3.1.3). Should be auto-fetched when installing with pip.
 * `wiringPi`: download and install [wiringPi](http://wiringpi.com/download-and-install/)
 * `Speedlib`: download and install [Speedlib](https://cristal-padrspeed.readthedocs.io/en/latest/documentation.html#installation)
 * Since `wiringPi` uses low-level mechanisms to access pins, dccpi programs **must be run as root**
 * `ROS` : Install [ROS](http://wiki.ros.org/ROS/Installation)


Hardware requirements
---------------------
 * Same as for speedlib see : [speedlib Hardware requirements](https://cristal-padrspeed.readthedocs.io/en/latest/documentation.html#hardware-requirements)


Prerequisite
------------
 * Knowing how to use ROS

Examples
^^^^^^^^ 
- Before starting, it is imperative to create a local working directory in which to clone the remote repository of [SpeedROS](https://github.com/CRIStAL-PADR/SpeedRos) .
- Once it's done, you have to build the packages in the SpeedRos workspace :
    $ cd ~/SpeedRos
    $ catkin_make
Once the workspace was built, it created a similar structure in the devel subfolder which you usually find under / opt / ros / $ ROSDISTRO_NAME



Controlling a Faller (c) crane model using ROS
----------------------------------------------


