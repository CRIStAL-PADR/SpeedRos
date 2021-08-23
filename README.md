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


Building a SpeedRos workspace and sourcing the setup file
-------------------------------------------------------
 * Knowing how to use ROS

Before starting, it is imperative to create a local working directory in which to clone the remote repository of [SpeedROS](https://github.com/CRIStAL-PADR/SpeedRos) .
Once it's done, you have to build the packages in the SpeedRos workspace :
 * $ cd ~/.../SpeedRos
 * $ catkin_make
Once the workspace was built, it created a similar structure in the devel subfolder which you usually find under / opt / ros / $ ROSDISTRO_NAME

To add the workspace to your ROS environment you need to source the generated setup file:
 * $ source ~/.../SpeedRos/devel/setup.bash

It should be noted that this will only work for cranes. In order to correctly source the generated setup file, because of the *wiringPi* used to control the trains and the switches, you must be in sudo :
 * $ sudo su

Once it's done we can now control our devices

Controlling a Faller (c) crane model using ROS
----------------------------------------------
You must first check that you are connected to the faller's wifi.

Open a terminal and run the following command: 
 * roscore

Open a second terminal and run the following command: 
 * rosrun crane crane_pilotpy "172.17.217.217"

Open a third terminal and run the following command: 
 * rostopic pub /crane/command std_msgs/String " data : ''"

### Example
Pour méthode start_for voici la commande : 
 >>> rostopic pub /crane/command std_msgs/String " data : ' crane_command : start_for; value : 5; motors_name : MotorChassis; motors_direction : MotorDirectionForward'"

Pour la méthode set_speed voici la commande : 
 >>> rostopic pub /crane/command std_msgs/String " data : ' crane_command : set_speed; speed_value : 5; motors_name : MotorChassis'"

# Controlling a DCC train and switch model

You must first be an administrator to be able to control the train or the switch because of the *wiringPiSetup*
 * sudo su

Il est également indispendable de sourcer le setup file (see *Building a SpeedRos workspace and sourcing the setup file*)

## Train
Open a terminal and run the following command: 
 * roscore

Open a second terminal and run the following command: 
 * rosrun train train_pilotpy 8 3

The first parameter is the number of train that we want to initialize.
The second parameter designates the address or number of the first train to be initialized

Open a third terminal and run the following command: 
 * rostopic pub /train/command std_msgs/String " data : ''"