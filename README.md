
# SpeedRos

An API to control speed devices using ROS

SpeedRos offers us the possibility to control miniature devices like Faller (c)
Crane, DCC trains or Switches using ROS

Software requirements
---------------------

* Ubuntu mate
* Python >= 3.6
* `bitstring` module [details](https://pypi.python.org/pypi/bitstring/3.1.3). Should be auto-fetched when installing with pip.
* `wiringPi`: download and install [wiringPi](http://wiringpi.com/download-and-install/)
* `Speedlib`: download and install [Speedlib](https://cristal-padrspeed.readthedocs.io/en/latest/documentation.html#installation)
* Since `wiringPi` uses low-level mechanisms to access pins,
dccpi programs **must be run as root**
* `ROS` : Install [ROS](http://wiki.ros.org/ROS/Installation)

Hardware requirements
---------------------

* Same for speedlib see : [speedlib Hardware requirements](https://cristal-padrspeed.readthedocs.io/en/latest/documentation.html#hardware-requirements)
* Added to that we will need a Tquad

Building a SpeedRos workspace and sourcing the setup file
-------------------------------------------------------

* Knowing how to use ROS

Before starting, it is imperative to create a local working directory in which
to clone the remote repository of [SpeedROS](https://github.com/CRIStAL-PADR/SpeedRos).

Once this is done, the tquad directory must be moved to
the raspberry which is on the tquad.
This means that we will have two working repertoires. The working directory
**SpeedRos** which will contain the **train**, **switch** and
**crane** and a second directory that we will have to move in the raspberry and
which will contain the **tquad**


:warning: **This step must be respected otherwise the `catkin_make` command will**
**generate errors.**: Be very careful here!

Once it's done, you have to build the packages in the SpeedRos workspace :

* $ cd ~/../SpeedRos
* $ `catkin_make`

Once the workspace was built, it created a similar structure in the devel
subfolder which you usually find under / opt / ros / $ ROSDISTRO_NAME

To add the workspace to your ROS environment you need to source the generated
setup file:

* $ source ~/.../SpeedRos/devel/setup.bash

It should be noted that this will only work for cranes. In order to correctly
source the generated setup file, because of the *wiringPi* used to control the
trains and the switches, you must be in sudo :

* $ sudo su

Once it's done we can now control our devices

## Controlling a Faller (c) crane model using ROS

You must first check that you are connected to the faller's wifi.

Open a terminal and run the following command: 
 * roscore

Open a second terminal and run the following command: 
 * rosrun crane crane_pilotpy "172.17.217.217"

Open a third terminal and run the following command: 
 * rostopic pub /crane/command std_msgs/String " data : '' "

### Example
For the start_for method here is the command:
 >>> rostopic pub /crane/command std_msgs/String " data : ' crane_command : start_for; value : 5; motors_name : MotorChassis; motors_direction : MotorDirectionForward' "

For the set_speed method here is the command: : 
 >>> rostopic pub /crane/command std_msgs/String " data : ' crane_command : set_speed; speed_value : 5; motors_name : MotorChassis' "

## Controlling a DCC train and switch model

You must first be an administrator to be able to control the train or the switch because of the *wiringPiSetup*
 * sudo su

Il est également indispendable de sourcer le setup file (see *Building a SpeedRos workspace and sourcing the setup file*)

### Train
Open a terminal and run the following command: 
 * roscore

Open a second terminal and run the following command: 
 * rosrun train train_pilotpy 8 3

The first parameter is the number of train that we want to initialize.
The second parameter designates the address or number of the first train to be initialized

Open a third terminal and run the following command: 
 * rostopic pub /train/command std_msgs/String " data : '' "

#### Example
For the faster method here is the command:
 >>> rostopic pub /train/command std_msgs/String " data : 'train_command : faster; train_number : 3' "

For the speed methode here is the command : 
 >>> rostopic pub /train/command std_msgs/String " data : 'train_command : speed; train_number : 5; speed_value : 15' "

For the fl methode here is the command:
 >>> rostopic pub /train/command std_msgs/String " data : 'train_command : fl; train_number : 5; accessories_value : True' "

### Switch
Open a terminal and run the following command: 
 * roscore

Open a second terminal and run the following command: 
 * rosrun switch switch_pilot.py 8 3

The first parameter is the number of switch that we want to initialize.
The second parameter designates the address or number of the first switch to be initialized

Open a third terminal and run the following command: 
 * rostopic pub /switch/command std_msgs/String " data : '' "

#### Example

For the biais method here is the command:
 >>> rostopic pub /switch/command std_msgs/String " data : 'switch_command : biais; switch_number : 6; biais_id : 1; biais_state : True' "

To print information about the switch her is the command :

 >>> rostopic pub /switch/command std_msgs/String " data : 'switch_command : biais_info; switch_number : 6' "

## Controlling T-Quad using ROS

You must install and configure the T-quad beforehand. To do this, follow the installation guide:

Installation Guide : [Installation guid](https://github.com/CRIStAL-PADR/SpeedRos/blob/master/src/tquad/Installation_Guide.md)

If you want to control the T-quad with the keyboard keys, open a terminal and run the following command :

    roslaunch tquad tquad_teleop.launch

If you want to control the tquad with a rosbridge client, open a terminal and run the following command :

    roslaunch tquad tquad_bridge.launch

## For the debugging
Usually we can use the tab to help us enter the ros command lines.
However, sometimes the tab does not work. This can be due to 2 potential errors:
* the node file is not an executable. To correct this, just write in the terminal: *chmod + x file.py*
* the Setup file is not well sourced.

We can also use a debugging command to find out if the ros node that we have launched has been properly initialized
* roswtf

This command allows you to know which ros nodes are running on the machine.

