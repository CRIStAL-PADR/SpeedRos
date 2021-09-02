# Programme de control des robots mobiles T-quads

## Configurations
* Ubuntu mate 16.04
* ros kinetic
* Python 2.7
## Installations
### Installation de Ubuntu mate
Télécharger ubuntu mate 16.04 et l'installer sur la carte mémoire de la raspberry pi.

Lien de téléchargement : [ici](https://releases.ubuntu-mate.org/archived/16.04/)
### Installation de ROS kinetic
Suivre le tutoriel suivant pour installer ROS.

Tutoriel d'installation : [ici](http://wiki.ros.org/kinetic/Installation/Ubuntu)

### Installations des packages additionnels ROS (*Rosbridge* et *Rosserial*)

    sudo apt-get install ros-kinetic-rosbridge-server
    sudo apt-get install ros-kinetic-rosserial-arduino
    
### Installation du paquet tquad ros
Créer et initialiser un espace de travail ros sur votre bureau

    mkdir ros_workspace/src
    cd ros_workspace/src
    catkin_make

Cloner le répertoire github

    https://github.com/sarifou/AGV-Robot.git

Copier-coller le package tquad du répertoire ros_package vers le src de votre espace de travail catkin et construiser à nouveau l'espace de travail : 

    catkin_make

### Installation du firmeware pour l'arduino méga.

Avec le logiciel arduino, compiler le fichier firmeware.ino dans l'arduino mega du tquad

## Liste des topics

## Utilisation
Pour contrôler le tquad avec un client rosbridge

    roslaunch tquad tquad_bridge.launch

Pour contrôler le tquad avec le teleop_key

    roslaunch tquad tquad_teleop_key.launch


