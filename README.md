# Interface ROS 2 - Bus CAN 
Ce dépôt permet d'envoyer/recevoir des trames CAN à partir de topics ros2 pour le tricycle. Un joystick est utilisé pour l'envoi des commandes, ces données sont en amont interpréter en trames CAN avant d'être envoyer sur le bus CAN. 

Réference : https://github.com/ROS4SPACE/ros2can_bridge

## Configuration requise: 
- Linux 
- ROS 2 
- Joystick (manette XBOX-360)
- Convertisseur CAN-USB

## Tester l'inteface avec un bus CAN simulé 
Install can-utils :             
> $ sudo apt-get update                  
  $ sudo apt-get -y install can-utils

Créer une interface CAN virtuelle:                  
> $ modprobe vcan             
  $ sudo ip link add dev vcan0 type vcan               
  $ sudo ip link set up vcan0            

**Note:** modprobe est nécessaire dans le cas où le pilote n'est toujours pas chargé.

Créer un Fork du le réferenciel, puis clonner et exécuter : 
> $ git clone URL_réferenciel
  $ cd ~/ros2can_bridge
  $ colcon build
  $ source install/setup.bash
  $ ros2can_bridge

