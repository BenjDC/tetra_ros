# TetraROS

tetraROS est une implémentation utilisant ROS du robot à transmission différentielle "Tetra" de Carre92. Le but est d'utiliser [la pile de navigation ROS](http://wiki.ros.org/navigation) pour le faire naviguer en SLAM de façon Autonome.

![Navigation ROS](./ressources/overview_tf.png)


## Architecture

L'architecture de tetraROS est basée sur les éléments suivants :

* Une carte à base de STM32F4, qui pilote les deux moteurs et remote les informations d'odometrie. L'API rosserial est utilisée pour communiquer via l'USART avec un Raspberry Pi
* Un Raspberry Pi 3, qui exécute le Coeur ROS, et l'algorithme SLAM.
* Un PC vers lequel est remonté la télémétrie, et les commandes de contrôle via un gamepad (testé avec une manette DualShock3)

## Dépendances

tetraROS utilise les paquets existants suivants

* **rosserial_python** : communication ROS entre RPi et STM32
* **joy** : gestion du gamepad
