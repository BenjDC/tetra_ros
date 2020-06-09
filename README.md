# TetraROS

tetraROS est une implémentation utilisant ROS du robot à transmission différentielle "Tetra" de Carre92. Le but est d'utiliser [la pile de navigation ROS](http://wiki.ros.org/navigation) pour le faire naviguer en SLAM de façon Autonome.

![Navigation ROS](./ressources/overview_tf.png)


## Répartition

tetraROS est basée sur les éléments suivants :

* Une carte à base de STM32F4, qui pilote les deux moteurs et remote les informations d'odometrie. L'API rosserial est utilisée pour communiquer via l'USART avec un Raspberry Pi
* Un Raspberry Pi 3, qui exécute le Coeur ROS, et l'algorithme SLAM.
* Un PC vers lequel est remonté la télémétrie, et les commandes de contrôle via un gamepad 


## Noeuds

**tetra_utility.py** : 
* récupère les inputs du gamepad pour modifier les modes :
    * ▲ : reset odometry
    * X : Enable left joystick for direction
    * □ : Get battery level
    * START : Kill all node
* publie les TF necessaires pour le lidar (laser) et odometrie (odom)
* formate l'odométrie

## Pilotage du robot

Le pilotage du robot fonctionne avec le fichier **tetra.launch**. Un gamepad PS3 est supporté pour la commande. (voir BUGS)

## Constuction de la carte

* Lancer **tetra.launch**
* Lancer gmapping avec la commande suivante : ``` rosrun gmapping slam_gmapping _scan:=scan _base_frame:=laser _xmin:="-10" _xmax:="10" _ymin:=-10 _ymax:=10 _delta:=0.01```
* Conduire le robot et utiliser rviz pour contrôler la construction de la carte
* Utiliser la commande suivante pour enregistrer sur la carte. ```rosrun map_server map_saver -f %file_name```


## Navigation

* configuraiton navigation.launch, pour utiliser la carte enregistrée. 
* Lancer **tetra.launch**, **navigation.lauch** et rviz.
* Voir [la configuration de la stack de navigation rviz](http://wiki.ros.org/navigation/Tutorials/Using%20rviz%20with%20the%20Navigation%20Stack) pour la configuration de rviz. 
* Conduire manuellement le robot jusqu'à ce que la costmap soit alignée avec la carte configurée. Utiliser un marqueur *initialpose* pour faciliter le processus
* Utiliser un marqueur 2D goal pour faire avancer le robot

## Bugs connus et améliorations à faire

* La stabilité de rosserial_stm32 est perfectible. Bug zoolander à corrgiger : quand le robot tourne à droite il perd la liaison (le watchdog aide à rendre les choses moins pénibles)
* configurer le launchfile pour pouvoir lancer des noeuds sur plusieurs machines à la fois (pour le moment le noeud joy doit être lancé manuellement)
* configurer le joypad directement sur le raspberry pi (voir https://www.piborg.org/blog/rpi-ps3-help)
* Vérifier le streaming camera (énormément de latence !)

