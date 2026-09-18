Proyecto Final - Robots II

Este proyecto implementa una estrategia de navegación autónoma para un robot TurtleBot3 simulado en Gazebo utilizando ROS 2. Se partió de la estructura base proporcionada para el desafío y se incorporó la lógica de navegación en el nodo TurtlebotCtrl.

Dependencias

Ubuntu 24.04
ROS 2 Jazzy
Gazebo
TurtleBot3
Paquetes turtlebot3, turtlebot3_msgs y turtlebot3_simulations
Python 3
NumPy

Preparación del workspace

Para el desarrollo se utilizó el workspace:

~/proyecto_final_tbot

Los paquetes necesarios se encuentran dentro de:

~/proyecto_final_tbot/src

Estructura utilizada:

proyecto_final_tbot/
src/
turtlebot3/
turtlebot3_control_ros2/
turtlebot3_msgs/
turtlebot3_simulations/
build/
install/
log/

Compilación

Desde la raíz del workspace:

cd ~/proyecto_final_tbot
colcon build
source install/setup.bash

Lanzamiento de la simulación

cd ~/proyecto_final_tbot
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

Ejecución del nodo

En una nueva terminal:

cd ~/proyecto_final_tbot
source install/setup.bash
ros2 run turtlebot3_control_ros2 turtlebot_ctrl

Arquitectura

La solución se implementa principalmente en el nodo TurtlebotCtrl, que integra percepción, toma de decisiones y control del movimiento.

El nodo utiliza tres tópicos principales:

/scan: recibe las mediciones del sensor LiDAR mediante mensajes LaserScan.

/odom: recibe información de odometría mediante mensajes Odometry.

/cmd_vel: publica los comandos de velocidad lineal y angular mediante mensajes TwistStamped.

Las mediciones del LiDAR se dividen en sectores alrededor del robot para detectar paredes, obstáculos y espacios libres. A partir de esta información se implementa una máquina de estados para buscar y seguir paredes, evitar obstáculos y detectar oportunidades para atravesar sectores libres.

El mapa discreto incluido en la estructura base se utiliza para registrar y calcular el porcentaje de cobertura alcanzado durante la ejecución. Las decisiones de navegación se realizan a partir de la información obtenida por los sensores.

Código principal

turtlebot3_control_ros2/turtlebot_ctrl.py

Repositorio

https://github.com/edu964/Priafinal

