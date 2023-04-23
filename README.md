# ElipsoidePuzzlebot
- Aqui estan todos los paquetes necesarios para probar la elipsoide de confianza de Puzzlebot.
Calculo de la elipsoide de confianza del Puzzlebot usando el simulador de Gazeboel siguiente comando:
- Corremos los siguientes comandos en una terminal diferente sin olvidar compilar nuestro ws.
- Iniciamos la simulacion en gazebo
```
roslaunch puzzlebot_world puzzlebot puzzlebot_simple_world.launch
```
- Corremos el codigo que calcula la elipse de confianza:
```
rosrun puzzlebot_rviz dead_reckoning.py
```
- Corremos la simulación del robot en RVIZ
```
roslaunch puzzlebot_rviz display.launch
```
- Corremos el modulo de control para teleoperar el robot:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
- En RVIZ agregamos el el mensaje de /odom desde add by topic.
- La localidad dell codigo que calcula de elipsoide de confianza es:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
