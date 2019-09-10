1. Turn on raspberry pi
2. Connect to Arduino and Pixhawk
3. Make sure Arduino address in rosserial_python/serial_node.py is correct
4. Make sure Pixhawk address in mavros/px4.launch is correct
5. In terminal: roslaunch mavros px4.launch //including serial_node.py
6. Arm with remote control
7. In terminal: rosrun rover_protospace control.py
8. Try to move the rover by running the [GUI](https://github.com/globibule/controllerGUIRaspi) on another ROS enabled device

Note: to edit program, use rosed roverprotospace control.py
