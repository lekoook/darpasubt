# tank_firmware

Provides Arduino control interfacing with the TGV. Able to receive Twist message commands on /cmd_vel and move accordingly. 

Also able to receive wheel odometry that is published constantly (publishes centimeters travelled by each wheel on /lwheel and /rwheel respectively).

### How to run
- Ensure your platformio is configured properly on vscode, especially the platformio.ini port name (eg: /dev/ttyUSB0).

- `rosrun serial_python serial_node.py` will run the twist subscriber for movement and wheel encoder publisher

- To test if controls is working, `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` for keyboard controller

- To obtain a wheel odometry estimate on /tf and /odom, `roslaunch differential_drive tgv_odom_from_wheels.launch` (other repo)
