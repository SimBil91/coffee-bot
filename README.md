# coffee-bot
ROS integration for the 3D printed eezybot MK2.

I. In order to print your own eezybot robot arm, please visit:
http://www.eezyrobots.it/eba_mk2.html


II. Afterwards, grab an Arduino Uno and flash the eezybot_ros_driver onto it. 

The 4 servos should be connected to pins 
*x-axis:(9);
*y-axis:(6);
*z-axis:(10);
*gripper:(5);


III. Connect the Arduino to your laptop and run:
```
roslaunch coffee_bot coffee_bot.launch 
```

If there is an error, you may have to adjust your usb port in the launch file.

You should now be able to move each servo of the robot by firstly selecting the axis (x,y,z,g) followed by +,-.
Pressing 'n', saves the current pose and pushes it into the sequence.

's' saves the current sequence.

The robot pose sequences are saved as yaml files and can be played by publishing a string to topic "pose_name":
```
rostopic pub /pose_name std_msgs/String "data: 'test'"
```

Please find a simple example trajectory of the robot operating a coffee machine.


