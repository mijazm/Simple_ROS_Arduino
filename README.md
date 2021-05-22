# Simple ROS Arduino
This code makes an Arduino Mega 2560 behave like a ROS node and controls a differential drive robot.  

The board controls a differential drive setup using L298N. It also records and publishes position of each motor wheel using encoders. It subscribes to a string topic to receive the movement command from Raspberry Pi.  

## Pubished Topics:
/lwheel : Position of left wheel
/rwheel : Position of right wheel

## Subsribed Topics
/rover_control : A String message : "w" : go forward  
                                    "a" : turn left  
                                    "s" : go backward  
                                    "d" : turn right  
                                    "any other key" : stop

## ROS Package
Correspoing ROS package is available at https://github.com/mijazm/simple_ros_robot
