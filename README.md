# Simple ROS Arduino
This project makes an Arduino Mega 2560 behave like a ROS node and controls a rover.

The Mega board controls a differential drive setup using L298N. It also records and publishes position and velocity of each motor wheel using encoders. It subscribes to a string topic to receive the movement command from a ROS master.

## Pubished Topics:

/lwheel : Position of left wheel
/rwheel : Position of right wheel
/lwheel_velocity : Velocity of left wheel in m/s
/rwheel_velocity: Velocity of right wheel in m/s

## Subsribed Topics
/rover_control : A String message : "w" : go forward
                                    "a" : turn left
                                    "s" : go backward
                                    "d" : turn right
                                    "any other key" : stop