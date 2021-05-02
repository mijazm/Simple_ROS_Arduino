<<<<<<< HEAD
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
=======
# SIMPLE ROS ROBOT
This project aims to create a basic ROS (Noetic) based robot.

The robot is powered by Raspberry Pi 4 running on ubuntu server 20.04.
The low level inputs are handled by an Arduino Mega 2560, it is connected to Raspberry Pi 4 via USB Serial.

Rosserial Package is used to communicate between Pi and Arduino.

In this project Joystick events from a gamepad are used to drive the robot. Localization is done using rotary encoders on motor.
>>>>>>> c3dd68ba27899a89174c11fc694fcca291cedf21
