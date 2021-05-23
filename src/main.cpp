/*** 
 This code makes an Arduino Mega 2560 board act as a ROS Node. 
For a simple diff drive robot it can publish the speed and positon
of left and right motor wheels, and subscribe to a control topic 
to accept control commands from a computer / Raspberry Pi running ROS.

Author: Mijaz Mukundan

References:
1. https://github.com/merose/ROSRobotControl
2. https://maker.pro/arduino/tutorial/how-to-use-arduino-with-robot-operating-system-ros
3. https://dronebotworkshop.com/rotary-encoders-arduino/

**/
#include <Arduino.h>
#include <rover.h>

// Arduino – ROS headers
#include <ros.h>
#include <std_msgs/Empty.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

//Define Motor Pins [IN1, IN2, IN3, IN4]
//OUT1 and OUT2 are connected to the right and OUT3 and OUT4 connected 
// to the left motor respectively
Rover rover(4,5,6,7);
uint8_t EN1 = 8;
uint8_t EN2 = 9;

//Defining Encoder Pins
// Pins for the SPG30E-200K DC Geared Motor with Encoder.
// Left Motor Encoder
const int ML_A = 2;
const int ML_B = 3;

//Right Motor Encoder
const int MR_A = 18;
const int MR_B = 19;

//Creating a Nodehandle object
ros::NodeHandle nh;

//Position Publisher Messages
std_msgs::Int16 lwheelMsg;
ros::Publisher lwheelPub("lwheel", &lwheelMsg);

std_msgs::Int16 rwheelMsg;
ros::Publisher rwheelPub("rwheel", &rwheelMsg);

//callback function when a control message is received
void rover_control(const std_msgs::String& ctrl_msg)
{ 
  
  if (ctrl_msg.data[0]=='w') rover.forward();
  else if (ctrl_msg.data[0]=='a') rover.turn_left();
  else if (ctrl_msg.data[0]=='d') rover.turn_right();
  else if (ctrl_msg.data[0]=='s') rover.backward();
  else rover.stop();
}

// Subscribing to "rover_control" ROS master serves the commands though this topic 
ros::Subscriber<std_msgs::String> controlSub("rover_control", rover_control );

// It is best to use volatile variables for the updates in interrupt service routines
volatile long lwheel = 0;
volatile long rwheel = 0;

unsigned long lastLoopTime;

// These are ISRs for reading the encoder ticks
// For the left wheel an anticlockwise rotation should increment the ticks
void leftAChange() {
  if (digitalRead(ML_A) != digitalRead(ML_B)) {
    ++lwheel;
  } else {
    --lwheel;
  }
}

// For the right wheel a clockwise rotation should increment the ticks
void rightAChange() {
  if (digitalRead(MR_A) == digitalRead(MR_B)) {
    ++rwheel;
  } else {
    --rwheel;
  }
}

void setup() {
  
  pinMode(EN1,OUTPUT);
  pinMode(EN2,OUTPUT);
  
  //Keeping enable pins high for full speed
  digitalWrite(EN1,HIGH);
  digitalWrite(EN2,HIGH);

  
  pinMode(ML_A,INPUT_PULLUP);
  pinMode(ML_B,INPUT_PULLUP);
  pinMode(MR_A,INPUT_PULLUP);
  pinMode(MR_B,INPUT_PULLUP);
  
  //Interrupts are the best way to read encoder values
  attachInterrupt(digitalPinToInterrupt(ML_A), leftAChange, RISING);
  attachInterrupt(digitalPinToInterrupt(MR_A), rightAChange, RISING);


  nh.initNode();
  
  nh.subscribe(controlSub);
  
  nh.advertise(lwheelPub);
  nh.advertise(rwheelPub);
  
  lastLoopTime = millis();
  
}

void loop() {
  
  long curLoopTime = millis();

   //This means that the Left and Right wheel encoder ticks is published every 100ms
  if (curLoopTime-lastLoopTime > 100){


    lwheelMsg.data = (int) lwheel;
    rwheelMsg.data = (int) rwheel;
    lwheelPub.publish(&lwheelMsg);
    rwheelPub.publish(&rwheelMsg);

    lastLoopTime = curLoopTime;
  }
  
  nh.spinOnce();
  

}

