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

//Define Motor Pins [left_forward, left_backward, right_forward, right_backward]
Rover rover(4,5,6,7);

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

//Position and Velocity Publisher Messages
std_msgs::Int16 lwheelMsg;
ros::Publisher lwheelPub("lwheel", &lwheelMsg);

std_msgs::Int16 rwheelMsg;
ros::Publisher rwheelPub("rwheel", &rwheelMsg);

std_msgs::Float32 lwheelVelocityMsg;
ros::Publisher lwheelVelocityPub("lwheel_velocity", &lwheelVelocityMsg);

std_msgs::Float32 rwheelVelocityMsg;
ros::Publisher rwheelVelocityPub("rwheel_velocity", &rwheelVelocityMsg);



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

long lastLwheel = 0;
long lastRwheel = 0;

// This value is different for every robot, you will have to experimentally determine this.
// It is the number of ticks it takes to drive the robot one meter in a straight line.
int ticksPerMeter=28028;

unsigned long lastLoopTime;

// These are ISRs for reading the encoder ticks
void leftAChange() {
  if (digitalRead(ML_A) == digitalRead(ML_B)) {
    ++lwheel;
  } else {
    --lwheel;
  }
}

void leftBChange() {
  if (digitalRead(ML_A) != digitalRead(ML_B)) {
    ++lwheel;
  } else {
    --lwheel;
  }
}

void rightAChange() {
  if (digitalRead(MR_A) != digitalRead(MR_B)) {
    ++rwheel;
  } else {
    --rwheel;
  }
}

void rightBChange() {
  if (digitalRead(MR_A) == digitalRead(MR_B)) {
    ++rwheel;
  } else {
    --rwheel;
  }
}

void setup() {
  
   //begin serial communication
  Serial1.begin(115200);
  
  pinMode(ML_A,INPUT_PULLUP);
  pinMode(ML_B,INPUT_PULLUP);
  pinMode(MR_A,INPUT_PULLUP);
  pinMode(MR_B,INPUT_PULLUP);
  
  //Interrupts are the best way to read encoder values
  attachInterrupt(digitalPinToInterrupt(ML_A), leftAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ML_B), leftBChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MR_A), rightAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MR_B), rightBChange, CHANGE);

  nh.initNode();
  
  nh.subscribe(controlSub);
  
  nh.advertise(lwheelPub);
  nh.advertise(rwheelPub);
  nh.advertise(lwheelVelocityPub);
  nh.advertise(rwheelVelocityPub);
  
  lastLoopTime = micros();
  
}

void loop() {
  
  delay(66);

  long curLoopTime = micros();
  
  
  noInterrupts();
  long curLwheel = lwheel;
  long curRwheel = rwheel;
  interrupts();

  lwheelMsg.data = (int) curLwheel;
  rwheelMsg.data = (int) curRwheel;
  lwheelPub.publish(&lwheelMsg);
  rwheelPub.publish(&rwheelMsg);

  float dt = (curLoopTime - lastLoopTime) / 1E6;

  float lwheelRate = ((curLwheel - lastLwheel) / dt);
  float rwheelRate = ((curRwheel - lastRwheel) / dt);

  lwheelVelocityMsg.data = lwheelRate / ticksPerMeter;
  rwheelVelocityMsg.data = rwheelRate / ticksPerMeter;
  lwheelVelocityPub.publish(&lwheelVelocityMsg);
  rwheelVelocityPub.publish(&rwheelVelocityMsg);
  
  lastLwheel = curLwheel;
  lastRwheel = curRwheel;
  
  lastLoopTime = curLoopTime;
  
  nh.spinOnce();
  

}

