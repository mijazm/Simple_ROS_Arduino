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

//Define Motor Pins
Rover rover(4,5,6,7);

//Defining Encoder Pins
// Pins for the A-Star motor encoder outputs.
const int M1_A = 2;
const int M1_B = 3;
const int M2_A = 18;
const int M2_B = 19;

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




void rover_control(const std_msgs::String& ctrl_msg)
{ 
  
  if (ctrl_msg.data[0]=='w') rover.forward();
  else if (ctrl_msg.data[0]=='a') rover.turn_left();
  else if (ctrl_msg.data[0]=='d') rover.turn_right();
  else if (ctrl_msg.data[0]=='s') rover.backward();
  else rover.stop();
}
ros::Subscriber<std_msgs::String> sub("rover_control", rover_control );


volatile long lwheel = 0;
volatile long rwheel = 0;

long lastLwheel = 0;
long lastRwheel = 0;

int ticksPerMeter=2727;

unsigned long lastLoopTime;

void leftAChange() {
  if (digitalRead(M1_A) == digitalRead(M1_B)) {
    ++lwheel;
  } else {
    --lwheel;
  }
}

void leftBChange() {
  if (digitalRead(M1_A) != digitalRead(M1_B)) {
    ++lwheel;
  } else {
    --lwheel;
  }
}

void rightAChange() {
  if (digitalRead(M2_A) != digitalRead(M2_B)) {
    ++rwheel;
  } else {
    --rwheel;
  }
}

void rightBChange() {
  if (digitalRead(M2_A) == digitalRead(M2_B)) {
    ++rwheel;
  } else {
    --rwheel;
  }
}

void setup() {
  
   //begin serial communication
  Serial1.begin(115200);

  pinMode(M1_A,INPUT_PULLUP);
  pinMode(M1_B,INPUT_PULLUP);
  pinMode(M2_A,INPUT_PULLUP);
  pinMode(M2_B,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(M1_A), leftAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_B), leftBChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_A), rightAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_B), rightBChange, CHANGE);

  nh.initNode();
  
  nh.subscribe(sub);
  
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

