/*
Rover library for ROS Robot
Mijaz Mukundan
MIT Lisence
*/

#include<Arduino.h>
#include "rover.h"

Rover::Rover(int motor_left_forward, int motor_left_backward, int motor_right_forward, int motor_right_backward){
    pinMode(motor_left_forward,OUTPUT);
    pinMode(motor_left_backward,OUTPUT);
    pinMode(motor_right_backward,OUTPUT);
    pinMode(motor_right_forward,OUTPUT);

    _motor_left_backward=motor_left_backward;
    _motor_left_forward=motor_left_forward;
    _motor_right_backward=motor_right_backward;
    _motor_right_forward=motor_right_forward;
}

void Rover::stop(){
    digitalWrite(_motor_left_forward,LOW);
    digitalWrite(_motor_right_forward,LOW);

    digitalWrite(_motor_left_backward,LOW);
    digitalWrite(_motor_right_backward,LOW);
}

void Rover::forward(){
    digitalWrite(_motor_left_forward,HIGH);
    digitalWrite(_motor_right_forward,HIGH);

    digitalWrite(_motor_left_backward,LOW);
    digitalWrite(_motor_right_backward,LOW);
}

void Rover::backward(){
    digitalWrite(_motor_left_forward,LOW);
    digitalWrite(_motor_right_forward,LOW);

    digitalWrite(_motor_left_backward,HIGH);
    digitalWrite(_motor_right_backward,HIGH);
}

void Rover::turn_left(){
    digitalWrite(_motor_left_forward,LOW);
    digitalWrite(_motor_right_forward,HIGH);

    digitalWrite(_motor_left_backward,HIGH);
    digitalWrite(_motor_right_backward,LOW);
}

void Rover::turn_right(){
    digitalWrite(_motor_left_forward,HIGH);
    digitalWrite(_motor_right_forward,LOW);

    digitalWrite(_motor_left_backward,LOW);
    digitalWrite(_motor_right_backward,HIGH);
}

//###########################################################################################################################

