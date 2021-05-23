/*
Rover library for ROS Robot
Mijaz Mukundan
MIT Lisence
*/

#include<Arduino.h>
#include "rover.h"

Rover::Rover(int in1, int in2, int in3, int in4){
    pinMode(in1,OUTPUT);
    pinMode(in2,OUTPUT);
    pinMode(in3,OUTPUT);
    pinMode(in4,OUTPUT);

    _in1 = in1;
    _in2 = in2;
    _in3 = in3;
    _in4 = in4;
}

void Rover::stop(){
    digitalWrite(_in1,LOW);
    digitalWrite(_in2,LOW);

    digitalWrite(_in3,LOW);
    digitalWrite(_in4,LOW);
}

void Rover::forward(){
    
    digitalWrite(_in1,HIGH);
    digitalWrite(_in2,LOW);

    digitalWrite(_in3,HIGH);
    digitalWrite(_in4,LOW);
}

void Rover::backward(){
    digitalWrite(_in1,LOW);
    digitalWrite(_in2,HIGH);

    digitalWrite(_in3,LOW);
    digitalWrite(_in4,HIGH);
}

void Rover::turn_left(){
    digitalWrite(_in1,HIGH);
    digitalWrite(_in2,LOW);

    digitalWrite(_in3,LOW);
    digitalWrite(_in4,HIGH);
}

void Rover::turn_right(){
    digitalWrite(_in1,LOW);
    digitalWrite(_in2,HIGH);

    digitalWrite(_in3,HIGH);
    digitalWrite(_in4,LOW);
}

//###########################################################################################################################

