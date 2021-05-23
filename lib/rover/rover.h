/*
Rover Library for ROS Robot
Mijaz Mukundan
MIT License
*/

#ifndef rover_h
#define rover_h

#include<Arduino.h>
class Rover
{
    public:
        Rover(int in1, int in2, int in3, int in4);
        void forward();
        void backward();
        void turn_left();
        void turn_right();
        void stop();
    private:
        int _in1, _in2, _in3, _in4;

};

#endif

