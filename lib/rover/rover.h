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
        Rover(int motor_left_forward, int motor_left_backward, int motor_right_forward, int motor_right_backward);
        void forward();
        void backward();
        void turn_left();
        void turn_right();
        void stop();
    private:
        int _motor_left_forward, _motor_left_backward, _motor_right_forward, _motor_right_backward;

};

#endif

