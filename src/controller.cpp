/**
 * @file controller.cpp
 *
 * @brief Simple PID controller
 * 
 * Based on https://gist.github.com/bradley219/5373998
 *
 * @author Walter Livingston
 */

#include "controller.hpp"

controller::controller(double Kp, double Ki, double Kd, double bias) :
    Kp(Kp), Ki(Ki), Kd(Kd), bias(bias), prev_error(0), prev_effort(0), integral(0) {}

double controller::calc(double error, double inMin, double inMax, double outMin, double outMax){
    double scaledError = (error - inMin) / (inMax - inMin);
    if(scaledError > 1.0) scaledError = 1.0;
    else if (scaledError < 0.0) scaledError = 0.0;

    if(!(Ki*integral >= 0.5*Kp*scaledError))
        integral += scaledError;
    else
        integral = 0;

    // if(error < 0.01)
    //     integral = 0;
        
    double dError = scaledError - prev_error;

    double ff = 0;
    if(bias != 0)
        ff = (bias - outMin) / (outMax - outMin);

    double out = ff + (Kp * scaledError) + (Ki * integral) - (Kd * dError);

    if(out < 0.0) out = 0.0;
    else if (out > 1.0) out = 1.0;

    prev_effort = out;
    prev_error = scaledError;

    double scaledOut = (out * (outMax - outMin)) + outMin;

    return scaledOut;
}

void controller::setGains(double Kp, double Ki, double Kd, double bias){
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->bias = bias;
}