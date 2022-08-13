#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <iostream>
#include <cmath>

#define PI 3.1415926

class controller {
    public:
        controller(double Kp, double Ki, double Kd, double bias);
        double calc(double error, double inMin, double inMax, double outMin, double outMax);
        void setGains(double Kp, double Ki, double Kd, double bias);
    private:
        double Kp;
        double Ki;
        double Kd;
        double bias;
        double prev_error;
        double prev_effort;
        double integral;
};

#endif