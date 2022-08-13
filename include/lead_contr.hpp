#ifndef LEAD_CONTR_HPP
#define LEAD_CONTR_HPP

#include <iostream>
#include <cmath>
#include "path_finder.hpp"

class lead_contr {
    private:
        double a; // Zero parameter
        double b; // Pole parameter
        double K; // Lead Gain
        double prev_delta; // Previous command delta
        double prev_yaw; // Previous yaw
        double lead(double yaw); // Calculation of lead controller command angle
        double feedforward(double x, double y); // Calculation of feedforward command angle
    public:
        lead_contr(double a, double b, double K); // Constructor
        double calc(double yaw, path_finder::Waypoint waypoint); // Calculate command delta
};

#endif