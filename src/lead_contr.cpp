#include "lead_contr.hpp"

lead_contr::lead_contr(double a, double b, double K) : 
    a(a), b(b), K(K), prev_delta(0), prev_yaw(0){}

double lead_contr::calc(double yaw, path_finder::Waypoint waypoint){
    // Calculate Feedforward and Lead command angles
    double delta_ff = feedforward(waypoint.x, waypoint.y);
    double delta_lead = lead(yaw);
    
    // Sum the command angles for an overall command angle
    double delta_comm = delta_lead + delta_ff;

    // Save previous values
    prev_delta = delta_comm;
    prev_yaw = yaw;

    return delta_comm;
}

double lead_contr::lead(double yaw){
    double delta_lead = K * (yaw) - K*a*(prev_yaw) + b*prev_delta;
    return delta_lead;
}

double lead_contr::feedforward(double x, double y){
    double delta_ff = (x*x + y*y)/(2*y) / 1000;
    return delta_ff;
}

//// TYLER LONG'S CPP SCRIPT FOR A LEAD CONTROLLER ////
// void lead(double yaw, double previous_yaw,double previous_delta) //lead controller that takes in yaw(k), yaw(k-1), and steer_angle(k-1)  ---> provides steer angle(k)    steer angle is delta
// {
//     double a = 0.05; //numerator value to tune
//     double b = 0.9; //denominator value to tune
//     double k = 1; //gain value on lead controller to tune
//     double ref = 0; //desired yaw (0 in our case)

//     double delta_lead; //output of the lead controller [delta_lead(k)]

//     delta_lead = k*(-1*yaw) - k*a*(-1*previous_yaw) + b*previous_delta; //math to produce delta_lead(k)
// }

// void path_follow(double x, double y) //feed foward term in control scheme (the weird Kyle shenanigans) 
// {
//     double track_width = 10; //not actually 10 but we need some value to start out

//     double delta_ff; //the steer angle used for feed foward [delta_ff(k)]

//     delta_ff = (pow(x,y))/(2*y); // math behind steer angle for feed foward 
// }

// void closed_loop(double delta_lead,double delta_ff) //the summation of the lead controller steer angle and feed foward steer angle which is fed to the car
// {
//     double delta_command; //the steer angle the car actually drives [delta_commanded(k)]

//     delta_command = delta_lead + delta_ff; //complicated mathematics (addition)
// }