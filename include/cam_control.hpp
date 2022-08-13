#ifndef CAM_CONTROL_HPP
#define CAM_CONTROL_HPP

// C++ STL
#include <chrono>
#include <thread>
#include <vector>
#include <cstdlib>
#include <memory>
#include <vector>
using namespace std::chrono_literals;

// OpenCV
#include "cv_bridge/cv_bridge.h"

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

// Message Types
#include <sensor_msgs/msg/image.hpp>
#include "softsys_msgs/msg/steer.hpp"
#include "softsys_msgs/msg/throttle.hpp"
#include "sally_msgs/msg/cam_control.hpp"
#include <marvelmind_ros2_msgs/msg/hedge_position.hpp>


// Path Finding
#include "path_finder.hpp"

// Controllers
#include "controller.hpp"
#include "lead_contr.hpp"

class cam_control : public rclcpp::Node {
    public:
        cam_control(); // Constructor
    private:
        path_finder* PF = new path_finder(); // Path finder object
        controller* steer_contr; // PID controller object for steering
        controller* throttle_contr; // PID controller object for throttle
        lead_contr* steer_contr_l; // Lead controller object for steering

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_; // Subscriber to camera image
        rclcpp::Subscription<marvelmind_ros2_msgs::msg::HedgePosition>::SharedPtr pos_sub_; // Subscriber to hedge position
        rclcpp::Publisher<softsys_msgs::msg::Steer>::SharedPtr steer_pub_; // Publisher to steering topic
        rclcpp::Publisher<softsys_msgs::msg::Throttle>::SharedPtr throttle_pub_; // Publisher to throttle topic
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_repub_; // Publisher to republish image
        rclcpp::Publisher<sally_msgs::msg::CamControl>::SharedPtr contr_pub_; // Publisher to publish control data
        
        double minThrottle; // Minimum throttle value
        double maxThrottle; // Maximum throttle value

        double lookAhead; // Look ahead distance in pixels
        double angleWeight; // Weight of line angle measurement

        double Kp_s; // Proportional gain
        double Ki_s; // Integral gain
        double Kd_s; // Derivative gain
        double bias_s; // Steering bias

        double Kp_t; // Proportional gain
        double Ki_t; // Integral gain
        double Kd_t; // Derivative gain
        double bias_t; // Throttle bias

        std::vector<double> zone_1_x; //throttle zone [Xmin,xmax]
        std::vector<double> zone_1_y; //throttle zone [Ymin,Ymax]
        double zone_1_K; //zone 1 throttle gain

        std::vector<double> zone_2_x; //throttle zone [Xmin,xmax]
        std::vector<double> zone_2_y; //throttle zone [Ymin,Ymax]
        double zone_2_K; //zone 1 throttle gain

        bool zone_1 = false;
        bool zone_2 = false;

        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg); // Callback function for image subscriber
        void pos_callback(const marvelmind_ros2_msgs::msg::HedgePosition msg); // Callback function for position subscriber
        void timer_callback(); // Callback function for controls timer
        void param_update(); // checks for and updates parameters in runtime 
        double calc_yaw(const path_finder::Waypoint waypoint, const path_finder::Waypoint origin); // Function for calculating yaw from waypoint
};

#endif