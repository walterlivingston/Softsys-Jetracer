/**
 * @file cam_control.cpp
 *
 * @brief ROS2 Galactic node for steering a car based on line following
 *
 * @author Walter Livingston
 */

#include "cam_control.hpp"


cam_control::cam_control() : Node("cam_control"){
    // Publishers and Subscribers
    this->image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/softsys/image_raw", 10, std::bind(&cam_control::image_callback, this, std::placeholders::_1));
    this->pos_sub_ = this->create_subscription<marvelmind_ros2_msgs::msg::HedgePosition>
        ("/hedgehog_pos", 1, std::bind(&cam_control::pos_callback, this, std::placeholders::_1));
    this->steer_pub_ = this->create_publisher<softsys_msgs::msg::Steer>("/softsys/steer_cmd", 10);
    this->throttle_pub_ = this->create_publisher<softsys_msgs::msg::Throttle>("/softsys/throttle_cmd", 10);
    this->image_repub_ = this->create_publisher<sensor_msgs::msg::Image>("/sally/path_image", 10);
    this->contr_pub_ = this->create_publisher<sally_msgs::msg::CamControl>("/sally/control_info", 10);
       
    // declare and retrieve parameters
    // Throttle parameters
    this->declare_parameter("minThrottle", 0.1);
    this->minThrottle = this->get_parameter("minThrottle").as_double();
    this->declare_parameter("maxThrottle", 0.1);
    this->maxThrottle = this->get_parameter("maxThrottle").as_double();

    // Look ahead parameter
    this->declare_parameter("lookAhead", 20.0);
    this->lookAhead = this->get_parameter("lookAhead").as_double();
    this->declare_parameter("angleWeight", 0.35);
    this->angleWeight = this->get_parameter("angleWeight").as_double();

    // Steer controller parameters
    this->declare_parameter("Kp_s", 10.0);
    this->Kp_s = this->get_parameter("Kp_s").as_double();
    this->declare_parameter("Ki_s", 0.0);
    this->Ki_s = this->get_parameter("Ki_s").as_double();
    this->declare_parameter("Kd_s", 0.0);
    this->Kd_s = this->get_parameter("Kd_s").as_double();
    this->declare_parameter("bias_s", 0.0);
    this->bias_s = this->get_parameter("bias_s").as_double();

    // Throttle controller parameters
    this->declare_parameter("Kp_t", 10.0);
    this->Kp_t = this->get_parameter("Kp_t").as_double();
    this->declare_parameter("Ki_t", 0.0);
    this->Ki_t = this->get_parameter("Ki_t").as_double();
    this->declare_parameter("Kd_t", 0.0);
    this->Kd_t = this->get_parameter("Kd_t").as_double();
    this->declare_parameter("bias_t", 0.0);
    this->bias_t = this->get_parameter("bias_t").as_double();

    // Throttle zone parameters
    this->declare_parameter("zone_1_x",std::vector<double>{0.0,5.0});
    this->zone_1_x = this->get_parameter("zone_1_x").as_double_array();
    this->declare_parameter("zone_1_y",std::vector<double>{2.0,5.5});
    this->zone_1_y = this->get_parameter("zone_1_y").as_double_array();
    this->declare_parameter("zone_1_K",2.0);
    this->zone_1_K = this->get_parameter("zone_1_K").as_double();

    this->declare_parameter("zone_2_x",std::vector<double>{3.8,5.0});
    this->zone_2_x = this->get_parameter("zone_2_x").as_double_array();
    this->declare_parameter("zone_2_y",std::vector<double>{4.5,6.0});
    this->zone_2_y = this->get_parameter("zone_2_y").as_double_array();
    this->declare_parameter("zone_2_K",0.7);
    this->zone_2_K = this->get_parameter("zone_2_K").as_double();


    // Controllers setup
    steer_contr = new controller(Kp_s, Ki_s, Kd_s, bias_s);
    throttle_contr = new controller(Kp_t, Ki_t, Kd_t, bias_t);
}

void cam_control::image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
    // Image Formatting
    Mat image = cv_bridge::toCvCopy(msg, msg->encoding)->image; // Convert to OpenCV image

    resize(image, image, Size(426, 240), INTER_LINEAR); // Resize Image

    // get new params
    this->param_update();

    // Error calculation
    double error_angle = PF->getPathAngle(image, this->lookAhead, angleWeight);
    Mat path_image = PF->getPathImage();

    // Publish commanded steering angle
    softsys_msgs::msg::Steer steer_msg;
    steer_msg.steer_angle = steer_contr->calc(error_angle, -PI/2, PI/2, -1, 1);
    steer_pub_->publish(steer_msg);

    // Publish commanded throttle
    softsys_msgs::msg::Throttle throttle_msg;
    
    throttle_msg.throttle = throttle_contr->calc(error_angle, -PI/2, PI/2, maxThrottle, minThrottle);
    if (this->zone_1==true){
        throttle_msg.throttle *= this->zone_1_K;
    }
    if (this->zone_2==true){
        throttle_msg.throttle *= this->zone_2_K;
    }

    throttle_pub_->publish(throttle_msg);
    

    // Publish control message
    sally_msgs::msg::CamControl contr_msg;
    contr_msg.header.frame_id = "contr_frame";
    contr_msg.header.stamp = this->get_clock()->now();
    contr_msg.steer_angle = steer_msg.steer_angle;
    contr_msg.throttle = throttle_msg.throttle;
    contr_msg.yaw = error_angle;
    contr_pub_->publish(contr_msg);

    // Publish processed image
    sensor_msgs::msg::Image image_msg;
    rclcpp::Time timestamp = this->get_clock()->now();
    image_msg.header.stamp = timestamp;
    image_msg.header.frame_id = "image_repub";
    image_msg.height = path_image.rows;
    image_msg.width = path_image.cols;
    image_msg.encoding = image_proc::mat_type2encoding(path_image.type());
    image_msg.is_bigendian = false;
    image_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(path_image.step);
    image_msg.data.assign(path_image.datastart, path_image.dataend);
    image_repub_->publish(image_msg);
}

void cam_control::timer_callback(){
   
}

void cam_control::param_update(){
    this->minThrottle = this->get_parameter("minThrottle").as_double();
    this->maxThrottle = this->get_parameter("maxThrottle").as_double();
    this->Kp_s = this->get_parameter("Kp_s").as_double();
    this->Ki_s = this->get_parameter("Ki_s").as_double();
    this->Kd_s = this->get_parameter("Kd_s").as_double();
    this->bias_s = this->get_parameter("bias_s").as_double();
    this->Kp_t = this->get_parameter("Kp_t").as_double();
    this->Ki_t = this->get_parameter("Ki_t").as_double();
    this->Kd_t = this->get_parameter("Kd_t").as_double();
    this->bias_t = this->get_parameter("bias_t").as_double();

    this->lookAhead = this->get_parameter("lookAhead").as_double();
    this->angleWeight = this->get_parameter("angleWeight").as_double();

    this->zone_1_x = this->get_parameter("zone_1_x").as_double_array();
    this->zone_1_y = this->get_parameter("zone_1_y").as_double_array();
    this->zone_1_K = this->get_parameter("zone_1_K").as_double();
    this->zone_2_x = this->get_parameter("zone_2_x").as_double_array();
    this->zone_2_y = this->get_parameter("zone_2_y").as_double_array();
    this->zone_2_K = this->get_parameter("zone_2_K").as_double();
    
    steer_contr->setGains(Kp_s, Ki_s, Kd_s, bias_s);
    throttle_contr->setGains(Kp_t, Ki_t, Kd_t, bias_t);
}

double cam_control::calc_yaw(const path_finder::Waypoint waypoint, const path_finder::Waypoint origin){
    // Camera yaw
    double theta = atan((waypoint.x - origin.x) / (waypoint.y - origin.y));

    return theta;
}

void cam_control::pos_callback(const marvelmind_ros2_msgs::msg::HedgePosition msg){
    if (msg.x_m > this->zone_1_x[0] && msg.x_m < this->zone_1_x[1]){
        if (msg.y_m > this->zone_1_y[0] && msg.y_m < this->zone_1_y[1]){
            this->zone_1 = true;
        }
        else{
            this->zone_1 = false;
        }
    }
    else{
        this->zone_1 = false;
    }

    if (msg.x_m > this->zone_2_x[0] && msg.x_m < this->zone_2_x[1]){
        if (msg.y_m > this->zone_2_y[0] && msg.y_m < this->zone_2_y[1]){
            this->zone_2 = true;
        }
        else{
            this->zone_2 = false;
        }
    }
    else{
        this->zone_2 = false;
    }

}




int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cam_control>());
    rclcpp::shutdown();
    return 0;
}
