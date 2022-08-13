/** 
 * Author: Bryce Karlins bjk0024@auburn.edu
 * Brief: RViz support node for car position and orientation data
 *        Subscribes to Marvelmind position and orientation topics
 *        and republishes to a stamped pose topic for RViz to use
 * File: rviz_repub_node.cpp
 */

// --------------------------
#include "rclcpp/rclcpp.hpp"
#include <fstream>

#include <marvelmind_ros2_msgs/msg/hedge_position.hpp>
#include <marvelmind_ros2_msgs/msg/hedge_imu_fusion.hpp>

// --------------------------

class LocalizationLogger : public rclcpp::Node
{
    public:
        LocalizationLogger() :
            Node("loc_log")
        {
            //main content

            //Declare Parameters
            this->declare_parameter<std::string>("input_topic1", "/hedgehog_pos");
            this->declare_parameter<std::string>("input_topic2", "/hedgehog_imu_fusion");
            this->declare_parameter<std::string>("output_file", "pos_log.csv");

            //Get Parameters
            std::string input_topic1, input_topic2, output_file;
            this->get_parameter<std::string>("input_topic1", input_topic1);
            this->get_parameter<std::string>("input_topic2", input_topic2);
            this->get_parameter<std::string>("output_file", output_file);

            
            //Publishers and Subscribers
            this->pos_sub = this->create_subscription<marvelmind_ros2_msgs::msg::HedgePosition>(input_topic1, 1, std::bind(&LocalizationLogger::posCallback, this, std::placeholders::_1));
            this->quat_sub = this->create_subscription<marvelmind_ros2_msgs::msg::HedgeImuFusion>(input_topic2, 1, std::bind(&LocalizationLogger::quatCallback, this, std::placeholders::_1));
            
            pose.open(output_file);
        }
        ~LocalizationLogger(){
            pose.close();
        }



    private:
        //Variables
        std::ofstream pose;
        

        rclcpp::Subscription<marvelmind_ros2_msgs::msg::HedgePosition>::SharedPtr pos_sub;
        rclcpp::Subscription<marvelmind_ros2_msgs::msg::HedgeImuFusion>::SharedPtr quat_sub;

        void posCallback(const marvelmind_ros2_msgs::msg::HedgePosition msg)
        {
            // this->posX = msg.x_m;
            // this->posY = msg.y_m;
            // this->posZ = msg.z_m;
            this->pose << msg.x_m << "," << msg.y_m << ',' << msg.z_m << "\n";
            RCLCPP_INFO(rclcpp::get_logger("loc_log"), "X: %f, Y: %f, Z: %f", msg.x_m, msg.y_m, msg.z_m);
        }

        void quatCallback(const marvelmind_ros2_msgs::msg::HedgeImuFusion msg)
        {
            // geometry_msgs::msg::PoseStamped carPose;
            // carPose.pose.position.x = this->posX;
            // carPose.pose.position.y = this->posY;
            // carPose.pose.position.z = this->posZ;
            
            // carPose.pose.orientation.x = msg.qx;
            // carPose.pose.orientation.y = msg.qy;
            // carPose.pose.orientation.z = msg.qz;
            // carPose.pose.orientation.w = msg.qw;
        }
}; //end class LocalizationLogger

int main(int argc, char* argv[])
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationLogger>());
    rclcpp::shutdown();
    return(0);
}