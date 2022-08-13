/** 
 * Author: Bryce Karlins bjk0024@auburn.edu
 * Brief: RViz support node for car position and orientation data
 *        Subscribes to Marvelmind position and orientation topics
 *        and republishes to a stamped pose topic for RViz to use
 * File: rviz_repub_node.cpp
 */

// --------------------------
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <marvelmind_ros2_msgs/msg/hedge_position.hpp>
#include <marvelmind_ros2_msgs/msg/hedge_imu_fusion.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <fstream>
#include "map.h"

// --------------------------

class RVizRepub : public rclcpp::Node
{
    public:
        RVizRepub() :
            Node("rviz_repub")
        {
            //main content

            //Declare Parameters
            this->declare_parameter<std::string>("input_topic1", "/hedgehog_pos");
            this->declare_parameter<std::string>("input_topic2", "/hedgehog_imu_fusion");
            this->declare_parameter<std::string>("output_topic1", "/visualization/pose");
            this->declare_parameter<std::string>("output_topic2", "/visualization/map");

            //Get Parameters
            std::string input_topic1, input_topic2, output_topic1, output_topic2;
            this->get_parameter<std::string>("input_topic1", input_topic1);
            this->get_parameter<std::string>("input_topic2", input_topic2);
            this->get_parameter<std::string>("output_topic1", output_topic1);
            this->get_parameter<std::string>("output_topic2", output_topic2);
            
            //Publishers and Subscribers
            this->pos_sub = this->create_subscription<marvelmind_ros2_msgs::msg::HedgePosition>(input_topic1, 1, std::bind(&RVizRepub::posCallback, this, std::placeholders::_1));
            this->quat_sub = this->create_subscription<marvelmind_ros2_msgs::msg::HedgeImuFusion>(input_topic2, 1, std::bind(&RVizRepub::quatCallback, this, std::placeholders::_1));
            this->pose_pub = this->create_publisher<nav_msgs::msg::Odometry>(output_topic1, 1);
            this->map_pub = this->create_publisher<geometry_msgs::msg::PolygonStamped>(output_topic2, 1);
            RCLCPP_DEBUG(this->get_logger(), "enter mapbuild");
            mapBuild();
            
            timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&RVizRepub::mapCallback, this));
            
            RCLCPP_DEBUG(this->get_logger(), "end of construct");
        }


    private:
        //Variables

        rclcpp::Subscription<marvelmind_ros2_msgs::msg::HedgePosition>::SharedPtr pos_sub;
        rclcpp::Subscription<marvelmind_ros2_msgs::msg::HedgeImuFusion>::SharedPtr quat_sub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub;
        rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr map_pub;

        rclcpp::TimerBase::SharedPtr timer_;

        OnSetParametersCallbackHandle::SharedPtr parameter_updater;


        geometry_msgs::msg::PolygonStamped map;

        void mapBuild(){
            RCLCPP_DEBUG(this->get_logger(), "begin of mapbuild");
            // map.points[120];

            for (int i=0; i < 120; i++){
                RCLCPP_DEBUG(this->get_logger(), "%i", i);

                geometry_msgs::msg::Point32 nod;
                nod.x = mapArr[i][0];
                nod.y = mapArr[i][1];
                nod.z = -2.5;
                map.polygon.points.push_back(nod);
                map.header.frame_id = "map";
            }
        }

        double posX = 0.0;
        double posY = 0.0;
        double posZ = 0.0; 

        void posCallback(const marvelmind_ros2_msgs::msg::HedgePosition msg)
        {
            this->posX = msg.x_m;
            this->posY = msg.y_m;
            this->posZ = msg.z_m;
        }

        void quatCallback(const marvelmind_ros2_msgs::msg::HedgeImuFusion msg)
        {
            // updateParameters();

            nav_msgs::msg::Odometry carPose;
            carPose.pose.pose.position.x = this->posX;
            carPose.pose.pose.position.y = this->posY;
            carPose.pose.pose.position.z = this->posZ;
            
            carPose.pose.pose.orientation.x = msg.qx;
            carPose.pose.pose.orientation.y = msg.qy;
            carPose.pose.pose.orientation.z = msg.qz;
            carPose.pose.pose.orientation.w = msg.qw;

            carPose.header.frame_id = "map";

            pose_pub->publish(carPose);
        }

        void mapCallback(){
            RCLCPP_DEBUG(this->get_logger(), "mapped");
            map_pub->publish(map);
        }

}; //end class RVizRepub

int main(int argc, char* argv[])
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RVizRepub>());
    rclcpp::shutdown();
    return(0);
}