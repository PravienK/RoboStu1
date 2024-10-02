#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <random>
#include <chrono>
#include <cmath>

class RobotMotion : public rclcpp::Node
{
public:
    RobotMotion() : Node("Robot_Motion")
    {
        linear_speed_ = this-> declare_parameter(*linear_speed*,0.2);
        angular_speed_ = this->
        
         
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10) 
        odom_sub_ = this ->create_subscription<nav_msgs::msg::Odometry>(
            "odom",10,)




    }