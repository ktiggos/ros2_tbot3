#include "tbot3_webots/tbot3_driver.hpp"

#include <cstdio>
#include <functional>
// #include <webots/Motor.hpp>
#include <webots/motor.h>
// #include <webots/Robot.hpp>
#include <webots/robot.h>

#include <pluginlib/class_list_macros.hpp>

// Values taken from URDF
#define WHEEL_HALF_DIS 0.144
#define WHEEL_RAD 0.033

PLUGINLIB_EXPORT_CLASS(tb3_driver::Tb3Driver, webots_ros2_driver::PluginInterface);

void tb3_driver::Tb3Driver::cmdVelCB(const geometry_msgs::msg::Twist::SharedPtr msg) {
    cmd_vel_msg.linear = msg->linear;
    cmd_vel_msg.angular = msg->angular;
}

void tb3_driver::Tb3Driver::init(webots_ros2_driver::WebotsNode *node,
                                 std::unordered_map<std::string, std::string> &parameters)
{
    std::cout<<"INITIALIZING DRIVER"<<std::endl;
    
    right_motor = wb_robot_get_device("wheel_right_joint");
    left_motor = wb_robot_get_device("wheel_left_joint");

    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);

    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(right_motor, 0.0);

    cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        rclcpp::SensorDataQoS().reliable(),
        [this](geometry_msgs::msg::Twist::SharedPtr msg) -> void {
            this->cmdVelCB(msg);
        }
    );
}

void tb3_driver::Tb3Driver::step(){
    // Differential Drive is used for Turtlebot3
    double fwd_speed = cmd_vel_msg.linear.x;
    double ang_speed = cmd_vel_msg.angular.z;

    // Rotational speeds for motors (req.speed/rad)
    double left_motor_cmd = (fwd_speed - ang_speed * WHEEL_HALF_DIS) / WHEEL_RAD;
    double right_motor_cmd = (fwd_speed + ang_speed * WHEEL_HALF_DIS) / WHEEL_RAD;

    wb_motor_set_velocity(left_motor, left_motor_cmd);
    wb_motor_set_velocity(right_motor, right_motor_cmd);
}