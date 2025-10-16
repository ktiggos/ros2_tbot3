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
#define LINEAR_COEFF 0.33/10.0
#define ANGULAR_COEFF 2.29/10.0

PLUGINLIB_EXPORT_CLASS(tb3_driver::Tb3Driver, webots_ros2_driver::PluginInterface);

void tb3_driver::Tb3Driver::cmdVelCB(const geometry_msgs::msg::Twist::SharedPtr msg) {
    cmd_vel_msg.linear = msg->linear;
    cmd_vel_msg.angular = msg->angular;
}

void tb3_driver::Tb3Driver::init(webots_ros2_driver::WebotsNode *node,
                                 std::unordered_map<std::string, std::string> &parameters)
{
    std::cout<<"Initializing differential driver for Turtlebot..."<<std::endl;

    this->node_ = node;
    
    right_motor = wb_robot_get_device("wheel_right_joint");
    left_motor = wb_robot_get_device("wheel_left_joint");

    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);

    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(right_motor, 0.0);

    cb_time = node->get_clock()->now();
    driver_time = node->get_clock()->now();

    cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        rclcpp::SensorDataQoS().reliable(),
        [this](geometry_msgs::msg::Twist::SharedPtr msg) -> void {
            this->cmdVelCB(msg);
            cb_time = node_->get_clock()->now();
        }
    );
}

void tb3_driver::Tb3Driver::step(){
    driver_time = node_->get_clock()->now();

    bool stale{ (driver_time - cb_time) < 
        rclcpp::Duration::from_seconds(timeout)
    };

    // Differential Drive is used for Turtlebot3
    double fwd_speed = stale ? (cmd_vel_msg.linear.x)*LINEAR_COEFF : 0.0;
    double ang_speed = stale ? (cmd_vel_msg.angular.z)*ANGULAR_COEFF : 0.0;

    // Rotational speeds for motors (req.speed/rad)
    double left_motor_cmd = (fwd_speed - ang_speed * WHEEL_HALF_DIS) / WHEEL_RAD;
    double right_motor_cmd = (fwd_speed + ang_speed * WHEEL_HALF_DIS) / WHEEL_RAD;

    left_motor_cmd = abs(left_motor_cmd) < 10.0 ? left_motor_cmd : (left_motor_cmd/abs(left_motor_cmd))*10.0;
    right_motor_cmd = abs(right_motor_cmd) < 10.0 ? right_motor_cmd : (right_motor_cmd/abs(right_motor_cmd))*10.0;

    wb_motor_set_velocity(left_motor, left_motor_cmd);
    wb_motor_set_velocity(right_motor, right_motor_cmd);
}