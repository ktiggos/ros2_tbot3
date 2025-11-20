#include "tbot3_webots/tbot3_driver.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cstdio>
#include <functional>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/position_sensor.h>

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
    this->node_ = node;

    RCLCPP_INFO(node_->get_logger(),"Initializing differential driver for Turtlebot...");
    
    left_motor = wb_robot_get_device("wheel_left_joint");
    right_motor = wb_robot_get_device("wheel_right_joint");
    lm_sensor = wb_robot_get_device("wheel_left_joint_sensor");
    rm_sensor = wb_robot_get_device("wheel_right_joint_sensor");

    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);

    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(right_motor, 0.0);

    wb_position_sensor_enable(lm_sensor, 10);
    wb_position_sensor_enable(rm_sensor, 10);

    RCLCPP_INFO(node_->get_logger(), "Initializing odometry measurements...");
    odom_msg.pose.pose.position = geometry_msgs::msg::Point(
        MessageInit::ZERO
    );
    odom_msg.pose.pose.orientation = geometry_msgs::msg::Quaternion(
        MessageInit::ZERO
    );

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
    double lm_cmd{ (fwd_speed - ang_speed * WHEEL_HALF_DIS) / WHEEL_RAD };
    double rm_cmd{ (fwd_speed + ang_speed * WHEEL_HALF_DIS) / WHEEL_RAD };

    lm_cmd = abs(lm_cmd) < 10.0 ? lm_cmd : (lm_cmd/abs(lm_cmd))*10.0;
    rm_cmd = abs(rm_cmd) < 10.0 ? rm_cmd : (rm_cmd/abs(rm_cmd))*10.0;

    wb_motor_set_velocity(left_motor, lm_cmd);
    wb_motor_set_velocity(right_motor, rm_cmd);

    // Odometry
    double ldis{ wb_position_sensor_get_value(lm_sensor) * WHEEL_RAD };
    double rdis{ wb_position_sensor_get_value(rm_sensor) * WHEEL_RAD };

    double dldis{ ldis - ldis_last };
    double drdis{ rdis - rdis_last};
    ldis_last = ldis;
    rdis_last = rdis;

    double dis_avg{ (dldis + drdis) / 2.0 };
    double dtheta{ (drdis - dldis) / (WHEEL_HALF_DIS * 2.0) };

    odom_msg.pose.pose.position.x += dis_avg * cos(theta + 0.5 * dtheta);
    odom_msg.pose.pose.position.y += dis_avg * sin(theta + 0.5 * dtheta);

    theta += dtheta;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);
}