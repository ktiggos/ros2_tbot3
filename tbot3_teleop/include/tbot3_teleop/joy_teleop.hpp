#ifndef JOY_TELEOP_HPP
#define JOY_TELEOP_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>

#define MAX_ANALOG 32768.0
#define MAX_TRIGGER 1023.0

typedef rosidl_runtime_cpp::MessageInitialization MessageInit;

class JoyTeleop : public rclcpp::Node {
    public:
        JoyTeleop();
    private:
        void joyCB(const sensor_msgs::msg::Joy::SharedPtr msg);
        void pubCB();
        void compute_velocities();

        sensor_msgs::msg::Joy::SharedPtr joy_msg_;
        geometry_msgs::msg::Twist cmd_vel_msg;

        rclcpp::Logger logger {this->get_logger()};
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
        rclcpp::TimerBase::SharedPtr pub_timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

#endif 