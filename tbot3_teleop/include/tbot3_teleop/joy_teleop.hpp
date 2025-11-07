#ifndef JOY_TELEOP_HPP
#define JOY_TELEOP_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

class JoyTeleop : public rclcpp::Node {
    public:
        JoyTeleop();
    private:
        void joyCB(const sensor_msgs::msg::Joy::SharedPtr msg);

        sensor_msgs::msg::Joy::SharedPtr joy_msg_;
        rclcpp::Logger logger {this->get_logger()};
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
};

#endif 