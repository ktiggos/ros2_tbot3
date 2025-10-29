#ifndef XBOX_DRIVER_HPP
#define XBOX_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <chrono>

class XboxDriver : public rclcpp::Node {
    public:
        XboxDriver(const char* dev);
    private:
        void eventCB();
        void pubCB();

        int fd;
        const char* m_dev;
        rclcpp::Logger logger {this->get_logger()};
        rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
        sensor_msgs::msg::Joy joy_msg;

        rclcpp::TimerBase::SharedPtr evtimer_, ptimer_;
};

#endif