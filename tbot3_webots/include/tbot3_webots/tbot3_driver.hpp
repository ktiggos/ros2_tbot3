#ifndef TBOT3_DRIVER_HPP
#define TBOT3_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/macros.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <webots_ros2_driver/WebotsNode.hpp>
#include <webots_ros2_driver/PluginInterface.hpp>

namespace tb3_driver
{
class Tb3Driver : public webots_ros2_driver::PluginInterface {
public:
    void step() override;
    void init(webots_ros2_driver::WebotsNode *node,
              std::unordered_map<std::string, std::string> &parameters) override;
private:
    void cmdVelCB(const geometry_msgs::msg::Twist::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    geometry_msgs::msg::Twist cmd_vel_msg;

    WbDeviceTag left_motor;
    WbDeviceTag right_motor;
};
}

#endif