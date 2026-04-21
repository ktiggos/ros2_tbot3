#ifndef TBOT3_DRIVER_HPP
#define TBOT3_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/macros.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <memory>

#include <webots_ros2_driver/WebotsNode.hpp>
#include <webots_ros2_driver/PluginInterface.hpp>

typedef rosidl_runtime_cpp::MessageInitialization MessageInit;

namespace tb3_driver
{
class Tb3Driver : public webots_ros2_driver::PluginInterface {
public:
    void step() override;
    void init(webots_ros2_driver::WebotsNode* node,
              std::unordered_map<std::string, std::string> &parameters) override;
private:
    void cmdVelCB(const geometry_msgs::msg::Twist::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::Twist cmd_vel_msg;
    nav_msgs::msg::Odometry odom_msg;

    WbDeviceTag left_motor;
    WbDeviceTag right_motor;
    WbDeviceTag lm_sensor;
    WbDeviceTag rm_sensor;

    rclcpp::Time cb_time, driver_time, dtime_last;
    double timeout{0.5};

    double ldis_last{0.0};
    double rdis_last{0.0};
    double theta{0.0};

    webots_ros2_driver::WebotsNode* node_;
    WbNodeRef self_node;

    double gt_x_last{0.0};
    double gt_y_last{0.0};
    double gt_yaw_last{0.0};
    bool gt_first{true};

    double lin_prev{0.0}, ang_prev{0.0};
};
}

#endif