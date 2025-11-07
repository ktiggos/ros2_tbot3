#include "tbot3_teleop/joy_teleop.hpp"

JoyTeleop::JoyTeleop() : Node("joy_teleop"){
    cmd_vel_msg.linear = geometry_msgs::msg::Vector3(
        MessageInit::ZERO
    );
    cmd_vel_msg.angular = geometry_msgs::msg::Vector3(
        MessageInit::ZERO
    );

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10
    );

    subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy",
        10,
        [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void {
            this->joyCB(msg);
        }
    );

    pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
        [this]() -> void {
            this->pubCB();
        }
    );
}

void JoyTeleop::joyCB(const sensor_msgs::msg::Joy::SharedPtr msg){
    this->joy_msg_ = msg;
}

void JoyTeleop::pubCB(){
    publisher_->publish(cmd_vel_msg);
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyTeleop>());
}