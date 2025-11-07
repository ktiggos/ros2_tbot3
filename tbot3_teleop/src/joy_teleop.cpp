#include "tbot3_teleop/joy_teleop.hpp"

JoyTeleop::JoyTeleop() : Node("joy_teleop"){
    sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy",
        10,
        [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void {
            this->joyCB(msg);
        }
    );
}

void JoyTeleop::joyCB(const sensor_msgs::msg::Joy::SharedPtr msg){
    this->joy_msg_ = msg;
    RCLCPP_INFO(logger, "SUB");
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyTeleop>());
}