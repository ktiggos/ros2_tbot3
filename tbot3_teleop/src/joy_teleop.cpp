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

void JoyTeleop::compute_velocities(){
    if(joy_msg_->axes[1]){
        cmd_vel_msg.linear.x = -10.0 * (joy_msg_->axes[1] / MAX_ANALOG);
        cmd_vel_msg.angular.z = -10.0 * (joy_msg_->axes[3] / MAX_ANALOG);
    }else if(joy_msg_->axes[2]){
        cmd_vel_msg.angular.z = 10.0 * (joy_msg_->axes[2] / MAX_TRIGGER);
    }else if(joy_msg_->axes[5]){
        cmd_vel_msg.angular.z = -10.0 * (joy_msg_->axes[5] / MAX_TRIGGER);
    }else{
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
    }
}

void JoyTeleop::joyCB(const sensor_msgs::msg::Joy::SharedPtr msg){
    this->joy_msg_ = msg;
    this->compute_velocities();
}

void JoyTeleop::pubCB(){
    publisher_->publish(cmd_vel_msg);
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyTeleop>());
}