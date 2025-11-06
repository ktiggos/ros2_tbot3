#include "tbot3_teleop/xbox_driver.hpp"

XboxDriver::XboxDriver(const char* dev)
: Node("xbox_driver"), m_dev{dev}{
    RCLCPP_INFO(logger,"Reading from joy device: %s",
        static_cast<std::string>(m_dev).c_str()
    );

    joy_msg.buttons = std::vector<int>(12,0);
    joy_msg.axes = std::vector<float>(6,0);

    publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy",10);

    fd = ::open(m_dev, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    
    evtimer_ = this->create_wall_timer(std::chrono::milliseconds(50),
        [this]() -> void {
            this->eventCB();
        }
    );

    ptimer_ = this->create_wall_timer(std::chrono::milliseconds(100),
        [this]() -> void {
            this->pubCB();
        }
    );
}

void XboxDriver::eventCB(){
    struct input_event ev[64];
    ssize_t n = ::read(fd, (void*)ev, sizeof(ev));

    if(n > 0){
        size_t cnt = n/sizeof(input_event);
        for(size_t i{0}; i<cnt; i++){
            joy_msg.header.stamp = this->now();
            if(ev[i].type == EV_ABS){
                joy_msg.axes[map_axes(ev[i].code)] = ev[i].value;
            }else if(ev[i].type == EV_KEY){
                joy_msg.buttons[map_buttons(ev[i].code)] = ev[i].value;
            }
        }
    }
}

void XboxDriver::pubCB(){
    publisher_->publish(joy_msg);
}

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);

    const char* dev = (argc > 1) ? argv[1] : "/dev/input/event14";

    rclcpp::spin(std::make_shared<XboxDriver>(dev));
    rclcpp::shutdown();

    return 0;
}