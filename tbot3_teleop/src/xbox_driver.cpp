#include "tbot3_teleop/xbox_driver.hpp"

XboxDriver::XboxDriver(const char* dev)
: Node("xbox_driver"), m_dev{dev}{
    RCLCPP_INFO(logger,"Reading from joy device: %s",
        static_cast<std::string>(m_dev).c_str()
    );

    fd = ::open(m_dev, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    
    evtimer_ = this->create_wall_timer(std::chrono::milliseconds(100),
        [this]() -> void {
            this->eventCB();
        }
    );

    ptimer_ = this->create_wall_timer(std::chrono::milliseconds(200),
        [this]() -> void {
            this->pubCB();
        }
    );
}

void XboxDriver::eventCB(){
    struct input_event ev[64];
    ssize_t n = ::read(fd, (void*)ev, 64);
    size_t cnt = n/sizeof(input_event);

    RCLCPP_INFO(logger,"%li", cnt);
}

void XboxDriver::pubCB(){
    // RCLCPP_INFO(logger,"PUBLISHING");
}

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);

    const char* dev = (argc > 1) ? argv[1] : "/dev/input/event14";

    rclcpp::spin(std::make_shared<XboxDriver>(dev));
    rclcpp::shutdown();

    return 0;
}