#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <sys/ioctl.h>

#include <rclcpp/rclcpp.hpp>

class XboxDriver : public rclcpp::Node {
    public:
        XboxDriver(std::string& dev)
        : Node("xbox_driver"), m_dev{dev} {
            RCLCPP_INFO(logger,"Reading from joy device: %s", m_dev.c_str());
        };

    private:
        std::string m_dev;
        rclcpp::Logger logger {this->get_logger()};
};

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);

    std::string dev = (argc > 1) ? argv[1] : "/dev/input/event14";

    rclcpp::spin(std::make_shared<XboxDriver>(dev));
    rclcpp::shutdown();

    return 0;
}