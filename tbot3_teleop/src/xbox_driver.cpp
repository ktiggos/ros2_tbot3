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
        XboxDriver(const char* dev)
        : Node("xbox_driver"), m_dev{dev} {
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
        };

    private:
        void eventCB(){
            int fd = open(m_dev, O_RDONLY);
            size_t buffer{64};

            RCLCPP_INFO(logger, "READING");
        }

        void pubCB(){
            RCLCPP_INFO(logger,"PUBLISHING");
        }

        int fd;
        const char* m_dev;
        rclcpp::Logger logger {this->get_logger()};
        rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
        sensor_msgs::msg::Joy joy_msg;

        rclcpp::TimerBase::SharedPtr evtimer_, ptimer_;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);

    const char* dev = (argc > 1) ? argv[1] : "/dev/input/event14";

    rclcpp::spin(std::make_shared<XboxDriver>(dev));
    rclcpp::shutdown();

    return 0;
}