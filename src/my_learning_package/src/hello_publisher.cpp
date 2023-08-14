#include <chrono>
#include <memory>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node // inheritance here  from ROS RCLCPP node type
{
    public:
        MinimalPublisher() // constructor
        : Node("hello_publisher")
        {
            publisher_ = this->create_publisher<std_msgs::msg::String>("hello_topic",1);
            i = 0;
            timer_ = this->create_wall_timer(1000ms, std::bind(&MinimalPublisher::publish_message, this));   
        }

    private:
        void publish_message()
        {
            auto message = std::make_unique<std_msgs::msg::String>();
            message->data= "Hello out there! #:" + std::to_string(i);
            RCLCPP_INFO(this->get_logger(), "Sending message: %s", message->data.c_str());
            publisher_->publish(std::move(message));
            i++;
        }

        std_msgs::msg::String msg_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        int i;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;

}