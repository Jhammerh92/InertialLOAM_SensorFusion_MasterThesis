#include <memory>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
    public:
        MinimalSubscriber()
        : Node("hello_subsriber")
        {
            subscriber_ = this->create_subscription<std_msgs::msg::String>("hello_topic",1, std::bind(&MinimalSubscriber::subscribe_message, this, _1));
        }
    private:
        void subscribe_message(const std_msgs::msg::String::SharedPtr message) const
            {
                RCLCPP_INFO(this->get_logger(),"Recieved message: %s", message->data.c_str());
            }
        
        
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}