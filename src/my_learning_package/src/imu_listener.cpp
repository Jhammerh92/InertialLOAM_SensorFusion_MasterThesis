#include <memory>
#include <cstdio>
#include <cmath>
#include <queue>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"


using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
    public:
        MinimalSubscriber() // constructer
        : Node("imu_listener")
        {
            //double meas_log[100];//empty array to log values in
            for (int i=0; i > log_size ; i++ ){ // fill array with a value
                meas_log[i] = 0.0;
            }
            subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("imu/acceleration", 1, std::bind(&MinimalSubscriber::subscribe_message, this, _1));
        }

    private:
        void subscribe_message(const geometry_msgs::msg::Vector3Stamped::SharedPtr message)
            {
                log_value(message->vector.z);
                double mean = mean_of_log();
                RCLCPP_INFO(this->get_logger(),"Z-axis MEAN accel from IMU: %f, log size %i", mean, log_size);                
            }

        void log_value(double value)
            {
                //float temp = meas_log[0];
                memmove(meas_log, meas_log+1, sizeof(double)*log_size-1); // shift the array elements left
                meas_log[log_size-1] = value; // add the new value
            }
        double mean_of_log(void)
        {
            double sum = 0.0;
            for (int i =0; i < log_size; i++) {
                sum += meas_log[i];
            }
            return sum/log_size;
        }
        
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr subscriber_;
        static const int log_size = 500;
        double meas_log[log_size];


        
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}