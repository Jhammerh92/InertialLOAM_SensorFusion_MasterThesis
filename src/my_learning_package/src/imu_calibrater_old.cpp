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
        : Node("imu_calibrater")
        {
            saturate_log(0.0);
            subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("imu/acceleration", 1, std::bind(&MinimalSubscriber::calibrate, this, _1));
        }

    private:
        void subscribe_message(const geometry_msgs::msg::Vector3Stamped::SharedPtr message)
            {
                log_value(message->vector.z);
                // double mean = mean_of_log();
                // RCLCPP_INFO(this->get_logger(),"Z-axis MEAN accel from IMU: %f, log size %i", mean, log_size);                
            }

        void calibrate(const geometry_msgs::msg::Vector3Stamped::SharedPtr message)
        {
            int step_times[] = {3, 8, 11, 16, 18, 1000};
            std::string instructions[] = {
                "Starting calibration - Put IMU on flat surface and hold still press enter to start.",
                "Recording positive Z-axis..  ",
                "Flip the IMU on its back, or hold it on the underside of a table and hold still..  ",
                "Recording negative Z-axis..  ",
                "Calibration result:",
                "Done:"
                };

            if (start_time == 0){ // set start time to track lapsed time via the message stamps
                RCLCPP_INFO(this->get_logger(), instructions[0]);
                std::cin.get(); // wait for enter to start
                start_time = message->header.stamp.sec;
                RCLCPP_INFO(this->get_logger(), "you clicked enter hurray! priming log.."); // see if it works.
            }
            int lapsed_time = message->header.stamp.sec - start_time; // get lapsed time
            if (lapsed_time >= step_times[step]) { // go to next calibration step at set times
                    step++;
            } 

            RCLCPP_INFO(this->get_logger(), instructions[step]);

            switch (step)
            {
                case 0:
                { 
                    log_value(message->vector.z);
                    break;
                }
                case 1:
                {
                    log_value(message->vector.z);
                    break;
                }
                case 2:
                {   
                    if (z_positive == 0.0 ){
                        z_positive = mean_of_log();
                    }
                    //log_value(message->vector.z);
                    break;
                }
                case 3:
                {
                    log_value(message->vector.z);
                    break;
                }
                case 4:
                {
                    if (z_negative == 0.0 ){
                        z_negative = mean_of_log();
                    }
                    log_value(message->vector.z);
                    break;
                }
                case 5:
                {
                    double estimated_g = (z_positive - z_negative) / 2;
                    double calibration_value = (z_positive + z_negative) / 2;

                    RCLCPP_INFO(this->get_logger(), "Positive recording: %f, Negative recording: %f, estimated g: %f, calibration value: %f", z_positive, z_negative, estimated_g, calibration_value );
                    std::cin.get();
                }
                
            }
          
            return;
        }

        void log_value(double value)
            {
                //float temp = meas_log[0];
                memmove(meas_log, meas_log+1, sizeof(double)*(log_size-1)); // shift the array elements left
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

        void saturate_log(double value)// fill array with a value
        {
            for (int i=0; i > log_size ; i++ ){ 
                meas_log[i] = value;
            }

        }
        
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr subscriber_;
        int start_time = 0;
        static const int log_size = 1000; // at 400 hz this is filled in 2.5 sec
        double meas_log[log_size];

        double z_positive = 0.0;
        double z_negative = 0.0;

        int step = 0;
        



        
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}