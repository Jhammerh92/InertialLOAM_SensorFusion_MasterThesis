#include <memory>
#include <cstdio>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node{
    public:
        MinimalSubscriber() // constructer
            : Node("imu_calibrater")
            {
                // saturate_log(0.0);
                subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("imu/acceleration", 1, std::bind(&MinimalSubscriber::subscribe_message, this, _1));

                calib_timer_ = this->create_wall_timer(5000ms, std::bind(&MinimalSubscriber::calibrate, this));   

                RCLCPP_INFO(this->get_logger(), "Starting calibration.. ");
                RCLCPP_INFO(this->get_logger(), "Flushing the log, recording in 5 sec");
                // std::cin.get();
            }

    private:
        void subscribe_message(const geometry_msgs::msg::Vector3Stamped::SharedPtr message)
            {
                log_value(message->vector.z);              
            }

        void calibrate()
        {
            if (step == -1){
                RCLCPP_INFO(get_logger(), "Recording, keep the IMU still..");
            }

            else if (step == 0)
            {
                z_positive = mean_of_log();
                z_std = std_of_log();
                RCLCPP_INFO(this->get_logger(), "z_positive logged as %f with std of %f", z_positive, z_std);
                RCLCPP_INFO(this->get_logger(), "flip the imu and hold still..");
                
                RCLCPP_INFO(this->get_logger(), "Recording in 5 sec..");
            }

            else if (step == 1)
            {
                RCLCPP_INFO(this->get_logger(), "Recording started, keep the imu still...");

            }

            else if (step == 2)
            {
                z_negative = mean_of_log();
                z_std = std_of_log();
                RCLCPP_INFO(this->get_logger(), "z_negative logged as %f with std of %f", z_negative, z_std);
            }


            else if (step == 3 || step == 2 )
            {
                double estimated_g = (z_positive - z_negative) / 2;
                double calibration_value = (z_positive + z_negative) / 2;

                RCLCPP_INFO(this->get_logger(), "Positive recording: %f, Negative recording: %f, estimated g: %f, calibration value: %f", z_positive,z_negative, estimated_g, calibration_value );
            }
            step++;            
            
            
        }

        void log_value(double value)
        {
            //float temp = meas_log[0];
            memmove(meas_log, meas_log+1, sizeof(double)*(log_size-1)); // shift the array elements left, also deletes the first value
            meas_log[log_size-1] = value; // add the new value to the end
        }

        double sum_of_log()
        {
            double sum = 0.0;
            for (int i =0; i < log_size; i++) {
                sum += meas_log[i];
            }
            return sum;
        }
        
        double mean_of_log()
        {
            return sum_of_log()/log_size;
        }

        double std_of_log()
        {
            double s_sum = 0.0;
            double mean = mean_of_log();
            for (int i=0; i< log_size; i++){
                double res = meas_log[i] - mean;
                s_sum +=  res * res;
            }
            
            return std::sqrt(s_sum/(log_size - 1)); // sampled standard deviation
        }

        void saturate_log(double value)// fill array with a value
        {
            for (int i=0; i > log_size ; i++ ){ 
                meas_log[i] = value;
            }

        }

        rclcpp::TimerBase::SharedPtr calib_timer_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr subscriber_; 
        static const int log_size = 1000; // at 400 hz this is filled in 2.5 sec
        double meas_log[log_size];

        double z_positive = 0.0;
        double z_negative = 0.0;
        double z_std = 0.0;

        int step = -1;

};



            // int step_times[] = {3, 8, 11, 16, 18, 1000};
            // std::string instructions[] = {
            //     "Starting calibration - Put IMU on flat surface and hold still press enter to start.",
            //     "Recording positive Z-axis..  ",
            //     "Flip the IMU on its back, or hold it on the underside of a table and hold still..  ",
            //     "Recording negative Z-axis..  ",
            //     "Calibration result:",
            //     "Done:"
            //     };

            // if (start_time == 0){ // set start time to track lapsed time via the message stamps
                
            //     RCLCPP_INFO(this->get_logger(), instructions[0]);
            //     std::cin.get(); // wait for enter to start
            //     start_time = message->header.stamp.sec;
            //     RCLCPP_INFO(this->get_logger(), "you clicked enter hurray! priming log.."); // see if it works.
            // }
            // int lapsed_time = message->header.stamp.sec - start_time; // get lapsed time
            // if (lapsed_time >= step_times[step]) { // go to next calibration step at set times
            //         step++;
            // } 

            // RCLCPP_INFO(this->get_logger(), instructions[step]);

            // switch (step)
            // {
            //     case 0:
            //     { 
            //         log_value(message->vector.z);
            //         break;
            //     }
            //     case 1:
            //     {
            //         log_value(message->vector.z);
            //         break;
            //     }
            //     case 2:
            //     {   
            //         if (z_positive == 0.0 ){
            //             z_positive = mean_of_log();
            //         }
            //         //log_value(message->vector.z);
            //         break;
            //     }
            //     case 3:
            //     {
            //         log_value(message->vector.z);
            //         break;
            //     }
            //     case 4:
            //     {
            //         if (z_negative == 0.0 ){
            //             z_negative = mean_of_log();
            //         }
            //         log_value(message->vector.z);
            //         break;
            //     }
            //     case 5:
            //     {
            //         double estimated_g = (z_positive - z_negative) / 2;
            //         double calibration_value = (z_positive + z_negative) / 2;

            //         RCLCPP_INFO(this->get_logger(), "Positive recording: %f, Negative recording: %f, estimated g: %f, calibration value: %f", z_positive, z_negative, estimated_g, calibration_value );
            //         std::cin.get();
            //     }
                
            // }
        
            // return;
             



            // void log_value(double value)
            // {
            //     //float temp = meas_log[0];
            //     memmove(meas_log, meas_log+1, sizeof(double)*(log_size-1)); // shift the array elements left, also deletes the first value
            //     meas_log[log_size-1] = value; // add the new value to the end
            // }


            // void saturate_log(double value)// fill array with a value
            // {
            //     for (int i=0; i > log_size ; i++ ){ 
            //         meas_log[i] = value;
            //     }

            // }


            // int start_time = 0;
            // static const int log_size = 1000; // at 400 hz this is filled in 2.5 sec
            // double meas_log_[];

  

            // int step = 0;




int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // rclcpp::executors::MultiThreadedExecutor executor;


    // executor.add_node(logger);
    // executor.add_node(calib);

    // executor.spin();
    rclcpp::spin(std::make_shared<MinimalSubscriber>()); // should be renamed to something with calibration

    rclcpp::shutdown();
    return 0;
}