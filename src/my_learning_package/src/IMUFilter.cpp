
#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/msg/imu.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <pcl/common/common.h>
// #include <pcl/common/transforms.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/conversions.h>
// #include <pcl/filters/filter.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl_conversions/pcl_conversions.h>




#include <memory>
#include <cstdio>
#include <cmath>
#include <queue>
#include <vector>
// #include <eigen>

using namespace std;
// using namespace Eigen;

using std::placeholders::_1;

struct PoseInfo
{
    double x;
    double y;
    double z;
    double qw;
    double qx;
    double qy;
    double qz;
    int idx;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PoseInfo,
                                   (double, x, x) (double, y, y) (double, z, z)
                                   (double, qw, qw) (double, qx, qx) (double, qy, qy) (double, qz, qz)
                                   (int, idx, idx) (double, time, time)
                                   )



// put the following in a genereal header..
// typedef pcl::PointXYZINormal PointType; // need to calculate and assign normals
// typedef pcl::PointXYZINormal PointType; // need to calculate and assign normals

class IMUFilter : public rclcpp::Node
{
    private:
        rclcpp::CallbackGroup::SharedPtr sub1_cb_group_;
        // rclcpp::CallbackGroup::SharedPtr sub2_cb_group_;

        // rclcpp::CallbackGroup::SharedPtr run_cb_group_;
        // rclcpp::TimerBase::SharedPtr run_timer;

        


        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr keyframe_odom_sub;

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_filter_pub;
        // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
        // rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

        // rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub;
     
      
        // rclcpp::Time time_latest_cloud;


        // NOTE: make filter or mechanization do the gravity estimate !!!


        double imu_dt_; 
        double g_acc; // gravity scalar value
        double g_acc_var;
        double g_acc_sum;
        double g_acc_sumsq;
        Eigen::Vector3d g_vec; // normal vector of gravity


        int step{};
        int step_since_keyframe{};
        int init_calib_steps_;


        // state of orientation, x with bias on orientation, [alpha, q_b]
        // Eigen::Vector2d state_pitch;  
        // Eigen::Vector2d state_yaw; 
        // Eigen::Vector2d state_roll; 

        // // state of position, [S, v, a_b]
        // Eigen::Vector3d state_x; 
        // Eigen::Vector3d state_y; 
        // Eigen::Vector3d state_z; 

        

        // prediction matrix, A aka F
        // Eigen::Matrix2d A_ori;
        // Eigen::Matrix2d A_ori_T;
        // // bias matrix, B
        // Eigen::Vector2d B_ori;
        // // measurement matrix H
        // Eigen::RowVector2d H_ori;
        // Eigen::Vector2d H_ori_T;

        // // prediction matrix, A aka F
        // Eigen::Matrix3d A_pos;
        // Eigen::Matrix3d A_pos_T;
        // // bias matrix, B
        // Eigen::Vector3d B_pos;
        // // measurement matrix H
        // Eigen::RowVector3d H_pos;
        // Eigen::Vector3d H_pos_T;



        Eigen::Vector3d acceleration;
        Eigen::Vector3d acceleration_measured;
        Eigen::Vector3d angular_velocity;
        Eigen::Vector3d angular_velocity_measured;

        Eigen::Vector3d jerk;
        Eigen::Vector3d angular_acceleration;

        double alpha_;
        double beta_;


        std::string imu_topic_;


        // Eigen::Quaterniond orientation_quaternion;
        // double roll;
        // double pitch;
        // double yaw;
        // double init_calib_roll;
        // double init_calib_pitch;
        // double init_calib_yaw;
        // double g_acc_var_measurement;
        // double k_gain;

        deque<sensor_msgs::msg::Imu::SharedPtr> imu_delay_buffer;

        int imu_step_delay_;

        bool print_states_{};

        // std_msgs::msg::Header imu_data_header;
        sensor_msgs::msg::Imu filtered_imu_data;

    
    
    public:
        IMUFilter() // constructer
        : Node("IMUFilter")
        {   
            declare_parameter("imu_dt", 0.01);
            get_parameter("imu_dt", imu_dt_);

            declare_parameter("alpha", 0.5);
            get_parameter("alpha", alpha_);

            declare_parameter("beta", 0.1);
            get_parameter("beta", beta_);

            // declare_parameter("imu_step_delay", 0);
            // get_parameter("imu_step_delay", imu_step_delay_);

            // declare_parameter("init_calib_steps", 0);
            // get_parameter("init_calib_steps", init_calib_steps_);
            
            declare_parameter("imu_topic", "/imu/data_raw");
            get_parameter("imu_topic", imu_topic_);

            // declare_parameter("print_states", true);
            // get_parameter("print_states", print_states_);

            RCLCPP_INFO(get_logger(),"alpha set to %f", alpha_);
            RCLCPP_INFO(get_logger(),"beta set to %f", beta_);
            RCLCPP_INFO(get_logger(),"listning on topic %s", imu_topic_.c_str());

            

            // run_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            // run_timer = this->create_wall_timer(1000ms, std::bind(&EKF::updateZeroMeasurement, this), run_cb_group_);

            sub1_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options1;
            options1.callback_group = sub1_cb_group_;

            // sub2_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            // rclcpp::SubscriptionOptions options2;
            // options2.callback_group = sub2_cb_group_;


            imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_, 100, std::bind(&IMUFilter::imuDataHandler, this, _1), options1);
            // odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 100, std::bind(&EKF::odometryHandler, this, _1), options2);
            // keyframe_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom_keyframe", 100, std::bind(&EKF::keyframeHandler, this, _1), options2);

            imu_filter_pub = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_filter", 100);
            // odometry_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom_kalman", 100);
            // path_pub = this->create_publisher<nav_msgs::msg::Path>("/path_kalman", 100);
            // initial_pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initial_pose", 100);
            

            initializeArrays();

        }
        ~IMUFilter(){}

        void initializeArrays()
        {   
            step = 0;
            // step_since_keyframe = 0;

            // g_acc = 9.822;
            // g_acc_var = 0.01;
            // g_acc_sum = 0.0;
            // g_acc_sumsq = 0.0;
            // g_vec << 0.0, 0.0, 1.0;

            acceleration << 0.0,0.0,0.0;
            jerk << 0.0,0.0,0.0;
            angular_velocity << 0.0,0.0,0.0;

            // imu_dt_ = 0.01; //TODO: make this a ros parameter
            // double imu_dt_sq = imu_dt_*imu_dt_;


            // // orientation
            // state_pitch = Eigen::Vector2d(0.0, 0.0);
            // state_yaw   = Eigen::Vector2d(0.0, 0.0);
            // state_roll  = Eigen::Vector2d(0.0, 0.0);

            // // position
            // state_x  = Eigen::Vector3d(0.0, 0.0, 0.0);
            // state_y  = Eigen::Vector3d(0.0, 0.0, 0.0);
            // state_z  = Eigen::Vector3d(0.0, 0.0, 0.0);

            // // orientation
            // A_ori   << 1.0, -imu_dt_,
            //            0.0, 1.0;
            // A_ori_T = A_ori.transpose();
            // B_ori   = Eigen::Vector2d(imu_dt_, 0.0);
            // H_ori   << 1.0, 0.0;
            // H_ori_T << 1.0, 0.0;

            // // position
            // A_pos   << 1.0, imu_dt_, -imu_dt_sq/2,
            //            0.0, 1.0, -imu_dt_,
            //            0.0, 0.0, 1.0;
            // A_pos_T = A_pos.transpose();
            // B_pos   = Eigen::Vector3d(imu_dt_sq/2, imu_dt_,  0.0);
            // H_pos   << 1.0, 0.0, 0.0;
            // H_pos_T << 1.0, 0.0, 0.0;


        }


        
        void imuDataHandler(const sensor_msgs::msg::Imu::SharedPtr imu_data)
        {
            RCLCPP_INFO_ONCE(get_logger(), "First imu msg recieved..");
            step++;
            // put data into buffer back
            imu_delay_buffer.push_back(imu_data);

            if (step < imu_step_delay_){ // if step is less than imu_step_delay return
                return;
            }

            // get data from buffer front
            sensor_msgs::msg::Imu::SharedPtr delayed_imu_data = imu_delay_buffer.back();
            imu_delay_buffer.pop_back();

            // imu_data_header = delayed_imu_data->header;
            filtered_imu_data = *delayed_imu_data;

            acceleration_measured << delayed_imu_data->linear_acceleration.x,
                                     delayed_imu_data->linear_acceleration.y,
                                     delayed_imu_data->linear_acceleration.z;

            angular_velocity_measured << delayed_imu_data->angular_velocity.x,
                                         delayed_imu_data->angular_velocity.y,
                                         delayed_imu_data->angular_velocity.z;


            updateAcceleration();
            updateAngularVelocity();


            publishFiltered();

        }   

        void updateAcceleration()
        {   
            if (step == 1) {
                acceleration = acceleration_measured;
                return;
            } 
            
            jerk = (jerk + (acceleration_measured - acceleration))/2 ;
            acceleration = acceleration + (1 - alpha_)*jerk;
        }

        void updateAngularVelocity()
        {   
            if (step == 1) {
                angular_velocity = angular_velocity_measured;
                return;
            } 
            
            angular_acceleration = (angular_acceleration + (angular_velocity_measured - angular_velocity))/2 ;
            angular_velocity = angular_velocity + (1 - beta_)*angular_acceleration;;
        }
        
        void publishFiltered()
        {
            
            // data
            filtered_imu_data.linear_acceleration.x = acceleration[0];
            filtered_imu_data.linear_acceleration.y = acceleration[1];
            filtered_imu_data.linear_acceleration.z = acceleration[2];

            filtered_imu_data.angular_velocity.x = angular_velocity[0];
            filtered_imu_data.angular_velocity.y = angular_velocity[1];
            filtered_imu_data.angular_velocity.z = angular_velocity[2];

            imu_filter_pub->publish(filtered_imu_data);


        }


        // void updateGravityCalibration(Eigen::Vector3d static_acc)
        // {
        //     if (step == 1)
        //         return;
        //     // using Naïve algotihm, see wiki "algorithm for calculating variance".
        //     double new_g_acc = static_acc.norm();
        //     Eigen::Vector3d new_g_vec = static_acc.normalized();

        //     g_acc_sum += new_g_acc;
        //     g_acc_sumsq += new_g_acc*new_g_acc;
        //     g_acc_var_measurement = (g_acc_sumsq - (g_acc_sum*g_acc_sum)/(double)step) / ((double)step - 1.0); // measuremet uncertainty 
        //     // get mean of running measurements
        //     double g_acc_mean = g_acc_sum / (double)step;

        //     // kalman gain
        //     k_gain = g_acc_var / (g_acc_var + g_acc_var_measurement);

        //     // update uncertainty of estimate
        //     g_acc_var = (1 - k_gain) * g_acc_var;

        //     // update estimate with mean 
        //     g_acc = g_acc + k_gain * (g_acc_mean - g_acc);


        //     g_vec = (1 - k_gain) *  new_g_vec;


        //     // find pitch and roll from gravity vector
        //     double calib_roll = std::atan2(g_vec[1], g_vec[2]);
        //     double z_prime = g_vec[1] * std::sin(-roll) + g_vec[2] * std::cos(-roll);
        //     double calib_pitch = std::atan2(-g_vec[0], z_prime);
        //     double calib_yaw = 0.0; // yaw is assumed zero as this is equal to the inital heading.

        //     if (step < 10){
        //         init_calib_roll  = calib_roll;
        //         init_calib_pitch = calib_pitch;
        //         init_calib_yaw   = calib_yaw;
        //     } else {
        //         init_calib_roll  +=  k_gain * (calib_roll  - init_calib_roll);
        //         init_calib_pitch +=  k_gain * (calib_pitch - init_calib_pitch);
        //         init_calib_yaw   +=  k_gain * (calib_yaw   - init_calib_yaw);
        //     }

        //     // update states and saturates biases
        //     Eigen::Vector2d k_gain_vec(0.9, 0.9);
        //     state_roll  += k_gain_vec * (init_calib_roll  - H_ori * state_roll); 
        //     state_pitch += k_gain_vec * (init_calib_pitch - H_ori * state_pitch); 
        //     state_yaw   += k_gain_vec * (init_calib_yaw   - H_ori * state_yaw); 



        //     // the orientation found needs to be send to some initial pose in ros or the lidar_odometry, have lidar_odometry subsribe to initial pose, the imu data comes before the lidar

        //     updateOrientationQuaternion();
        //     geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg;
        //     initial_pose_msg.pose.pose.orientation.w =  orientation_quaternion.w();
        //     initial_pose_msg.pose.pose.orientation.x =  orientation_quaternion.x();
        //     initial_pose_msg.pose.pose.orientation.y =  orientation_quaternion.y();
        //     initial_pose_msg.pose.pose.orientation.z =  orientation_quaternion.z();
        //     initial_pose_msg.pose.pose.position.x = 0.0;
        //     initial_pose_msg.pose.pose.position.y = 0.0;
        //     initial_pose_msg.pose.pose.position.z = 0.0;

        //     initial_pose_pub->publish(initial_pose_msg);

        // }

        // void printStates()
        // {   
        //     if (print_states_) {

        //         // if init_calib_steps > 0
        //         RCLCPP_INFO(get_logger(), "test..");
        //         // RCLCPP_INFO(get_logger(), "Static Calibration: gravity estimate: %f m/s², std: %f m/s², measurement std: %f, update gain: %f", g_acc, sqrt(g_acc_var), sqrt(g_acc_var_measurement), k_gain);
        //         // RCLCPP_INFO(get_logger(), "Static Calibration: gravity norm vector: %f %f %f", g_vec[0], g_vec[1], g_vec[2] );
        //         // RCLCPP_INFO(get_logger(), "Static Calibration: pitch: %f deg, roll: %f deg", init_calib_pitch *57.3, init_calib_roll*57.3 );

        //         // RCLCPP_INFO(get_logger(), "Lidar measurement: ori = %f, %f, %f", roll, pitch, yaw);
        //         // // RCLCPP_INFO(get_logger(), "Lidar measurement: pos = %f, %f, %f",odom_message->pose.pose.position.x, odom_message->pose.pose.position.y, odom_message->pose.pose.position.z);
        //         // RCLCPP_INFO(get_logger(), "Lidar measurement: pos state = %f, %f, %f, vel = %f, %f, %f, bias = %f, %f, %f, gain = %f, %f, %f", state_x[0], state_y[0], state_z[0], state_x[1], state_y[1], state_z[1], state_x[2], state_y[2], state_z[2], gain_x[0], gain_y[0], gain_z[0]);
        //         // RCLCPP_INFO(get_logger(), "lidar measurement: pos uncertainty = %f, %f, %f", uncertainty_x(0,0), uncertainty_y(0,0), uncertainty_z(0,0));

        //         // RCLCPP_INFO(get_logger(), "update: pos state = %f, %f, %f, vel = %f, %f, %f, bias = %f %f %f", state_x[0], state_y[0], state_z[0], state_x[1], state_y[1], state_z[1], state_x[2], state_y[2], state_z[2]);
        //         // RCLCPP_INFO(get_logger(), "update: pos uncertainty = %f, %f, %f", uncertainty_x(0,0), uncertainty_y(0,0), uncertainty_z(0,0));
        //         // RCLCPP_INFO(get_logger(), "update: ori state = %f, %f, %f, bias = %f %f %f", state_roll[0], state_pitch[0], state_yaw[0], state_roll[1], state_pitch[1], state_yaw[1]);
        //         // RCLCPP_INFO(get_logger(), "update: ori uncertainty = %f, %f, %f", uncertainty_roll(0,0), uncertainty_pitch(0,0), uncertainty_yaw(0,0));
        //     }
        // }





};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto ekf_node = std::make_shared<IMUFilter>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(ekf_node);
    
    executor.spin();
    rclcpp::shutdown();
    return 0;
}