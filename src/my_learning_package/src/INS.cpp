
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

// struct PoseInfo
// {
//     double x;
//     double y;
//     double z;
//     double qw;
//     double qx;
//     double qy;
//     double qz;
//     int idx;
//     double time;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;

// POINT_CLOUD_REGISTER_POINT_STRUCT (PoseInfo,
//                                    (double, x, x) (double, y, y) (double, z, z)
//                                    (double, qw, qw) (double, qx, qx) (double, qy, qy) (double, qz, qz)
//                                    (int, idx, idx) (double, time, time)
//                                    )



// put the following in a genereal header..
// typedef pcl::PointXYZINormal PointType; // need to calculate and assign normals
// typedef pcl::PointXYZINormal PointType; // need to calculate and assign normals

class INS : public rclcpp::Node
{
    private:
        rclcpp::CallbackGroup::SharedPtr sub1_cb_group_;
        // rclcpp::CallbackGroup::SharedPtr sub2_cb_group_;

        rclcpp::CallbackGroup::SharedPtr run_cb_group_;
        rclcpp::TimerBase::SharedPtr run_timer;

        


        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr keyframe_odom_sub;

        
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ins_pub;
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
        double g_acc_var_measurement;
        double k_gain;
        Eigen::Vector3d g_vec; // normal vector of gravity


        int step{};
        int step_since_keyframe{};

        Eigen::Vector3d gravity_vector;

        Eigen::Vector3d acceleration;
        Eigen::Vector3d acceleration_post;
        Eigen::Vector3d velocity;
        Eigen::Vector3d position;
 

        Eigen::Vector3d angular_velocity;
        // Eigen::Vector3d angular_velocity_measured;

        Eigen::Quaterniond orientation;
        Eigen::Quaterniond orientation_post;

        // Eigen::Vector3d jerk;
        // Eigen::Vector3d angular_acceleration;

        // double alpha_;
        // double beta_;


        std::string imu_topic_;


        deque<sensor_msgs::msg::Imu::SharedPtr> imu_delay_buffer;

        nav_msgs::msg::Odometry INS_odometry;

        int imu_step_delay_;

        bool print_states_{};

        // std_msgs::msg::Header imu_data_header;
        // sensor_msgs::msg::Imu filtered_imu_data;

    
    
    public:
        INS() // constructer
        : Node("IMUFilter")
        {   
            declare_parameter("imu_dt", 0.01);
            get_parameter("imu_dt", imu_dt_);

            // declare_parameter("alpha", 0.5);
            // get_parameter("alpha", alpha_);

            // declare_parameter("beta", 0.1);
            // get_parameter("beta", beta_);

            // declare_parameter("imu_step_delay", 0);
            // get_parameter("imu_step_delay", imu_step_delay_);

            // declare_parameter("init_calib_steps", 0);
            // get_parameter("init_calib_steps", init_calib_steps_);
            
            declare_parameter("imu_topic", "/imu/data");
            get_parameter("imu_topic", imu_topic_);

            // declare_parameter("print_states", true);
            // get_parameter("print_states", print_states_);

            // RCLCPP_INFO(get_logger(),"alpha set to %f", alpha_);
            // RCLCPP_INFO(get_logger(),"beta set to %f", beta_);
            RCLCPP_INFO(get_logger(),"INS listning on topic %s", imu_topic_.c_str());

            

            run_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            run_timer = this->create_wall_timer(5000ms, std::bind(&INS::correctINSPose, this), run_cb_group_);

            sub1_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options1;
            options1.callback_group = sub1_cb_group_;

            // sub2_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            // rclcpp::SubscriptionOptions options2;
            // options2.callback_group = sub2_cb_group_;


            imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_, 100, std::bind(&INS::imuDataHandler, this, _1), options1);
            // odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 100, std::bind(&EKF::odometryHandler, this, _1), options2);
            // keyframe_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom_keyframe", 100, std::bind(&EKF::keyframeHandler, this, _1), options2);

            ins_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom_ins", 100);
            // odometry_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom_kalman", 100);
            // path_pub = this->create_publisher<nav_msgs::msg::Path>("/path_kalman", 100);
            // initial_pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initial_pose", 100);
            

            initializeArrays();

        }
        ~INS(){}

        void initializeArrays()
        {   
            step = 0;

            g_acc = 9.825;
            g_acc_var = 0.01;
            g_acc_sum = 0.0;
            g_acc_sumsq = 0.0;
            g_vec << 0.0, 0.0, 1.0;
   
            acceleration << 0.0,0.0,0.0;
            acceleration_post << 0.0,0.0,0.0;
            velocity << 0.0,0.0,0.0;
            position << 0.0,0.0,0.0;
            gravity_vector << 0.0,0.0,1.0;
            // jerk << 0.0,0.0,0.0;
            // angular_velocity << 0.0,0.0,0.0;
            orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
            // orientation_post = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

            INS_odometry.child_frame_id = "ins";

        }


        
        void imuDataHandler(const sensor_msgs::msg::Imu::SharedPtr imu_data)
        {
            RCLCPP_INFO_ONCE(get_logger(), "INS First imu msg recieved..");
            step++;
            // put data into buffer back
            imu_delay_buffer.push_back(imu_data);

            if (step < imu_step_delay_){ // if step is less than imu_step_delay return
                return;
            }

            // get data from buffer front
            sensor_msgs::msg::Imu::SharedPtr delayed_imu_data = imu_delay_buffer.back();
            imu_delay_buffer.pop_back();

            INS_odometry.header = delayed_imu_data->header;
            INS_odometry.header.frame_id = "odom";

            // imu_data_header = delayed_imu_data->header;
            // filtered_imu_data = *delayed_imu_data;

            Eigen::Vector3d acceleration_in(delayed_imu_data->linear_acceleration.x,
                                            delayed_imu_data->linear_acceleration.y,
                                            delayed_imu_data->linear_acceleration.z);

            angular_velocity << delayed_imu_data->angular_velocity.x,
                                delayed_imu_data->angular_velocity.y,
                                delayed_imu_data->angular_velocity.z;

            Eigen::Quaterniond orientation_in(delayed_imu_data->orientation.w,
                                              delayed_imu_data->orientation.x,
                                              delayed_imu_data->orientation.y, 
                                              delayed_imu_data->orientation.z);

            if (step < 10000){
                updateGravityCalibration(acceleration_in);
            }


            updateOrientation(orientation_in);
            updateAcceleration(acceleration_in);
            updateVelocity();
            updatePosition();


            publishINS( );

        }   

        void updateOrientation(Eigen::Quaterniond q_in ){
            if (step == 1){
                orientation_post = q_in;
            } 
            orientation = orientation_post;
            orientation_post = q_in; // spherical linear interpolation between q_in and ori.
        }

        void updateAcceleration(Eigen::Vector3d acc_in)
        {   
            Eigen::Quaterniond avg_quat = orientation_post.slerp(0.5, orientation);
            // if (step == 1){
            //     acceleration_post = avg_quat*acc_in;
            // } 
            gravity_vector << 0.0, 0.0, -g_acc;
            acceleration = avg_quat*acc_in;
            acceleration = acceleration + gravity_vector;
        }

        void updateVelocity()
        {   
            velocity = velocity + acceleration * imu_dt_;
        }

        void updatePosition()
        {   
            position = position + velocity * imu_dt_ - acceleration * imu_dt_*imu_dt_ *0.5;
        }
        
        void publishINS()
        {
            
            // data
            INS_odometry.pose.pose.position.x = position[0];
            INS_odometry.pose.pose.position.y = position[1];
            INS_odometry.pose.pose.position.z = position[2];
            
            INS_odometry.pose.pose.orientation.w = orientation.w();
            INS_odometry.pose.pose.orientation.x = orientation.x();
            INS_odometry.pose.pose.orientation.y = orientation.y();
            INS_odometry.pose.pose.orientation.z = orientation.z();

            INS_odometry.twist.twist.linear.x = velocity[0];
            INS_odometry.twist.twist.linear.y = velocity[1];
            INS_odometry.twist.twist.linear.z = velocity[2];

            INS_odometry.twist.twist.angular.x = angular_velocity[0];
            INS_odometry.twist.twist.angular.y = angular_velocity[1];
            INS_odometry.twist.twist.angular.z = angular_velocity[2];


            ins_pub->publish(INS_odometry);


        }

        void correctINSPose(){
            // update position and velocity from the kalman filter
            
            position << 0.0,0.0,0.0;
            velocity << 0.0,0.0,0.0;
            
            // INS_odometry.pose.pose.orientation.w = orientation.w();
            // INS_odometry.pose.pose.orientation.x = orientation.x();
            // INS_odometry.pose.pose.orientation.y = orientation.y();
            // INS_odometry.pose.pose.orientation.z = orientation.z();

            // INS_odometry.twist.twist.linear.x = 0.0;
            // INS_odometry.twist.twist.linear.y = 0.0;
            // INS_odometry.twist.twist.linear.z = 0.0;

            // INS_odometry.twist.twist.angular.x = 0.0;
            // INS_odometry.twist.twist.angular.y = 0.0;
            // INS_odometry.twist.twist.angular.z = 0.0;
        }


        void updateGravityCalibration(Eigen::Vector3d static_acc)
        {
            if (step == 1)
                return;
            // using Naïve algotihm, see wiki "algorithm for calculating variance".
            double new_g_acc = static_acc.norm();
            Eigen::Vector3d new_g_vec = static_acc.normalized();

            g_acc_sum += new_g_acc;
            g_acc_sumsq += new_g_acc*new_g_acc;
            g_acc_var_measurement = (g_acc_sumsq - (g_acc_sum*g_acc_sum)/(double)step) / ((double)step - 1.0); // measuremet uncertainty 
            // get mean of running measurements
            double g_acc_mean = g_acc_sum / (double)step;

            if (step < 50)
                return;
            // kalman gain
            k_gain = g_acc_var / (g_acc_var + g_acc_var_measurement);

            // update uncertainty of estimate
            g_acc_var = (1 - k_gain) * g_acc_var;

            // update estimate with mean 
            g_acc = g_acc + k_gain * (g_acc_mean - g_acc);

            g_vec = (1 - k_gain) *  new_g_vec;


            RCLCPP_INFO(get_logger(), "Static Calibration: gravity estimate: %f m/s²", g_acc);
            RCLCPP_INFO(get_logger(), "Static Calibration: std: %f m/s²", sqrt(g_acc_var));
            RCLCPP_INFO(get_logger(), "Static Calibration: measurement std: %f",sqrt(g_acc_var_measurement));
            RCLCPP_INFO(get_logger(), "Static Calibration: update gain: %f", k_gain);
            RCLCPP_INFO(get_logger(), "Static Calibration: gravity norm vector: %f %f %f", g_vec[0], g_vec[1], g_vec[2] );
                    // RCLCPP_INFO(get_logger(), "Static Calibration: pitch: %f deg, roll: %f deg", init_calib_pitch *57.3, init_calib_roll*57.3 );


            // find pitch and roll from gravity vector
            // double calib_roll = std::atan2(g_vec[1], g_vec[2]);
            // double z_prime = g_vec[1] * std::sin(-roll) + g_vec[2] * std::cos(-roll);
            // double calib_pitch = std::atan2(-g_vec[0], z_prime);
            // double calib_yaw = 0.0; // yaw is assumed zero as this is equal to the inital heading.

            // if (step < 10){
            //     init_calib_roll  = calib_roll;
            //     init_calib_pitch = calib_pitch;
            //     init_calib_yaw   = calib_yaw;
            // } else {
            //     init_calib_roll  +=  k_gain * (calib_roll  - init_calib_roll);
            //     init_calib_pitch +=  k_gain * (calib_pitch - init_calib_pitch);
            //     init_calib_yaw   +=  k_gain * (calib_yaw   - init_calib_yaw);
            // }

            // // update states and saturates biases
            // Eigen::Vector2d k_gain_vec(0.5, 0.5);
            // state_roll  += k_gain_vec * (init_calib_roll  - H_ori * state_roll); 
            // state_pitch += k_gain_vec * (init_calib_pitch - H_ori * state_pitch); 
            // state_yaw   += k_gain_vec * (init_calib_yaw   - H_ori * state_yaw); 



            // the orientation found needs to be send to some initial pose in ros or the lidar_odometry, have lidar_odometry subsribe to initial pose, the imu data comes before the lidar

            // geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg;
            // if (use_madgwick_orientation_) {
            //     initial_pose_msg.pose.pose.orientation.w =  madgwick_orientation_quaternion.w();
            //     initial_pose_msg.pose.pose.orientation.x =  madgwick_orientation_quaternion.x();
            //     initial_pose_msg.pose.pose.orientation.y =  madgwick_orientation_quaternion.y();
            //     initial_pose_msg.pose.pose.orientation.z =  madgwick_orientation_quaternion.z();
            // } else {
            //     // updateOrientationQuaternion(init_calib_roll, init_calib_pitch, init_calib_yaw);
            //     initial_pose_msg.pose.pose.orientation.w =  orientation_quaternion.w();
            //     initial_pose_msg.pose.pose.orientation.x =  orientation_quaternion.x();
            //     initial_pose_msg.pose.pose.orientation.y =  orientation_quaternion.y();
            //     initial_pose_msg.pose.pose.orientation.z =  orientation_quaternion.z();
            // }
            // initial_pose_msg.pose.pose.position.x = 0.0;
            // initial_pose_msg.pose.pose.position.y = 0.0;
            // initial_pose_msg.pose.pose.position.z = 0.0;

            // initial_pose_pub->publish(initial_pose_msg);

        }



};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto ins_node = std::make_shared<INS>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(ins_node);
    
    executor.spin();
    rclcpp::shutdown();
    return 0;
}