
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

class IMUConverter : public rclcpp::Node
{
    private:
        rclcpp::CallbackGroup::SharedPtr sub1_cb_group_;
        // rclcpp::CallbackGroup::SharedPtr sub2_cb_group_;

        // rclcpp::CallbackGroup::SharedPtr run_cb_group_;
        // rclcpp::TimerBase::SharedPtr run_timer;

        


        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr keyframe_odom_sub;

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
        // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
        // rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

        // rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub;
     
      
        // rclcpp::Time time_latest_cloud;


        // NOTE: make filter or mechanization do the gravity estimate !!!


        // double imu_dt_; 
        double g_scale_; // gravity scalar value


        int step{};
        int step_since_keyframe{};
        int init_calib_steps_;




        Eigen::Vector3d acceleration;
        Eigen::Vector3d acceleration_measured;
        Eigen::Vector3d angular_velocity;
        Eigen::Vector3d angular_velocity_measured;

        Eigen::Vector3d jerk;
        Eigen::Vector3d angular_acceleration;

        // double alpha_;
        // double beta_;


        std::string imu_topic_;


        deque<sensor_msgs::msg::Imu::SharedPtr> imu_delay_buffer;

        int imu_step_delay_;

        bool print_states_{};

        // std_msgs::msg::Header imu_data_header;
        sensor_msgs::msg::Imu filtered_imu_data;

    
    
    public:
        IMUConverter() // constructer
        : Node("imu_hap_converter")
        {   
            // declare_parameter("imu_dt", 0.01);
            // get_parameter("imu_dt", imu_dt_);

            declare_parameter("g_scale", 9.815);
            get_parameter("g_scale", g_scale_);

            // declare_parameter("alpha", 0.5);
            // get_parameter("alpha", alpha_);

            // declare_parameter("beta", 0.1);
            // get_parameter("beta", beta_);

            // declare_parameter("imu_step_delay", 0);
            // get_parameter("imu_step_delay", imu_step_delay_);

            // declare_parameter("init_calib_steps", 0);
            // get_parameter("init_calib_steps", init_calib_steps_);
            
            declare_parameter("imu_topic", "/livox/imu");
            get_parameter("imu_topic", imu_topic_);

            // declare_parameter("print_states", true);
            // get_parameter("print_states", print_states_);

            // RCLCPP_INFO(get_logger(),"alpha set to %f", alpha_);
            // RCLCPP_INFO(get_logger(),"beta set to %f", beta_);
            // RCLCPP_INFO(get_logger(),"listning on topic %s", imu_topic_.c_str());

            

            // run_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            // run_timer = this->create_wall_timer(1000ms, std::bind(&EKF::updateZeroMeasurement, this), run_cb_group_);

            sub1_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options1;
            options1.callback_group = sub1_cb_group_;

            // sub2_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            // rclcpp::SubscriptionOptions options2;
            // options2.callback_group = sub2_cb_group_;


            imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_, 100, std::bind(&IMUConverter::imuDataHandler, this, _1), options1);
            // odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 100, std::bind(&EKF::odometryHandler, this, _1), options2);
            // keyframe_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom_keyframe", 100, std::bind(&EKF::keyframeHandler, this, _1), options2);

            imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 100);
            // odometry_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom_kalman", 100);
            // path_pub = this->create_publisher<nav_msgs::msg::Path>("/path_kalman", 100);
            // initial_pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initial_pose", 100);
            

            initializeArrays();

        }
        ~IMUConverter(){}

        void initializeArrays()
        {   
            step = 0;
   
            acceleration << 0.0,0.0,0.0;
            jerk << 0.0,0.0,0.0;
            angular_velocity << 0.0,0.0,0.0;

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


            publish();

        }   

        void updateAcceleration()
        {   
            
            acceleration = acceleration_measured * g_scale_;

        }

        void updateAngularVelocity()
        {   

            angular_velocity = angular_velocity_measured;
        }
        
        void publish()
        {
            // data
            filtered_imu_data.linear_acceleration.x = acceleration[0];
            filtered_imu_data.linear_acceleration.y = acceleration[1];
            filtered_imu_data.linear_acceleration.z = acceleration[2];

            filtered_imu_data.angular_velocity.x = angular_velocity[0];
            filtered_imu_data.angular_velocity.y = angular_velocity[1];
            filtered_imu_data.angular_velocity.z = angular_velocity[2];

            imu_pub->publish(filtered_imu_data);
        }


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto convert_node = std::make_shared<IMUConverter>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(convert_node);
    
    executor.spin();
    rclcpp::shutdown();
    return 0;
}