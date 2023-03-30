
#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
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

struct IMUwrench
{
    Eigen::Vector3d acc;
    Eigen::Vector3d ang;
    double time;
};

struct INSstate
{
    Eigen::Vector3d acc;
    Eigen::Vector3d vel;
    Eigen::Vector3d pos;
    Eigen::Quaterniond ori;
    double time;
    IMUwrench bias;
};

class INS : public rclcpp::Node
{
    private:
        rclcpp::CallbackGroup::SharedPtr sub1_cb_group_;
        rclcpp::CallbackGroup::SharedPtr sub2_cb_group_;

        rclcpp::CallbackGroup::SharedPtr run_cb_group_;
        rclcpp::CallbackGroup::SharedPtr correction_cb_group_;
        rclcpp::TimerBase::SharedPtr run_timer;
        rclcpp::TimerBase::SharedPtr correction_timer;

        


        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr bias_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr keyframe_odom_sub;

        
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ins_pub;
        
        // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

        // rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub;
     
      
        // rclcpp::Time time_latest_cloud;


        // NOTE: make filter or mechanization do the gravity estimate !!!


        double imu_dt_; 
        double scan_dt{};
        double scan_timestamp{};
        double process_time{};
        double prediction_time{};

        double calibration_time{};
        
        double g_acc_; // gravity scalar value
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
        Eigen::Vector3d acceleration_bias;
        Eigen::Vector3d acceleration_offset;
        Eigen::Vector3d acceleration_post;
        Eigen::Vector3d velocity;
        Eigen::Vector3d position;

        Eigen::Vector3d omega;
        Eigen::Vector3d alpha;
 

        Eigen::Vector3d angular_velocity;
        Eigen::Vector3d previous_angular_velocity;
        Eigen::Vector3d angular_velocity_bias;
        Eigen::Vector3d angular_velocity_offset;
        // Eigen::Vector3d angular_velocity_measured;

        Eigen::Quaterniond orientation;
        Eigen::Quaterniond orientation_post;
        Eigen::Quaterniond orientation_dt;

        Eigen::Vector3d ego_to_imu_offset;

        // Eigen::Vector3d jerk;
        Eigen::Vector3d angular_acceleration;

        // double alpha_;
        // double beta_;


        std::string imu_topic_;


        deque<double> g_buffer;
        deque<Eigen::Vector3d> g_vec_buffer;

        deque<IMUwrench> bias_buffer;
        deque<IMUwrench> predict_buffer;  // the inputs used in the prediction step that need to be reapplied from the anchor with corrected a bias
        INSstate prediction_anchor;
        IMUwrench previous_bias;
        IMUwrench current_bias;
        

        deque<sensor_msgs::msg::Imu::SharedPtr> imu_delay_buffer;

        deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer;
        sensor_msgs::msg::Imu::SharedPtr delayed_imu_data;

        nav_msgs::msg::Odometry INS_odometry;
        nav_msgs::msg::Path path;

        int imu_step_delay_;
        int turn_on_bias_estimation_period_;
        int gravity_estimation_period_;

        bool print_states_{};
        bool use_madgwick_orientation_{};
        bool use_ficticiuos_force_correction_{};
        bool use_kalman_bias_correction_{};
        bool predict_next_scan_{};

        // std_msgs::msg::Header imu_data_header;
        // sensor_msgs::msg::Imu filtered_imu_data;

    
    
    public:
        INS() // constructer
        : Node("IMUFilter")
        {   
            declare_parameter("imu_dt", 0.01);
            get_parameter("imu_dt", imu_dt_);

            declare_parameter("g_acc", 9.88);
            get_parameter("g_acc", g_acc_);

            declare_parameter("turn_on_bias_estimation_period", 0);
            get_parameter("turn_on_bias_estimation_period", turn_on_bias_estimation_period_);

            declare_parameter("gravity_estimation_period", 100);
            get_parameter("gravity_estimation_period", gravity_estimation_period_);

            // declare_parameter("imu_step_delay", 0);
            // get_parameter("imu_step_delay", imu_step_delay_);

            // declare_parameter("init_calib_steps", 0);
            // get_parameter("init_calib_steps", init_calib_steps_);
            
            declare_parameter("imu_topic", "/imu/data");
            get_parameter("imu_topic", imu_topic_);

            // declare_parameter("print_states", true);
            // get_parameter("print_states", print_states_);

            declare_parameter("use_madgwick_orientation", false);
            get_parameter("use_madgwick_orientation", use_madgwick_orientation_);

            declare_parameter("use_ficticiuos_force_correction", false);
            get_parameter("use_ficticiuos_force_correction", use_ficticiuos_force_correction_);

            declare_parameter("use_kalman_bias_correction", false);
            get_parameter("use_kalman_bias_correction", use_kalman_bias_correction_);

            declare_parameter("predict_next_scan", false);
            get_parameter("predict_next_scan", predict_next_scan_);


            RCLCPP_INFO(get_logger(),"INS listning on topic %s", imu_topic_.c_str());
 

            // run_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            // run_timer = this->create_wall_timer(1ms, std::bind(&INS::runINS, this), run_cb_group_);

            // correction_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            // correction_timer = this->create_wall_timer(5000ms, std::bind(&INS::correctINSPose, this), correction_cb_group_);

            sub1_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options1;
            options1.callback_group = sub1_cb_group_;

            sub2_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options2;
            options2.callback_group = sub2_cb_group_;


            imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_, 100, std::bind(&INS::imuDataHandler, this, _1), options1);
            bias_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>("/kalman_bias", 100, std::bind(&INS::biasHandler, this, _1), options2);
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 100, std::bind(&INS::odomHandler, this, _1), options2);
            // odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 100, std::bind(&EKF::odometryHandler, this, _1), options2);
            // keyframe_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom_keyframe", 100, std::bind(&EKF::keyframeHandler, this, _1), options2);

            ins_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom_ins", 100);
            // odometry_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom_kalman", 100);
            // path_pub = this->create_publisher<nav_msgs::msg::Path>("/path_kalman", 100);
            path_pub = this->create_publisher<nav_msgs::msg::Path>("/path_ins", 100);
            // initial_pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initial_pose", 100);
            

            initializeArrays();
            

        }
        ~INS(){}

        void setPredictionAnchor()
        {
            prediction_anchor.acc = acceleration;
            prediction_anchor.vel = velocity;
            prediction_anchor.pos = position;
            prediction_anchor.time = process_time;
            prediction_anchor.ori = orientation;
            // prediction_anchor.bias.acc = acceleration;
            // prediction_anchor.bias.ang = angular_velocity;
            prediction_anchor.bias = current_bias;
        }

        void getPredictionAnchor()
        {
            acceleration = prediction_anchor.acc;
            velocity = prediction_anchor.vel;
            position = prediction_anchor.pos;
            // process_time = prediction_anchor.time;
            orientation = prediction_anchor.ori;
            current_bias = prediction_anchor.bias;
        }

        void initializeArrays()
        {   
            step = 0;

            // g_acc_ = 9.825;
            g_acc_var = 0.1;
            g_acc_sum = 0.0;
            g_acc_sumsq = 0.0;
            g_vec << 0.0, 0.0, 1.0;
   
            acceleration << 0.0,0.0,0.0;
            acceleration_bias << 0.0,0.0,0.0;
            acceleration_offset << 0.0,0.0,0.0;
            acceleration_post << 0.0,0.0,0.0;
            velocity << 0.0,0.0,0.0;
            position << 0.0,0.0,0.0;
            gravity_vector << 0.0,0.0,1.0;
            // jerk << 0.0,0.0,0.0;
            previous_angular_velocity << 0.0,0.0,0.0;
            angular_velocity << 0.0,0.0,0.0;
            angular_velocity_bias << 0.0,0.0,0.0;
            angular_velocity_offset << 0.0,0.0,0.0;


            omega << 0.0,0.0,0.0;
            alpha << 0.0,0.0,0.0;


            current_bias.acc = acceleration_bias;
            current_bias.ang = angular_velocity_bias;

            orientation = Eigen::Quaterniond::Identity();
            orientation_dt = Eigen::Quaterniond::Identity();
            // orientation_post = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
            
            ego_to_imu_offset = Eigen::Vector3d(0.25, 0.0, 0.1); // forward 25 cm, raised 10cm?


            INS_odometry.child_frame_id = "ins";

            setPredictionAnchor();

        }

        void imuDataHandler(const sensor_msgs::msg::Imu::SharedPtr imu_data)
        {
            // put data into buffer back
            RCLCPP_INFO_ONCE(get_logger(),"First IMU message recieved..");
            imu_buffer.push_back(imu_data);

        }

        void biasHandler(const geometry_msgs::msg::WrenchStamped::SharedPtr bias_msg){

            IMUwrench new_bias;
            Eigen::Vector3d acc_bias;
            Eigen::Vector3d ang_bias;
            // RCLCPP_INFO(get_logger(), "bias recieved!");
            // if (use_kalman_bias_correction_){
            acc_bias.x() = bias_msg->wrench.force.x;
            acc_bias.y() = bias_msg->wrench.force.y;
            acc_bias.z() = bias_msg->wrench.force.z;
            // }
            ang_bias.x() = bias_msg->wrench.torque.x;
            ang_bias.y() = bias_msg->wrench.torque.y;
            ang_bias.z() = bias_msg->wrench.torque.z;

            new_bias.acc = acc_bias;
            new_bias.ang = ang_bias;
            new_bias.time = toSec(bias_msg->header.stamp);

            bias_buffer.push_back(new_bias);
        }

        void odomHandler(const nav_msgs::msg::Odometry::SharedPtr odom_data)
        {
            // scan dt is embedded in the top right position (idx 5) of 6x6 covariance matrix from LOAM node
            scan_timestamp = toSec(odom_data->header.stamp);
            scan_dt = odom_data->pose.covariance[5];

            // Eigen::Vector3d lidar_pos(odom_data->pose.pose.position.x,
            //                           odom_data->pose.pose.position.y,
            //                           odom_data->pose.pose.position.z);
            // Eigen::Vector3d lidar_vel(odom_data->twist.twist.linear.x,
            //                           odom_data->twist.twist.linear.y,
            //                           odom_data->twist.twist.linear.z);

            // correctINSPose(position, lidar_vel);
            // saveCorrectionPose(position, lidar_vel);

            processINS(scan_timestamp);
            if (predict_next_scan_)
                processINSPrediction(scan_timestamp + scan_dt);
        }

        double toSec(builtin_interfaces::msg::Time header_stamp)
        {
            rclcpp::Time time = header_stamp;
            double nanoseconds = time.nanoseconds();

            return nanoseconds * 1e-9;
        }

        void getCurrentBias(double cutoff_time)
        {
            previous_bias = current_bias;
            while (current_bias.time < cutoff_time && !bias_buffer.empty()) {
                current_bias = bias_buffer.front();
                bias_buffer.pop_front();
            }
            RCLCPP_INFO(get_logger(), "Current Bias acc: %f %f %f ang %f %f %f", current_bias.acc.x(), current_bias.acc.y(), current_bias.acc.z(), current_bias.ang.x(), current_bias.ang.y(), current_bias.ang.z() );

        }

        void processINS(double cutoff_time)
        {
            getCurrentBias(process_time);
            // predict_buffer.clear();

            while (process_time <= cutoff_time && !imu_buffer.empty()){

                delayed_imu_data = imu_buffer.front();
                imu_buffer.pop_front();

                // RCLCPP_INFO_ONCE(get_logger(), "INS First imu msg recieved..");
                step++;

                INS_odometry.header = delayed_imu_data->header;
                INS_odometry.header.frame_id = "odom";
                process_time = toSec(delayed_imu_data->header.stamp);

                // imu_data_header = delayed_imu_data->header;
                // filtered_imu_data = *delayed_imu_data;

                Eigen::Vector3d acceleration_in(delayed_imu_data->linear_acceleration.x,
                                                delayed_imu_data->linear_acceleration.y,
                                                delayed_imu_data->linear_acceleration.z);

                angular_velocity << delayed_imu_data->angular_velocity.x,
                                    delayed_imu_data->angular_velocity.y,
                                    delayed_imu_data->angular_velocity.z;

                Eigen::Quaterniond orientation_madgwick(delayed_imu_data->orientation.w,
                                                delayed_imu_data->orientation.x,
                                                delayed_imu_data->orientation.y, 
                                                delayed_imu_data->orientation.z);

                IMUwrench imu_wrench_prediction_step;
                imu_wrench_prediction_step.acc = acceleration_in;
                imu_wrench_prediction_step.ang = angular_velocity;
                imu_wrench_prediction_step.time = process_time;
                predict_buffer.push_back(imu_wrench_prediction_step);
                
 

                if (step < turn_on_bias_estimation_period_){
                    turnOnBiasEstimation( angular_velocity);
                }
                


                if (use_madgwick_orientation_){
                    updateOrientation(orientation_madgwick);
                } else {
                    if (step <= 10){ // use the madgwick found orientation as the initial orientation
                        initOrientationMadgwick(orientation_madgwick);
                    } else {
                        updateOrientation(angular_velocity);
                    }
                }
                updateAcceleration(acceleration_in);
                updateVelocity();
                updatePosition();

                if (step < gravity_estimation_period_){
                    updateGravityCalibration(acceleration_in, toSec(delayed_imu_data->header.stamp));
                }

                publishINS();


            }
            setPredictionAnchor();
        }

        void processINSPrediction(double cutoff_time)
        {
            getPredictionAnchor();
            getCurrentBias(process_time);
            IMUwrench imu;
            while (!predict_buffer.empty()){
                imu = predict_buffer.front();
                predict_buffer.pop_front();
                Eigen::Vector3d acc_in = imu.acc;
                Eigen::Vector3d ang_in = imu.ang;

                // if (use_madgwick_orientation_){
                    // updateOrientation(orientation_madgwick);
                // } else {
                updateOrientation(ang_in);
                // }
                updateAcceleration(acc_in);
                updateVelocity();
                updatePosition();
                publishINS(); // make seperate predict topic?
            }

            while (process_time <= cutoff_time && !imu_buffer.empty()){

                delayed_imu_data = imu_buffer.front();
                imu_buffer.pop_front();

                RCLCPP_INFO(get_logger(), "Predicting..");

                INS_odometry.header = delayed_imu_data->header;
                INS_odometry.header.frame_id = "odom";
                process_time = toSec(delayed_imu_data->header.stamp);

                // imu_data_header = delayed_imu_data->header;
                // filtered_imu_data = *delayed_imu_data;

                Eigen::Vector3d acceleration_in(delayed_imu_data->linear_acceleration.x,
                                                delayed_imu_data->linear_acceleration.y,
                                                delayed_imu_data->linear_acceleration.z);

                angular_velocity << delayed_imu_data->angular_velocity.x,
                                    delayed_imu_data->angular_velocity.y,
                                    delayed_imu_data->angular_velocity.z;

                Eigen::Quaterniond orientation_madgwick(delayed_imu_data->orientation.w,
                                                delayed_imu_data->orientation.x,
                                                delayed_imu_data->orientation.y, 
                                                delayed_imu_data->orientation.z);

                IMUwrench imu_wrench_prediction_step;
                imu_wrench_prediction_step.acc = acceleration_in;
                imu_wrench_prediction_step.ang = angular_velocity;
                imu_wrench_prediction_step.time = process_time;
                predict_buffer.push_back(imu_wrench_prediction_step);
                
 

                if (use_madgwick_orientation_){
                    updateOrientation(orientation_madgwick);
                } else {
                    updateOrientation(angular_velocity);

                }
                updateAcceleration(acceleration_in);
                updateVelocity();
                updatePosition();


                // publishINS( ); // make seperate predict topic?

            }
        }


        
        // void runINS()
        // {
            
        //     // // get data from buffer front
        //     // if (!imu_buffer.empty()){
        //     //     delayed_imu_data = imu_buffer.front();
        //     //     imu_buffer.pop_front();
        //     // } else {
        //     //     return;
        //     // }
        // }   

        void initOrientationMadgwick(Eigen::Quaterniond q_in ){
            orientation_post = q_in;
            orientation = orientation_post;
        }

        void updateOrientation(Eigen::Quaterniond q_in ){
            if (step == 1){
                orientation_post = q_in;
            } 
            orientation = orientation_post;
            orientation_post = q_in; // spherical linear interpolation between q_in and ori. ?
        }

        // get delta quaternion from rotation base -> matrix, vector 
        template <typename Derived>
        Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
        {
            typedef typename Derived::Scalar Scalar_t;

            Eigen::Quaternion<Scalar_t> dq;
            Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
            half_theta /= static_cast<Scalar_t>(2.0);
            dq.w() = static_cast<Scalar_t>(1.0);
            dq.x() = half_theta.x();
            dq.y() = half_theta.y();
            dq.z() = half_theta.z();
            return dq;
        }

        void updateOrientation(Eigen::Vector3d angular_velocity ){
            
            orientation_post = orientation;
            angular_acceleration = (previous_angular_velocity - angular_velocity) / imu_dt_;
            Eigen::Vector3d average_angular_velocity = 0.5 * ((previous_angular_velocity - previous_bias.ang) + (angular_velocity - current_bias.ang)) - angular_velocity_offset;
            // first half is for average, second is dq_dt = 1/2 q w
            // but this should be taken care of in deltaQ??, ran two instances of madgwick that sent the rate twice..
            orientation_dt = deltaQ(average_angular_velocity * imu_dt_);
            orientation *= orientation_dt;
            previous_angular_velocity = angular_velocity; // the previos bias is baked in


            Eigen::Vector3d omega_old(omega);
            Eigen::AngleAxisd axangomega(orientation_dt);
            omega = axangomega.axis();
            omega *= axangomega.angle() / imu_dt_;

            alpha = (omega - omega_old);

        }

        Eigen::Vector3d calculateFicticiousAcceleration(){
            
            // RCLCPP_INFO(get_logger(), "omega %f %f %f", omega.x(), omega.y(), omega.z() );
            // RCLCPP_INFO(get_logger(), "alpha %f %f %f", alpha.x(), alpha.y(), alpha.z() );

            // no coriolis force as the imu is not moving relative to the ugv/robot
            Eigen::Vector3d centrifugal_acc;
            Eigen::Vector3d euler_acc; // requires angular acc
            centrifugal_acc = omega.cross(omega.cross(ego_to_imu_offset));
            euler_acc = alpha.cross(ego_to_imu_offset);
            // RCLCPP_INFO(get_logger(), "centrifugal acc %f %f %f", centrifugal_acc.x(), centrifugal_acc.y(), centrifugal_acc.z() );
            // RCLCPP_INFO(get_logger(), "euler acc %f %f %f", euler_acc.x(), euler_acc.y(), euler_acc.z() );
            // euler_acc = - 
            return centrifugal_acc + euler_acc;
            // return centrifugal_acc; 
        }

        void updateAcceleration(Eigen::Vector3d acc_in)
        {   
            Eigen::Quaterniond avg_quat = orientation_post.slerp(0.5, orientation);
            // if (step == 1){
            //     acceleration_post = avg_quat*acc_in;
            // } 
            Eigen::Vector3d ficticious_acc(0.0, 0.0, 0.0);
            if (use_ficticiuos_force_correction_){
                ficticious_acc = calculateFicticiousAcceleration();
            } 

            
            gravity_vector << 0.0, 0.0, -g_acc_;
            acceleration =  (acc_in - ficticious_acc ) ; // ficticious forces are in the body frame.
            
            
            // acceleration = orientation*(acc_in - acceleration_bias);
            acceleration =  avg_quat * acceleration - current_bias.acc + gravity_vector; // gravity vector is in world frame.

            // Just false...
            // the rejection vector between dv
            // Eigen::Vector3d centrifugal_acc_curve_path(0.0,0.0,0.0);
            // if (velocity.norm() > 0.0){
            //     Eigen::Vector3d dv(acceleration * imu_dt_);
            //     Eigen::Vector3d proj = (velocity) * (velocity+dv).dot(velocity) /( velocity.dot(velocity));
            //     centrifugal_acc_curve_path = (velocity+dv - proj);
            // }
            // RCLCPP_INFO(get_logger(), "centrifugal acc curved path %f %f %f", centrifugal_acc_curve_path.x(), centrifugal_acc_curve_path.y(), centrifugal_acc_curve_path.z() );
            // acceleration -= avg_quat.inverse() * centrifugal_acc_curve_path;

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
            // header etc has been set earlier
            
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

            
            vector<double> cov_diag{0.5,0.5,0.5, 0.1,0.1,0.1};
            Eigen::MatrixXd cov_mat = createCovarianceEigen(cov_diag);
            // Eigen::Matrix3d rot_mat = last_odometry_pose.block<3,3>(0,0);
            // Eigen::Matrix3d rot_mat(Eigen::Quaterniond(latestPoseInfo.qw, latestPoseInfo.qx, latestPoseInfo.qy, latestPoseInfo.qz ));
            // rotateCovMatrix(cov_mat, rot_mat);
            setCovariance(INS_odometry.pose.covariance, cov_mat);

            ins_pub->publish(INS_odometry);

            geometry_msgs::msg::PoseStamped poseStamped;
            poseStamped.header = INS_odometry.header;
            poseStamped.pose = INS_odometry.pose.pose;
            poseStamped.header.stamp = INS_odometry.header.stamp;
            path.header.stamp = INS_odometry.header.stamp;
            path.poses.push_back(poseStamped);
            // path.header.frame_id = frame_id;
            path.header.frame_id = "odom";
            path_pub->publish(path);

        }

        Eigen::MatrixXd createCovarianceEigen(vector<double> cov_diag)
        {
            Eigen::MatrixXd cov_mat(6,6);
            cov_mat.fill(0.0);
            for (int i=0 ; i < 6; i++){
                cov_mat(i*6 + i) = cov_diag[i];
            }

            return cov_mat;
        }

        void setCovariance(array<double, 36> &cov, vector<double> cov_diag)
        // assumes a 6x6 covariance array ie. size 36
        {
            for (int i=0 ; i < 6; i++){
                cov[i*6 + i] = cov_diag[i];
            }

        }

        void setCovariance(array<double, 36> &cov, Eigen::MatrixXd cov_mat)
        // assumes a 6x6 covariance array ie. size 36
        {
            for (int i=0 ; i < 36; i++){
                double value = cov_mat(i);
                if (value < 0.0) {
                    value = 0.0;
                }
                cov[i] = value; // no negative values
            }

        }
        
        void saveCorrectionPose(Eigen::Vector3d position_, Eigen::Vector3d velocity_)
        {

        }

        void correctINSPose(Eigen::Vector3d position_, Eigen::Vector3d velocity_)
        {
            // update position and velocity from the kalman filter
            
            position = position_;
            velocity = velocity_;
            // orientation = Eigen::Quaterniond(1.0,0.0,0.0,0.0);
            
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


        void updateGravityCalibration(Eigen::Vector3d static_acc, double time_stamp)
        {
            g_vec_buffer.push_back(static_acc);

            // when enough time has passed trigger the calibration
            if (time_stamp > (calibration_time + imu_dt_ * gravity_estimation_period_)){
                double dt = time_stamp - calibration_time;
                Eigen::Vector3d average_g_vec = Eigen::Vector3d::Zero();
                size_t n = g_vec_buffer.size();
                while (!g_vec_buffer.empty())
                {
                    average_g_vec += g_vec_buffer.front();
                    g_vec_buffer.pop_front();
                }
                average_g_vec /= (float)n ;
                Eigen::Vector3d average_g_norm = average_g_vec.normalized();
                // double average_g = average_g_vec.norm();

                double displacement_from_gravity = position.dot(average_g_norm);
                // double displacement_from_gravity = position.norm();

                double g0 = (2* displacement_from_gravity/(dt*dt) + g_acc_);
                

                // g_acc_ = g0;

                // kalman gain
                g_acc_var_measurement = 0.02;
                k_gain = g_acc_var / (g_acc_var + g_acc_var_measurement);

                // update uncertainty of estimate
                g_acc_var = (1 - k_gain) * g_acc_var;

                // update estimate with mean 
                RCLCPP_INFO(get_logger(), "Estimation of gravity: %f, old gravity: %f ", g0, g_acc_);
                g_acc_ = g_acc_ + k_gain * (g0 - g_acc_);

                RCLCPP_INFO(get_logger(), "Updated gravity value: %f +- %f", g_acc_, sqrt(g_acc_var));

                position = Eigen::Vector3d::Zero();

                calibration_time = time_stamp;
            }
        }

        void updateGravityCalibrationKalman(Eigen::Vector3d static_acc)
        {
            size_t mean_steps = 50;
            // using Naïve algotihm, see wiki "algorithm for calculating variance".
            double new_g_acc = static_acc.norm();
            // Eigen::Vector3d new_g_vec = static_acc.normalized();
            if (step == 1)
                g_acc_ = new_g_acc;

            g_buffer.push_back(new_g_acc);

            if (g_buffer.size() < mean_steps) {
                return;
            }

            g_acc_sum = 0.0;
            g_acc_sumsq = 0.0;
            for (size_t i=0; i> mean_steps; i++)
            {
                double g_ = g_buffer.front();
                g_acc_sum += g_;
                g_acc_sumsq += g_*g_;
                g_buffer.pop_front();
            }
            g_acc_var_measurement = (g_acc_sumsq - (g_acc_sum*g_acc_sum)/(double)mean_steps) / ((double)mean_steps - 1.0); // measuremet uncertainty 
            double g_acc_mean = g_acc_sum / (double)mean_steps;

            // // kalman gain
            k_gain = g_acc_var / (g_acc_var + g_acc_var_measurement);

            // // update uncertainty of estimate
            g_acc_var = (1 - k_gain) * g_acc_var;

            // // update estimate with mean 
            g_acc_ = g_acc_ + k_gain * (g_acc_mean - g_acc_);

            // g_vec = (1 - k_gain) *  new_g_vec;


            RCLCPP_INFO(get_logger(), "Static Calibration: gravity estimate: %f m/s²", g_acc_);
            RCLCPP_INFO(get_logger(), "Static Calibration: std: %f m/s²", sqrt(g_acc_var));
            RCLCPP_INFO(get_logger(), "Static Calibration: measurement std: %f",sqrt(g_acc_var_measurement));
            RCLCPP_INFO(get_logger(), "Static Calibration: update gain: %f", k_gain);
            // RCLCPP_INFO(get_logger(), "Static Calibration: gravity norm vector: %f %f %f", g_vec[0], g_vec[1], g_vec[2] );
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

        void turnOnBiasEstimation(Eigen::Vector3d angular_velocity){
            angular_velocity_offset /= (float)turn_on_bias_estimation_period_;
            angular_velocity_offset += angular_velocity/(float)turn_on_bias_estimation_period_;
            angular_velocity_offset *= (step-1);
            angular_velocity_offset /= step;
            angular_velocity_offset *= (float)turn_on_bias_estimation_period_;
            RCLCPP_INFO(get_logger(), "angular rate static calibration: : %f %f %f", angular_velocity_offset[0], angular_velocity_offset[1], angular_velocity_offset[2] );
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