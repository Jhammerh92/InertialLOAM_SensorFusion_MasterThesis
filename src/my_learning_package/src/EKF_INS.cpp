
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

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>


#include <memory>
#include <cstdio>
#include <cmath>
#include <queue>
#include <vector>
#include <fstream>
// #include <eigen>

using namespace std;


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

class EKF_INS : public rclcpp::Node
{
    private:
        rclcpp::CallbackGroup::SharedPtr sub1_cb_group_;
        rclcpp::CallbackGroup::SharedPtr sub2_cb_group_;

        rclcpp::CallbackGroup::SharedPtr run_cb_group_;
        rclcpp::TimerBase::SharedPtr run_timer;

        
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ins_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr keyframe_odom_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr lidar_odometry_transformation_sub;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr bias_pub;
     
      
        // rclcpp::Time time_latest_cloud;



        double imu_dt_; 
        double g_acc; // gravity scalar value
        double g_acc_var;
        double g_acc_sum;
        double g_acc_sumsq;
        Eigen::Vector3d g_vec; // normal vector of gravity


        int step{};
        int step_since_keyframe{};

        Eigen::MatrixXd Eye9;
        Eigen::MatrixXd Eye6;
        Eigen::Matrix3d Eye3;

        // ORIENTATION

        // state of orientation, x with bias on orientation, [alpha, q_b]
        Eigen::MatrixXd state_ori; 
        Eigen::Vector3d state_euler;

        // uncertainty / covariance matrix, P  -> check in docs for xsens imu if these are not equal on all axes

        Eigen::MatrixXd uncertainty_ori; 

        // process noise matrix, Q -> check in docs for xsens imu if these are not equal on all axes

        Eigen::MatrixXd noise_ori; 

        // prediction matrix, A aka F
        Eigen::MatrixXd A_ori;
        Eigen::MatrixXd A_ori_T;
        // bias matrix, B
        Eigen::MatrixXd B_ori;

        // measurement matrix H
        Eigen::MatrixXd H_ori;
        Eigen::MatrixXd H_ori_T;

        // Kalman gain
        Eigen::MatrixXd gain_ori_ins;
        Eigen::MatrixXd gain_ori_lidar;

        // measurement noise, R
        Eigen::Matrix3d measurement_covariance_ori_ins;
        Eigen::Matrix3d measurement_covariance_ori_lidar;




        // POSITION

        // state vector of position, [S, v, a_b]
        Eigen::MatrixXd state_pos; // should be 3 parts, [S, v, a_b], for each axis , total -> 9 parts

        // covariance Matrix P
        Eigen::MatrixXd uncertainty_pos; // 9x9 matrix, also has uncertainty of biases


        Eigen::MatrixXd noise_pos; 
        // prediction matrix, A aka F

        Eigen::MatrixXd A_pos;
        Eigen::MatrixXd A_pos_T;

        // input matrix, B
        Eigen::MatrixXd B_pos;

        // Obsevation matrix H
        Eigen::MatrixXd H_pos;
        Eigen::MatrixXd H_pos_T;

        Eigen::MatrixXd H_ins;
        Eigen::MatrixXd H_ins_T;

        Eigen::Vector3d state_pos_post;

        //kalman gain matrix, K
        Eigen::MatrixXd gain_pos;
        Eigen::MatrixXd gain_ins;

        // R matrix, the covariance matrix of the measurement
        Eigen::Matrix3d measurement_covariance_pos;
        Eigen::MatrixXd measurement_covariance_pos_vel_ins;


        // OTHER
        double roll;
        double pitch;
        double yaw;
        double init_calib_roll;
        double init_calib_pitch;
        double init_calib_yaw;
        double g_acc_var_measurement;
        double k_gain;
        deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer;
        deque<nav_msgs::msg::Odometry::SharedPtr> ins_buffer;
        double measurement_time{};
        double process_time{};
        double ins_time{};

        Eigen::Vector3d body_acc_bias;


        Eigen::Vector3d acceleration;
        Eigen::Vector3d angular_velocity;

        Eigen::Vector3d ins_position;
        Eigen::Vector3d ins_velocity;
        tf2::Quaternion ins_orientation;

        PoseInfo latestPoseInfo;

        nav_msgs::msg::Path path;

        // Eigen::Quaterniond orientation_quaternion;
        tf2::Quaternion orientation_quaternion;

        // Eigen::Quaterniond madgwick_orientation_quaternion;
        tf2::Quaternion madgwick_orientation_quaternion;

        // ROS PARAMETERS
        std::string imu_topic_;
       
        double process_noise_roll_;
        double process_noise_pitch_;
        double process_noise_yaw_;
        double measurement_noise_roll_; 
        double measurement_noise_pitch_; 
        double measurement_noise_yaw_; 

        double process_noise_pos_;
        double measurement_noise_pos_; 


        int init_calib_steps_;

        int imu_step_delay_;

        bool print_states_{};
        bool use_madgwick_orientation_{};
        bool relative_mode_{};
        bool publish_bias_{};

        builtin_interfaces::msg::Time kalman_stamp;

        std::ofstream data_file;

    
    public:
        EKF_INS() // constructer
        : Node("EKF_INS")
        {   
            declare_parameter("imu_dt", 0.01);
            get_parameter("imu_dt", imu_dt_);

            declare_parameter("imu_step_delay", 10);
            get_parameter("imu_step_delay", imu_step_delay_);

            declare_parameter("measurement_noise_roll", 0.0001);
            get_parameter("measurement_noise_roll", measurement_noise_roll_);
            declare_parameter("measurement_noise_pitch", 0.0001);
            get_parameter("measurement_noise_pitch", measurement_noise_pitch_);
            declare_parameter("measurement_noise_yaw", 0.0001);
            get_parameter("measurement_noise_yaw", measurement_noise_yaw_);

            declare_parameter("measurement_noise_pos", 0.000001);
            get_parameter("measurement_noise_pos", measurement_noise_pos_);

            declare_parameter("process_noise_roll", 0.0001);
            get_parameter("process_noise_roll", process_noise_roll_);
            declare_parameter("process_noise_pitch", 0.0001);
            get_parameter("process_noise_pitch", process_noise_pitch_);
            declare_parameter("process_noise_yaw", 0.005);
            get_parameter("process_noise_yaw", process_noise_yaw_);

            declare_parameter("process_noise_pos", 0.00001);
            get_parameter("process_noise_pos", process_noise_pos_);

            declare_parameter("init_calib_steps", 0);
            get_parameter("init_calib_steps", init_calib_steps_);
            
            declare_parameter("imu_topic", "/imu/data_raw");
            get_parameter("imu_topic", imu_topic_);

            declare_parameter("relative_mode", false);
            get_parameter("relative_mode", relative_mode_);

            declare_parameter("print_states", false);
            get_parameter("print_states", print_states_);

            declare_parameter("use_madgwick_orientation", false);
            get_parameter("use_madgwick_orientation", use_madgwick_orientation_);
            declare_parameter("publish_bias", false);
            get_parameter("publish_bias", publish_bias_);
            

            RCLCPP_INFO(get_logger(), "Listning for IMU on %s", imu_topic_.c_str());
            // run_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            // run_timer = this->create_wall_timer(1ms, std::bind(&EKF::runProcess, this), run_cb_group_);

            sub1_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options1;
            options1.callback_group = sub1_cb_group_;

            sub2_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options2;
            options2.callback_group = sub2_cb_group_;


            imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_, 100, std::bind(&EKF_INS::imuDataHandler, this, _1), options1);
            ins_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom_ins", 100, std::bind(&EKF_INS::insHandler, this, _1), options2);
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 100, std::bind(&EKF_INS::odometryHandler, this, _1), options2);
            lidar_odometry_transformation_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/transformation/lidar", 100, std::bind(&EKF_INS::transformationHandler, this, _1), options2);
            // keyframe_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom_keyframe", 100, std::bind(&EKF::keyframeHandler, this, _1), options2);

            odometry_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom_kalman", 100);
            path_pub = this->create_publisher<nav_msgs::msg::Path>("/path_kalman", 100);
            initial_pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initial_pose", 100);
            bias_pub = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/kalman_bias", 100);
            

            initializeArrays();
            initDataFile();
        
        }
        ~EKF_INS(){}

        void initDataFile()
        {   
            // if (!save_running_data_)
            //     return;

            data_file.open ("temp_saved_odometry_data/kalman/kalman_data.csv");

            data_file << "time, x, y, z, vx, vy, vz, ex, ey, ez, qw, qx, qy, qz, bias_acc_x, bias_acc_y, bias_acc_z, bias_ang_x, bias_ang_y, bias_ang_z";

        }
        void saveRunningData()
        {
            // if (!save_running_data_)
            //     return;

            // order of data: "time, x, y, z, vx, vy, vz, obs_vx, obs_vy, obs_vz, qw, qx, qy, qz, fitness, residual_x, residual_y, residual_z, residual_qw, residual_qx, residual_qy, residual_qz, cov_x, cov_y, cov_z, bias_acc_x, bias_acc_y, bias_acc_z, bias_ang_x, bias_ang_y, bias_ang_z "
            deque<double> print_qeue;
            print_qeue.push_back(measurement_time);         // time
            print_qeue.push_back(state_pos(0));             // x          
            print_qeue.push_back(state_pos(1));             // y          
            print_qeue.push_back(state_pos(2));             // z  
            print_qeue.push_back(state_pos(3));             // vx          
            print_qeue.push_back(state_pos(4));             // vy          
            print_qeue.push_back(state_pos(5));             // vz        
            print_qeue.push_back(state_ori(0));             // ex        
            print_qeue.push_back(state_ori(1));             // ey        
            print_qeue.push_back(state_ori(2));             // ez        
            print_qeue.push_back(latestPoseInfo.qw);        // qw         
            print_qeue.push_back(latestPoseInfo.qx);        // qx         
            print_qeue.push_back(latestPoseInfo.qy);        // qy         
            print_qeue.push_back(latestPoseInfo.qz);        // qz         
            print_qeue.push_back(body_acc_bias.x());             // bias_acc_x          
            print_qeue.push_back(body_acc_bias.y());             // bias_acc_y         
            print_qeue.push_back(body_acc_bias.z());             // bias_acc_z          
            print_qeue.push_back(state_ori(3));             // bias_ang_x          
            print_qeue.push_back(state_ori(4));             // bias_ang_y         
            print_qeue.push_back(state_ori(5));             // bias_ang_z          

            int length =  (int)print_qeue.size();

            data_file << "\n";
            for(int i=0; i < length; i++){
                data_file << std::to_string(print_qeue.front()); 
                if (i < length-1){
                    data_file << ","; 
                }
                print_qeue.pop_front();
            }
            print_qeue.clear(); // just to make sure it is empty

        }

        void initializeArrays()
        {   
            step = 0;
            step_since_keyframe = 0;

            g_acc = 9.825;
            g_acc_var = 0.01;
            g_acc_sum = 0.0;
            g_acc_sumsq = 0.0;
            g_vec << 0.0, 0.0, 1.0;

            process_time = 0.0;
            ins_time = 0.0;


            ins_position = Eigen::Vector3d::Zero();
            ins_velocity = Eigen::Vector3d::Zero();
            ins_orientation = tf2::Quaternion::getIdentity();

            body_acc_bias = Eigen::Vector3d::Zero();

            // imu_dt_ = 0.01; //TODO: make this a ros parameter, done

            double imu_dt_sq = imu_dt_*imu_dt_;

            Eye9 = Eigen::Matrix<double,9,9>();
            Eye9 << Eigen::MatrixXd::Identity(9,9);

            Eye6 = Eigen::Matrix<double,6,6>();
            Eye6 << Eigen::MatrixXd::Identity(6,6);

            Eye3 = Eigen::Matrix3d::Identity();
            
            state_pos_post.fill(0.0);

            // orientation
            // state_pitch = Eigen::Vector2d(0.0, 0.0);
            // state_yaw   = Eigen::Vector2d(0.0, 0.0);
            // state_roll  = Eigen::Vector2d(0.0, 0.0);
            state_ori = Eigen::Matrix<double,6,1>();
            state_ori.fill(0.0);
            state_euler.fill(0.0);

            // orientation
            A_ori = Eigen::Matrix<double,6,6>();
            A_ori_T = Eigen::Matrix<double,6,6>();
            A_ori   << 1.0, 0.0, 0.0, -imu_dt_, 0.0,      0.0,    
                       0.0, 1.0, 0.0, 0.0,      -imu_dt_, 0.0,    
                       0.0, 0.0, 1.0, 0.0,      0.0,      -imu_dt_,
                       0.0, 0.0, 0.0, 1.0,      0.0,      0.0,
                       0.0, 0.0, 0.0, 0.0,      1.0,      0.0,
                       0.0, 0.0, 0.0, 0.0,      0.0,      1.0;
                       
            A_ori_T = A_ori.transpose();

            B_ori   = Eigen::Matrix<double,6,3>();
            B_ori   << imu_dt_, 0.0,      0.0,
                       0.0,     imu_dt_,  0.0,
                       0.0,     0.0,      imu_dt_,
                       0.0,     0.0,      0.0,
                       0.0,     0.0,      0.0,
                       0.0,     0.0,      0.0;

            H_ori = Eigen::Matrix<double,3,6>();
            H_ori_T = Eigen::Matrix<double,3,6>();
            H_ori   << Eye3, Eigen::Matrix3d::Zero();
            H_ori_T = H_ori.transpose();

            double init_uncertainty_ori_var = 0.001;
            // uncertainty_pitch = Eigen::Matrix2d::Identity() * init_uncertainty_ori_var;
            // uncertainty_yaw   = Eigen::Matrix2d::Identity() * init_uncertainty_ori_var;
            // uncertainty_roll  = Eigen::Matrix2d::Identity() * init_uncertainty_ori_var;
            uncertainty_ori = Eigen::Matrix<double,6,6>();

            Eigen::VectorXd unc_cov_diag_ori = Eigen::VectorXd(6);
            unc_cov_diag_ori.fill(init_uncertainty_ori_var);
            uncertainty_ori = unc_cov_diag_ori.array().matrix().asDiagonal();



            // BUILD Q matrix
            Eigen::VectorXd noise_ori_vec(2);
            Eigen::Vector3d noise_ori_values(process_noise_roll_ *process_noise_roll_ , process_noise_pitch_ *process_noise_pitch_ ,process_noise_yaw_ *process_noise_yaw_);
            noise_ori = Eigen::Matrix<double,6,6>();
            Eigen::Matrix2d noise_ori_2matrix;

            noise_ori_vec <<  imu_dt_, 0.1 ;
            // noise_ori_vec <<  imu_dt_, imu_dt_;
            noise_ori_2matrix = noise_ori_vec * noise_ori_vec.transpose();

            Eigen::Vector3d temp_vec;
            for (int i=0; i < noise_ori_2matrix.cols(); i++){
                for (int j=0; j < noise_ori_2matrix.rows(); j++){ 
                    temp_vec = Eigen::Vector3d(noise_ori_2matrix(j,i), noise_ori_2matrix(j,i), noise_ori_2matrix(j,i));
                    temp_vec.cwiseProduct(noise_ori_values);
                    noise_ori.block<3,3>(3*j,3*i) = temp_vec.array().matrix().asDiagonal();
                }
            }


            // double process_noise_ori_var = 0.0001;
            // noise_roll  = Eigen::Matrix2d::Identity() * process_noise_roll_  *process_noise_roll_; 
            // noise_pitch = Eigen::Matrix2d::Identity() * process_noise_pitch_ *process_noise_pitch_; 
            // noise_yaw   = Eigen::Matrix2d::Identity() * process_noise_yaw_   *process_noise_yaw_;  
            // noise_roll(0,0)  *= imu_dt_sq;
            // noise_pitch(0,0) *= imu_dt_sq;
            // noise_yaw(0,0)   *= imu_dt_sq;

            
            // measurement_noise_ori = 0.0001; // radians²
            // measurement_noise_roll = Eigen::VectorXd(1);
            // measurement_noise_roll << measurement_noise_roll_*measurement_noise_roll_;
            // measurement_noise_pitch = Eigen::VectorXd(1);
            // measurement_noise_pitch << measurement_noise_pitch_*measurement_noise_pitch_;
            // measurement_noise_yaw = Eigen::VectorXd(1);
            // measurement_noise_yaw << measurement_noise_yaw_*measurement_noise_yaw_;
            Eigen::Vector3d meas_cov_vec(measurement_noise_roll_*measurement_noise_roll_,measurement_noise_pitch_*measurement_noise_pitch_ , measurement_noise_yaw_*measurement_noise_yaw_);

            measurement_covariance_ori_lidar = meas_cov_vec.array().matrix().asDiagonal();
            measurement_covariance_ori_ins = meas_cov_vec.array().matrix().asDiagonal();

            // gain_roll   = uncertainty_roll  * H_ori_T * (H_ori * uncertainty_roll   * H_ori_T + measurement_noise_roll).inverse(); // + R in the inverse  but not at foregone at the init
            // gain_pitch  = uncertainty_pitch * H_ori_T * (H_ori * uncertainty_pitch  * H_ori_T + measurement_noise_pitch).inverse(); // + R in the inverse  but not at the init
            // gain_yaw    = uncertainty_yaw   * H_ori_T * (H_ori * uncertainty_yaw    * H_ori_T + measurement_noise_yaw).inverse(); // + R in the inverse  but not at foregone at the init

            gain_ori_ins = Eigen::Matrix<double,6,3>();
            gain_ori_lidar = Eigen::Matrix<double,6,3>();
            gain_ori_lidar    = uncertainty_ori   * H_ori_T * (H_ori * uncertainty_ori    * H_ori_T + measurement_covariance_ori_lidar).inverse(); // + R in the inverse  but not at foregone at the init
            gain_ori_ins    = uncertainty_ori   * H_ori_T * (H_ori * uncertainty_ori    * H_ori_T + measurement_covariance_ori_ins).inverse(); // + R in the inverse  but not at foregone at the init

            // -------------- POSITION --------------

            // state_x  = Eigen::Vector3d::Zero();
            // state_y  = Eigen::Vector3d::Zero();
            // state_z  = Eigen::Vector3d::Zero();
            // Eigen::MatrixXd state_x(3,1);
            // state_x << 0.0, 0.0, 0.0;
            // Eigen::MatrixXd state_y(3,1);
            // state_y << 0.0, 0.0, 0.0;
            // Eigen::MatrixXd state_z(3,1);
            // state_z << 0.0, 0.0, 0.0;
            // state_pos = Eigen::MatrixXd(9,1);
            state_pos = Eigen::Matrix<double,9,1>();
            state_pos.fill(0.0);


            // position
            // A_pos   << 1.0, imu_dt_, -imu_dt_sq/2,
            //            0.0, 1.0, -imu_dt_,
            //            0.0, 0.0, 1.0;
            A_pos = Eigen::Matrix<double,9,9>();
            A_pos_T = Eigen::Matrix<double,9,9>();
            A_pos   << 1.0, 0.0, 0.0, imu_dt_, 0.0,     0.0,     -imu_dt_sq/2.0, 0.0,            0.0,
                       0.0, 1.0, 0.0, 0.0,     imu_dt_, 0.0,     0.0,            -imu_dt_sq/2.0, 0.0, 
                       0.0, 0.0, 1.0, 0.0,     0.0,     imu_dt_, 0.0,            0.0,            -imu_dt_sq/2.0, 
                       0.0, 0.0, 0.0, 1.0,     0.0,     0.0,     -imu_dt_,       0.0,            0.0,
                       0.0, 0.0, 0.0, 0.0,     1.0,     0.0,     0.0,            -imu_dt_,       0.0, 
                       0.0, 0.0, 0.0, 0.0,     0.0,     1.0,     0.0,            0.0,            -imu_dt_, 
                       0.0, 0.0, 0.0, 0.0,     0.0,     0.0,     1.0,            0.0,            0.0,
                       0.0, 0.0, 0.0, 0.0,     0.0,     0.0,     0.0,            1.0,            0.0, 
                       0.0, 0.0, 0.0, 0.0,     0.0,     0.0,     0.0,            0.0,            1.0;
            // A_pos = Eye9;
            A_pos_T = A_pos.transpose();
            // A_pos = A_pos_T.transpose();

            // B_pos   = Eigen::Vector3d(imu_dt_sq/2, imu_dt_,  0.0);
            B_pos   = Eigen::Matrix<double,9,3>();
            B_pos   << imu_dt_sq/2.0, 0.0,           0.0,
                       0.0,           imu_dt_sq/2.0, 0.0,
                       0.0,           0.0,           imu_dt_sq/2.0,
                       imu_dt_,       0.0,           0.0,
                       0.0,           imu_dt_,       0.0,
                       0.0,           0.0,           imu_dt_,
                       0.0,           0.0,           0.0,
                       0.0,           0.0,           0.0,
                       0.0,           0.0,           0.0;
            // B_pos.fill(0.0);

            // H_pos   << 1.0, 0.0, 0.0;
            // H_pos_T << 1.0, 0.0, 0.0;

            // H_ins   << 1.0, 1.0, 0.0;
            // H_ins_T << 1.0, 1.0, 0.0;
            // Eigen::Matrix3d Eye = Eigen::Matrix3d::Identity();
            H_pos_T = Eigen::Matrix<double,9,3>();
            H_pos = Eigen::Matrix<double,3,9>();
            H_pos   << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero();
            H_pos_T = H_pos.transpose();


            // Eigen::Matrix3d Eye = Eigen::Matrix3d::Identity();
            H_ins_T = Eigen::Matrix<double,9,6>();
            H_ins_T  << Eye3, Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eye3, Eigen::Matrix3d::Zero();
            H_ins = Eigen::Matrix<double,6, 9>();
            H_ins = H_ins_T.transpose();

            


            // double init_uncertainty_pos_std = 0.001;

            // uncertainty_x = Eigen::Matrix3d::Identity() * init_uncertainty_pos_std*init_uncertainty_pos_std;
            // uncertainty_y = Eigen::Matrix3d::Identity() * init_uncertainty_pos_std*init_uncertainty_pos_std;
            // uncertainty_z = Eigen::Matrix3d::Identity() * init_uncertainty_pos_std*init_uncertainty_pos_std;

            Eigen::VectorXd unc_cov_diag_pos = Eigen::VectorXd(9);
            unc_cov_diag_pos << 0.001, 0.001, 0.001, 0.005, 0.005, 0.005, 0.02, 0.02, 0.02;
            // unc_cov_diag << 2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0;
            uncertainty_pos = Eigen::Matrix<double,9,9>();
            // uncertainty_pos << Eigen::MatrixXd::Identity(9, 9) * init_uncertainty_pos_std *init_uncertainty_pos_std;
            uncertainty_pos = unc_cov_diag_pos.array().matrix().asDiagonal();



            // double process_noise_pos_var = 0.1;
            // Eigen::Vector3d noise_pos_vec(3);
            // noise_pos_vec << imu_dt_sq/2.0, imu_dt_,  0.0;

            // BUILD Q matrix
            Eigen::VectorXd noise_pos_vec(3);
            noise_pos = Eigen::Matrix<double,9,9>();
            Eigen::Matrix3d noise_pos_3matrix;

            noise_pos_vec << imu_dt_sq/2.0, imu_dt_, 0.0 ;
            // noise_pos_vec << imu_dt_sq/2.0, imu_dt_, imu_dt_ ;
            // noise_pos_vec * noise_pos_vec.transpose() * process_noise_pos_*process_noise_pos_;
            noise_pos_3matrix = noise_pos_vec * noise_pos_vec.transpose() * process_noise_pos_*process_noise_pos_;

            // Eigen::Vector3d temp_vec;
            for (int i=0; i < noise_pos_3matrix.cols(); i++){
                for (int j=0; j < noise_pos_3matrix.rows(); j++){ 
                    temp_vec = Eigen::Vector3d(noise_pos_3matrix(j,i), noise_pos_3matrix(j,i), noise_pos_3matrix(j,i));
                    noise_pos.block<3,3>(3*j,3*i) = temp_vec.array().matrix().asDiagonal();
                }
            }

            //     noise_pos << noise_pos_3matrix.rowStride coeff(i).array().matrix().asDiagonal();
            // }

            // noise_pos_3matrix << imu_dt_sq*imu_dt_sq/4.0 * process_noise_pos_*process_noise_pos_, imu_dt_sq*imu_dt_/2 * process_noise_pos_*process_noise_pos_, 0.0,
            //                      imu_dt_sq*imu_dt_/2 * process_noise_pos_*process_noise_pos_, imu_dt_sq * process_noise_pos_*process_noise_pos_, 0.0,
            //                      0.0, 0.0, 0.0;


            RCLCPP_INFO(get_logger(),"pos process noise %f", noise_pos(0,0)*1e10);
            RCLCPP_INFO(get_logger(),"prev pos process noise %f", imu_dt_sq*imu_dt_sq/4.0 * process_noise_pos_*process_noise_pos_*1e10);



            // measurement_noise_pos = 0.00001; // meters²
            // measurement_covariance_x = Eigen::VectorXd(1);
            // measurement_covariance_x << measurement_noise_pos_*measurement_noise_pos_;
            // measurement_covariance_y = Eigen::VectorXd(1);
            // measurement_covariance_y << measurement_noise_pos_*measurement_noise_pos_;
            // measurement_covariance_z = Eigen::VectorXd(1);
            // measurement_covariance_z << measurement_noise_pos_*measurement_noise_pos_;

            Eigen::VectorXd ins_cov_diag = Eigen::VectorXd(6);
            ins_cov_diag << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
            measurement_covariance_pos_vel_ins = Eigen::Matrix<double,6, 6>();
            measurement_covariance_pos_vel_ins = Eigen::MatrixXd::Identity(6, 6) *0.1;
            // measurement_covariance_pos_vel_ins = ins_cov_diag.array().matrix().asDiagonal();

            // initial/static covariance
            Eigen::VectorXd pos_cov_diag = Eigen::Vector3d(measurement_noise_pos_* measurement_noise_pos_, measurement_noise_pos_* measurement_noise_pos_, measurement_noise_pos_* measurement_noise_pos_);
            measurement_covariance_pos = pos_cov_diag.array().matrix().asDiagonal();

            // gain_x = uncertainty_x * H_pos_T * (H_pos * uncertainty_x * H_pos_T + measurement_covariance_x).inverse(); 
            // gain_y = uncertainty_y * H_pos_T * (H_pos * uncertainty_y * H_pos_T + measurement_covariance_y).inverse(); 
            // gain_z = uncertainty_z * H_pos_T * (H_pos * uncertainty_z * H_pos_T + measurement_covariance_z).inverse(); 
            gain_pos = Eigen::Matrix<double,9, 3>();
            gain_pos = uncertainty_pos * H_pos_T * (H_pos * uncertainty_pos * H_pos_T + measurement_covariance_pos).inverse(); 


            gain_ins = Eigen::Matrix<double, 9, 6>();
            // (H_ins * uncertainty_pos * H_ins_T + measurement_covariance_ins).inverse();
            // RCLCPP_INFO(get_logger(),"initialize test");
            gain_ins = uncertainty_pos * H_ins_T * (H_ins * uncertainty_pos * H_ins_T + measurement_covariance_pos_vel_ins).inverse(); 
            
        
            latestPoseInfo.qw = 1.0;
            latestPoseInfo.qx = 0.0;
            latestPoseInfo.qy = 0.0;
            latestPoseInfo.qz = 0.0;
            latestPoseInfo.x = 0.0;
            latestPoseInfo.y = 0.0;
            latestPoseInfo.z = 0.0;

            orientation_quaternion = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
            madgwick_orientation_quaternion = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);

            RCLCPP_INFO(get_logger(),"Parameters and Arrays initialized..");
        }

        void updateMeasurementCovariance(double new_error_std)
        {
            // measurement_noise_pos = 0.00001; // meters²
            measurement_covariance_pos = Eigen::Matrix3d::Identity() * new_error_std*new_error_std;
            // measurement_covariance_x << new_error_std*new_error_std;
            // measurement_covariance_y << new_error_std*new_error_std;
            // measurement_covariance_z << new_error_std*new_error_std;
        }

        void updateMeasurementCovariance(double new_error_std_x, double new_error_std_z, double new_error_std_y)
        {
            measurement_covariance_pos = Eigen::Matrix3d::Identity();
            measurement_covariance_pos(0,0) *= new_error_std_x*new_error_std_y;
            measurement_covariance_pos(1,1) *= new_error_std_y*new_error_std_y;
            measurement_covariance_pos(2,2) *= new_error_std_z*new_error_std_z;

            // measurement_covariance_x << new_error_std_x*new_error_std_y;
            // measurement_covariance_y << new_error_std_y*new_error_std_y;
            // measurement_covariance_z << new_error_std_z*new_error_std_z;
        }

        void updateMeasurementCovariance(Eigen::Vector3d cov_diagonal)
        {   
            measurement_covariance_pos = Eigen::Matrix3d::Identity();
            measurement_covariance_pos(0,0) *= cov_diagonal[0] * cov_diagonal[0];
            measurement_covariance_pos(1,1) *= cov_diagonal[1] * cov_diagonal[1];
            measurement_covariance_pos(2,2) *= cov_diagonal[2] * cov_diagonal[2];

            // measurement_covariance_x << cov_diagonal[0] * cov_diagonal[0];
            // measurement_covariance_y << cov_diagonal[1] * cov_diagonal[1];
            // measurement_covariance_z << cov_diagonal[2] * cov_diagonal[2];
        }

        void updateOrientationQuaternionFromEulerState()
        {
            // orientation_quaternion.setEuler(state_yaw[0], state_pitch[0], state_roll[0]);
            // orientation_quaternion.setRPY(state_roll[0], state_pitch[0], state_yaw[0]); // this seems better, but still don't seem right
            orientation_quaternion.setRPY(state_ori(0), state_ori(1), state_ori(2)); // 
        }

        // if no input is given function uses the current state
        // void updateOrientationQuaternion()
        // // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        // {
        //     double cr = cos(state_roll[0] * 0.5);
        //     double sr = sin(state_roll[0] * 0.5);
        //     double cp = cos(state_pitch[0] * 0.5);
        //     double sp = sin(state_pitch[0]  * 0.5);
        //     double cy = cos(state_yaw[0] * 0.5);
        //     double sy = sin(state_yaw[0] * 0.5);

        //     double w = cr * cp * cy + sr * sp * sy;
        //     double x = sr * cp * cy - cr * sp * sy;
        //     double y = cr * sp * cy + sr * cp * sy;
        //     double z = cr * cp * sy - sr * sp * cy;

        //     orientation_quaternion.setW(w);
        //     orientation_quaternion.setX(x);
        //     orientation_quaternion.setY(y);
        //     orientation_quaternion.setZ(z);

        //     orientation_quaternion.normalize();
        // }

        // overload if input is given this will be used.
        // void updateOrientationQuaternion(double new_roll, double new_pitch, double new_yaw)
        // // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        // {
        //     double cr = cos(new_roll * 0.5);
        //     double sr = sin(new_roll * 0.5);
        //     double cp = cos(new_pitch * 0.5);
        //     double sp = sin(new_pitch  * 0.5);
        //     double cy = cos(new_yaw * 0.5);
        //     double sy = sin(new_yaw * 0.5);

        //     double w = cr * cp * cy + sr * sp * sy;
        //     double x = sr * cp * cy - cr * sp * sy;
        //     double y = cr * sp * cy + sr * cp * sy;
        //     double z = cr * cp * sy - sr * sp * cy;

        //     orientation_quaternion.w() = w;
        //     orientation_quaternion.x() = x;
        //     orientation_quaternion.y() = y;
        //     orientation_quaternion.z() = z;

        //     orientation_quaternion.normalize();
        // }

        // void orientationQuaternionToEuler(Eigen::Quaterniond q) 
        // {
        //     // EulerAngles angles;

        //     // roll (x-axis rotation)
        //     double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
        //     double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
        //     roll = std::atan2(sinr_cosp, cosr_cosp);

        //     // pitch (y-axis rotation)
        //     double sinp = std::sqrt(1 + 2 * (q.w() * q.y() - q.x() * q.z()));
        //     double cosp = std::sqrt(1 - 2 * (q.w() * q.y() - q.x() * q.z()));
        //     pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2; 

        //     // yaw (z-axis rotation)
        //     double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
        //     double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
        //     yaw = std::atan2(siny_cosp, cosy_cosp); // limited to pi 

        // }

        // void orientationQuaternionToEuler(tf2::Quaternion q) 
        // {          
        //     tf2::Matrix3x3 orientation_matrix(q);

        //     // orientation_matrix.getEulerYPR(yaw, pitch, roll);
        //     orientation_matrix.getRPY(roll, pitch, yaw);
        //     limitEulerAngles();
        // }

        Eigen::Vector3d orientationQuaternionToEuler(tf2::Quaternion q) 
        {          
            tf2::Matrix3x3 orientation_matrix(q);

            // orientation_matrix.getEulerYPR(yaw, pitch, roll);
            Eigen::Vector3d euler;
            orientation_matrix.getRPY(euler.x(), euler.y(), euler.z());
            limitEulerAngles();
            return euler;
        }

        void limitEulerAngles()
        {
            while (roll > M_PI_2) // limited to +-pi/2 because it cannot be upside down
                roll -= M_PI_2;
            while (roll < -M_PI_2)
                roll += M_PI_2;

            while (pitch > M_PI_2) // limited to +-pi/2 because it cannot be upside down
                pitch -= M_PI_2;
            while (pitch < -M_PI_2)
                pitch += M_PI_2;

            while (yaw > M_PI)
                yaw -= M_PI;
            while (yaw < -M_PI)
                yaw += M_PI; 
        }

        void limitEulerAngles(Eigen::Vector3d &euler_vec)
        {
            while (euler_vec[0] > M_PI_2) // limited to +-pi/2 because it cannot be upside down
                euler_vec[0] -= M_PI_2;
            while (euler_vec[0] < -M_PI_2)
                euler_vec[0] += M_PI_2;

            while (euler_vec[1] > M_PI_2) // limited to +-pi/2 because it cannot be upside down
                euler_vec[1] -= M_PI_2;
            while (euler_vec[1] < -M_PI_2)
                euler_vec[1] += M_PI_2;

            while (euler_vec[2] > M_PI)
                euler_vec[2] -= M_PI;
            while (euler_vec[2] < -M_PI)
                euler_vec[2] += M_PI; 
        }

        void limitStateEulerAngles()
        {
            while (state_ori(0) > M_PI_2) // limited to +-pi/2 becuase it cannot be upside down
                state_ori(0) -= M_PI_2;
            while (state_ori(0) < -M_PI_2)
                state_ori(0) += M_PI_2;

            while (state_ori(1) > M_PI_2) // limited to +-pi/2 becuase it cannot be upside down
                state_ori(1) -= M_PI_2;
            while (state_ori(1) < -M_PI_2)
                state_ori(1) += M_PI_2;

            while (state_ori(2) > M_PI)
                state_ori(2) -= 2.0*M_PI;
            while (state_ori(2) < -M_PI)
                state_ori(2) += 2.0*M_PI; 
        }

        void imuDataHandler(const sensor_msgs::msg::Imu::SharedPtr imu_data)
        {  
            // put data into buffer back
            RCLCPP_INFO_ONCE(get_logger(),"First IMU message recieved..");
            imu_buffer.push_back(imu_data);    
        }

        void insHandler(const nav_msgs::msg::Odometry::SharedPtr ins_data)
        {          
            // put data into buffer back
            RCLCPP_INFO_ONCE(get_logger(),"First INS message recieved..");
            ins_buffer.push_back(ins_data);    
        }

        void progressINS(double cutoff_time)
        {
            //  ins_msg;
            // RCLCPP_INFO(get_logger(), "INS time test %i %i", ins_time < cutoff_time, ins_buffer.empty());
            if (ins_buffer.empty()){
                return;
            }
            // nav_msgs::msg::Odometry::SharedPtr ins_msg = ins_buffer.front();
            // ins_time = toSec(ins_msg->header.stamp);
            // RCLCPP_INFO(get_logger(), "syncing INS");
            while (ins_time + imu_dt_ < cutoff_time ){
                // while(ins_buffer.empty()){
                //     RCLCPP_INFO_ONCE(get_logger(), "waiting for INS");
                if (ins_buffer.empty())
                    break;
                    RCLCPP_INFO_ONCE(get_logger(), "empty INS buffer");
                // }
                nav_msgs::msg::Odometry::SharedPtr ins_msg = ins_buffer.front();
                // RCLCPP_INFO(get_logger(), "loop test INS");

                ins_position.x() = ins_msg->pose.pose.position.x;
                ins_position.y() = ins_msg->pose.pose.position.y;
                ins_position.z() = ins_msg->pose.pose.position.z;

                ins_velocity.x() = ins_msg->twist.twist.linear.x;
                ins_velocity.y() = ins_msg->twist.twist.linear.y;
                ins_velocity.z() = ins_msg->twist.twist.linear.z;

                ins_orientation.setW(ins_msg->pose.pose.orientation.w);
                ins_orientation.setX(ins_msg->pose.pose.orientation.x);
                ins_orientation.setY(ins_msg->pose.pose.orientation.y);
                ins_orientation.setZ(ins_msg->pose.pose.orientation.z);

                measurement_covariance_pos_vel_ins.block<3,3>(0,0) = getCovariancePos(ins_msg->pose.covariance);
                measurement_covariance_pos_vel_ins.block<3,3>(3,3) = getCovariancePos(ins_msg->twist.covariance);
                

                kalman_stamp = ins_msg->header.stamp;
                ins_time = toSec(ins_msg->header.stamp);
                ins_buffer.pop_front();
            }
            // RCLCPP_INFO(get_logger(), "INS synced");
            // measurementUpdateINSPosVel();

        }

        void runProcess(double cutoff_time)
        {   
            RCLCPP_INFO(get_logger(), "updating process to measurement.. cutoff: %f, current process time: %f",cutoff_time, process_time);
            while ((process_time < cutoff_time) && !imu_buffer.empty()){
                
                step++;
                // if (step < imu_step_delay_){ // if step is less than imu_step_delay return
                //     return;
                // }

                // get data from buffer front
                sensor_msgs::msg::Imu::SharedPtr delayed_imu_data = imu_buffer.front();
                kalman_stamp = delayed_imu_data->header.stamp;
                imu_buffer.pop_front();
                process_time = toSec(imu_buffer.front()->header.stamp); // get time stamp of next to compare with cutoff.


                // process_time = toSec(delayed_imu_data->header.stamp);

                
                madgwick_orientation_quaternion.setW(delayed_imu_data->orientation.w);
                madgwick_orientation_quaternion.setX(delayed_imu_data->orientation.x);
                madgwick_orientation_quaternion.setY(delayed_imu_data->orientation.y);
                madgwick_orientation_quaternion.setZ(delayed_imu_data->orientation.z);
                // tf2::Matrix3x3 orient_matrix(madgwick_orientation_quaternion);
                // orient_matrix.getEulerYPR(yaw, pitch, roll);
                // limitEulerAngles();

                acceleration << delayed_imu_data->linear_acceleration.x,
                                delayed_imu_data->linear_acceleration.y,
                                delayed_imu_data->linear_acceleration.z;

                angular_velocity << delayed_imu_data->angular_velocity.x,
                                    delayed_imu_data->angular_velocity.y,
                                    delayed_imu_data->angular_velocity.z;
    
                // RCLCPP_INFO(get_logger(), "angluar vel raw: %f %f %f", angular_velocity[0], angular_velocity[1], angular_velocity[2]);
                // RCLCPP_INFO(get_logger(), "acceleration raw: %f %f %f", acceleration[0], acceleration[1], acceleration[2]);
                // RCLCPP_INFO(get_logger(), "recieved madgwick pose roll: %f pitch: %f yaw: %f", roll*57.3, pitch*57.3, yaw*57.3);

                processUpdate();
            }
        }


        void processUpdate()
        {
            if (step < init_calib_steps_){
                // updateGravityCalibration(acceleration);
                updateZeroMeasurement();
            }

            // update orientaion state
            state_ori   = A_ori * state_ori    +  B_ori * angular_velocity;
            limitStateEulerAngles(); // limits angles within ranges of +-pi

            // convert acceleration input to world frame by rotating by the imu orientation
            // updateOrientationQuaternion();
            updateOrientationQuaternionFromEulerState();
            // tf2::Matrix3x3 rot_mat(orientation_quaternion);
            // Eigen::Matrix3d rot_mat_eigen(rot_mat);
            Eigen::Quaterniond quat_eigen(orientation_quaternion.getX(),
                                          orientation_quaternion.getY(),
                                          orientation_quaternion.getZ(),
                                          orientation_quaternion.getW());

            acceleration = quat_eigen * acceleration; // if offset orientation is to be used apply it here
            // RCLCPP_INFO(get_logger(), "acceleration rotated: %f %f %f", acceleration[0], acceleration[1], acceleration[2]);

            // subtract gravity
            // Eigen::Vector3d gravity(0.0,0.0, g_acc);
            // acceleration = acceleration - gravity;

            // RCLCPP_INFO(get_logger(), "acceleration sub gravity: %f %f %f", acceleration[0], acceleration[1], acceleration[2]);
            // prediction step

            // RCLCPP_INFO(get_logger(), "test matrix IMU ");
            state_pos = A_pos * state_pos + B_pos * acceleration;
            // RCLCPP_INFO(get_logger(), "test matrix IMU ");

            // update uncertainty and add process noise
            uncertainty_ori     = A_ori * uncertainty_ori   * A_ori_T + noise_ori;
            uncertainty_pos = A_pos * uncertainty_pos * A_pos_T + noise_pos;
            // RCLCPP_INFO(get_logger(), "test matrix IMU ");
            
            // RCLCPP_INFO(get_logger(), "uncertaint_pitch = %f", uncertainty_pitch(0,0));

            updateOrientationQuaternionFromEulerState();

            latestPoseInfo.qw = orientation_quaternion.getW();
            latestPoseInfo.qx = orientation_quaternion.getX();
            latestPoseInfo.qy = orientation_quaternion.getY();
            latestPoseInfo.qz = orientation_quaternion.getZ();
            // latestPoseInfo.x = state_x(0); 
            // latestPoseInfo.y = state_y(0);
            // latestPoseInfo.z = state_z(0);
            latestPoseInfo.x = state_pos(0); 
            latestPoseInfo.y = state_pos(1);
            latestPoseInfo.z = state_pos(2);

            
            publishOdometry();
            // printStates();
        }

        Eigen::Matrix3d getCovariancePos(std::array<double, 36> &cov_msg)
        {
            vector<int> pos_cov_array_idx{0,1,2,6,7,8,12,13,14};
            Eigen::Matrix3d cov_mat;
            for (int i=0 ; i < 9; i++){
                // RCLCPP_INFO(get_logger(),"%i, %f", pos_cov_array_idx[i], cov_msg[pos_cov_array_idx[i]]);
                cov_mat(i) = cov_msg[pos_cov_array_idx[i]] ;
            }

            return cov_mat;
        }

        Eigen::Matrix3d getCovarianceOri(std::array<double, 36> &cov_msg)
        {
            vector<int> pos_cov_array_idx{0,1,2,6,7,8,12,13,14};
            Eigen::Matrix3d cov_mat;
            for (int i=0 ; i < 9; i++){
                // RCLCPP_INFO(get_logger(),"%i, %f", pos_cov_array_idx[i], cov_msg[pos_cov_array_idx[i]]);
                cov_mat(i) = cov_msg[pos_cov_array_idx[i]+ 21] ;
            }

            return cov_mat;
        }

        Eigen::Vector3d getCovarianceDiagonalPos(std::array<double, 36> &cov_msg)
        {
            // vector<int> pos_cov_array_idx{0,1,2,6,7,8,12,13,14};
            vector<int> pos_cov_array_idx{0,7,14};
            Eigen::Vector3d cov_vec;
            for (int i=0 ; i < 3; i++){
                // RCLCPP_INFO(get_logger(),"%i, %f", pos_cov_array_idx[i], cov_msg[pos_cov_array_idx[i]]);
                cov_vec[i] = cov_msg[pos_cov_array_idx[i]] ;
            }

            return cov_vec;
        }


        void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odom_message)
        {
            // NOTE: the lidar measurement are (almost) absolute mearuements and should not be transformed
            // DONE !! TODO IMPORTANT: IN THE CASE THAT YAW CROSSOVER FROM -180 TO 180 ( 360 TO 0) THAT THE ERROR IS FOUND AS THE SHORTEST PATH !!
            // TODO get timestamp and only use imu measurements up to this timestamp? DONE
            // rclcpp::Time new_time = odom_message->header.stamp; // make an update time-stamp
            // odom_message.header.stamp = cloud_header.stamp;

            if (relative_mode_)
                return;
            
            // Eigen::Quaterniond lidar_orientation(odom_message->pose.pose.orientation.w, 
            //                                      odom_message->pose.pose.orientation.x,
            //                                      odom_message->pose.pose.orientation.y,
            //                                      odom_message->pose.pose.orientation.z);


            // process imu up till the measurement time step to make the state up to date for the incoming measurement
            measurement_time = toSec(odom_message->header.stamp);
            runProcess(measurement_time); // runs process from imu buffer up till the input time;
            // RCLCPP_INFO(get_logger(), "test INS");

            // RCLCPP_INFO(get_logger(),"What happens? %i", imu_buffer.size());
            progressINS(measurement_time);// set the ins input to that of the lidar

            Eigen::Vector3d lidar_position(odom_message->pose.pose.position.x,
                                           odom_message->pose.pose.position.y,
                                           odom_message->pose.pose.position.z);

            tf2::Quaternion lidar_orientation(odom_message->pose.pose.orientation.x, 
                                              odom_message->pose.pose.orientation.y,
                                              odom_message->pose.pose.orientation.z,
                                              odom_message->pose.pose.orientation.w);

            measurement_covariance_pos = getCovariancePos(odom_message->pose.covariance);
            measurement_covariance_ori_lidar = getCovarianceOri(odom_message->pose.covariance);
            // Eigen::Vector3d cov_diagonal = getCovarianceDiagonalPos(odom_message->pose.covariance);
            // updateMeasurementCovariance(cov_diagonal);
            // updateMeasurementCovariance()

            // for (int i=0 ; i < 3; i++){
            //     RCLCPP_INFO(get_logger(),"cov[%i]= %f", i, cov_diagonal[i]);
            // }


            // ORIENTATION
            // measurementUpdateOrientation(lidar_orientation);
            if (use_madgwick_orientation_){
                measurementUpdateOrientationMadgwick(madgwick_orientation_quaternion);
            } else {
                // measurementUpdateOrientationINS(ins_orientation);
                measurementUpdateOrientationLidar(lidar_orientation);
            }

            // // POSITION
            // measurementUpdateINSPosVel();
            // RCLCPP_INFO(get_logger(), "problem finder..");
            measurementUpdatePosition(lidar_position);


            updateOrientationQuaternionFromEulerState();

            state_pos_post = Eigen::Vector3d(state_pos(0), state_pos(1), state_pos(2));

            // latestPoseInfo.time = odom_message->header.stamp;
            latestPoseInfo.qw = orientation_quaternion.getW();
            latestPoseInfo.qx = orientation_quaternion.getX();
            latestPoseInfo.qy = orientation_quaternion.getY();
            latestPoseInfo.qz = orientation_quaternion.getZ();
            latestPoseInfo.x = state_pos(0); 
            latestPoseInfo.y = state_pos(1);
            latestPoseInfo.z = state_pos(2);


            step_since_keyframe++;
            // updateMeasurementNoise(measurement_noise_pos_* pow(1.1 , step_since_keyframe) );

            // RCLCPP_INFO(get_logger(), "Absolute is running");
            publishOdometry();

            
            publishBias();

            printStates();

            saveRunningData();

        }

        void transformationHandler(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr transform_message)
        {
            // NOTE: the lidar measurement are (almost) absolute mearuements and should not be transformed
            // DONE !! TODO IMPORTANT: IN THE CASE THAT YAW CROSSOVER FROM -180 TO 180 ( 360 TO 0) THAT THE ERROR IS FOUND AS THE SHORTEST PATH !!
            // TODO get timestamp and only use lidar measurements up to this timestamp? DONE
            // rclcpp::Time new_time = odom_message->header.stamp; // make an update time-stamp
            // odom_message.header.stamp = cloud_header.stamp;


            // Eigen::Quaterniond lidar_orientation(odom_message->pose.pose.orientation.w, 
            //                                      odom_message->pose.pose.orientation.x,
            //                                      odom_message->pose.pose.orientation.y,
            //                                      odom_message->pose.pose.orientation.z);


            // process imu up till the measurement time step to make the state up to date for the incoming measurement
            if (!relative_mode_) {
                return;
            }

            

            measurement_time = toSec(transform_message->header.stamp);
            runProcess(measurement_time); // runs process from imu buffer up till the input time;

            Eigen::Vector3d translation(transform_message->pose.pose.position.x,
                                           transform_message->pose.pose.position.y,
                                           transform_message->pose.pose.position.z);

            // tf2::Quaternion rotation_orientation(transform_message->pose.pose.orientation.x, 
            //                                   transform_message->pose.pose.orientation.y,
            //                                   transform_message->pose.pose.orientation.z,
            //                                   transform_message->pose.pose.orientation.w);

            // measurement_covariance_pos = getCovariancePos(odom_message->pose.covariance);
            Eigen::Vector3d cov_diagonal = getCovarianceDiagonalPos(transform_message->pose.covariance);
            updateMeasurementCovariance(cov_diagonal);

            // for (int i=0 ; i < 3; i++){
            //     RCLCPP_INFO(get_logger(),"cov[%i]= %f", i, cov_diagonal[i]);
            // }


            // ORIENTATION
            // measurementUpdateOrientation(lidar_orientation);
            if (use_madgwick_orientation_){

                measurementUpdateOrientationLidar(madgwick_orientation_quaternion);
            }
            updateOrientationQuaternionFromEulerState();

            // // POSITION
            // rotate translation
            
            // SHOULD BE ROTATED  BY THE LIDAR ORIENTATION AND NOT THE STATE?
            translation = tf2Eigen(orientation_quaternion) * translation;
            measurementUpdateTranslation(translation);



            // RCLCPP_INFO(get_logger(), "Lidar measurement: pos = %f, %f %f", lidar_position[0], lidar_position[1], lidar_position[2]);
            // RCLCPP_INFO(get_logger(), "Lidar measurement: rpy = %f, %f, %f", roll*57.3, pitch*57.3, yaw*57.3);
            
            state_pos_post = Eigen::Vector3d( state_pos(0), state_pos(1), state_pos(2));

            // latestPoseInfo.time = odom_message->header.stamp;
            // latestPoseInfo.qw = odom_message->pose.pose.orientation.w;
            // latestPoseInfo.qx = odom_message->pose.pose.orientation.x;
            // latestPoseInfo.qy = odom_message->pose.pose.orientation.y;
            // latestPoseInfo.qz = odom_message->pose.pose.orientation.z;
            latestPoseInfo.qw = orientation_quaternion.getW();
            latestPoseInfo.qx = orientation_quaternion.getX();
            latestPoseInfo.qy = orientation_quaternion.getY();
            latestPoseInfo.qz = orientation_quaternion.getZ();
            // latestPoseInfo.x = state_x(0);
            // latestPoseInfo.y = state_y(0);
            // latestPoseInfo.z = state_z(0);
            latestPoseInfo.x = state_pos(0); 
            latestPoseInfo.y = state_pos(1);
            latestPoseInfo.z = state_pos(2);

            step_since_keyframe++;
            // updateMeasurementNoise(measurement_noise_pos_* pow(1.1 , step_since_keyframe) );

            RCLCPP_INFO(get_logger(), "Relative is running");
            // publishOdometry();

        }

        Eigen::Quaterniond tf2Eigen(tf2::Quaternion tf2q)
        {
            Eigen::Quaterniond Eq(tf2q.getW(),tf2q.getX(), tf2q.getY(),tf2q.getZ() );
            return Eq;
        }

        void measurementUpdateINSPosVel()
        {  
            // POSITION
            // update kalman gain
            // gain_x = uncertainty_x * H_ins_T * (H_ins * uncertainty_x * H_ins_T + measurement_covariance_x).inverse(); 
            // gain_y = uncertainty_y * H_ins_T * (H_ins * uncertainty_y * H_ins_T + measurement_covariance_y).inverse(); 
            // gain_z = uncertainty_z * H_ins_T * (H_ins * uncertainty_z * H_ins_T + measurement_covariance_z).inverse(); 
            // RCLCPP_INFO(get_logger(), "test INS matrix 0");
            // H_ins * uncertainty_pos * H_ins_T + measurement_covariance_pos_vel_ins ;
            // RCLCPP_INFO(get_logger(), "problem finder 2 electric boogaloo..");
            // gain_ins = uncertainty_pos * H_ins_T * (H_ins * uncertainty_pos * H_ins_T + measurement_covariance_pos_vel_ins).inverse(); 
            gain_ins = uncertainty_pos * H_ins_T * (H_ins * uncertainty_pos * H_ins_T + measurement_covariance_pos_vel_ins).inverse();
            Eigen::MatrixXd ins_full(6,1); 
            // Eigen::MatrixXd ins_y(2,1);
            // Eigen::MatrixXd ins_z(2,1);

            // RCLCPP_INFO(get_logger(), "test INS matrix 1");
            
            // ins_full << ins_position[0], ins_velocity[0];
            // ins_full << ins_position[1], ins_velocity[1];
            // ins_full << ins_position[2], ins_velocity[2];
            ins_full << ins_position, ins_velocity;

            // RCLCPP_INFO(get_logger(), "test INS matrix 2");

            // update state with kalman gain
            // state_x += gain_x * (ins_x - H_ins * state_x);
            // state_y += gain_y * (ins_y - H_ins * state_y);
            // state_z += gain_z * (ins_z - H_ins * state_z);
            state_pos += gain_ins * (ins_full - H_ins * state_pos);
            // RCLCPP_INFO(get_logger(), "test INS matrix 3");
          
            // recalculate uncertainty after measurement
            // uncertainty_x = (Eigen::Matrix3d::Identity() - gain_x * H_ins) * uncertainty_x;
            // uncertainty_y = (Eigen::Matrix3d::Identity() - gain_y * H_ins) * uncertainty_y;
            // uncertainty_z = (Eigen::Matrix3d::Identity() - gain_z * H_ins) * uncertainty_z;

            // uncertainty_pos = (Eye9 - gain_pos * H_pos) * uncertainty_pos; // this is unstable apparently
            uncertainty_pos = (Eye9 - gain_ins * H_ins) * uncertainty_pos * (Eye9 - gain_ins * H_ins).transpose() + gain_ins * measurement_covariance_pos_vel_ins * gain_ins.transpose(); 
        }

        void measurementUpdatePosition(Eigen::Vector3d lidar_position)
        {  
            // POSITION
            // update kalman gain
            // gain_x = uncertainty_x * H_pos_T * (H_pos * uncertainty_x * H_pos_T + measurement_covariance_x).inverse(); 
            // gain_y = uncertainty_y * H_pos_T * (H_pos * uncertainty_y * H_pos_T + measurement_covariance_y).inverse(); 
            // gain_z = uncertainty_z * H_pos_T * (H_pos * uncertainty_z * H_pos_T + measurement_covariance_z).inverse(); 
            Eigen::Matrix3d HPHt = H_pos * uncertainty_pos * H_pos_T;
            // RCLCPP_INFO(get_logger(), "HPHt1 %f %f %f", HPHt.col(0)[0], HPHt.col(0)[1], HPHt.col(0)[2]);
            // RCLCPP_INFO(get_logger(), "HPHt2 %f %f %f", HPHt.col(1)[0], HPHt.col(1)[1], HPHt.col(1)[2]);
            // RCLCPP_INFO(get_logger(), "HPHt3 %f %f %f", HPHt.col(2)[0], HPHt.col(2)[1], HPHt.col(2)[2]);



            // RCLCPP_INFO(get_logger(), "gain1 %f %f %f %f %f %f %f %f %f", gain_pos.col(0)[0], gain_pos.col(0)[1], gain_pos.col(0)[2], gain_pos.col(0)[3], gain_pos.col(0)[4], gain_pos.col(0)[5], gain_pos.col(0)[6], gain_pos.col(0)[7], gain_pos.col(0)[8] );
            // RCLCPP_INFO(get_logger(), "gain2 %f %f %f %f %f %f %f %f %f", gain_pos.col(1)[0], gain_pos.col(1)[1], gain_pos.col(1)[2], gain_pos.col(1)[3], gain_pos.col(1)[4], gain_pos.col(1)[5], gain_pos.col(1)[6], gain_pos.col(1)[7], gain_pos.col(1)[8] );
            // RCLCPP_INFO(get_logger(), "gain3 %f %f %f %f %f %f %f %f %f", gain_pos.col(2)[0], gain_pos.col(2)[1], gain_pos.col(2)[2], gain_pos.col(2)[3], gain_pos.col(2)[4], gain_pos.col(2)[5], gain_pos.col(2)[6], gain_pos.col(2)[7], gain_pos.col(2)[8] );
            gain_pos = uncertainty_pos * H_pos_T * (H_pos * uncertainty_pos * H_pos_T + measurement_covariance_pos).inverse(); 
            // RCLCPP_INFO(get_logger(), "gain1 %f %f %f %f %f %f %f %f %f", gain_pos.col(0)[0], gain_pos.col(0)[1], gain_pos.col(0)[2], gain_pos.col(0)[3], gain_pos.col(0)[4], gain_pos.col(0)[5], gain_pos.col(0)[6], gain_pos.col(0)[7], gain_pos.col(0)[8] );
            // RCLCPP_INFO(get_logger(), "gain2 %f %f %f %f %f %f %f %f %f", gain_pos.col(1)[0], gain_pos.col(1)[1], gain_pos.col(1)[2], gain_pos.col(1)[3], gain_pos.col(1)[4], gain_pos.col(1)[5], gain_pos.col(1)[6], gain_pos.col(1)[7], gain_pos.col(1)[8] );
            // RCLCPP_INFO(get_logger(), "gain3 %f %f %f %f %f %f %f %f %f", gain_pos.col(2)[0], gain_pos.col(2)[1], gain_pos.col(2)[2], gain_pos.col(2)[3], gain_pos.col(2)[4], gain_pos.col(2)[5], gain_pos.col(2)[6], gain_pos.col(2)[7], gain_pos.col(2)[8] );
         


            Eigen::Vector3d innovation(lidar_position - H_pos * state_pos);
            // RCLCPP_INFO(get_logger(), "innovation %f %f %f", innovation[0], innovation[1], innovation[2] );

            Eigen::VectorXd gained_innovation(9);
            gained_innovation = gain_pos * innovation;
            // RCLCPP_INFO(get_logger(), "gained innovation %f %f %f %f %f %f %f %f %f", gained_innovation[0], gained_innovation[1], gained_innovation[2], gained_innovation[3], gained_innovation[4], gained_innovation[5], gained_innovation[6], gained_innovation[7], gained_innovation[8] );

            state_pos += gain_pos * (lidar_position - H_pos * state_pos);
            // RCLCPP_INFO(get_logger(), "test lidar matrix2");
          
            // recalculate uncertainty after measurement

            Eigen::MatrixXd Eye9_gain_H(9,9);
            Eye9_gain_H << (Eye9 - gain_pos * H_pos);
            // uncertainty_pos = (Eye9 - gain_pos * H_pos) * uncertainty_pos; // this is unstable apparently
            uncertainty_pos = Eye9_gain_H * uncertainty_pos * Eye9_gain_H.transpose() + gain_pos * measurement_covariance_pos * gain_pos.transpose(); 
            // RCLCPP_INFO(get_logger(), "test lidar matrix3");

            HPHt = H_pos * uncertainty_pos * H_pos_T;
            // RCLCPP_INFO(get_logger(), "HPHt1 %f %f %f", HPHt.col(0)[0], HPHt.col(0)[1], HPHt.col(0)[2]);
            // RCLCPP_INFO(get_logger(), "HPHt2 %f %f %f", HPHt.col(1)[0], HPHt.col(1)[1], HPHt.col(1)[2]);
            // RCLCPP_INFO(get_logger(), "HPHt3 %f %f %f", HPHt.col(2)[0], HPHt.col(2)[1], HPHt.col(2)[2]);
            // print state

            Eigen::Quaterniond ori = tf2Eigen(orientation_quaternion);
            body_acc_bias = Eigen::Vector3d(state_pos(6), state_pos(7), state_pos(8));
            body_acc_bias = ori * body_acc_bias;
            
        }

        void measurementUpdateTranslation(Eigen::Vector3d translation)
        {  
            // POSITION
            Eigen::Vector3d new_position = state_pos_post + translation;

            // update kalman gain
            // gain_x = uncertainty_x * H_pos_T * (H_pos * uncertainty_x * H_pos_T + measurement_covariance_x).inverse(); 
            // gain_y = uncertainty_y * H_pos_T * (H_pos * uncertainty_y * H_pos_T + measurement_covariance_y).inverse(); 
            // gain_z = uncertainty_z * H_pos_T * (H_pos * uncertainty_z * H_pos_T + measurement_covariance_z).inverse(); 
            gain_pos = uncertainty_pos * H_pos_T * (H_pos * uncertainty_pos * H_pos_T + measurement_covariance_pos ).inverse(); 

            // Eigen::MatrixXd x(1,1);
            // Eigen::MatrixXd y(1,1);
            // Eigen::MatrixXd z(1,1);
            // x << new_position[0];
            // y << new_position[1];
            // z << new_position[2];


            // update state with kalman gain
            // state_x += gain_x * (x - H_pos * state_x);
            // state_y += gain_y * (y - H_pos * state_y);
            // state_z += gain_z * (z - H_pos * state_z);
            state_pos += gain_pos * (new_position - H_pos * state_pos);
          
            // recalculate uncertainty after measurement
            // uncertainty_x = (Eigen::Matrix3d::Identity() - gain_x * H_pos) * uncertainty_x;
            // uncertainty_y = (Eigen::Matrix3d::Identity() - gain_y * H_pos) * uncertainty_y;
            // uncertainty_z = (Eigen::Matrix3d::Identity() - gain_z * H_pos) * uncertainty_z;

            // uncertainty_pos = (Eye9 - gain_pos * H_pos) * uncertainty_pos; // this is unstable apparently
            uncertainty_pos = (Eye9 - gain_pos * H_pos) * uncertainty_pos * (Eye9 - gain_pos * H_pos).transpose() + gain_pos * measurement_covariance_pos * gain_pos.transpose(); 
        }

        void measurementUpdateOrientationLidar(tf2::Quaternion orientation)
        {
            Eigen::Vector3d euler_orientation = orientationQuaternionToEuler(orientation); // updates values roll, pitch, and yaw from the input quaternion
            limitEulerAngles(euler_orientation);
            // ORIENTATION
            // update kalman gain
            // gain_roll   = uncertainty_roll  * H_ori_T * (H_ori * uncertainty_roll  * H_ori_T + measurement_noise_roll ).inverse(); 
            // gain_pitch  = uncertainty_pitch * H_ori_T * (H_ori * uncertainty_pitch * H_ori_T + measurement_noise_pitch).inverse(); 
            // gain_yaw    = uncertainty_yaw   * H_ori_T * (H_ori * uncertainty_yaw   * H_ori_T + measurement_noise_yaw  ).inverse(); 
            gain_ori_lidar    = uncertainty_ori   * H_ori_T * (H_ori * uncertainty_ori   * H_ori_T + measurement_covariance_ori_lidar  ).inverse(); 

            // double yaw_innovation = yaw  - H_ori * state_yaw;
            Eigen::Vector3d innovation = euler_orientation - H_ori * state_ori;
            
            RCLCPP_INFO(get_logger(), "Lidar measured RPY: %f %f %f", euler_orientation.x()*57.3, euler_orientation.y()*57.3, euler_orientation.z()*57.3 );
            RCLCPP_INFO(get_logger(), "current RPY: %f %f %f", state_ori(0)*57.3, state_ori(1)*57.3, state_ori(2)*57.3 );
            // RCLCPP_INFO(get_logger(), "yaw innovation before: %f", yaw_innovation*57.3 );
            if (innovation[2] >= M_PI){
                innovation[2] -= M_PI;
                // yaw_innovation *= -1.0;
            } else if (innovation[2] <= - M_PI){
                innovation[2] += M_PI;
                // yaw_innovation *= -1.0;
            }
            // RCLCPP_INFO(get_logger(), "yaw innovation after: %f", yaw_innovation*57.3 );
            // only yaw is expected to have crossover error else the object is upside down..

            // update state with kalman gain
            // state_roll  += gain_roll  * (roll  - H_ori * state_roll);
            // state_pitch += gain_pitch * (pitch - H_ori * state_pitch); 
            // state_yaw   += gain_yaw   * yaw_innovation; // <-- YAW CROSSOVER ERROR CORRECTED HERE ! see note at top of function
            state_ori   += gain_ori_lidar   * innovation; //

            // recalculate uncertainty after measurement
            // uncertainty_roll  = (Eigen::Matrix2d::Identity() - gain_roll  * H_ori) * uncertainty_roll ;
            // uncertainty_pitch = (Eigen::Matrix2d::Identity() - gain_pitch * H_ori) * uncertainty_pitch;
            // uncertainty_yaw   = (Eigen::Matrix2d::Identity() - gain_yaw   * H_ori) * uncertainty_yaw  ;
            uncertainty_ori   = (Eye6 - gain_ori_lidar   * H_ori) * uncertainty_ori  ;
        }

        void measurementUpdateOrientationINS(tf2::Quaternion orientation)
        {
            Eigen::Vector3d euler_orientation = orientationQuaternionToEuler(orientation); // updates values roll, pitch, and yaw from the input quaternion
            limitEulerAngles(euler_orientation);
            // ORIENTATION
            // update kalman gain
            gain_ori_ins    = uncertainty_ori   * H_ori_T * (H_ori * uncertainty_ori   * H_ori_T + measurement_covariance_ori_ins  ).inverse(); 

            // double yaw_innovation = yaw  - H_ori * state_yaw;
            Eigen::Vector3d innovation = euler_orientation - H_ori * state_ori;
            
            RCLCPP_INFO(get_logger(), "INS measured RPY: %f %f %f", euler_orientation.x()*57.3, euler_orientation.y()*57.3, euler_orientation.z()*57.3 );
            RCLCPP_INFO(get_logger(), "current RPY: %f %f %f", state_ori(0)*57.3, state_ori(1)*57.3, state_ori(2)*57.3 );
            // RCLCPP_INFO(get_logger(), "yaw innovation before: %f", yaw_innovation*57.3 );
            if (innovation[2] >= M_PI){
                innovation[2] -= M_PI;
                // yaw_innovation *= -1.0;
            } else if (innovation[2] <= - M_PI){
                innovation[2] += M_PI;
                // yaw_innovation *= -1.0;
            }
            // RCLCPP_INFO(get_logger(), "yaw innovation after: %f", yaw_innovation*57.3 );
            // only yaw is expected to have crossover error else the object is upside down..

            // update state with kalman gain
            state_ori   += gain_ori_ins   * innovation; //
            

            // recalculate uncertainty after measurement 
            uncertainty_ori   = (Eye6 - gain_ori_ins   * H_ori) * uncertainty_ori  ;
        }
        
        void measurementUpdateOrientationMadgwick(tf2::Quaternion orientation)
        {
            Eigen::Vector3d euler_orientation = orientationQuaternionToEuler(orientation); // updates values roll, pitch, and yaw from the input quaternion
            limitEulerAngles(euler_orientation);
            // ORIENTATION
            // update kalman gain
            Eigen::Matrix3d meas_cov = measurement_covariance_ori_ins;
            meas_cov(2,2) = measurement_covariance_ori_ins(2,2) * 10;
            gain_ori_ins    = uncertainty_ori   * H_ori_T * (H_ori * uncertainty_ori   * H_ori_T + meas_cov ).inverse(); 

            // double yaw_innovation = yaw  - H_ori * state_yaw;
            Eigen::Vector3d innovation = euler_orientation - H_ori * state_ori;
            
            RCLCPP_INFO(get_logger(), "madgwick measured RPY: %f %f %f", euler_orientation.x()*57.3, euler_orientation.y()*57.3, euler_orientation.z()*57.3 );
            RCLCPP_INFO(get_logger(), "current RPY: %f %f %f", state_ori(0)*57.3, state_ori(1)*57.3, state_ori(2)*57.3 );
            // RCLCPP_INFO(get_logger(), "yaw innovation before: %f", yaw_innovation*57.3 );
            if (innovation[2] >= M_PI){
                innovation[2] -= M_PI;
                // yaw_innovation *= -1.0;
            } else if (innovation[2] <= - M_PI){
                innovation[2] += M_PI;
                // yaw_innovation *= -1.0;
            }
            // RCLCPP_INFO(get_logger(), "yaw innovation after: %f", yaw_innovation*57.3 );
            // only yaw is expected to have crossover error else the object is upside down..

            // update state with kalman gain
            state_ori  += gain_ori_ins   * innovation; //
            

            // recalculate uncertainty after measurement 
            uncertainty_ori   = (Eye6 - gain_ori_ins   * H_ori) * uncertainty_ori  ;
        }
        

        // void keyframeHandler(const nav_msgs::msg::Odometry::SharedPtr odom_message)
        // {   
        //     // reset the position noise when a keyframe arrives
        //     updateMeasurementNoise(measurement_noise_pos_);
        //     step_since_keyframe = 1;
        // }

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

        void publishOdometry()
        {
            // PoseInfo latestPoseInfo = odometry_pose_info->points[latest_keyframe_idx - 1];
            nav_msgs::msg::Odometry odom;
            // odom.header.stamp = cloud_header.stamp;
            odom.header.frame_id = "odom";
            odom.header.stamp = kalman_stamp;
 
            odom.pose.pose.orientation.w = latestPoseInfo.qw;
            odom.pose.pose.orientation.x = latestPoseInfo.qx;
            odom.pose.pose.orientation.y = latestPoseInfo.qy;
            odom.pose.pose.orientation.z = latestPoseInfo.qz;
            odom.pose.pose.position.x = latestPoseInfo.x;
            odom.pose.pose.position.y = latestPoseInfo.y;
            odom.pose.pose.position.z = latestPoseInfo.z;

            // vector<double> cov_diag{0.0,0.0,0.0, 0.0,0.0,0.0};
            // Eigen::MatrixXd cov_mat = createCovarianceEigen(cov_diag);
            // Eigen::Matrix3d rot_mat = last_odometry_pose.block<3,3>(0,0);
            // Eigen::Matrix3d rot_mat(Eigen::Quaterniond(latestPoseInfo.qw, latestPoseInfo.qx, latestPoseInfo.qy, latestPoseInfo.qz ));
            Eigen::MatrixXd cov_mat = Eigen::Matrix<double,6,6>();
            cov_mat = uncertainty_pos.block<6,6>(0,0);
            // Eigen::Matrix3d ori_cov;
            // ori_cov(0,0) = uncertainty_yaw(0,0);
            // ori_cov(1,1) = uncertainty_pitch(0,0);
            // ori_cov(2,2) = uncertainty_roll(0,0);
            cov_mat.block<3,3>(3,3) = uncertainty_ori.block<3,3>(0,0); // set ori covariances
            // rotateCovMatrix(cov_mat, rot_mat);
            setCovariance(odom.pose.covariance, cov_mat);



            // odom.twist.twist.linear.x // add the velocities in twist
            odometry_pub->publish(odom);



            // odom -> base_link transform
            // geometry_msgs::msg::TransformStamped t_;
            // // t_.header.stamp = this->get_clock()->now();
            // t_.header.stamp = cloud_header.stamp;
            // t_.header.frame_id = "odom";//"lidar_odom";
            // t_.child_frame_id = "base_link"; // "livox_frame"

            // t_.transform.rotation.w = odom.pose.pose.orientation.w;
            // t_.transform.rotation.x = odom.pose.pose.orientation.x;
            // t_.transform.rotation.y = odom.pose.pose.orientation.y;
            // t_.transform.rotation.z = odom.pose.pose.orientation.z;
            // t_.transform.translation.x = odom.pose.pose.position.x;            
            // t_.transform.translation.y = odom.pose.pose.position.y;  
            // t_.transform.translation.z = odom.pose.pose.position.z;
            // tf_broadcaster_->sendTransform(t_);


            geometry_msgs::msg::PoseStamped poseStamped;
            poseStamped.header = odom.header;
            poseStamped.pose = odom.pose.pose;
            poseStamped.header.stamp = odom.header.stamp;
            path.header.stamp = odom.header.stamp;
            path.poses.push_back(poseStamped);
            // path.header.frame_id = frame_id;
            path.header.frame_id = "odom"; // "livox_frame"
            path_pub->publish(path);
        }

        void publishBias()
        {   
            if (!publish_bias_)
                return;
            // using a wrench as data transfer as it has two vec3 elements,
            geometry_msgs::msg::WrenchStamped bias_msg;
            bias_msg.header.frame_id = "kalman_bias";
            bias_msg.header.stamp = kalman_stamp;


            int pos_bias_loc = state_pos.size() - 3;
            // force is acc bias
            bias_msg.wrench.force.x = state_pos(pos_bias_loc);
            bias_msg.wrench.force.y = state_pos(pos_bias_loc + 1);
            bias_msg.wrench.force.z = state_pos(pos_bias_loc + 2);

            int ori_bias_loc = state_ori.size() - 3;
            // torque is angular bias
            bias_msg.wrench.torque.x = state_ori(ori_bias_loc);
            bias_msg.wrench.torque.y = state_ori(ori_bias_loc + 1);
            bias_msg.wrench.torque.z = state_ori(ori_bias_loc + 2);

            bias_pub->publish(bias_msg);

        }

        

    
        void updateZeroMeasurement() // fake update, giving a zero value measurement to estimate bias. This is run when the rover is assumend to be stationary in the inital part of the recordings
        {

            RCLCPP_INFO(get_logger(), "Zero-update!");

            // kalman update step, becuase uncertainty has increased during process of IMU inputs
            // gain_pitch  = uncertainty_pitch * H_ori_T * (H_ori * uncertainty_pitch  * H_ori_T + measurement_noise_pitch).inverse(); 
            // gain_yaw    = uncertainty_yaw   * H_ori_T * (H_ori * uncertainty_yaw    * H_ori_T + measurement_noise_yaw).inverse(); 
            // gain_roll   = uncertainty_roll  * H_ori_T * (H_ori * uncertainty_roll   * H_ori_T + measurement_noise_roll).inverse(); 
            gain_ori_lidar   = uncertainty_ori  * H_ori_T * (H_ori * uncertainty_ori   * H_ori_T).inverse(); 

            // gain_x = uncertainty_x * H_pos_T * (H_pos * uncertainty_x * H_pos_T + measurement_covariance_x).inverse(); 
            // gain_y = uncertainty_y * H_pos_T * (H_pos * uncertainty_y * H_pos_T + measurement_covariance_y).inverse(); 
            // gain_z = uncertainty_z * H_pos_T * (H_pos * uncertainty_z * H_pos_T + measurement_covariance_z).inverse();


            gain_pos = uncertainty_pos * H_pos_T * (H_pos * uncertainty_pos * H_pos_T + measurement_covariance_pos).inverse(); 


            // update state from measurement in this case it is an assumed static postion and orientation
            Eigen::Vector3d init_calib_vec(init_calib_roll, init_calib_pitch, init_calib_yaw);
            // state_pitch +=  gain_pitch  * (init_calib_pitch - H_ori * state_pitch);
            // state_yaw   +=  gain_yaw    * (init_calib_yaw - H_ori * state_yaw);
            // state_roll  +=  gain_roll   * (init_calib_roll - H_ori * state_roll);
            state_ori  +=  gain_ori_lidar   * (init_calib_vec - H_ori * state_ori);

            // Eigen::MatrixXd zero(1,1);
            // zero << 0.0;

            // state_x += gain_x * (zero - H_pos * state_x);
            // state_y += gain_y * (zero - H_pos * state_y);
            // state_z += gain_z * (zero - H_pos * state_z);
            Eigen::Vector3d zero_vector(0.0, 0.0, 0.0);
            state_pos += gain_pos * (zero_vector - H_pos * state_pos);
          

            RCLCPP_INFO(get_logger(), "zero measurement: ori state = %f, %f, %f, bias = %f, %f, %f", state_ori(0), state_ori(1), state_ori(2), state_ori(3), state_ori(4), state_ori(5));
            RCLCPP_INFO(get_logger(), "zero measurement: pos state = %f, %f, %f, vel = %f, %f, %f, bias = %f, %f, %f", state_pos(0), state_pos(1), state_pos(2), state_pos(3), state_pos(4), state_pos(5), state_pos(6), state_pos(7), state_pos(8));
            // RCLCPP_INFO(get_logger(), "zero measurement: pos state = %f, %f, %f, vel = %f, %f, %f, bias = %f, %f, %f, gain = %f, %f, %f", state_x(0), state_y(0), state_z(0), state_x(1), state_y(1), state_z(1), state_x(2), state_y(2), state_z(2), gain_x[0], gain_y[0], gain_z[0]);


            // update uncertainties
            // uncertainty_pitch   = (Eigen::Matrix2d::Identity() - gain_pitch * H_ori) * uncertainty_pitch;
            // uncertainty_yaw     = (Eigen::Matrix2d::Identity() - gain_yaw   * H_ori) * uncertainty_yaw;
            // uncertainty_roll    = (Eigen::Matrix2d::Identity() - gain_roll  * H_ori) * uncertainty_roll;
            uncertainty_ori    = (Eye6 - gain_ori_lidar  * H_ori) * uncertainty_ori;

            // uncertainty_x = (Eigen::Matrix3d::Identity() - gain_x * H_pos) * uncertainty_x;
            // uncertainty_y = (Eigen::Matrix3d::Identity() - gain_y * H_pos) * uncertainty_y;
            // uncertainty_z = (Eigen::Matrix3d::Identity() - gain_z * H_pos) * uncertainty_z;

            uncertainty_pos = (Eye9 - gain_pos * H_pos) * uncertainty_pos;

            RCLCPP_INFO(get_logger(), "zero measurement: ori uncertainty = %f, %f, %f", uncertainty_ori(0,0), uncertainty_ori(1,1), uncertainty_ori(2,2));
            RCLCPP_INFO(get_logger(), "zero measurement: pos uncertainty = %f, %f, %f", uncertainty_pos(0,0), uncertainty_pos(1,1), uncertainty_pos(2,2));
            // RCLCPP_INFO(get_logger(), "zero measurement: pos uncertainty = %f, %f, %f", uncertainty_x(0,0), uncertainty_y(0,0), uncertainty_z(0,0));

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
        //     Eigen::Vector2d k_gain_vec(0.5, 0.5);
        //     // state_roll  += k_gain_vec * (init_calib_roll  - H_ori * state_roll); 
        //     // state_pitch += k_gain_vec * (init_calib_pitch - H_ori * state_pitch); 
        //     // state_yaw   += k_gain_vec * (init_calib_yaw   - H_ori * state_yaw); 
        //     state_ori   += k_gain_vec * (init_calib_yaw   - H_ori * state_yaw); 



        //     // the orientation found needs to be send to some initial pose in ros or the lidar_odometry, have lidar_odometry subsribe to initial pose, the imu data comes before the lidar

        //     geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg;
        //     // if (use_madgwick_orientation_) {
        //     //     initial_pose_msg.pose.pose.orientation.w =  madgwick_orientation_quaternion.w();
        //     //     initial_pose_msg.pose.pose.orientation.x =  madgwick_orientation_quaternion.x();
        //     //     initial_pose_msg.pose.pose.orientation.y =  madgwick_orientation_quaternion.y();
        //     //     initial_pose_msg.pose.pose.orientation.z =  madgwick_orientation_quaternion.z();
        //     // } else {
        //     //     // updateOrientationQuaternion(init_calib_roll, init_calib_pitch, init_calib_yaw);
        //     //     initial_pose_msg.pose.pose.orientation.w =  orientation_quaternion.w();
        //     //     initial_pose_msg.pose.pose.orientation.x =  orientation_quaternion.x();
        //     //     initial_pose_msg.pose.pose.orientation.y =  orientation_quaternion.y();
        //     //     initial_pose_msg.pose.pose.orientation.z =  orientation_quaternion.z();
        //     // }
        //     initial_pose_msg.pose.pose.position.x = 0.0;
        //     initial_pose_msg.pose.pose.position.y = 0.0;
        //     initial_pose_msg.pose.pose.position.z = 0.0;

        //     initial_pose_pub->publish(initial_pose_msg);

        // }

        void printStates()
        {   
            if (print_states_) {

                // if (init_calib_steps_ > 0) {

                //     RCLCPP_INFO(get_logger(), "Static Calibration: gravity estimate: %f m/s², std: %f m/s², measurement std: %f, update gain: %f", g_acc, sqrt(g_acc_var), sqrt(g_acc_var_measurement), k_gain);
                //     RCLCPP_INFO(get_logger(), "Static Calibration: gravity norm vector: %f %f %f", g_vec[0], g_vec[1], g_vec[2] );
                //     RCLCPP_INFO(get_logger(), "Static Calibration: pitch: %f deg, roll: %f deg", init_calib_pitch *57.3, init_calib_roll*57.3 );
                // }


                

                // RCLCPP_INFO(get_logger(), "update: pos state = %f, %f, %f, vel = %f, %f, %f, bias = %f %f %f", state_pos(0), state_pos(1), state_pos(2), state_pos(3), state_pos(4), state_pos(5), state_pos(6), state_pos(7), state_pos(8));
                RCLCPP_INFO(get_logger(), "update: state pos  = %f, %f, %f", state_pos(0), state_pos(1), state_pos(2));
                RCLCPP_INFO(get_logger(), "update: state vel = %f, %f, %f, ", state_pos(3), state_pos(4), state_pos(5));
                RCLCPP_INFO(get_logger(), "update: state bias = %f %f %f", state_pos(6), state_pos(7), state_pos(8));
                RCLCPP_INFO(get_logger(), "update: state uncertainty pos = %f, %f, %f", sqrt(uncertainty_pos(0,0)), sqrt(uncertainty_pos(1,1)), sqrt(uncertainty_pos(2,2)));
                RCLCPP_INFO(get_logger(), "update: state uncertainty vel = %f, %f, %f", sqrt(uncertainty_pos(3,3)), sqrt(uncertainty_pos(4,4)), sqrt(uncertainty_pos(5,5)));
                RCLCPP_INFO(get_logger(), "update: state uncertainty bias = %f, %f, %f", sqrt(uncertainty_pos(6,6)), sqrt(uncertainty_pos(7,7)), sqrt(uncertainty_pos(8,8)));
                RCLCPP_INFO(get_logger(), "update: state ori state = %f, %f, %f", state_ori(0)*57.3, state_ori(1)*57.3, state_ori(2)*57.3);
                RCLCPP_INFO(get_logger(), "update: state ori bias = %f %f %f",  state_ori(3)*57.3, state_ori(4)*57.3, state_ori(5)*57.3);
                RCLCPP_INFO(get_logger(), "update: state ori uncertainty ang = %f, %f, %f", sqrt(uncertainty_ori(0,0))*57.3, sqrt(uncertainty_ori(1,1))*57.3, sqrt(uncertainty_ori(2,2))*57.3);
                RCLCPP_INFO(get_logger(), "update: state ori uncertainty bias = %f, %f, %f", sqrt(uncertainty_ori(3,3))*57.3, sqrt(uncertainty_ori(4,4))*57.3, sqrt(uncertainty_ori(5,5))*57.3);
            }
        }

        double toSec(builtin_interfaces::msg::Time header_stamp)
        {
            rclcpp::Time time = header_stamp;
            double nanoseconds = time.nanoseconds();

            return nanoseconds * 1e-9;
        }





};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto ekf_node = std::make_shared<EKF_INS>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(ekf_node);
    
    executor.spin();
    rclcpp::shutdown();
    return 0;
}