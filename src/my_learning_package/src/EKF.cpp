
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

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>



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

class EKF : public rclcpp::Node
{
    private:
        rclcpp::CallbackGroup::SharedPtr sub1_cb_group_;
        rclcpp::CallbackGroup::SharedPtr sub2_cb_group_;

        rclcpp::CallbackGroup::SharedPtr run_cb_group_;
        rclcpp::TimerBase::SharedPtr run_timer;

        
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr keyframe_odom_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr lidar_odometry_transformation_sub;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub;
     
      
        // rclcpp::Time time_latest_cloud;



        double imu_dt_; 
        double g_acc; // gravity scalar value
        double g_acc_var;
        double g_acc_sum;
        double g_acc_sumsq;
        Eigen::Vector3d g_vec; // normal vector of gravity


        int step{};
        int step_since_keyframe{};

        // Eigen::MatrixXd Eye9;
        Eigen::Matrix3d Eye3;

        // ORIENTATION

        // state of orientation, x with bias on orientation, [alpha, q_b]
        Eigen::Vector2d state_pitch;  
        Eigen::Vector2d state_yaw; 
        Eigen::Vector2d state_roll; 

        // uncertainty / covariance matrix, P  -> check in docs for xsens imu if these are not equal on all axes
        Eigen::Matrix2d uncertainty_pitch;  
        Eigen::Matrix2d uncertainty_yaw;  
        Eigen::Matrix2d uncertainty_roll; 
        // Eigen::Matrix2d uncertainty_ori; 

        // process noise matrix, Q -> check in docs for xsens imu if these are not equal on all axes
        Eigen::Matrix2d noise_pitch; 
        Eigen::Matrix2d noise_yaw;  
        Eigen::Matrix2d noise_roll; 

        // prediction matrix, A aka F
        Eigen::Matrix2d A_ori;
        Eigen::Matrix2d A_ori_T;
        // bias matrix, B
        Eigen::Vector2d B_ori;

        // measurement matrix H
        Eigen::RowVector2d H_ori;
        Eigen::Vector2d H_ori_T;

        // Kalman gain
        Eigen::Vector2d gain_pitch;
        Eigen::Vector2d gain_yaw;
        Eigen::Vector2d gain_roll;

        // measurement noise, R
        Eigen::VectorXd measurement_noise_pitch;
        Eigen::VectorXd measurement_noise_yaw;
        Eigen::VectorXd measurement_noise_roll;



        // POSITION

        // state of position, [S, v, a, a_b]
        Eigen::Vector4d state_x; 
        Eigen::Vector4d state_y; 
        Eigen::Vector4d state_z; 
        // Eigen::VectorXd state_pos; // should be 3 parts, [S, v, a_b], for each axis , total -> 9 parts

        Eigen::Matrix4d uncertainty_x; 
        Eigen::Matrix4d uncertainty_y; 
        Eigen::Matrix4d uncertainty_z; 
        // Eigen::MatrixXd uncertainty_pos; // 9x9 matrix, also has uncertainty of biases

        Eigen::Matrix4d noise_pos; 
        // Eigen::MatrixXd noise_pos; 
        // prediction matrix, A aka F
        Eigen::Matrix4d A_pos;
        Eigen::Matrix4d A_pos_T;
        // Eigen::MatrixXd A_pos;
        // Eigen::MatrixXd A_pos_T;
        // input matrix, B
        // Eigen::Vector3d B_pos;
        // Eigen::MatrixXd B_pos;
        // measurement matrix H
        Eigen::RowVector4d H_pos;
        Eigen::Vector4d H_pos_T;
        Eigen::RowVector4d H_acc;
        Eigen::Vector4d H_acc_T;
        // Eigen::MatrixXd H_pos;
        // Eigen::MatrixXd H_pos_T;

        Eigen::Vector3d state_pos_post;

        //kalman gain matrix, K

        Eigen::Vector4d gain_x;
        Eigen::Vector4d gain_y;
        Eigen::Vector4d gain_z;
        
        // Eigen::MatrixXd gain_pos;

        Eigen::VectorXd measurement_covariance_x;
        Eigen::VectorXd measurement_covariance_y;
        Eigen::VectorXd measurement_covariance_z;

        Eigen::VectorXd measurement_covariance_x_imu;
        Eigen::VectorXd measurement_covariance_y_imu;
        Eigen::VectorXd measurement_covariance_z_imu;

        // R matrix, the covariance matrix of the measurement
        // Eigen::Matrix3d measurement_covariance_pos;



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
        double measurement_time{};
        double process_time{};

        Eigen::Vector3d acceleration;
        Eigen::Vector3d angular_velocity;


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
        double measurement_noise_imu_; 


        int init_calib_steps_;

        int imu_step_delay_;

        bool print_states_{};
        bool use_madgwick_orientation_{};
        bool relative_mode_{};


    
    
    public:
        EKF() // constructer
        : Node("EKF")
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
            declare_parameter("measurement_noise_imu", 0.000001);
            get_parameter("measurement_noise_imu", measurement_noise_imu_);

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

            declare_parameter("relative_mode", true);
            get_parameter("relative_mode", relative_mode_);

            declare_parameter("print_states", false);
            get_parameter("print_states", print_states_);

            declare_parameter("use_madgwick_orientation", false);
            get_parameter("use_madgwick_orientation", use_madgwick_orientation_);
            

            // run_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            // run_timer = this->create_wall_timer(1ms, std::bind(&EKF::runProcess, this), run_cb_group_);

            sub1_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options1;
            options1.callback_group = sub1_cb_group_;

            sub2_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options2;
            options2.callback_group = sub2_cb_group_;


            imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_, 100, std::bind(&EKF::imuDataHandler, this, _1), options1);
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 100, std::bind(&EKF::odometryHandler, this, _1), options2);
            lidar_odometry_transformation_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/transformation/lidar", 100, std::bind(&EKF::transformationHandler, this, _1), options2);
            // keyframe_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom_keyframe", 100, std::bind(&EKF::keyframeHandler, this, _1), options2);

            odometry_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom_kalman", 100);
            path_pub = this->create_publisher<nav_msgs::msg::Path>("/path_kalman", 100);
            initial_pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initial_pose", 100);
            

            initializeArrays();
        

        }
        ~EKF(){}

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


            // imu_dt_ = 0.01; //TODO: make this a ros parameter, done
            double imu_dt_sq = imu_dt_*imu_dt_;

            // Eye9 = Eigen::Matrix<double,9,9>();
            Eye3 = Eigen::Matrix3d::Identity();
            state_pos_post.fill(0.0);

            // orientation
            state_pitch = Eigen::Vector2d(0.0, 0.0);
            state_yaw   = Eigen::Vector2d(0.0, 0.0);
            state_roll  = Eigen::Vector2d(0.0, 0.0);
            // orientation
            A_ori   << 1.0, -imu_dt_,
                       0.0, 1.0;
            A_ori_T = A_ori.transpose();
            B_ori   = Eigen::Vector2d(imu_dt_, 0.0);
            H_ori   << 1.0, 0.0;
            H_ori_T << 1.0, 0.0;

            double init_uncertainty_ori_var = 0.00001;
            uncertainty_pitch = Eigen::Matrix2d::Identity() * init_uncertainty_ori_var;
            uncertainty_yaw   = Eigen::Matrix2d::Identity() * init_uncertainty_ori_var;
            uncertainty_roll  = Eigen::Matrix2d::Identity() * init_uncertainty_ori_var;

            // double process_noise_ori_var = 0.0001;
            noise_roll  = Eigen::Matrix2d::Identity() * process_noise_roll_  *process_noise_roll_;
            noise_pitch = Eigen::Matrix2d::Identity() * process_noise_pitch_ *process_noise_pitch_;
            noise_yaw   = Eigen::Matrix2d::Identity() * process_noise_yaw_   *process_noise_yaw_;
            noise_roll(0,0)  *= imu_dt_sq;
            noise_pitch(0,0) *= imu_dt_sq;
            noise_yaw(0,0)   *= imu_dt_sq;

            
            // measurement_noise_ori = 0.0001; // radians²
            measurement_noise_roll = Eigen::VectorXd(1);
            measurement_noise_roll << measurement_noise_roll_*measurement_noise_roll_;
            measurement_noise_pitch = Eigen::VectorXd(1);
            measurement_noise_pitch << measurement_noise_pitch_*measurement_noise_pitch_;
            measurement_noise_yaw = Eigen::VectorXd(1);
            measurement_noise_yaw << measurement_noise_yaw_*measurement_noise_yaw_;

            gain_roll   = uncertainty_roll  * H_ori_T * (H_ori * uncertainty_roll   * H_ori_T + measurement_noise_roll).inverse(); // + R in the inverse  but not at foregone at the init
            gain_pitch  = uncertainty_pitch * H_ori_T * (H_ori * uncertainty_pitch  * H_ori_T + measurement_noise_pitch).inverse(); // + R in the inverse  but not at the init
            gain_yaw    = uncertainty_yaw   * H_ori_T * (H_ori * uncertainty_yaw    * H_ori_T + measurement_noise_yaw).inverse(); // + R in the inverse  but not at foregone at the init




            // POSITION
            state_x  = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
            state_y  = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
            state_z  = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
            // state_pos = Eigen::VectorXd(9);
            // state_pos.fill(0.0);
            RCLCPP_INFO(get_logger(),"HERE?");


            // position
            A_pos   << 1.0, imu_dt_, imu_dt_sq/2, -imu_dt_sq/2,
                       0.0, 1.0,     imu_dt_,     -imu_dt_,
                       0.0, 0.0,     1.0,          0.0,
                       0.0, 0.0,     0.0,          1.0;
            // A_pos = Eigen::Matrix<double,9,9>();
            // A_pos   << 1.0, 0.0, 0.0, imu_dt_, 0.0, 0.0, -imu_dt_sq/2.0, 0.0, 0.0,
            //            0.0, 1.0, 0.0, 0.0, imu_dt_, 0.0, 0.0, -imu_dt_sq/2.0, 0.0, 
            //            0.0, 0.0, 1.0, 0.0, 0.0, imu_dt_, 0.0, 0.0, -imu_dt_sq/2.0, 
            //            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -imu_dt_, 0.0, 0.0,
            //            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -imu_dt_, 0.0, 
            //            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -imu_dt_, 
            //            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            //            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 
            //            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;


            // A_pos_T = Eigen::Matrix<double,9,9>();
            A_pos_T = A_pos.transpose();

            // B_pos   = Eigen::Vector3d(imu_dt_sq/2, imu_dt_,  0.0);
            // B_pos   = Eigen::Matrix<double,9,3>();
            // B_pos   << imu_dt_sq/2.0, 0.0, 0.0,
            //            0.0, imu_dt_sq/2.0, 0.0,
            //            0.0, 0.0, imu_dt_sq/2.0,
            //            imu_dt_, 0.0, 0.0,
            //            0.0, imu_dt_, 0.0,
            //            0.0, 0.0, imu_dt_,
            //            0.0, 0.0, 0.0,
            //            0.0, 0.0, 0.0,
            //            0.0, 0.0, 0.0;


            H_pos   << 1.0, 0.0, 0.0, 0.0;
            H_pos_T << 1.0, 0.0, 0.0, 0.0;
            H_acc   << 0.0, 0.0, 1.0, 0.0;
            H_acc_T << 0.0, 0.0, 1.0, 0.0;
            // Eigen::Matrix3d Eye = Eigen::Matrix3d::Identity();
            // H_pos_T = Eigen::Matrix<double,9,3>();
            // H_pos_T   << Eye, Eye*0.0, Eye*0.0;
  
            // H_pos = Eigen::Matrix<double,3,9>();
            // H_pos = H_pos_T.transpose();

            


            double init_uncertainty_pos_std = 0.001;
            uncertainty_x = Eigen::Matrix4d::Identity() * init_uncertainty_pos_std;
            uncertainty_y = Eigen::Matrix4d::Identity() * init_uncertainty_pos_std;
            uncertainty_z = Eigen::Matrix4d::Identity() * init_uncertainty_pos_std;
            // uncertainty_pos = Eigen::Matrix<double,9,9>();
            // uncertainty_pos << Eigen::MatrixXd::Identity(9, 9) * init_uncertainty_pos_std *init_uncertainty_pos_std;



            // double process_noise_pos_var = 0.1;
            Eigen::Vector4d noise_pos_vec(4);
            noise_pos_vec << imu_dt_sq/2.0, imu_dt_,  1.0, 0.0;
            // Eigen::VectorXd noise_pos_vec(9);
            // noise_pos_vec << imu_dt_sq/2.0, imu_dt_sq/2.0, imu_dt_sq/2.0, imu_dt_, imu_dt_, imu_dt_, 0.0, 0.0, 0.0 ;
            // noise_pos = Eigen::Matrix<double,9,9>();
            noise_pos = noise_pos_vec * noise_pos_vec.transpose() * process_noise_pos_*process_noise_pos_;
            // noise_pos << imu_dt_sq*imu_dt_sq/4.0 * process_noise_pos_*process_noise_pos_, imu_dt_sq*imu_dt_/2 * process_noise_pos_*process_noise_pos_, 0.0,
            //              imu_dt_sq*imu_dt_/2 * process_noise_pos_*process_noise_pos_, imu_dt_sq * process_noise_pos_*process_noise_pos_, 0.0,
            //              0.0, 0.0, 0.0;
            RCLCPP_INFO(get_logger(),"pos process noise %f", noise_pos(0,0)*1e10);
            RCLCPP_INFO(get_logger(),"prev pos process noise %f", imu_dt_sq*imu_dt_sq/4.0 * process_noise_pos_*process_noise_pos_*1e10);



            // measurement_noise_pos = 0.00001; // meters²
            measurement_covariance_x = Eigen::VectorXd(1);
            measurement_covariance_y = Eigen::VectorXd(1);
            measurement_covariance_z = Eigen::VectorXd(1);

            measurement_covariance_x_imu = Eigen::VectorXd(1);
            measurement_covariance_y_imu = Eigen::VectorXd(1);
            measurement_covariance_z_imu = Eigen::VectorXd(1);

            measurement_covariance_x << measurement_noise_pos_*measurement_noise_pos_;
            measurement_covariance_y << measurement_noise_pos_*measurement_noise_pos_;
            measurement_covariance_z << measurement_noise_pos_*measurement_noise_pos_;

            measurement_covariance_x_imu << measurement_noise_imu_*measurement_noise_imu_;
            measurement_covariance_y_imu << measurement_noise_imu_*measurement_noise_imu_;
            measurement_covariance_z_imu << measurement_noise_imu_*measurement_noise_imu_;

            // initial/static covariance
            // measurement_covariance_pos = Eigen::Matrix3d::Identity() * measurement_noise_pos_* measurement_noise_pos_;

            gain_x = uncertainty_x * H_pos_T * (H_pos * uncertainty_x * H_pos_T + measurement_covariance_x).inverse(); 
            gain_y = uncertainty_y * H_pos_T * (H_pos * uncertainty_y * H_pos_T + measurement_covariance_y).inverse(); 
            gain_z = uncertainty_z * H_pos_T * (H_pos * uncertainty_z * H_pos_T + measurement_covariance_z).inverse(); 
            // gain_pos = Eigen::Matrix<double,9,3>();
            // gain_pos = uncertainty_pos * H_pos_T * (H_pos * uncertainty_pos * H_pos_T + measurement_covariance_pos).inverse(); 
            
        
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
            // measurement_covariance_pos = Eigen::Matrix3d::Identity() * new_error_std*new_error_std;
            measurement_covariance_x << new_error_std*new_error_std;
            measurement_covariance_y << new_error_std*new_error_std;
            measurement_covariance_z << new_error_std*new_error_std;
        }

        void updateMeasurementCovariance(double new_error_std_x, double new_error_std_z, double new_error_std_y)
        {
            measurement_covariance_x << new_error_std_x*new_error_std_y;
            measurement_covariance_y << new_error_std_y*new_error_std_y;
            measurement_covariance_z << new_error_std_z*new_error_std_z;
        }

        void updateMeasurementCovariance(Eigen::Vector3d cov_diagonal)
        {
            measurement_covariance_x << cov_diagonal[0] * cov_diagonal[0];
            measurement_covariance_y << cov_diagonal[1] * cov_diagonal[1];
            measurement_covariance_z << cov_diagonal[2] * cov_diagonal[2];
        }

        void updateOrientationQuaternionFromState()
        {
            // orientation_quaternion.setEuler(state_yaw[0], state_pitch[0], state_roll[0]);
            orientation_quaternion.setRPY(state_roll[0], state_pitch[0], state_yaw[0]); // this seems better, but still don't seem right
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

        void orientationQuaternionToEuler(tf2::Quaternion q) 
        {          
            tf2::Matrix3x3 orientation_matrix(q);

            // orientation_matrix.getEulerYPR(yaw, pitch, roll);
            orientation_matrix.getRPY(roll, pitch, yaw);
            limitEulerAngles();
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

        void limitStateEulerAngles()
        {
            while (state_roll[0] > M_PI_2) // limited to +-pi/2 becuase it cannot be upside down
                state_roll[0] -= M_PI_2;
            while (state_roll[0] < -M_PI_2)
                state_roll[0] += M_PI_2;

            while (state_pitch[0] > M_PI_2) // limited to +-pi/2 becuase it cannot be upside down
                state_pitch[0] -= M_PI_2;
            while (state_pitch[0] < -M_PI_2)
                state_pitch[0] += M_PI_2;

            while (state_yaw[0] > M_PI)
                state_yaw[0] -= 2.0*M_PI;
            while (state_yaw[0] < -M_PI)
                state_yaw[0] += 2.0*M_PI; 
        }

        void imuDataHandler(const sensor_msgs::msg::Imu::SharedPtr imu_data)
        {
            
            // put data into buffer back
            RCLCPP_INFO_ONCE(get_logger(),"First IMU message recieved..");
            imu_buffer.push_back(imu_data);

            
             
        }

        void runProcess(double cutoff_time)
        {   
            while (process_time <= cutoff_time && !imu_buffer.empty()){
                RCLCPP_INFO(get_logger(), "updating process to measurement.. cutoff: %f, current process time: %f",cutoff_time, process_time);
                
                step++;
                // if (step < imu_step_delay_){ // if step is less than imu_step_delay return
                //     return;
                // }

                // get data from buffer front
                sensor_msgs::msg::Imu::SharedPtr delayed_imu_data = imu_buffer.front();
                imu_buffer.pop_front();

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

                processUpdate(); // prediction step
                process_time = toSec(imu_buffer.front()->header.stamp); // get time stamp of next to compare with cutoff.
                
                measurementUpdateAcceleration(acceleration);
                
                // RCLCPP_INFO(get_logger(),"Where's the poop?");
            }
        }


        void processUpdate()
        {
            if (step < init_calib_steps_){
                updateGravityCalibration(acceleration);
            }

            // update orientaion state
            state_roll  = A_ori * state_roll   +  B_ori * angular_velocity[0];
            state_pitch = A_ori * state_pitch  +  B_ori * angular_velocity[1];
            state_yaw   = A_ori * state_yaw    +  B_ori * angular_velocity[2];
            limitStateEulerAngles(); // limits angles within ranges of +-pi

            // convert acceleration input to world frame by rotating by the imu orientation
            // updateOrientationQuaternion();
            updateOrientationQuaternionFromState();
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

            state_x = A_pos * state_x;
            state_y = A_pos * state_y;
            state_z = A_pos * state_z;
            // state_pos = A_pos * state_pos + B_pos * acceleration;

            // RCLCPP_INFO(get_logger(), "incoming data = %f", imu_data->angular_velocity.x);
            // RCLCPP_INFO(get_logger(), "state_pitch = %f, pitch_bias %f", state_pitch[0], state_pitch[1]);


            // update uncertainty and add process noise
            uncertainty_roll    = A_ori * uncertainty_roll  * A_ori_T + noise_roll;
            uncertainty_pitch   = A_ori * uncertainty_pitch * A_ori_T + noise_pitch;
            uncertainty_yaw     = A_ori * uncertainty_yaw   * A_ori_T + noise_yaw;

            uncertainty_x = A_pos * uncertainty_x * A_pos_T + noise_pos;
            uncertainty_y = A_pos * uncertainty_y * A_pos_T + noise_pos;
            uncertainty_z = A_pos * uncertainty_z * A_pos_T + noise_pos;
            // uncertainty_pos = A_pos * uncertainty_pos * A_pos_T + noise_pos;


            // RCLCPP_INFO(get_logger(), "update: pos state = %f, %f, %f, vel = %f, %f, %f, bias = %f %f %f", state_x[0], state_y[0], state_z[0], state_x[1], state_y[1], state_z[1], state_x[2], state_y[2], state_z[2]);
            // RCLCPP_INFO(get_logger(), "update: pos uncertainty = %f, %f, %f", uncertainty_x(0,0), uncertainty_y(0,0), uncertainty_z(0,0));
            // RCLCPP_INFO(get_logger(), "update: ori state = %f, %f, %f, bias = %f %f %f", state_roll[0], state_pitch[0], state_yaw[0], state_roll[1], state_pitch[1], state_yaw[1]);
            // RCLCPP_INFO(get_logger(), "update: ori uncertainty = %f, %f, %f", uncertainty_roll(0,0), uncertainty_pitch(0,0), uncertainty_yaw(0,0));
            
            // RCLCPP_INFO(get_logger(), "uncertaint_pitch = %f", uncertainty_pitch(0,0));

            updateOrientationQuaternionFromState();

            latestPoseInfo.qw = orientation_quaternion.getW();
            latestPoseInfo.qx = orientation_quaternion.getX();
            latestPoseInfo.qy = orientation_quaternion.getY();
            latestPoseInfo.qz = orientation_quaternion.getZ();
            latestPoseInfo.x = state_x[0]; 
            latestPoseInfo.y = state_y[0];
            latestPoseInfo.z = state_z[0];

            
            publishOdometry();
            printStates();
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
            // TODO get timestamp and only use lidar measurements up to this timestamp? DONE
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

            Eigen::Vector3d lidar_position(odom_message->pose.pose.position.x,
                                           odom_message->pose.pose.position.y,
                                           odom_message->pose.pose.position.z);

            tf2::Quaternion lidar_orientation(odom_message->pose.pose.orientation.x, 
                                              odom_message->pose.pose.orientation.y,
                                              odom_message->pose.pose.orientation.z,
                                              odom_message->pose.pose.orientation.w);

            // measurement_covariance_pos = getCovariancePos(odom_message->pose.covariance);
            Eigen::Vector3d cov_diagonal = getCovarianceDiagonalPos(odom_message->pose.covariance);
            updateMeasurementCovariance(cov_diagonal);

            // for (int i=0 ; i < 3; i++){
            //     RCLCPP_INFO(get_logger(),"cov[%i]= %f", i, cov_diagonal[i]);
            // }


            // ORIENTATION
            // measurementUpdateOrientation(lidar_orientation);
            if (use_madgwick_orientation_){
                measurementUpdateOrientation(madgwick_orientation_quaternion);
            }

            // // POSITION
            measurementUpdatePosition(lidar_position);

            updateOrientationQuaternionFromState();


            RCLCPP_INFO(get_logger(), "Lidar measurement: pos = %f, %f %f", lidar_position[0], lidar_position[1], lidar_position[2]);
            // RCLCPP_INFO(get_logger(), "Lidar measurement: rpy = %f, %f, %f", roll*57.3, pitch*57.3, yaw*57.3);
            
            state_pos_post = Eigen::Vector3d( state_x[0], state_y[0], state_z[0]);

            // latestPoseInfo.time = odom_message->header.stamp;
            // latestPoseInfo.qw = odom_message->pose.pose.orientation.w;
            // latestPoseInfo.qx = odom_message->pose.pose.orientation.x;
            // latestPoseInfo.qy = odom_message->pose.pose.orientation.y;
            // latestPoseInfo.qz = odom_message->pose.pose.orientation.z;
            latestPoseInfo.qw = orientation_quaternion.getW();
            latestPoseInfo.qx = orientation_quaternion.getX();
            latestPoseInfo.qy = orientation_quaternion.getY();
            latestPoseInfo.qz = orientation_quaternion.getZ();
            latestPoseInfo.x = state_x[0]; 
            latestPoseInfo.y = state_y[0];
            latestPoseInfo.z = state_z[0];


            step_since_keyframe++;
            // updateMeasurementNoise(measurement_noise_pos_* pow(1.1 , step_since_keyframe) );

            RCLCPP_INFO(get_logger(), "Absolute is running");
            // publishOdometry();

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

                measurementUpdateOrientation(madgwick_orientation_quaternion);
            }
            updateOrientationQuaternionFromState();

            // // POSITION
            // rotate translation
            
            // SHOULD BE ROTATED  BY THE LIDAR ORIENTATION AND NOT THE STATE?
            translation = tf2Eigen(orientation_quaternion) * translation;
            measurementUpdateTranslation(translation);



            // RCLCPP_INFO(get_logger(), "Lidar measurement: pos = %f, %f %f", lidar_position[0], lidar_position[1], lidar_position[2]);
            // RCLCPP_INFO(get_logger(), "Lidar measurement: rpy = %f, %f, %f", roll*57.3, pitch*57.3, yaw*57.3);
            
            state_pos_post = Eigen::Vector3d( state_x[0], state_y[0], state_z[0]);

            // latestPoseInfo.time = odom_message->header.stamp;
            // latestPoseInfo.qw = odom_message->pose.pose.orientation.w;
            // latestPoseInfo.qx = odom_message->pose.pose.orientation.x;
            // latestPoseInfo.qy = odom_message->pose.pose.orientation.y;
            // latestPoseInfo.qz = odom_message->pose.pose.orientation.z;
            latestPoseInfo.qw = orientation_quaternion.getW();
            latestPoseInfo.qx = orientation_quaternion.getX();
            latestPoseInfo.qy = orientation_quaternion.getY();
            latestPoseInfo.qz = orientation_quaternion.getZ();
            latestPoseInfo.x = state_x[0]; 
            latestPoseInfo.y = state_y[0];
            latestPoseInfo.z = state_z[0];


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


        void measurementUpdatePosition(Eigen::Vector3d lidar_position)
        {  
            // POSITION
            // update kalman gain
            gain_x = uncertainty_x * H_pos_T * (H_pos * uncertainty_x * H_pos_T + measurement_covariance_x).inverse(); 
            gain_y = uncertainty_y * H_pos_T * (H_pos * uncertainty_y * H_pos_T + measurement_covariance_y).inverse(); 
            gain_z = uncertainty_z * H_pos_T * (H_pos * uncertainty_z * H_pos_T + measurement_covariance_z).inverse(); 
            // gain_pos = uncertainty_pos * H_pos_T * (H_pos * uncertainty_pos * H_pos_T + measurement_covariance_pos ).inverse(); 

            // update state with kalman gain
            state_x += gain_x * (lidar_position[0] - H_pos * state_x);
            state_y += gain_y * (lidar_position[1] - H_pos * state_y);
            state_z += gain_z * (lidar_position[2] - H_pos * state_z);
            // state_pos += gain_pos * (lidar_position - H_pos * state_pos);
          
            // recalculate uncertainty after measurement
            uncertainty_x = (Eigen::Matrix4d::Identity() - gain_x * H_pos) * uncertainty_x;
            uncertainty_y = (Eigen::Matrix4d::Identity() - gain_y * H_pos) * uncertainty_y;
            uncertainty_z = (Eigen::Matrix4d::Identity() - gain_z * H_pos) * uncertainty_z;

            // uncertainty_pos = (Eye9 - gain_pos * H_pos) * uncertainty_pos; // this is unstable apparently
            // uncertainty_pos = (Eye9 - gain_pos * H_pos) * uncertainty_pos * (Eye9 - gain_pos * H_pos).transpose() + gain_pos * measurement_covariance_pos * gain_pos.transpose(); 
        }

        void measurementUpdateAcceleration(Eigen::Vector3d imu_acceleration)
        {  
            // POSITION
            // update kalman gain
            gain_x = uncertainty_x * H_acc_T * (H_acc * uncertainty_x * H_acc_T + measurement_covariance_x_imu).inverse(); 
            gain_y = uncertainty_y * H_acc_T * (H_acc * uncertainty_y * H_acc_T + measurement_covariance_y_imu).inverse(); 
            gain_z = uncertainty_z * H_acc_T * (H_acc * uncertainty_z * H_acc_T + measurement_covariance_z_imu).inverse(); 
            // gain_pos = uncertainty_pos * H_pos_T * (H_pos * uncertainty_pos * H_pos_T + measurement_covariance_pos ).inverse(); 

            // update state with kalman gain
            state_x += gain_x * (imu_acceleration[0] - H_acc * state_x);
            state_y += gain_y * (imu_acceleration[1] - H_acc * state_y);
            state_z += gain_z * (imu_acceleration[2] - H_acc * state_z);
            // state_pos += gain_pos * (lidar_position - H_pos * state_pos);
          
            // recalculate uncertainty after measurement
            uncertainty_x = (Eigen::Matrix4d::Identity() - gain_x * H_acc) * uncertainty_x;
            uncertainty_y = (Eigen::Matrix4d::Identity() - gain_y * H_acc) * uncertainty_y;
            uncertainty_z = (Eigen::Matrix4d::Identity() - gain_z * H_acc) * uncertainty_z;

            // uncertainty_pos = (Eye9 - gain_pos * H_pos) * uncertainty_pos; // this is unstable apparently
            // uncertainty_pos = (Eye9 - gain_pos * H_pos) * uncertainty_pos * (Eye9 - gain_pos * H_pos).transpose() + gain_pos * measurement_covariance_pos * gain_pos.transpose(); 
        }

        void measurementUpdateTranslation(Eigen::Vector3d translation)
        {  
            // POSITION
            Eigen::Vector3d new_position = state_pos_post + translation;

            // update kalman gain
            gain_x = uncertainty_x * H_pos_T * (H_pos * uncertainty_x * H_pos_T + measurement_covariance_x).inverse(); 
            gain_y = uncertainty_y * H_pos_T * (H_pos * uncertainty_y * H_pos_T + measurement_covariance_y).inverse(); 
            gain_z = uncertainty_z * H_pos_T * (H_pos * uncertainty_z * H_pos_T + measurement_covariance_z).inverse(); 
            // gain_pos = uncertainty_pos * H_pos_T * (H_pos * uncertainty_pos * H_pos_T + measurement_covariance_pos ).inverse(); 

            // update state with kalman gain
            state_x += gain_x * (new_position[0] - H_pos * state_x);
            state_y += gain_y * (new_position[1] - H_pos * state_y);
            state_z += gain_z * (new_position[2] - H_pos * state_z);
            // state_pos += gain_pos * (lidar_position - H_pos * state_pos);
          
            // recalculate uncertainty after measurement
            uncertainty_x = (Eigen::Matrix4d::Identity() - gain_x * H_pos) * uncertainty_x;
            uncertainty_y = (Eigen::Matrix4d::Identity() - gain_y * H_pos) * uncertainty_y;
            uncertainty_z = (Eigen::Matrix4d::Identity() - gain_z * H_pos) * uncertainty_z;

            // uncertainty_pos = (Eye9 - gain_pos * H_pos) * uncertainty_pos; // this is unstable apparently
            // uncertainty_pos = (Eye9 - gain_pos * H_pos) * uncertainty_pos * (Eye9 - gain_pos * H_pos).transpose() + gain_pos * measurement_covariance_pos * gain_pos.transpose(); 
        }

        void measurementUpdateOrientation(tf2::Quaternion orientation)
        {
            orientationQuaternionToEuler(orientation); // updates values roll, pitch, and yaw from the input quaternion
            limitEulerAngles();
            // ORIENTATION
            // update kalman gain
            gain_roll   = uncertainty_roll  * H_ori_T * (H_ori * uncertainty_roll  * H_ori_T + measurement_noise_roll ).inverse(); 
            gain_pitch  = uncertainty_pitch * H_ori_T * (H_ori * uncertainty_pitch * H_ori_T + measurement_noise_pitch).inverse(); 
            gain_yaw    = uncertainty_yaw   * H_ori_T * (H_ori * uncertainty_yaw   * H_ori_T + measurement_noise_yaw  ).inverse(); 

            double yaw_innovation = yaw  - H_ori * state_yaw;
            
            RCLCPP_INFO(get_logger(), "measured yaw: %f", yaw*57.3 );
            RCLCPP_INFO(get_logger(), "current yaw: %f", state_yaw[0]*57.3 );
            RCLCPP_INFO(get_logger(), "yaw innovation before: %f", yaw_innovation*57.3 );
            if (yaw_innovation >= M_PI){
                yaw_innovation -= M_PI;
                // yaw_innovation *= -1.0;
            } else if (yaw_innovation <= - M_PI){
                yaw_innovation += M_PI;
                // yaw_innovation *= -1.0;
            }
            RCLCPP_INFO(get_logger(), "yaw innovation after: %f", yaw_innovation*57.3 );
            // only yaw is expected to have crossover error else the object is upside down..

            // update state with kalman gain
            state_roll  += gain_roll  * (roll  - H_ori * state_roll);
            state_pitch += gain_pitch * (pitch - H_ori * state_pitch); 
            state_yaw   += gain_yaw   * yaw_innovation; // <-- YAW CROSSOVER ERROR HERE ! see note at top of function

            // recalculate uncertainty after measurement
            uncertainty_roll  = (Eigen::Matrix2d::Identity() - gain_roll  * H_ori) * uncertainty_roll ;
            uncertainty_pitch = (Eigen::Matrix2d::Identity() - gain_pitch * H_ori) * uncertainty_pitch;
            uncertainty_yaw   = (Eigen::Matrix2d::Identity() - gain_yaw   * H_ori) * uncertainty_yaw  ;
        }


        // void keyframeHandler(const nav_msgs::msg::Odometry::SharedPtr odom_message)
        // {   
        //     // reset the position noise when a keyframe arrives
        //     updateMeasurementNoise(measurement_noise_pos_);
        //     step_since_keyframe = 1;
        // }

        void publishOdometry()
        {
            // PoseInfo latestPoseInfo = odometry_pose_info->points[latest_keyframe_idx - 1];
            nav_msgs::msg::Odometry odom;
            // odom.header.stamp = cloud_header.stamp;
            odom.header.frame_id = "odom";
            odom.pose.pose.orientation.w = latestPoseInfo.qw;
            odom.pose.pose.orientation.x = latestPoseInfo.qx;
            odom.pose.pose.orientation.y = latestPoseInfo.qy;
            odom.pose.pose.orientation.z = latestPoseInfo.qz;
            odom.pose.pose.position.x = latestPoseInfo.x;
            odom.pose.pose.position.y = latestPoseInfo.y;
            odom.pose.pose.position.z = latestPoseInfo.z;
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

        

    
        void updateZeroMeasurement() // fake update, giving a zero value measurement to estimate bias. This is run when the rover is assumend to be stationary in the inital part of the recordings
        {

            RCLCPP_INFO(get_logger(), "Zero-update!");

            // kalman update step, becuase uncertainty has increased during process of IMU inputs
            gain_pitch  = uncertainty_pitch * H_ori_T * (H_ori * uncertainty_pitch  * H_ori_T + measurement_noise_pitch).inverse(); 
            gain_yaw    = uncertainty_yaw   * H_ori_T * (H_ori * uncertainty_yaw    * H_ori_T + measurement_noise_yaw).inverse(); 
            gain_roll   = uncertainty_roll  * H_ori_T * (H_ori * uncertainty_roll   * H_ori_T + measurement_noise_roll).inverse(); 

            gain_x = uncertainty_x * H_pos_T * (H_pos * uncertainty_x * H_pos_T + measurement_covariance_x).inverse(); 
            gain_y = uncertainty_y * H_pos_T * (H_pos * uncertainty_y * H_pos_T + measurement_covariance_y).inverse(); 
            gain_z = uncertainty_z * H_pos_T * (H_pos * uncertainty_z * H_pos_T + measurement_covariance_z).inverse();

            // gain_pos = uncertainty_pos * H_pos_T * (H_pos * uncertainty_pos * H_pos_T + measurement_covariance_pos).inverse(); 


            // update state from measurement in this case it is an assumed static postion and orientation
            state_pitch +=  gain_pitch  * (init_calib_pitch - H_ori * state_pitch);
            state_yaw   +=  gain_yaw    * (init_calib_yaw - H_ori * state_yaw);
            state_roll  +=  gain_roll   * (init_calib_roll - H_ori * state_roll);

            state_x += gain_x * (0.0 - H_pos * state_x);
            state_y += gain_y * (0.0 - H_pos * state_y);
            state_z += gain_z * (0.0 - H_pos * state_z);
            // Eigen::Vector3d zero_vector(0.0, 0.0, 0.0);
            // state_pos += gain_pos * (zero_vector - H_pos * state_pos);
          

            RCLCPP_INFO(get_logger(), "zero measurement: ori state = %f, %f, %f, bias = %f, %f, %f, gain = %f, %f, %f", state_pitch[0], state_yaw[0], state_roll[0], state_pitch[1], state_yaw[1], state_roll[1], gain_pitch[0], gain_yaw[0], gain_roll[0]);
            // RCLCPP_INFO(get_logger(), "zero measurement: pos state = %f, %f, %f, vel = %f, %f, %f, bias = %f, %f, %f, gain = %f, %f, %f", state_pos[0], state_pos[1], state_pos[2], state_pos[3], state_pos[4], state_pos[5], state_pos[6], state_pos[7], state_pos[8]);
            RCLCPP_INFO(get_logger(), "zero measurement: pos state = %f, %f, %f, vel = %f, %f, %f, bias = %f, %f, %f, gain = %f, %f, %f", state_x[0], state_y[0], state_z[0], state_x[1], state_y[1], state_z[1], state_x[2], state_y[2], state_z[2], gain_x[0], gain_y[0], gain_z[0]);


            // update uncertainties
            uncertainty_pitch   = (Eigen::Matrix2d::Identity() - gain_pitch * H_ori) * uncertainty_pitch;
            uncertainty_yaw     = (Eigen::Matrix2d::Identity() - gain_yaw   * H_ori) * uncertainty_yaw;
            uncertainty_roll    = (Eigen::Matrix2d::Identity() - gain_roll  * H_ori) * uncertainty_roll;

            uncertainty_x = (Eigen::Matrix4d::Identity() - gain_x * H_pos) * uncertainty_x;
            uncertainty_y = (Eigen::Matrix4d::Identity() - gain_y * H_pos) * uncertainty_y;
            uncertainty_z = (Eigen::Matrix4d::Identity() - gain_z * H_pos) * uncertainty_z;

            // uncertainty_pos = (Eye9 - gain_pos * H_pos) * uncertainty_pos;

            RCLCPP_INFO(get_logger(), "zero measurement: ori uncertainty = %f, %f, %f", uncertainty_pitch(0,0), uncertainty_yaw(0,0), uncertainty_roll(0,0));
            // RCLCPP_INFO(get_logger(), "zero measurement: pos uncertainty = %f, %f, %f", uncertainty_pos(0,0), uncertainty_pos(1,1), uncertainty_pos(2,2));
            RCLCPP_INFO(get_logger(), "zero measurement: pos uncertainty = %f, %f, %f", uncertainty_x(0,0), uncertainty_y(0,0), uncertainty_z(0,0));

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

            // kalman gain
            k_gain = g_acc_var / (g_acc_var + g_acc_var_measurement);

            // update uncertainty of estimate
            g_acc_var = (1 - k_gain) * g_acc_var;

            // update estimate with mean 
            g_acc = g_acc + k_gain * (g_acc_mean - g_acc);


            g_vec = (1 - k_gain) *  new_g_vec;


            // find pitch and roll from gravity vector
            double calib_roll = std::atan2(g_vec[1], g_vec[2]);
            double z_prime = g_vec[1] * std::sin(-roll) + g_vec[2] * std::cos(-roll);
            double calib_pitch = std::atan2(-g_vec[0], z_prime);
            double calib_yaw = 0.0; // yaw is assumed zero as this is equal to the inital heading.

            if (step < 10){
                init_calib_roll  = calib_roll;
                init_calib_pitch = calib_pitch;
                init_calib_yaw   = calib_yaw;
            } else {
                init_calib_roll  +=  k_gain * (calib_roll  - init_calib_roll);
                init_calib_pitch +=  k_gain * (calib_pitch - init_calib_pitch);
                init_calib_yaw   +=  k_gain * (calib_yaw   - init_calib_yaw);
            }

            // update states and saturates biases
            Eigen::Vector2d k_gain_vec(0.5, 0.5);
            state_roll  += k_gain_vec * (init_calib_roll  - H_ori * state_roll); 
            state_pitch += k_gain_vec * (init_calib_pitch - H_ori * state_pitch); 
            state_yaw   += k_gain_vec * (init_calib_yaw   - H_ori * state_yaw); 



            // the orientation found needs to be send to some initial pose in ros or the lidar_odometry, have lidar_odometry subsribe to initial pose, the imu data comes before the lidar

            geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg;
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
            initial_pose_msg.pose.pose.position.x = 0.0;
            initial_pose_msg.pose.pose.position.y = 0.0;
            initial_pose_msg.pose.pose.position.z = 0.0;

            initial_pose_pub->publish(initial_pose_msg);

        }

        void printStates()
        {   
            if (print_states_) {

                if (init_calib_steps_ > 0) {

                    RCLCPP_INFO(get_logger(), "Static Calibration: gravity estimate: %f m/s², std: %f m/s², measurement std: %f, update gain: %f", g_acc, sqrt(g_acc_var), sqrt(g_acc_var_measurement), k_gain);
                    RCLCPP_INFO(get_logger(), "Static Calibration: gravity norm vector: %f %f %f", g_vec[0], g_vec[1], g_vec[2] );
                    RCLCPP_INFO(get_logger(), "Static Calibration: pitch: %f deg, roll: %f deg", init_calib_pitch *57.3, init_calib_roll*57.3 );
                }


                

                // RCLCPP_INFO(get_logger(), "update: pos state = %f, %f, %f, vel = %f, %f, %f, bias = %f %f %f", state_pos[0], state_pos[1], state_pos[2], state_pos[3], state_pos[4], state_pos[5], state_pos[6], state_pos[7], state_pos[8]);
                RCLCPP_INFO(get_logger(), "update: pos state = %f, %f, %f", state_x[0], state_y[0], state_z[0]);
                RCLCPP_INFO(get_logger(), "update: pos vel = %f, %f, %f, ", state_x[1], state_y[1], state_z[1]);
                RCLCPP_INFO(get_logger(), "update: pos bias = %f %f %f", state_x[2], state_y[2], state_z[2]);
                RCLCPP_INFO(get_logger(), "update: pos uncertainty = %f, %f, %f", sqrt(uncertainty_x(0,0)), sqrt(uncertainty_y(0,0)), sqrt(uncertainty_z(0,0)));
                RCLCPP_INFO(get_logger(), "update: ori state = %f, %f, %f", state_roll[0]*57.3, state_pitch[0]*57.3, state_yaw[0]*57.3);
                RCLCPP_INFO(get_logger(), "update: ori bias = %f %f %f",  state_roll[1]*57.3, state_pitch[1]*57.3, state_yaw[1]*57.3);
                RCLCPP_INFO(get_logger(), "update: ori uncertainty = %f, %f, %f", sqrt(uncertainty_roll(0,0))*57.3, sqrt(uncertainty_pitch(0,0))*57.3, sqrt(uncertainty_yaw(0,0))*57.3);
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
    auto ekf_node = std::make_shared<EKF>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(ekf_node);
    
    executor.spin();
    rclcpp::shutdown();
    return 0;
}