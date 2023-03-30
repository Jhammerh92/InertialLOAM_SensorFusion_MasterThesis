// #include <memory>
// #include <chrono>


#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"
// #include "utils/common.h"
// #include "common.h"

// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/msg/int64.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
// #include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
// #include "geometry_msgs/msg/transform_stamped.hpp"

// #include "example_interfaces/srv/add_two_ints.hpp"
// #include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>


#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/ndt.h>
// #include <fast_pcl/registration/ndt.h>
// #include <fast_pcl/registration/ndt.h>

// #include <pclomp/gicp_omp.h>
// #include <pclomp/gicp_omp.h>

#include <cmath>
// #include <ctime>
// #include <array>
// #include <string>
#include <vector>
// #include <algorithm>
#include <iostream>
#include <fstream>
// #include <thread>
// #include <mutex>
#include <queue>
// #include <assert.h>

#define _USE_MATH_DEFINES

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
typedef pcl::PointXYZINormal PointType; // need to calculate and assign normals
typedef sensor_msgs::msg::PointCloud2 PC_msg;


struct Pose
{
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond orientation;
    pcl::PointCloud<PointType>::Ptr pointcloud;
    int idx;
    int frame_idx;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct Transformation
{
    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
    int idx;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;



using namespace std;

using std::placeholders::_1;


// keyframe class like synthesis?


class LidarOdometry : public rclcpp::Node 
{
    private:
        //callback groups
        rclcpp::CallbackGroup::SharedPtr subscriber_cb_group_;
        rclcpp::CallbackGroup::SharedPtr run_cb_group_;

        rclcpp::TimerBase::SharedPtr run_timer;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_data_service;

        // pose representation: [quaternion: w, x, y, z | transition: x, y, z]
        // double abs_pose[7];   //absolute pose from current frame to the first frame / odometry
        // double rel_pose[7];   //relative pose between two frames

        
        bool system_initialized = false;
        bool new_cloud_ready = false;
        bool init_map_built = false;

        size_t latest_frame_idx;
        size_t latest_keyframe_idx;

        double icp_fitness = 0.0;
        deque<double> fitnesses;

        double scan_dt{};
        double current_scan_time{};

        double cloud_scale;
        double prev_cloud_scale;

        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg;

        Eigen::Matrix4d initial_pose_matrix;
        Pose initial_pose;

        // transformation matrices
        Transformation registration_transformation;
        Transformation last_odometry_transformation;
        Pose last_odometry_pose;

        Pose ins_pose_new;
        Pose ins_pose;
        Transformation ins_relative;

        // Eigen::Matrix4d registration_transformation;
        // Eigen::Matrix4d last_odometry_transformation;
        // Eigen::Matrix4d last_odometry_pose;
        Eigen::Matrix4d odometry_transformation_guess;

        Eigen::Matrix4f init_guess;


        // deque<Eigen::Matrix4d> keyframe_poses; // keyframes class instead?
        Pose keyframe_pose;
        deque<Pose> keyframe_poses; // keyframes class instead?
        // deque<size_t> keyframe_index; // keyframes class instead?



        // headers and header information
        rclcpp::Time time_new_cloud;
        std_msgs::msg::Header cloud_header;

        nav_msgs::msg::Odometry odom;
        nav_msgs::msg::Path path;

        geometry_msgs::msg::PoseWithCovarianceStamped transformation_geommsg;

        // pcl filters downsampling
        pcl::VoxelGrid<PointType> down_size_filter;
        pcl::VoxelGrid<PointType> down_size_filter_local_map;

        //point clouds and vector containing pointclouds
        pcl::PointCloud<PointType>::Ptr cloud_in = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr cloud_in_ds = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr cloud_prev = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr cloud_prev_ds = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr cloud_keyframe = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr cloud_keyframe_ds = boost::make_shared<pcl::PointCloud<PointType>>();

        std::deque<sensor_msgs::msg::PointCloud2> cloud_queue;
        sensor_msgs::msg::PointCloud2 current_cloud_msg;

        pcl::PointCloud<PointType>::Ptr local_map = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr inverse_local_map = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr local_map_ds = boost::make_shared<pcl::PointCloud<PointType>>();

        deque<pcl::PointCloud<PointType>::Ptr> recent_frames;


        pcl::PointCloud<PointType>::Ptr global_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
        vector<boost::shared_ptr<pcl::PointCloud<PointType>>> all_clouds;

        // "point cloud" containing a specific type that has the odometry information
        pcl::PointCloud<PoseInfo>::Ptr odometry_pose_info = boost::make_shared<pcl::PointCloud<PoseInfo>>(); // add keyframe pose info?
        pcl::PointCloud<PoseInfo>::Ptr keyframe_pose_info = boost::make_shared<pcl::PointCloud<PoseInfo>>(); // add keyframe pose info?
        // pcl::PointCloud<pcl::PointXYZI>::Ptr odometry_pose_positions = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

        // subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ins_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

        //publishers
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloudguess_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_cloud_pub;
        // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalcloud_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr localcloud_pub;

        rclcpp::Publisher<std_msgs::msg::Int64 >::SharedPtr keyframe_idx_pub;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr keyframe_odometry_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr lidar_odometry_transformation_pub;

        deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        double translation_std_x;
        double translation_std_y;
        double translation_std_z;

        double translation_std_min_x_;
        double translation_std_min_y_;
        double translation_std_min_z_;


        Eigen::Quaterniond preint_quat ;
        Eigen::Vector3d preint_position;
        Eigen::Vector3d preint_velocity;


        Eigen::Vector3d average_translation;
        deque<Eigen::Vector3d> recent_translations;

        bool initial_pose_recieved;

        // parameters
        // ds_voxelsize
        // ds_voxelsize_lc
        float ds_voxel_size_;
        float ds_voxel_size_lc_;
        // strings
        std::string frame_id = "lidar_odom";
        // this->declare_parameter("my_parameter", "world");
        double keyframe_threshold_angle_;
        double keyframe_threshold_length_;
        double keyframe_threshold_fitness_;
        int keyframe_threshold_index_; // max frames between keyframes
        int icp_max_iterations_;
        int icp_max_coarse_iterations_;
        double icp_max_correspondence_distance_;
        int coarse_correspondence_factor_;
        int local_map_width_;

        int points_per_cloud_scale_;
        int scan_matching_method_;

        int local_map_init_frames_count_;

        bool use_cloud_scale_for_ds_{};
        bool use_wheel_constraint_{};
        bool use_ins_guess_{};
        bool use_preint_imu_guess_{};
        bool use_lidar_odometry_guess_{};
        bool use_linear_undistortion_{};

        double ds_lc_voxel_size_ratio_;
        double cloud_scale_previuos_cloud_weight_;

        std::string imu_topic_;

        // publish topics as parameters so they can be changed?

    

    public:
        LidarOdometry()
        : Node("lidar_odometry")
        {

            declare_parameter("ds_voxel_size", 0.1f);
            get_parameter("ds_voxel_size", ds_voxel_size_);

            declare_parameter("ds_voxel_size_lc", 0.1f);
            get_parameter("ds_voxel_size_lc", ds_voxel_size_lc_);

            declare_parameter("keyframe_threshold_length", 0.3);
            get_parameter("keyframe_threshold_length", keyframe_threshold_length_);

            declare_parameter("keyframe_threshold_angle", 2.0);
            get_parameter("keyframe_threshold_angle", keyframe_threshold_angle_);

            declare_parameter("keyframe_threshold_fitness", 0.1); // 0.5 * icp_correspondance threshold
            get_parameter("keyframe_threshold_fitness", keyframe_threshold_fitness_);

            declare_parameter("keyframe_threshold_index", 0); 
            get_parameter("keyframe_threshold_index", keyframe_threshold_index_);

            declare_parameter("scan_matching_method", 0); 
            get_parameter("scan_matching_method", scan_matching_method_);

            declare_parameter("icp_max_iterations", 25); 
            get_parameter("icp_max_iterations", icp_max_iterations_);

            declare_parameter("icp_max_coarse_iterations", 5); 
            get_parameter("icp_max_coarse_iterations", icp_max_coarse_iterations_);

            declare_parameter("icp_max_correspondance_distance", 0.5); 
            get_parameter("icp_max_correspondance_distance", icp_max_correspondence_distance_);

            declare_parameter("course_correspondence_factor", 10); 
            get_parameter("course_correspondence_factor", coarse_correspondence_factor_);

            declare_parameter("local_map_width", 20); 
            get_parameter("local_map_width", local_map_width_);

            declare_parameter("local_map_init_frames_count", 0); 
            get_parameter("local_map_init_frames_count", local_map_init_frames_count_);

            declare_parameter("use_cloud_scale_for_ds", true); 
            get_parameter("use_cloud_scale_for_ds", use_cloud_scale_for_ds_);

            declare_parameter("points_per_cloud_scale", 25); 
            get_parameter("points_per_cloud_scale", points_per_cloud_scale_);

            declare_parameter("use_wheel_constraint", false); 
            get_parameter("use_wheel_constraint", use_wheel_constraint_);

            declare_parameter("imu_topic", "/imu/data"); 
            get_parameter("imu_topic", imu_topic_);

            declare_parameter("use_ins_guess", false); 
            get_parameter("use_ins_guess", use_ins_guess_);

            declare_parameter("use_preint_imu_guess", false); 
            get_parameter("use_preint_imu_guess", use_preint_imu_guess_);

            declare_parameter("use_lidar_odometry_guess", true); 
            get_parameter("use_lidar_odometry_guess", use_lidar_odometry_guess_);

            declare_parameter("use_linear_undistortion", true); 
            get_parameter("use_linear_undistortion", use_linear_undistortion_);

            declare_parameter("cloud_scale_previuos_cloud_weight", 0.5); 
            get_parameter("cloud_scale_previuos_cloud_weight", cloud_scale_previuos_cloud_weight_);

            declare_parameter("ds_lc_voxel_size_ratio", 3.0); 
            get_parameter("ds_lc_voxel_size_ratio", ds_lc_voxel_size_ratio_);

            declare_parameter("translation_std_min_x", 0.01); 
            get_parameter("translation_std_min_x", translation_std_min_x_);
            declare_parameter("translation_std_min_y", 0.01); 
            get_parameter("translation_std_min_y", translation_std_min_y_);
            declare_parameter("translation_std_min_z", 0.01); 
            get_parameter("translation_std_min_z", translation_std_min_z_);

            // RCLCPP_INFO(get_logger(), "ds_voxel_size in constructor is: %f", ds_voxel_size_);

            initializeParameters();
            allocateMemory();

            // setup callback groups
            run_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            // publsiher callback groud added?
            subscriber_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // create subscriber callback group
            rclcpp::SubscriptionOptions options; // create subscribver options
            options.callback_group = subscriber_cb_group_; // add callbackgroup to subscriber options

            run_timer = this->create_wall_timer(1ms, std::bind(&LidarOdometry::run, this), run_cb_group_); // the process timer 

            // save_data_service = this->create_service<std_srvs::srv::Trigger>("save_odometry_data", &LidarOdometry::save_data, rmw_qos_profile_services_default , subscriber_cb_group_);
            save_data_service = this->create_service<std_srvs::srv::Trigger>("save_odometry_data", std::bind(&LidarOdometry::saveDataService,this, std::placeholders::_1, std::placeholders::_2));


            pointcloud_sub_ = this->create_subscription<PC_msg>("/preprocessed_point_cloud", 100, std::bind(&LidarOdometry::pointCloudHandler, this, _1), options);
            // pointcloud_sub_ = this->create_subscription<PC_msg>("/surf_features", 100, std::bind(&LidarOdometry::pointCloudHandler, this, _1), options);
            // pointcloud_sub_ = this->create_subscription<PC_msg>("/edge_features", 100, std::bind(&LidarOdometry::pointCloudHandler, this, _1), options);
            initial_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initial_pose", 100, std::bind(&LidarOdometry::initialPoseHandler, this, _1), options);
            ins_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom_ins", 100, std::bind(&LidarOdometry::insHandler, this, _1), options);
            imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_, 100, std::bind(&LidarOdometry::imuDataHandler, this, _1), options);


            pointcloud_pub = this->create_publisher<PC_msg>("/full_point_cloud_odom", 100);
            pointcloudguess_pub = this->create_publisher<PC_msg>("/full_point_cloud_transform_guess", 100);
            keyframe_cloud_pub = this->create_publisher<PC_msg>("/full_point_cloud_keyframe", 100);

            // globalcloud_pub = this->create_publisher<PC_msg>("/global_point_cloud", 100);
            localcloud_pub = this->create_publisher<PC_msg>("/local_point_cloud", 100);

            // current odometry publisher
            odometry_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 100);
            keyframe_odometry_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom_keyframe", 100);
            path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 100);

            lidar_odometry_transformation_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/transformation/lidar", 100);

            keyframe_idx_pub = this->create_publisher<std_msgs::msg::Int64>("keyframe_idx", 100);

            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  
            initial_pose_matrix = Eigen::Matrix4d::Identity();
            initial_pose.orientation = Eigen::Quaterniond::Identity();
            initial_pose.position = Eigen::Vector3d::Zero();

            preint_quat = Eigen::Quaterniond::Identity();
            preint_position = Eigen::Vector3d(0.0,0.0,0.0);
            preint_velocity = Eigen::Vector3d(0.0,0.0,0.0);
            
        }
        ~LidarOdometry(){}

        void saveDataService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response>  response){
        // void save_data(){
            RCLCPP_INFO(get_logger(), "Request to save data received %d.", request);
            bool data_to_be_saved = fitnesses.size() > 0;
            size_t elements_to_save = fitnesses.size();

            if (data_to_be_saved){

                std::ofstream file;
                file.open ("temp_saved_odometry_data/fitness.csv");
                for(size_t i=0; i < elements_to_save; i++){
                    double fitn = fitnesses.front();
                    file << std::to_string(fitn) + "\n"; 
                    fitnesses.pop_front();
                }

                // file << "This is the first cell in the first column.\n";
                // file << "a,b,c,\n";
                // file << "c,s,v,\n";
                // file << "1,2,3.456\n";
                // file << "semi;colon";
                file.close();
            } 

            bool data_emptied_to_file = fitnesses.size() == 0;

            response->success = data_to_be_saved && data_emptied_to_file;
            response->message = "End of service" ;

            RCLCPP_INFO(get_logger(), "Data has been saved, Data: %s, Emptied: %s ", data_to_be_saved? "true":"false", data_emptied_to_file?"true":"false");
        }
    

        void pointCloudHandler(const PC_msg::SharedPtr lidar_cloud_msg )
        {   
            RCLCPP_INFO_ONCE(get_logger(), "First Lidar cloud received..");
            cloud_queue.push_back(*lidar_cloud_msg);

        }

        void initialPoseHandler(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initial_pose_msg)
        {
            double qw = initial_pose_msg->pose.pose.orientation.w;
            double qx = initial_pose_msg->pose.pose.orientation.x;
            double qy = initial_pose_msg->pose.pose.orientation.y;
            double qz = initial_pose_msg->pose.pose.orientation.z;
            double x = initial_pose_msg->pose.pose.position.x;
            double y = initial_pose_msg->pose.pose.position.y;
            double z = initial_pose_msg->pose.pose.position.z;

            Eigen::Quaterniond quat( qx, qy, qz, qw );
            
            initial_pose_matrix.block<3,3>(0,0) = quat.toRotationMatrix();

            // also include a position offset at some point
            initial_pose_matrix.block<3,1>(0,3) = Eigen::Vector3d(x, y, z);

            RCLCPP_INFO_ONCE(get_logger(), "Initial pose recieved q: %f %f %f %f p: %f %f %f", qx, qy, qz, qw, x, y, z);
            initial_pose_recieved = true;

        }

        void insHandler(const nav_msgs::msg::Odometry::SharedPtr ins_odom_msg){
            // the INS propagates to the time of the next frame so this pose is a "guess"
            ins_pose_new.orientation.w() = ins_odom_msg->pose.pose.orientation.w;
            ins_pose_new.orientation.x() = ins_odom_msg->pose.pose.orientation.x;
            ins_pose_new.orientation.y() = ins_odom_msg->pose.pose.orientation.y;
            ins_pose_new.orientation.z() = ins_odom_msg->pose.pose.orientation.z;

            ins_pose_new.position.x() = ins_odom_msg->pose.pose.position.x;
            ins_pose_new.position.y() = ins_odom_msg->pose.pose.position.y;
            ins_pose_new.position.z() = ins_odom_msg->pose.pose.position.z;

            ins_pose_new.velocity.x() = ins_odom_msg->twist.twist.linear.x;
            ins_pose_new.velocity.y() = ins_odom_msg->twist.twist.linear.y;
            ins_pose_new.velocity.z() = ins_odom_msg->twist.twist.linear.z;

            if (!initial_pose_recieved) {
                ins_pose = ins_pose_new;
                initial_pose = ins_pose_new;
                initial_pose_recieved = true;
            }

        }

        void imuDataHandler(const sensor_msgs::msg::Imu::SharedPtr imu_data)
        {
            // put data into buffer back
            RCLCPP_INFO_ONCE(get_logger(),"First IMU message recieved..");
            imu_buffer.push_back(imu_data);

        }


        void publishCurrentCloud()
        {   
            sensor_msgs::msg::PointCloud2 msgs;
            pcl::PointCloud<PointType> transformed_cloud;
            pcl::transformPointCloudWithNormals<PointType>(*cloud_in_ds, transformed_cloud, last_odometry_pose.position, last_odometry_pose.orientation);
            #ifndef __INTELLISENSE__ 
            pcl::toROSMsg(transformed_cloud, msgs);
            #endif
            msgs.header.stamp = cloud_header.stamp;
            // msgs.header.frame_id = frame_id;
            msgs.header.frame_id = frame_id;

            pointcloud_pub->publish(msgs);
            // RCLCPP_INFO(get_logger(), "Header: %s,  time of PCL: %f, num points: %i", cloud_header.frame_id, time_new_cloud.nanoseconds()/1e9, cloud_in->points.size());
        }

        void publishGuessCloud(Eigen::Matrix4f transformation)
        {   
            sensor_msgs::msg::PointCloud2 msgs;
            pcl::PointCloud<PointType> transformed_cloud;
            pcl::transformPointCloudWithNormals<PointType>(*cloud_in_ds, transformed_cloud, transformation);
            #ifndef __INTELLISENSE__ 
            pcl::toROSMsg(transformed_cloud, msgs);
            #endif
            msgs.header.stamp = cloud_header.stamp;
            // msgs.header.frame_id = frame_id;
            msgs.header.frame_id = frame_id;

            pointcloudguess_pub->publish(msgs);
        }
        

        void publishKeyframeCloud()
        {   
            sensor_msgs::msg::PointCloud2 msgs;
            #ifndef __INTELLISENSE__ 
            pcl::toROSMsg(*cloud_keyframe, msgs);
            #endif
            msgs.header.stamp = cloud_header.stamp;
            // msgs.header.frame_id = frame_id;
            msgs.header.frame_id = frame_id;

            keyframe_cloud_pub->publish(msgs);
            // RCLCPP_INFO(get_logger(), "Header: %s,  time of PCL: %f, num points: %i", cloud_header.frame_id, time_new_cloud.nanoseconds()/1e9, cloud_in->points.size());
        }


        void initializeParameters()
        {
            RCLCPP_INFO(get_logger(), "Initializing Parameters..");
            // abs_pose[0] = 1;
            // rel_pose[0] = 1;

            // for (int i = 1; i < 7; ++i) {
            //     abs_pose[i] = 0;
            //     rel_pose[i] = 0;
            // }

            if (initial_pose_recieved){
                RCLCPP_INFO(get_logger(), "Using Initial Pose quat from INS:  q_init: %f, %f, %f, %f",initial_pose.orientation.w(), initial_pose.orientation.x(), initial_pose.orientation.y(), initial_pose.orientation.z() );
                last_odometry_pose.orientation = initial_pose.orientation;
            }

            last_odometry_pose.orientation = Eigen::Quaterniond::Identity();
            last_odometry_pose.position = Eigen::Vector3d::Zero();

            registration_transformation.rotation = Eigen::Quaterniond::Identity();
            registration_transformation.translation = Eigen::Vector3d::Zero();
            last_odometry_transformation.rotation = Eigen::Quaterniond::Identity();
            last_odometry_transformation.translation = Eigen::Vector3d::Zero();

            ins_pose_new.orientation = Eigen::Quaterniond::Identity();
            ins_pose_new.position = Eigen::Vector3d::Zero();
            ins_pose.orientation = Eigen::Quaterniond::Identity();
            ins_pose.position = Eigen::Vector3d::Zero();
            ins_relative.rotation = Eigen::Quaterniond::Identity();
            ins_relative.translation = Eigen::Vector3d::Zero();

            // last_odometry_transformation = Eigen::Matrix4d::Identity();
            odometry_transformation_guess = Eigen::Matrix4d::Identity();
            init_guess = Eigen::Matrix4f::Identity();

            average_translation = Eigen::Vector3d::Zero();
            recent_translations.push_back(average_translation);

            // get_parameter("ds_voxel_size", ds_voxel_size_);

            
            odom.header.frame_id = "odom";
            odom.child_frame_id = frame_id;

            transformation_geommsg.header.frame_id = "odom";

            // // ds_voxel_size = 0.1f;
            // ds_voxel_size = 0.2f;
            // RCLCPP_INFO(get_logger(), "ds_voxel_size in function is: %f", ds_voxel_size_);
            down_size_filter.setLeafSize(ds_voxel_size_, ds_voxel_size_, ds_voxel_size_);
            down_size_filter_local_map.setLeafSize(ds_voxel_size_lc_, ds_voxel_size_lc_, ds_voxel_size_lc_);

            latest_frame_idx = 0;
            latest_keyframe_idx = 0;

            system_initialized = false;

        }


        void initializeSystem()
        {   
            RCLCPP_INFO(get_logger(), "Initializing System..");

            updateTransformationError();
            calcCloudScale();
            prev_cloud_scale = cloud_scale;


            // *global_cloud += *cloud_in;
            downSampleClouds();

            pushKeyframe(); // first keyframe pose is a identity matrix

            publishCurrentCloud();

            savePose();
            // savePointCloud();
            publishOdometry();
            // publishGlobalCloud();

            system_initialized = true;
        }


        void allocateMemory()
        {
            RCLCPP_INFO(get_logger(), "Allocating Point Cloud Memory..");

            cloud_in.reset(new pcl::PointCloud<PointType>());
            cloud_in_ds.reset(new pcl::PointCloud<PointType>());
            cloud_prev.reset(new pcl::PointCloud<PointType>());
            cloud_prev_ds.reset(new pcl::PointCloud<PointType>());

            local_map.reset(new pcl::PointCloud<PointType>());
            inverse_local_map.reset(new pcl::PointCloud<PointType>());
            local_map_ds.reset(new pcl::PointCloud<PointType>());
            global_cloud.reset(new pcl::PointCloud<PointType>());

            odometry_pose_info.reset(new pcl::PointCloud<PoseInfo>());
            keyframe_pose_info.reset(new pcl::PointCloud<PoseInfo>());
            // odometry_pose_positions.reset(new pcl::PointCloud<pcl::PointXYZI>());
        }

        void pushKeyframe()
        {
            // RCLCPP_INFO(get_logger(), "New keyframe added! index: %i", latest_keyframe_idx);

            pcl::PointCloud<PointType>::Ptr full = boost::make_shared<pcl::PointCloud<PointType>>();
            // full.reset(new pcl::PointCloud<PointType>());
            pcl::copyPointCloud(*cloud_in, *full);
            *cloud_keyframe = *full;

            Pose new_keyframe_pose;
            // new_keyframe_pose.orientation = Eigen::Quaterniond(last_odometry_pose.block<3,3>(0,0)); // this needs to change at some point
            new_keyframe_pose.orientation = last_odometry_pose.orientation; // this needs to change at some point
            // new_keyframe_pose.position = last_odometry_pose.block<3,1>(0,3);
            new_keyframe_pose.position = last_odometry_pose.position;
            new_keyframe_pose.idx = keyframe_poses.size();
            new_keyframe_pose.frame_idx = latest_frame_idx;
            new_keyframe_pose.pointcloud = cloud_keyframe;
            new_keyframe_pose.time = current_scan_time; // TODO make this the stamp of the cloud

            keyframe_pose = new_keyframe_pose;
            // keyframe_poses.push_back(last_odometry_pose); 
            keyframe_poses.push_back(new_keyframe_pose); 
            // keyframe_index.push_back(latest_frame_idx); // the index to actual frames
            // keyframe_index.push_back(latest_frame_idx); // the keyframe index to all frames

            publishKeyframeCloud();

            savePointCloud();
            saveKeyframePose();
            publishKeyframeOdometry();

            // odometry_transformation_guess = Eigen::Matrix4d::Identity(); //last_odometry_transformation;
            // odometry_transformation_guess = last_odometry_transformation;
            // registration_transformation = Eigen::Matrix4d::Identity(); // this has to be "reset" as it it is used to determine the next guess
            registration_transformation.rotation = Eigen::Quaterniond::Identity(); // this has to be "reset" as it it is used to determine the next guess
            registration_transformation.translation = Eigen::Vector3d::Zero(); // this has to be "reset" as it it is used to determine the next guess


            latest_keyframe_idx++; // the count of keyframes
        }   

        

        // put this stuff in a header
        // function pulled from pcl_conversions.h to make it work explicitly
        // template<typename T>
        void fromROSMsg(const sensor_msgs::msg::PointCloud2 &cloud, pcl::PointCloud<PointType> &pcl_cloud)
        {   
            // sensor_msgs::PointCloud2 cloud_PC2 = cloud;
            pcl::PCLPointCloud2 pcl_pc2;
            #ifndef __INTELLISENSE__ // this is to disable an error squiggle from pcl_conversions not having the proper overload for input type.
            pcl_conversions::toPCL(cloud, pcl_pc2);
            #endif
            pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
        }

        template <typename PointT>
        void lineSampling(const boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_in, boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_out, int numberPerSample)
        {

            RCLCPP_INFO(get_logger(), "Line Sampling?? %i, %i", numberPerSample, cloud_in->points.size());
            int n = 0;
            PointT point;
            for (size_t i = 0; i < cloud_in->points.size(); i++){
                if (n < numberPerSample)
                {
                    n++;
                    // RCLCPP_INFO(get_logger(), "n? %i", n);
                    continue;
                }
                // RCLCPP_INFO(get_logger(), "point sampled %i", i);
                point = cloud_in->points[i];
                cloud_out->points.push_back(point);
                n = 0; 
            }
        }

        template <typename PointT>
        void lineDensitySampling(const boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_in, boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_out, int numberPerSample)
        {

            // RCLCPP_INFO(get_logger(), "Line Sampling?? %i, %i", numberPerSample, cloud_in->points.size());
            double line_distance{};
            double threshold = 10.0/(double)numberPerSample;
            PointT point;
            for (size_t i = 0; i < cloud_in->points.size()-1; i++){
                double dist = (cloud_in->points[i].getVector4fMap() - 
                            cloud_in->points[i+1].getVector4fMap()).norm ();
                line_distance += dist;
                if (line_distance < threshold)
                {
                    continue;
                }
                point = cloud_in->points[i];
                cloud_out->points.push_back(point);
                line_distance = 0.0;
            }
        }


        template <typename PointT>
        void farthestPointSampling(const boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_in, boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_out, int numberSamples)
        {
            vector<int> points_left_idx;
            vector<int> points_sampled_idx;
            vector<double> dists;
            vector<double> dists_to_last_added;

            points_sampled_idx.push_back(0);
            // cloud_out->points.push_back(cloud_in->points[0]); // push first point to sampled

            for (size_t i = 0; i < cloud_in->points.size(); i++){
                dists.push_back(std::numeric_limits<double>::max ()); // infinite distances
                if (i == 0)
                    continue;
                points_left_idx.push_back(i); // 0 is not added because if statement above

            }

            PointType point;
            // for (size_t i = 0; i < cloud_in->points.size(); i++)
            for (int i = 0; i < numberSamples; i++) // number of samples wanted
            {
                // get the last added index
                int last_added_idx = points_sampled_idx.back();

                // calculate distances to the last added point
                dists_to_last_added.clear();
                for (size_t j = 0; j < points_left_idx.size() - i; j++)
                {
                    double dist = (cloud_in->points[points_left_idx[j]].getVector4fMap () - 
                            cloud_in->points[last_added_idx].getVector4fMap ()).squaredNorm ();
                    dists_to_last_added.push_back(dist);
                }

                // update distances ie. get shortest of dist_to_last_added and dists , mayeb combine this with the next loop to save a loop,
                for (size_t k = 0; k < points_left_idx.size(); k++)
                {
                    if (dists_to_last_added[k] < dists[points_left_idx[k]]){
                        dists[points_left_idx[k]] = dists_to_last_added[k];
                    }
                }

                // find the index of the point in points_left with the largest distance
                double max_dist = std::numeric_limits<double>::min ();
                int idx_max{};
                int selected{};
                for (size_t l = 0; l < points_left_idx.size(); l++) {
                    if (dists[points_left_idx[l]] <= max_dist)
                        continue;
                    max_dist = dists[points_left_idx[l]];
                    idx_max = l;
                    selected = points_left_idx[l];
                }

                // add that index to sampled and remove from points_left
                points_sampled_idx.push_back(selected);
                points_left_idx.erase(points_left_idx.begin()+idx_max);

                // point = lidar_cloud_in->points[i];
            }

            pcl::PointCloud<PointType>::Ptr sampled = boost::make_shared<pcl::PointCloud<PointType>>();

            // assign points to sampled from found indices
            for (size_t i = 0; i < points_sampled_idx.size(); i++)
            {
                sampled->points.push_back(cloud_in->points[points_sampled_idx[i]]);
            }

            cloud_out = sampled;
        }

        void downSampleClouds() {


            setCloudScale();


            if (ds_voxel_size_ > 0.0) {
                cloud_in_ds->clear();
                down_size_filter.setInputCloud(cloud_in);
                down_size_filter.filter(*cloud_in_ds);
            } else {
                cloud_in_ds = cloud_in;
            }
            // farthestPointSampling(cloud_in, cloud_in_ds, 1000);

            RCLCPP_INFO(get_logger(), "Downsampling clouds..");
            // lineSampling(cloud_in, cloud_in_ds, points_per_cloud_scale_);
            // lineDensitySampling(cloud_in, cloud_in_ds, points_per_cloud_scale_);


            // cloud_keyframe_ds->clear();
            if (ds_voxel_size_ > 0.0) {
                down_size_filter.setInputCloud(cloud_keyframe);
                down_size_filter.filter(*cloud_keyframe_ds);
            } else {
                cloud_keyframe_ds = cloud_keyframe;
            }
        }

        void setCloudScale()
        {
            calcCloudScale();
            if (use_cloud_scale_for_ds_) {
                double prev_cloud_weight = cloud_scale_previuos_cloud_weight_;
                double new_cloud_scale = prev_cloud_scale * prev_cloud_weight + cloud_scale * (1 - prev_cloud_weight); // calculate new scale as weighted average between old and new
                RCLCPP_DEBUG(get_logger(), "cloud scale is: %f, prev: %f, new scale is: %f", cloud_scale, prev_cloud_scale, new_cloud_scale);
                float temp_leafsize = new_cloud_scale / float(points_per_cloud_scale_); // 25 points from side to side by default
                float temp_leafsize_lc = temp_leafsize / float(ds_lc_voxel_size_ratio_); // best have as uneven number to have center point ?
                down_size_filter.setLeafSize(temp_leafsize, temp_leafsize, temp_leafsize);
                down_size_filter_local_map.setLeafSize(temp_leafsize_lc, temp_leafsize_lc, temp_leafsize_lc);
                prev_cloud_scale = new_cloud_scale;
            }
        }

        void calcCloudScale()
        {
            // cloud_scale = getMaxSegment(*cloud_in); // was! very slow! for a 10 000 point cloud it does 100 mil iterations ie. O(n²)
            cloud_scale = getMaxLeftRight(*cloud_in); // this only uses O(n) but is not guaranteed to find the largest distance, only a usable distance
        }

        template <typename PointT> double inline
        getMaxSegment (const pcl::PointCloud<PointT> &cloud)
        {
            double max_dist = std::numeric_limits<double>::min ();
            double max_left = std::numeric_limits<double>::min ();
            int i_min = -1, i_max = -1;
            int i_left = -1;

            // new method N*2 iterations
            // find leftmost point
            for (size_t i = 0; i < cloud.points.size (); ++i)
            {        
                double y_val = cloud.points[i].y;
                // Compute the distance 
                // double dist = (cloud.points[i].getVector4fMap () - 
                //             cloud.points[j].getVector4fMap ()).squaredNorm ();
                if (y_val <= max_left) {
                    max_left = y_val;
                    i_left = i;
                }
            }
            i_min = i_left;

            // need find the distance to farhest point from this.
            for (size_t i = 0; i < cloud.points.size (); ++i)
            {   
                // Compute the distance 
                double dist = (cloud.points[i_left].getVector4fMap () - 
                            cloud.points[i].getVector4fMap ()).squaredNorm ();
                if (dist <= max_dist)
                    continue;

                max_dist = dist;
                // i_min = i_left;
                i_max = i;
                // max_right = y_val;
                // i_right = i;
            }
            
            // Old method -> N² iterations, very slow
            // for (size_t i = 0; i < cloud.points.size (); ++i)
            // {
            //     for (size_t j = i; j < cloud.points.size (); ++j)
            //     {
            //         // Compute the distance 
            //         double dist = (cloud.points[i].getVector4fMap () - 
            //                     cloud.points[j].getVector4fMap ()).squaredNorm ();
            //         if (dist <= max_dist)
            //         continue;

            //         max_dist = dist;
            //         i_min = i;
            //         i_max = j;
            //     }
            // }

            if (i_min == -1 || i_max == -1)
            return (max_dist = std::numeric_limits<double>::min ());

            return (std::sqrt (max_dist));
        }

        template <typename PointT> double inline
        getMaxLeftRight(const pcl::PointCloud<PointT> &cloud)
        {
            double max_left = std::numeric_limits<double>::max ();
            double max_right = std::numeric_limits<double>::min ();
            int i_left = -1, i_right = -1;

            for (size_t i = 0; i < cloud.points.size (); ++i)
            {        
                double y_val = cloud.points[i].y;
                // Compute the distance 
                // double dist = (cloud.points[i].getVector4fMap () - 
                //             cloud.points[j].getVector4fMap ()).squaredNorm ();
                if (y_val <= max_left) {
                    max_left = y_val;
                    i_left = i;

                } else if (y_val >= max_right) {
                    max_right = y_val;
                    i_right = i;
                }
            }

            if (i_left == -1 || i_right == -1)
            return (max_left = std::numeric_limits<double>::min ());

            // double dist = (cloud.points[i_left].getVector4fMap () - 
            //                 cloud.points[i_right].getVector4fMap ()).squaredNorm ();


            // double dist = cloud.points[i_right].y - cloud.points[i_left].y;
            double dist = max_right - max_left;
            // RCLCPP_INFO(get_logger(), "max_left is: %f - max_right is: %f, difference is %f", max_left, max_right, dist);

            // pmin = cloud.points[i_left];
            // pmax = cloud.points[i_right];
            return dist;
        }



        void undistortCLoudLinear()
        {   

            Eigen::Vector3d linear_translation = last_odometry_transformation.translation;
            double scan_dt = cloud_in->points[cloud_in->points.size()-1].intensity - (int)cloud_in->points[cloud_in->points.size()-1].intensity;

            for (auto &point : cloud_in->points) {
                double intensity = point.intensity;
                double dt_i = intensity - (int)intensity;
                double ratio_i = dt_i / scan_dt;
                if(ratio_i >= 1.0) {
                    ratio_i = 1.0;
                }

                Eigen::Vector3d point_vector(point.x, point.y, point.z);
                // point_vector -= linear_translation * (1 - ratio_i);
                point_vector += linear_translation * ratio_i;

                point.x = point_vector.x();
                point.y = point_vector.y();
                point.z = point_vector.z();

            }
        }


        void updateINSRelative()
        {
            ins_relative.rotation = ins_pose.orientation.inverse() * ins_pose_new.orientation; 
            ins_pose.orientation = ins_pose_new.orientation;

            ins_relative.translation = (ins_pose_new.position - ins_pose.position);
        }

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

        void integrateImu(Eigen::Vector3d acc, Eigen::Vector3d ang_vel, double dt)
        {
            Eigen::Quaterniond ori_pre(preint_quat);
            preint_quat *= deltaQ(ang_vel * dt);
            Eigen::Quaterniond ori_avg = ori_pre.slerp(0.5, preint_quat);

            acc = ori_avg * acc; // needs biases and gravity compensation, might use madgwick for gravity removal
            preint_velocity += acc * dt;
            preint_position += preint_velocity * dt - acc * dt*dt *0.5;

        }


        void preintegrateINSGuess()
        {
            // clear buffer up till keyframe scan time stamp
            while(keyframe_pose.time >= toSec(imu_buffer.front()->header.stamp) )
            {
                RCLCPP_INFO(get_logger(), "imu msg removed");
                imu_buffer.pop_front();
            }

            sensor_msgs::msg::Imu imu_msg;
            // double next_scan_time = current_scan_time + scan_dt; 
            double next_scan_time = current_scan_time; // current scan time is the unmatched frame at this point

            preint_quat = Eigen::Quaterniond::Identity();
            preint_position  << 0.0,0.0,0.0;
            // preint_velocity << 0.0,0.0,0.0;
            preint_velocity = last_odometry_pose.velocity;


            // run buffer til next scan time
            // for (size_t i=0 ; i < imu_buffer.size(); i++ )
            size_t i = 0;
            RCLCPP_INFO(get_logger(), "STARTING INTEGRATION..%i", i);
            while(true)
            {
                if (next_scan_time < toSec(imu_buffer[i]->header.stamp) ){
                    RCLCPP_INFO(get_logger(), "imu frames %i", i);
                    return;
                }
                imu_msg = *imu_buffer[i];

                Eigen::Vector3d acc_in(imu_msg.linear_acceleration.x,
                                       imu_msg.linear_acceleration.y,
                                       imu_msg.linear_acceleration.z);
                // Eigen::Vector3d ang_vel_in(imu_msg.angular_velocity.x,
                //                            imu_msg.angular_velocity.y,
                //                            imu_msg.angular_velocity.z);
                Eigen::Vector3d ang_vel_in(0.0,
                                           0.0,
                                           imu_msg.angular_velocity.z);
                double dt = 0.01;
                integrateImu(acc_in, ang_vel_in, dt);

                i++;
            }
            RCLCPP_INFO(get_logger(), "FAILED INTEGRATION.. shouldn't happen");

        }   


        void calculateInitialTransformationGuess(){

            init_guess = Eigen::Matrix4f::Identity(); 
            

            // Eigen::Matrix3d last_odometry_transformation_rot = last_odometry_transformation.block<3,3>(0,0);

            // odometry_transformation_guess = registration_transformation * last_odometry_transformation; // prediction of the next transformation using the previuosly found transformation - used for ICP initial guess
            // odometry_transformation_guess = registration_transformation; // prediction of the next transformation using the previuosly found transformation - used for ICP initial guess
            // odometry_transformation_guess = registration_transformation * last_odometry_transformation_rot; // prediction of the next transformation using the previuosly found transformation - used for ICP initial guess
            // odometry_transformation_guess = Eigen::Matrix4d::Identity(); // prediction of the next transformation using the previuosly found transformation - used for ICP initial guess

            Eigen::Matrix4f K;
            K.block<3, 3>(0, 0) = Eigen::Matrix3f(Eigen::Matrix3d(keyframe_pose.orientation).cast<float>());
            K.block<3, 1>(0, 3) = keyframe_pose.position.cast<float>();
            // K.block<3, 1>(0, 3) = Eigen::Vector3f::Zero();

            if (use_preint_imu_guess_ && !imu_buffer.empty()){  // and with "ins_is_updated" -- need also to make guarentee that last ins msg has been recieved.
                preintegrateINSGuess();
                preint_position.z() = 0.0;
                init_guess.block<3, 3>(0, 0) = Eigen::Matrix3f((preint_quat).cast<float>());
                // init_guess.block<3, 3>(0, 0) = Eigen::Matrix3f((ins_relative.rotation).cast<float>());
                // init_guess.block<3, 1>(0, 3) = (registration_transformation.translation + preint_position).cast<float>();
                // init_guess.block<3, 1>(0, 3) = (preint_position).cast<float>();
                init_guess.block<3, 1>(0, 3) = (registration_transformation.translation).cast<float>();
                // init_guess.block<3, 1>(0, 3) = (average_translation).cast<float>();
                init_guess(1,3) = 0.0;
                init_guess(2,3) = - keyframe_pose.position.z();
            }  else if (use_ins_guess_ ){  // and with "ins_is_updated" -- need also to make guarentee that last ins msg has been recieved.
                // RCLCPP_INFO(get_logger(), "INS guess used");
                updateINSRelative();

                Eigen::Matrix3f ins_rot(Eigen::Matrix3d(ins_pose.orientation).cast<float>());
                Eigen::Matrix3f rot_guess(( K.block<3, 3>(0, 0).inverse() * ins_rot));

                init_guess.block<3, 3>(0, 0) = rot_guess;
                // init_guess.block<3, 1>(0, 3) = (ins_relative.translation).cast<float>();
                init_guess.block<3, 1>(0, 3) = (ins_relative.translation).cast<float>();
                // init_guess.block<3, 1>(0, 3) = (registration_transformation.translation + rot_guess*last_odometry_transformation.translation).cast<float>();
                // init_guess.block<3, 1>(0, 3) = Eigen::Vector3f::Zero();
                // init_guess.block<3, 1>(0, 3) = (average_translation).cast<float>();
                init_guess(2,3) = - keyframe_pose.position.z();
            } else if (use_lidar_odometry_guess_ ) {
                RCLCPP_INFO(get_logger(), "Odometry guess used");
                init_guess.block<3, 3>(0, 0) = Eigen::Matrix3f((registration_transformation.rotation * last_odometry_transformation.rotation).cast<float>());
                init_guess.block<3, 1>(0, 3) = (registration_transformation.translation + last_odometry_transformation.translation).cast<float>();
                // init_guess.block<3, 3>(0, 0) = Eigen::Matrix3f((registration_transformation.rotation).cast<float>());
                // init_guess.block<3, 1>(0, 3) = (registration_transformation.translation).cast<float>();
                // init_guess.block<3, 1>(0, 3) = average_translation.cast<float>();
                init_guess(1,3) = 0.0;
                init_guess(2,3) = 0.0;
            } 
            // else just identity guess..


            publishGuessCloud(K*init_guess);
        }



        Eigen::Matrix4d regisrationICP(boost::shared_ptr<pcl::PointCloud<PointType>> source, const boost::shared_ptr<pcl::PointCloud<PointType>> target)
        {
            typedef pcl::registration::TransformationEstimationPointToPlane<PointType, PointType> PointToPlane;
            // typedef pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<PointType, PointType> PointToPlane;
            // maybe not setup the icp on every use
            // pcl::IterativeClosestPoint<PointType, PointType> icp;
         
            pcl::IterativeClosestPointWithNormals<PointType, PointType> icp;
            
            boost::shared_ptr<PointToPlane> p2pl(new PointToPlane);
            icp.setTransformationEstimation(p2pl);
            icp.setUseReciprocalCorrespondences(true);

            // double max_correspondance_distance = cloud_scale * icp_max_correspondence_distance_  ;
            double max_correspondance_distance = icp_max_correspondence_distance_  ;

            icp.setUseSymmetricObjective(true);
            icp.setEnforceSameDirectionNormals(true);
            
            // RCLCPP_INFO(get_logger(),"source size %i target size %i", source->points.size(), target->points.size());

            icp.setInputSource(source);
            icp.setInputTarget(target);
            icp.setEuclideanFitnessEpsilon(1e-3);
            // icp.setTransformationRotationEpsilon(0.99); // cos(angle)

            // icp.setRANSACIterations(10);
            // icp.setRANSACOutlierRejectionThreshold(1.5);

            pcl::PointCloud<PointType>::Ptr aligned_cloud(new pcl::PointCloud<PointType>());
            // Eigen::Matrix4f init_guess = odometry_transformation_guess.cast<float>(); // should be a better guess anyway

            

            // course estimation first to get a better initial estimate of transformation
            icp.setMaxCorrespondenceDistance(coarse_correspondence_factor_*max_correspondance_distance); // 10*
            icp.setMaximumIterations(icp_max_coarse_iterations_);
            icp.setTransformationEpsilon(1e-1);

            icp.align(*aligned_cloud, init_guess);

            //get icp transformation 
            init_guess = icp.getFinalTransformation();//.cast<double>(); // why cast to double??
            

            // second iteration with finer correspondence limit
            icp.setMaxCorrespondenceDistance(max_correspondance_distance);
            icp.setMaximumIterations(icp_max_iterations_);
            icp.setTransformationEpsilon(1e-9);
            icp.align(*aligned_cloud, init_guess);

            // Eigen::Matrix4f registration_transform;
            // registration_transform = icp.getFinalTransformation();//.cast<double>(); // why cast to double??

            
            Eigen::Matrix4d registration_transform_double = icp.getFinalTransformation().cast<double>();


            icp_fitness = icp.getFitnessScore();
            
            RCLCPP_INFO(get_logger(), "ICP %i,  fitness: %f", icp.hasConverged(), icp_fitness);


            return registration_transform_double;
        }


        Eigen::Matrix4d regisrationICP_gcip(boost::shared_ptr<pcl::PointCloud<PointType>> source, const boost::shared_ptr<pcl::PointCloud<PointType>> target)
        {

            // maybe not setup the icp on every use
     
            pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp;


            // double max_correspondance_distance = icp_max_correspondence_distance_;
            double max_correspondance_distance = cloud_scale * icp_max_correspondence_distance_  ;

            icp.setInputSource(source);
            icp.setInputTarget(target);
            icp.setEuclideanFitnessEpsilon(1e-1);
            // icp.setRANSACIterations(10);
            // icp.setRANSACOutlierRejectionThreshold(1.5);
            icp.setCorrespondenceRandomness(20);
            icp.setMaximumOptimizerIterations(50);

            // course estimation first to get a better initial estimate of transformation
            icp.setMaxCorrespondenceDistance(coarse_correspondence_factor_*max_correspondance_distance);
            icp.setMaximumIterations(icp_max_coarse_iterations_);
            icp.setTransformationEpsilon(1e-1);

            pcl::PointCloud<PointType>::Ptr aligned_cloud(new pcl::PointCloud<PointType>());
            icp.align(*aligned_cloud, init_guess);

            //get icp transformation and use 
            Eigen::Matrix4f registration_transform;
            registration_transform = icp.getFinalTransformation();//.cast<double>(); // why cast to double??

            // second iteration with finer correspondence limit
            icp.setMaxCorrespondenceDistance(max_correspondance_distance);
            icp.setMaximumIterations(icp_max_iterations_);
            icp.setTransformationEpsilon(1e-5);
            icp.align(*aligned_cloud, registration_transform);

            registration_transform = icp.getFinalTransformation();//.cast<double>(); // why cast to double??

            // icp_nl.align(*aligned_cloud, registration_transform);
            // // Eigen::Matrix4d registration_transform;
            // registration_transform = icp_nl.getFinalTransformation();//.cast<double>(); // why cast to double??
            
            Eigen::Matrix4d registration_transform_double = registration_transform.cast<double>();
            
            // registration_transformation = registration_transform_double;

            // make transform matrix into quaternion and vector
            // Eigen::Quaterniond reg_quarternion(registration_transform_double.block<3, 3>(0, 0)); 
            // Eigen::Vector3d reg_translation(registration_transform_double.block<3, 1>(0, 3));

            icp_fitness = icp.getFitnessScore();
            
            RCLCPP_INFO(get_logger(), "ICP: %i,  fitness: %f", icp.hasConverged(), icp_fitness);


            return registration_transform_double;
        }

        Eigen::Matrix4d regisrationNDT(boost::shared_ptr<pcl::PointCloud<PointType>> source, const boost::shared_ptr<pcl::PointCloud<PointType>> target)
        {
            // Initializing Normal Distributions Transform (NDT).
            pcl::NormalDistributionsTransform<PointType, PointType> ndt;
            // fast_pcl::NormalDistributionsTransform<PointType, PointType> ndt;

            // Setting scale dependent NDT parameters
        
            ndt.setResolution (0.5);
            // Setting point cloud to be aligned.
            ndt.setInputSource(source);
            // Setting point cloud to be aligned to.
            ndt.setInputTarget(target);

            ndt.setOulierRatio(0.01);
        
            pcl::PointCloud<PointType>::Ptr aligned_cloud(new pcl::PointCloud<PointType>());
            // Eigen::Matrix4f init_guess = odometry_transformation_guess.cast<float>(); // should be a better guess anyway

            if (true) {

                // course estimation first to get a better initial estimate of transformation

                ndt.setStepSize (coarse_correspondence_factor_* icp_max_correspondence_distance_);
                //Setting Resolution of NDT grid structure (VoxelGridCovariance).
                // ndt.setResolution (0.1);
                ndt.setMaximumIterations(icp_max_coarse_iterations_);
                ndt.setTransformationEpsilon(1e-1);

                ndt.align(*aligned_cloud, init_guess);
                // RCLCPP_INFO(get_logger(),"Where's the poop Robin?");

                //get icp transformation 
                init_guess = ndt.getFinalTransformation();//.cast<double>(); // why cast to double??
            }

            // Setting max number of registration iterations final.
            ndt.setMaximumIterations(icp_max_iterations_);
            // Setting maximum step size for More-Thuente line search.
            ndt.setStepSize(icp_max_correspondence_distance_);
            //Setting Resolution of NDT grid structure (VoxelGridCovariance).

            // Setting minimum transformation difference for termination condition.
            ndt.setTransformationEpsilon (1e-5);


            ndt.align(*aligned_cloud, init_guess);
        
            // double max_correspondance_distance = cloud_scale * icp_max_correspondence_distance_  ;

            // RCLCPP_INFO(get_logger(),"source size %i target size %i", source->points.size(), target->points.size());

            Eigen::Matrix4f registration_transform;
            registration_transform = ndt.getFinalTransformation();//.cast<double>(); // why cast to double??

            
            Eigen::Matrix4d registration_transform_double = registration_transform.cast<double>();
            
            // registration_transformation = registration_transform_double;

            // make transform matrix into quaternion and vector
            // Eigen::Quaterniond reg_quarternion(registration_transform_double.block<3, 3>(0, 0)); 
            // Eigen::Vector3d reg_translation(registration_transform_double.block<3, 1>(0, 3));

            icp_fitness = ndt.getFitnessScore();
            
            RCLCPP_INFO(get_logger(), "NDT %i,  fitness: %f", ndt.hasConverged(), icp_fitness);

            return registration_transform_double;
        }

        void scanMatchRegistration(boost::shared_ptr<pcl::PointCloud<PointType>> source, const boost::shared_ptr<pcl::PointCloud<PointType>> target)
        {
            Eigen::Matrix4d registration_transform_matrix;
            switch(scan_matching_method_) {
                case 1:
                    registration_transform_matrix = regisrationICP_gcip(source, target);
                    break;
                case 2:
                    registration_transform_matrix = regisrationNDT(source, target); // source, target - such that t = s*T, where T is the transformation. If source is the new frame the transformation found is the inverse of the odometry transformation 
                    break;
                default:
                    registration_transform_matrix = regisrationICP(source, target);
            }

            registration_transformation.rotation = Eigen::Quaterniond(registration_transform_matrix.block<3,3>(0,0)).normalized();
            registration_transformation.translation = Eigen::Vector3d(registration_transform_matrix.block<3,1>(0,3));

            saveFitness();
        }

        void saveFitness(){
            fitnesses.push_back(icp_fitness);
        }

        // Eigen::Matrix4d wheelConstrainTransformation(Eigen::Matrix4d input_transformation)
        // {
        //     // attempts to remove any "sliding" in y, by assuming movement is in an circle arc

        //     Eigen::Matrix3d rotation_mat_pose = last_odometry_pose.block<3,3>(0,0);
        //     Eigen::Matrix3d rotation_mat_transform = input_transformation.block<3,3>(0,0);
        //     Eigen::Vector3d translation(input_transformation.block<3,1>(0,3));
        //     double transformation_distance = translation.norm();


        //     Eigen::Quaterniond quat_transform_eigen(rotation_mat_transform);
        //     tf2::Quaternion quat_transform_tf(quat_transform_eigen.x(),
        //                                         quat_transform_eigen.y(),
        //                                         quat_transform_eigen.z(),
        //                                         quat_transform_eigen.w());
        //     tf2::Matrix3x3 rotation_mat_transform_tf(quat_transform_tf);
        //     double r, p, y;
        //     rotation_mat_transform_tf.getEulerYPR(y,p,r);
        //     RCLCPP_INFO(get_logger(), "\nRoll, Pitch, Yaw  of transform: %f %f %f", r*57.3, p*57.3, y*57.3);

        //     RCLCPP_INFO(get_logger(), "translation: %f %f %f", translation[0], translation[1] , translation[2]);
        //     RCLCPP_INFO(get_logger(), "norm of translation: %f", transformation_distance);

        //     // Eigen::Vector3d translation_prime = rotation_mat_pose.inverse() * translation;  // the translation in the the moving frame of the odometry
        //     Eigen::Vector3d translation_prime = translation;  // the translation in the the moving frame of the odometry
        //     RCLCPP_INFO(get_logger(), "translation prime: %f %f %f", translation_prime[0], translation_prime[1] , translation_prime[2]);

        //     double yaw_radius = transformation_distance / (abs(sin(y)) + 1e-8);
        //     RCLCPP_INFO(get_logger(), "Yaw radius: %f", yaw_radius);
        //     double y_translation_max = yaw_radius * (1 - cos(asin(translation_prime[0]/yaw_radius)));

        //     RCLCPP_INFO(get_logger(), "max y: %f", y_translation_max);
        //     RCLCPP_INFO(get_logger(), "transform y: %f", translation_prime[1]);


        //     // if (y*57.3 < 0.2) {
        //         // return input_transformation;
        //     // }

        //     // if (abs(translation_prime[1]) > y_translation_max)  // maybe add scaling coefficient?
        //     // {
        //         // int sign = abs(translation_prime[1]) / translation_prime[1];
        //         // translation_prime[1] =  sign *y_translation_max;
        //         translation_prime[1] *=  0.2;
        //     // }

        //     // translation = rotation_mat_pose * translation_prime;
        //     translation = translation_prime;
            
        //     RCLCPP_INFO(get_logger(), "final translation: %f %f %f", translation[0], translation[1] , translation[2]);

        //     Eigen::Matrix4d constrained_transformation(input_transformation);
        //     constrained_transformation.block<3,1>(0,3) = translation;

        //     return constrained_transformation;
        // }

        void updateTransformationError()
        {
            // make transform matrix into quaternion and vector
            // Eigen::Quaterniond quarternion_pose(last_odometry_pose.block<3, 3>(0, 0)); 
            // Eigen::Matrix3d rot_mat(last_odometry_transformation.block<3, 3>(0, 0)); 
            Eigen::Vector3d translation = last_odometry_transformation.translation;

            Eigen::Vector3d translation_deviation;
            translation_deviation = average_translation - translation;
            // translation = rot_mat * translation;
            // RCLCPP_INFO(get_logger(), "translation %f %f %f", translation[0], translation[1], translation[2]);

            translation_std_x = ( abs(translation_deviation[0]))  ;//+ translation_std_min_x_;
            translation_std_y = ( abs(translation_deviation[1]))  ;//+ translation_std_min_y_;
            translation_std_z = ( abs(translation_deviation[2]))  ;//+ translation_std_min_z_;

            translation_std_x = max(translation_std_x, translation_std_min_x_);
            translation_std_y = max(translation_std_y, translation_std_min_y_);
            translation_std_z = max(translation_std_z, translation_std_min_z_);

            RCLCPP_INFO(get_logger(), "trans deviation %f %f %f", translation_std_x, translation_std_y, translation_std_z);

            // length of translation
            // double distance = translation.norm();

            // angle of rotation
            // 0.1 rad is approx 57.3 deg
            // Eigen::AngleAxisd angleAxis = Eigen::AngleAxisd(quarternion);
            // double angle = angleAxis.angle()*180.0/M_PI;
        }

        void calculateAverageTranslation(){

            average_translation << 0.0,0.0,0.0;
            for (size_t i=0; i < recent_translations.size(); i++){
                average_translation += recent_translations[i];
            }
            average_translation = average_translation/ ((float)recent_translations.size());
    
        }

        void updateOdometryPose(Transformation update_transformation)
        {   
            Pose next_odometry_pose;
            // RCLCPP_INFO(get_logger(), "key %f %f %f", translation_std_x, translation_std_y, translation_std_z);
            // next_odometry_pose.position =  keyframe_pose.orientation * ( keyframe_pose.position - registration_transformation.translation)  + registration_transformation.translation;
            // Eigen::Isometry3d Tr = Eigen::Isometry3d(registration_transformation.rotation);
            // Tr.pretranslate(registration_transformation.translation);
            // Eigen::Isometry3d Tk = Eigen::Isometry3d(keyframe_pose.orientation);
            // Tk.pretranslate(keyframe_pose.position );

            Eigen::Vector3d rotated_translation = keyframe_poses.back().orientation * update_transformation.translation;
            // RCLCPP_INFO(get_logger(), "translation %f %f %f", update_transformation.translation[0] , registration_transformation.translation[1], update_transformation.translation[2]);
            // RCLCPP_INFO(get_logger(), "rotated translation %f %f %f", rotated_translation[0] , rotated_translation[1], rotated_translation[2]);
            // RCLCPP_INFO(get_logger(), "keyframe position %f %f %f", keyframe_pose.position[0] , keyframe_pose.position[1], keyframe_pose.position[2]);


            // Eigen::Isometry3d Tn = Tr*Tk;

            next_odometry_pose.position =   keyframe_poses.back().position  + rotated_translation;
            // next_odometry_pose.position =   keyframe_poses.back().position  + registration_transformation.rotation * rotated_translation;
            // next_odometry_pose.position = Tn.translation();
            // RCLCPP_INFO(get_logger(), "next position %f %f %f", next_odometry_pose.position[0] , next_odometry_pose.position[1], next_odometry_pose.position[2]);
            next_odometry_pose.orientation = (update_transformation.rotation * keyframe_poses.back().orientation).normalized();
            // next_odometry_pose.orientation = Eigen::Quaterniond(Tn.rotation()).normalized();

            
            // next_odometry_pose.position = keyframe_pose.orientation * registration_transformation.translation;
            // next_odometry_pose.orientation = (keyframe_pose.orientation * registration_transformation.rotation);

            // Eigen::Matrix4d next_odometry_pose = keyframe_poses.back() * registration_transformation; // the next evaluted odometry pose, not neccesarily a keyframe
            // last_odometry_transformation =  next_odometry_pose * last_odometry_pose.inverse(); // transformation between 2 consecutive odometry poses
            last_odometry_transformation.rotation = last_odometry_pose.orientation.inverse() * next_odometry_pose.orientation; // transformation between 2 consecutive odometry poses
            last_odometry_transformation.translation = last_odometry_pose.orientation.inverse() * (next_odometry_pose.position - last_odometry_pose.position) ; // transformation between 2 consecutive odometry poses
            // last_odometry_transformation.block<3,1>(0,3) = - last_odometry_transformation.block<3,1>(0,3);

            calculateAverageTranslation();
            updateTransformationError();

            // RCLCPP_INFO(get_logger(), "average translation %f %f %f", average_translation[0], average_translation[1], average_translation[2]);
            // calculateAverageRotation();

            // if (use_wheel_constraint_ ){
            //     // make wheel constraint here on last last_odometry_transformation moving from last_odometry_pose
            //     Eigen::Matrix4d constrained_odometry_transformation = wheelConstrainTransformation(last_odometry_transformation);
            //     next_odometry_pose =  last_odometry_pose * constrained_odometry_transformation;
            //     // registration_transformation =  next_odometry_pose * keyframe_poses.back().inverse();
            //     registration_transformation =  keyframe_poses.back().inverse() * next_odometry_pose;
            // }


            last_odometry_pose.position = Eigen::Vector3d(next_odometry_pose.position);
            last_odometry_pose.orientation = Eigen::Quaterniond(next_odometry_pose.orientation);
            last_odometry_pose.velocity = last_odometry_transformation.translation / scan_dt;
            last_odometry_pose.time = current_scan_time;
        }

        void pushRecentTranslation()
        {
            // RCLCPP_INFO(get_logger(), "reg_translation %f %f %f", reg_translation[0], reg_translation[1], reg_translation[2]);    
            // Eigen::Vector3d reg_translation(registration_transformation.block<3, 1>(0, 3));
            Eigen::Vector3d recent_translation = last_odometry_transformation.translation;
            recent_translations.push_back(recent_translation);
            if (recent_translations.size() > 3){
                recent_translations.pop_front();
            }

        }

        bool newKeyframeRequired()
        {
            // make transform matrix into quaternion and vector
            Eigen::Quaterniond reg_quarternion = registration_transformation.rotation; 
            Eigen::Vector3d reg_translation = registration_transformation.translation;

            // length of translation
            double distance = reg_translation.norm();

            // angle of rotation
            // 0.1 rad is approx 5.73 deg
            Eigen::AngleAxisd angleAxis = Eigen::AngleAxisd(reg_quarternion);
            double angle = angleAxis.angle()*180.0/M_PI;

            RCLCPP_DEBUG(get_logger(), "found transform parameters: length: %f, angle: %f, fitness: %f", distance, angle, icp_fitness);
            RCLCPP_DEBUG(get_logger(), "threshold parameters: length: %f, angle: %f, fitness: %f", keyframe_threshold_length_, keyframe_threshold_angle_, keyframe_threshold_fitness_);
            RCLCPP_DEBUG(get_logger(), "should this be true: %i", (distance > keyframe_threshold_length_ || angle > keyframe_threshold_angle_ || icp_fitness > keyframe_threshold_fitness_ ));


            // and get icp fitness

            // assert if any is beyond thresholds

            bool dist = distance > keyframe_threshold_length_ && keyframe_threshold_length_ > 0.0;
            bool ang = angle > keyframe_threshold_angle_ && keyframe_threshold_angle_ > 0.0;
            bool fitness = icp_fitness > keyframe_threshold_fitness_ && keyframe_threshold_fitness_ > 0.0;
            bool index = (int)latest_frame_idx > (keyframe_threshold_index_ + keyframe_poses.back().frame_idx -1 ) && keyframe_threshold_index_ > 0;

            bool covariance_x = translation_std_x < 0.1;
            bool covariance_y = translation_std_y < 0.1;
            bool covariance_z = translation_std_z < 0.05;

            // if transformation has big covaraince it is not used as keyframe!
            bool good_covariance = (covariance_x && covariance_y && covariance_z);

            // good_covariance = true; // this is an override!
            if (!good_covariance){
                registration_transformation.rotation = Eigen::Quaterniond::Identity();
                registration_transformation.translation = Eigen::Vector3d::Zero();
            }


            return ((dist || ang || fitness ) && good_covariance) || index;
        }


        void savePose()
        {
            // function to push new_pose to odometries, saved as quaternion and translation

            Eigen::Quaterniond reg_quarternion = last_odometry_pose.orientation; 
            // reg_quarternion.normalize();
            Eigen::Vector3d reg_translation = last_odometry_pose.position;

            PoseInfo new_pose;
            new_pose.qw = reg_quarternion.w();
            new_pose.qx = reg_quarternion.x();
            new_pose.qy = reg_quarternion.y();
            new_pose.qz = reg_quarternion.z();
            new_pose.x = reg_translation.x();
            new_pose.y = reg_translation.y();
            new_pose.z = reg_translation.z();
            new_pose.idx = odometry_pose_info->points.size();
            new_pose.time = time_new_cloud.nanoseconds();

            odometry_pose_info->push_back(new_pose);

        }

        void saveKeyframePose()
        {
            // function to push new_pose to odometries, saved as quaternion and translation
            Eigen::Quaterniond reg_quarternion = keyframe_pose.orientation; 
            // reg_quarternion.normalize();
            Eigen::Vector3d reg_translation = keyframe_pose.position;



            PoseInfo new_pose;
            new_pose.qw = reg_quarternion.w();
            new_pose.qx = reg_quarternion.x();
            new_pose.qy = reg_quarternion.y();
            new_pose.qz = reg_quarternion.z();
            new_pose.x = reg_translation.x();
            new_pose.y = reg_translation.y();
            new_pose.z = reg_translation.z();
            new_pose.idx = keyframe_pose.frame_idx;
            new_pose.time = time_new_cloud.nanoseconds();

            keyframe_pose_info->push_back(new_pose);

        }



        void savePointCloud()
        {
            // pcl::PointCloud<PointType>::Ptr full(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr full = boost::make_shared<pcl::PointCloud<PointType>>();
            // full.reset(new pcl::PointCloud<PointType>());

            pcl::copyPointCloud(*cloud_in, *full);
            all_clouds.push_back(full);
        }
        


        template <typename PointT>
        void cropLocalMap(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out)
        {
            size_t j = 0;
            for (size_t i = 0; i < cloud_in.points.size(); ++i)
            {
                if (cloud_in.points[i].x < 0.0) // remove points with negative x
                    continue;
                cloud_out.points[j] = cloud_in.points[i]; // if they are beyond the threshold assign the point to cloud out
                j++;                                      // count exchanged points
            }

            if (j != cloud_in.points.size())
            {
                cloud_out.points.resize(j);
            }

            cloud_out.height = 1;
            cloud_out.width = static_cast<uint32_t>(j);
            cloud_out.is_dense = true;
        }


        // function adapted from lili-om
        void buildLocalMapAroundKeyFrame() 
        {
            size_t max_frames = local_map_init_frames_count_;
            if (local_map_width_ > 0)
                max_frames = local_map_width_;
   
            // int latest_keyframe_idx = keyframe_index.back();

            // IDEA: get closest frames from as KD knn search as the odometry points are saved as a point cloud, and use the neighbourhood clouds as local map. -> this requires and is close to loop closure 

            // Initialization
            // if (local_map->points.size() <= 1) { // if there are no point in the local map
            //     // ROS_INFO("Initialization for odometry local map");
            //     *local_map += *cloud_keyframe;
            //     RCLCPP_INFO(get_logger(), "Local Map initialized.");
            //     return;
            // }


            // else {
            //     RCLCPP_INFO(get_logger(), "Building Local Map.");
            //     local_map->clear(); 
            //     size_t j = 0;
            //     // for (size_t i : keyframe_index)
            //     for (auto i = keyframe_index.rbegin();  i != keyframe_index.rend(); i++ )
            //     {
            //         if ( j >= max_frames){
            //             break;
            //         }
            //         // *local_map += *transformCloud(all_clouds[*i], &odometry_pose_info->points[*i]);
            //         // *local_map += *pcltransformCloud(all_clouds[*i], keyframe_poses[*i]);
            //         pcl::PointCloud<PointType> transformed_cloud;
            //         pcl::transformPointCloudWithNormals<PointType>(*all_clouds[*i], transformed_cloud, keyframe_poses[*i]);
            //         *local_map += transformed_cloud;
            //         j++;
            //     }
            // }



            // If already more than max frames, pop the frames at the beginning
            size_t i = keyframe_poses.size()-1;
            if (recent_frames.size() >= max_frames ) {
                
                // RCLCPP_INFO(get_logger(), "Less than limit (%i) frames in local map, size: %i, odomsize: %i, index:%i", local_map_width_ ,recent_frames.size(),i, latest_frame_idx);
                // pcl::transformPointCloudWithNormals<PointType>(*all_clouds[i], *transformed_cloud, keyframe_poses[i]);
        
                // RCLCPP_INFO(get_logger(), "More than limit (%i) frames in local map, size:%i  index:%i", local_map_width_, recent_frames.size(), latest_frame_idx);
                // if (latest_frame_idx != odometry_pose_info->points.size() - 1) {

                recent_frames.pop_front();
                // latest_frame_idx = odometry_pose_info->points.size() - 1;
                // pcl::transformPointCloudWithNormals<PointType>(*all_clouds[i], *transformed_cloud, keyframe_poses[i]);
                // recent_frames.push_back(transformed_cloud);
            }
            pcl::PointCloud<PointType>::Ptr transformed_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
            pcl::transformPointCloudWithNormals<PointType>(*all_clouds[i], *transformed_cloud,  keyframe_pose.position, keyframe_pose.orientation);
            recent_frames.push_back(transformed_cloud);
            *inverse_local_map += *transformed_cloud;
            
            
            // local_map->clear();
            // local_map_ds->clear();
            // pcl::PointCloud<PointType>::Ptr downsampled_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
            // for (size_t i = 0; i < recent_frames.size(); ++i){

            //     down_size_filter_local_map.setInputCloud(recent_frames[i]);
            //     down_size_filter_local_map.filter(*downsampled_cloud);

            //     // *local_map += *recent_frames[i];
            //     *local_map_ds += *downsampled_cloud;
            // }


            local_map->clear();
            for (size_t i = 0; i < recent_frames.size(); ++i){
                *local_map += *recent_frames[i];
            }

            // try and downsample each keyframe in the local map instead of 
            if (ds_voxel_size_lc_ > 0.0){
                // local_map_ds->clear();
                down_size_filter_local_map.setInputCloud(local_map);
                down_size_filter_local_map.filter(*local_map_ds);
            } else {
                local_map_ds = local_map;
            }

            // transform the local map back to latest keyframe/"origo" for it to be matchable with the most recent cloud
            // Eigen::Matrix4d inverse_keyframe_pose = keyframe_poses[keyframe_poses.size() - 1].inverse();
            Pose inverse_keyframe_pose;
            inverse_keyframe_pose.orientation = keyframe_pose.orientation.inverse();
            inverse_keyframe_pose.position =  - keyframe_pose.position;

            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3,3>(0,0) = Eigen::Matrix3d(keyframe_pose.orientation);
            T.block<3,1>(0,3) = (keyframe_pose.position);
            T = T.inverse().eval();
            pcl::transformPointCloudWithNormals<PointType>(*local_map_ds, *local_map_ds, T);
            
            // cropLocalMap(*local_map_ds, *local_map_ds);


            publishLocalMap();


            // // transform the local map back to latest keyframe/"origo" for it to be matchable with the most recent cloud
            // // Eigen::Matrix4d inverse_keyframe_pose = keyframe_poses[keyframe_poses.size() - 1].inverse();
            // Pose inverse_keyframe_pose;
            // inverse_keyframe_pose.orientation = keyframe_pose.orientation.inverse();
            // inverse_keyframe_pose.position =  - keyframe_pose.position;

            // Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            // T.block<3,3>(0,0) = Eigen::Matrix3d(keyframe_pose.orientation);
            // T.block<3,1>(0,3) = (keyframe_pose.position);
            // T = T.inverse().eval();
            // pcl::transformPointCloudWithNormals<PointType>(*local_map_ds, *local_map_ds, T);
            
            // cropLocalMap(*local_map_ds, *local_map_ds);

            // pcl::PointCloud<PointType>::Ptr cropped_local_map = boost::make_shared<pcl::PointCloud<PointType>>();
            // pcl::copyPointCloud(*local_map_ds, *cropped_local_map);
            // pcl::transformPointCloudWithNormals<PointType>(*cropped_local_map, *cropped_local_map, T.inverse().eval());

            // sensor_msgs::msg::PointCloud2 msgs;
            // #ifndef __INTELLISENSE__ 
            // pcl::toROSMsg(*cropped_local_map, msgs);
            // #endif
            // // msgs.header.stamp = ros::Time().fromSec(time_new_cloud);
            // msgs.header.stamp = time_new_cloud; 
            // // msgs.header.frame_id = frame_id;
            // msgs.header.frame_id = "lidar_odom";
            // localcloud_pub->publish(msgs);
            // // RCLCPP_INFO(get_logger(), "Local map published!");
            // // global_cloud->clear();

            

        }

        void addToLocalMap()
        {   

            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3,3>(0,0) = Eigen::Matrix3d(keyframe_pose.orientation);
            T.block<3,1>(0,3) = (keyframe_pose.position);
            // T = T.inverse().eval();
            pcl::PointCloud<PointType>::Ptr transformed_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
            pcl::transformPointCloudWithNormals<PointType>(*all_clouds[keyframe_poses.size()-1], *transformed_cloud,  T);
            // pcl::transformPointCloudWithNormals<PointType>(*local_map, *local_map, T.inverse().eval());
            
            *inverse_local_map += *transformed_cloud;

            local_map->clear();
            pcl::transformPointCloudWithNormals<PointType>(*inverse_local_map, *local_map,  T.inverse().eval());

            // try and downsample each keyframe in the local map instead of 
            if (ds_voxel_size_lc_ > 0.0){
                // RCLCPP_INFO(get_logger(), "Downsample much..?");
                local_map_ds->clear();
                down_size_filter_local_map.setInputCloud(local_map);
                down_size_filter_local_map.filter(*local_map_ds);
            } else {
                local_map_ds = local_map;
            }
            // cropLocalMap(*local_map_ds, *local_map_ds);

            // pcl::transformPointCloudWithNormals<PointType>(*local_map_ds, *local_map_ds, T);
            

            publishLocalMap();
        }



        void publishLocalMap()
        {
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3,3>(0,0) = Eigen::Matrix3d(keyframe_pose.orientation);
            T.block<3,1>(0,3) = (keyframe_pose.position);

            pcl::PointCloud<PointType>::Ptr cropped_local_map = boost::make_shared<pcl::PointCloud<PointType>>();
            pcl::copyPointCloud(*local_map_ds, *cropped_local_map);
            pcl::transformPointCloudWithNormals<PointType>(*cropped_local_map, *cropped_local_map, T);

            sensor_msgs::msg::PointCloud2 msgs;
            #ifndef __INTELLISENSE__ 
            pcl::toROSMsg(*cropped_local_map, msgs);
            #endif
            // msgs.header.stamp = ros::Time().fromSec(time_new_cloud);
            msgs.header.stamp = time_new_cloud; 
            // msgs.header.frame_id = frame_id;
            msgs.header.frame_id = "lidar_odom";
            localcloud_pub->publish(msgs);
            // RCLCPP_INFO(get_logger(), "Local map published!");
            // global_cloud->clear();
        }




        pcl::PointCloud<PointType>::Ptr transformCloud(const pcl::PointCloud<PointType>::Ptr &cloudIn, PoseInfo * PointInfoIn)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut = boost::make_shared<pcl::PointCloud<PointType>>();;

            Eigen::Quaterniond quaternion(PointInfoIn->qw,
                                        PointInfoIn->qx,
                                        PointInfoIn->qy,
                                        PointInfoIn->qz);
            Eigen::Vector3d transition(PointInfoIn->x,
                                    PointInfoIn->y,
                                    PointInfoIn->z);

            int numPts = cloudIn->points.size();
            cloudOut->resize(numPts);

            for (int i = 0; i < numPts; ++i)
            {
                Eigen::Vector3d ptIn(cloudIn->points[i].x, cloudIn->points[i].y, cloudIn->points[i].z);
                Eigen::Vector3d ptOut = quaternion * ptIn + transition;

                Eigen::Vector3d normIn(cloudIn->points[i].normal_x, cloudIn->points[i].normal_y, cloudIn->points[i].normal_z);
                Eigen::Vector3d normOut = quaternion * normIn;

                PointType pt;
                pt.x = ptOut.x();
                pt.y = ptOut.y();
                pt.z = ptOut.z();
                pt.intensity = cloudIn->points[i].intensity;
                pt.curvature = cloudIn->points[i].curvature;
                pt.normal_x = normOut.x();
                pt.normal_y = normOut.y();
                pt.normal_z = normOut.z();

                cloudOut->points[i] = pt;
            }

            return cloudOut;
        }

        void publishTransformation()
        {
            Eigen::Quaterniond quarternion = last_odometry_transformation.rotation; 
            quarternion.normalize();
            Eigen::Vector3d translation = last_odometry_transformation.translation;


            transformation_geommsg.header.stamp = cloud_header.stamp;

            transformation_geommsg.pose.pose.orientation.w = quarternion.w();
            transformation_geommsg.pose.pose.orientation.x = quarternion.x();
            transformation_geommsg.pose.pose.orientation.y = quarternion.y();
            transformation_geommsg.pose.pose.orientation.z = quarternion.z();


            transformation_geommsg.pose.pose.position.x = translation.x();
            transformation_geommsg.pose.pose.position.y = translation.y();
            transformation_geommsg.pose.pose.position.z = translation.z();

            vector<double> cov_diag{translation_std_x*translation_std_x, translation_std_y* translation_std_y, translation_std_z * translation_std_z, 0.0,0.0,0.0};
            Eigen::MatrixXd cov_mat = createCovarianceEigen(cov_diag);
            Eigen::Matrix3d rot_mat(last_odometry_pose.orientation);
            rotateCovMatrix(cov_mat, rot_mat);
            setCovariance(transformation_geommsg.pose.covariance, cov_mat);

            // handle covariance depending on registration, eg. use deviation from guess 

            lidar_odometry_transformation_pub->publish(transformation_geommsg);

        }

        void rotateCovMatrix(Eigen::MatrixXd &cov_mat, Eigen::Matrix3d rot_mat)
        {
            Eigen::MatrixXd R(6,6);
            R.fill(0.0);
            R.block<3,3>(0,0) = rot_mat;
            R.block<3,3>(3,3) = rot_mat;

            // cov_mat = R.transpose() * cov_mat * R;
            cov_mat = R * cov_mat * R.transpose();
            // cov_mat.block<3,3>(0,0) = rot_mat.transpose() * cov_mat.block<3,3>(0,0) * rot_mat;
            // cov_mat.block<3,3>(3,3) = rot_mat * cov_mat.block<3,3>(3,3) * rot_mat.transpose();

            // for (int i =0; i<36; i++){
            //     if (cov_mat(i) < 1e-8){
            //         cov_mat(i) = 0.0;
            //     }
            // }
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

        void publishOdometry()
        {
            // PoseInfo latestPoseInfo = odometry_pose_info->points[latest_keyframe_idx - 1];
            PoseInfo latestPoseInfo = odometry_pose_info->points[odometry_pose_info->points.size() -1 ];
            odom.header.stamp = cloud_header.stamp;
            odom.pose.pose.orientation.w = latestPoseInfo.qw;
            odom.pose.pose.orientation.x = latestPoseInfo.qx;
            odom.pose.pose.orientation.y = latestPoseInfo.qy;
            odom.pose.pose.orientation.z = latestPoseInfo.qz;
            odom.pose.pose.position.x = latestPoseInfo.x;
            odom.pose.pose.position.y = latestPoseInfo.y;
            odom.pose.pose.position.z = latestPoseInfo.z;

            vector<double> cov_diag{translation_std_x*translation_std_x, translation_std_y* translation_std_y, translation_std_z * translation_std_z, 0.0,0.0,0.0};
            Eigen::MatrixXd cov_mat = createCovarianceEigen(cov_diag);
            // Eigen::Matrix3d rot_mat = last_odometry_pose.block<3,3>(0,0);
            Eigen::Matrix3d rot_mat(Eigen::Quaterniond(latestPoseInfo.qw, latestPoseInfo.qx, latestPoseInfo.qy, latestPoseInfo.qz ));
            rotateCovMatrix(cov_mat, rot_mat);
            setCovariance(odom.pose.covariance, cov_mat);
            // set scan dt in the top right position (idx 5) of 6x6 covariance matrix
            odom.pose.covariance[5] = scan_dt;

            // odom.twist.twist.linear.x // add the velocities in twist
            odometry_pub->publish(odom);


            // odom -> base_link transform
            geometry_msgs::msg::TransformStamped t_;
            // t_.header.stamp = this->get_clock()->now();
            t_.header.stamp = cloud_header.stamp;
            t_.header.frame_id = "odom";//"lidar_odom";
            t_.child_frame_id = "base_link"; // "livox_frame"

            t_.transform.rotation.w = odom.pose.pose.orientation.w;
            t_.transform.rotation.x = odom.pose.pose.orientation.x;
            t_.transform.rotation.y = odom.pose.pose.orientation.y;
            t_.transform.rotation.z = odom.pose.pose.orientation.z;
            t_.transform.translation.x = odom.pose.pose.position.x;            
            t_.transform.translation.y = odom.pose.pose.position.y;  
            t_.transform.translation.z = odom.pose.pose.position.z;
            tf_broadcaster_->sendTransform(t_);


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


       

        void publishKeyframeOdometry()
        {
            // PoseInfo latestPoseInfo = odometry_pose_info->points[latest_keyframe_idx - 1];
            PoseInfo latestPoseInfo = keyframe_pose_info->points[keyframe_pose_info->points.size() - 1 ];
            // odom.header.stamp = keyframe_pose_info.time;
            odom.pose.pose.orientation.w = latestPoseInfo.qw;
            odom.pose.pose.orientation.x = latestPoseInfo.qx;
            odom.pose.pose.orientation.y = latestPoseInfo.qy;
            odom.pose.pose.orientation.z = latestPoseInfo.qz;
            odom.pose.pose.position.x = latestPoseInfo.x;
            odom.pose.pose.position.y = latestPoseInfo.y;
            odom.pose.pose.position.z = latestPoseInfo.z;

            

            // odom.twist.twist.linear.x // add the velocities in twist

            keyframe_odometry_pub->publish(odom);


            // // odom -> base_link transform
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


            // geometry_msgs::msg::PoseStamped poseStamped;
            // poseStamped.header = odom.header;
            // poseStamped.pose = odom.pose.pose;
            // poseStamped.header.stamp = odom.header.stamp;
            // path.header.stamp = odom.header.stamp;
            // path.poses.push_back(poseStamped);
            // // path.header.frame_id = frame_id;
            // path.header.frame_id = "odom"; // "livox_frame"
            // path_pub->publish(path);
        }

        double toSec(builtin_interfaces::msg::Time header_stamp)
        {
            rclcpp::Time time = header_stamp;
            double nanoseconds = time.nanoseconds();

            return nanoseconds * 1e-9;
        }

        bool getNextInBuffer()
        {
            // get next frame in buffer..
            if (cloud_queue.size() <= 0) { 
                return false; 
            } else {
                current_cloud_msg = cloud_queue.front(); // puts the next cloud to variable current
                cloud_queue.pop_front(); // removes element from the queue

                // set header id and timestamp
                cloud_header = current_cloud_msg.header; 
                cloud_header.frame_id = frame_id;
    
                time_new_cloud = current_cloud_msg.header.stamp;
                current_scan_time = toSec(time_new_cloud);

                if (cloud_queue.size() > 0) {
                    RCLCPP_INFO(get_logger(), "Buffer size is: %i", cloud_queue.size());
                }
            } 

            // cloud_header = pcl_msg->header;
            fromROSMsg(current_cloud_msg, *cloud_in);
            scan_dt = cloud_in->points[cloud_in->points.size() -1].intensity - (int)cloud_in->points[cloud_in->points.size() -1].intensity;
            return true;
        }

        void run()
        {
            if (!getNextInBuffer()){
                return;
            }
            if (use_linear_undistortion_)
                undistortCLoudLinear();
            downSampleClouds();
            
            

            if (!system_initialized){
                // save first pose and point cloud and initialize the system

                initializeSystem();
                buildLocalMapAroundKeyFrame(); // only uses the first cloud for map.
                RCLCPP_INFO(get_logger(), "LOAM first frame is initialized");
                return;
            }
            
            latest_frame_idx++;

            if (local_map_init_frames_count_ > 0 && !init_map_built){
                if (keyframe_poses.size() <= size_t(local_map_init_frames_count_)) {
                    RCLCPP_INFO(get_logger(), "Adding frame to initial local map by assuming no movement! ");
                    savePose();
                    
                    publishCurrentCloud();
                    pushKeyframe();
                    
                    publishOdometry(); 
                    buildLocalMapAroundKeyFrame();
                    return;
                }
                if (keyframe_poses.size() >= size_t(local_map_init_frames_count_)) {
                    RCLCPP_INFO(get_logger(), "Building initial local map! ");
                    buildLocalMapAroundKeyFrame();
                    init_map_built = true;
                    // return;
                }
                // for (int i = 0; i < local_map_init_frames_count_; i++ ){
                // }

            }


            // RCLCPP_INFO(get_logger(), "This is from RUN: frame_id: %s,  time of PCL: %f, num points: %i", cloud_header.frame_id.c_str(), time_new_cloud.nanoseconds()/1e9, cloud_in_ds->points.size());
            
             
            

            // do icp with last key frame or  local map
            calculateInitialTransformationGuess();
            scanMatchRegistration(cloud_in_ds, local_map_ds);

            // regisrationICP(cloud_in_ds, cloud_keyframe_ds); // source, target - such that t = s*T, where T is the transformation. If source is the new frame the transformation found is the inverse of the odometry transformation 
            
            // regisrationICP(cloud_in_ds, local_map_ds, true); // source, target - such that t = s*T, where T is the transformation. If source is the new frame the transformation found is the inverse of the odometry transformation 
            
            // regisrationNDT(cloud_in_ds, local_map_ds); // source, target - such that t = s*T, where T is the transformation. If source is the new frame the transformation found is the inverse of the odometry transformation 
            // regisrationICP_gcip(cloud_in_ds, local_map_ds);
            
            // TODO: make check for degenerate estimated transformation 
            // RCLCPP_INFO(get_logger(), "did ICP happen?!...");


            updateOdometryPose(registration_transformation); 
            savePose(); 
            // publishTransformation(); // publishes the odometry transformation

            // send found transform to backend / kalman filter later
            publishCurrentCloud();

            // savePointCloud(); // testing this here
            publishOdometry(); 
            publishTransformation();

            if (newKeyframeRequired()) {
                pushKeyframe();
                pushRecentTranslation();
                // savePointCloud();
                if (local_map_width_ > 0) {
                    buildLocalMapAroundKeyFrame();
                } else {
                    addToLocalMap();
                }

            } else { // keyframe rejection ei. 

            }


        }



};


int main(int argc, char ** argv){

    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto lidar_odometry_node = std::make_shared<LidarOdometry>();
    executor.add_node(lidar_odometry_node);


    // rclcpp::spin(std::make_shared<LidarOdometry>());
    executor.spin();
    rclcpp::shutdown();
    return 0;


}