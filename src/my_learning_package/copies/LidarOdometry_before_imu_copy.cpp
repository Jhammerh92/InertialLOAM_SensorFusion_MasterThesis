// #include <memory>
// #include <chrono>

#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"
// #include "utils/common.h"
// #include "common.h"

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"


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

#include <cmath>
// #include <ctime>
// #include <array>
// #include <string>
#include <vector>
// #include <algorithm>
// #include <iostream>
// #include <fstream>
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


        // pose representation: [quaternion: w, x, y, z | transition: x, y, z]
        // double abs_pose[7];   //absolute pose from current frame to the first frame / odometry
        // double rel_pose[7];   //relative pose between two frames


        
        bool system_initialized = false;
        bool new_cloud_ready = false;

        size_t latest_frame_idx;
        size_t latest_keyframe_idx;

        double icp_fitness = 0.0;

        double cloud_scale;

        // transformation matrices
        Eigen::Matrix4d registration_transformation;
        Eigen::Matrix4d last_odometry_pose;
        Eigen::Matrix4d last_odometry_transformation;
        Eigen::Matrix4d odometry_transformation_guess;

        deque<Eigen::Matrix4d> keyframe_poses; // keyframes class instead?
        deque<size_t> keyframe_index; // keyframes class instead?



        // headers and header information
        rclcpp::Time time_new_cloud;
        std_msgs::msg::Header cloud_header;

        nav_msgs::msg::Odometry odom;
        nav_msgs::msg::Path path;

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
        pcl::PointCloud<PointType>::Ptr local_map_ds = boost::make_shared<pcl::PointCloud<PointType>>();

        deque<pcl::PointCloud<PointType>::Ptr> recent_frames;


        pcl::PointCloud<PointType>::Ptr global_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
        vector<boost::shared_ptr<pcl::PointCloud<PointType>>> all_clouds;

        // "point cloud" containing a specific type that has the odometry information
        pcl::PointCloud<PoseInfo>::Ptr odometry_pose_info = boost::make_shared<pcl::PointCloud<PoseInfo>>(); // add keyframe pose info?
        // pcl::PointCloud<pcl::PointXYZI>::Ptr odometry_pose_positions = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

        // subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

        //publishers
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_cloud_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalcloud_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr localcloud_pub;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        // parameters
        // ds_voxelsize
        // ds_voxelsize_lc
        float ds_voxel_size_;
        float ds_voxel_size_lc_;
        // strings
        std::string frame_id;
        // this->declare_parameter("my_parameter", "world");
        double keyframe_threshold_angle_;
        double keyframe_threshold_length_;
        double keyframe_threshold_fitness_;
        int keyframe_threshold_index_; // max frames between keyframes
        int icp_max_iterations_;
        double icp_max_correspondance_distance_;
        int local_map_width_;

        bool use_cloud_scale_for_ds_{};
        bool use_fusion_with_imu_{};

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

                declare_parameter("keyframe_treshold_fitness", 0.5); // 0.5 * icp_correspondance threshold
                get_parameter("keyframe_treshold_fitness", keyframe_threshold_fitness_);

                declare_parameter("keyframe_threshold_index", 0); 
                get_parameter("keyframe_threshold_index", keyframe_threshold_index_);

                declare_parameter("icp_max_iterations", 50); 
                get_parameter("icp_max_iterations", icp_max_iterations_);

                declare_parameter("icp_max_correspondance_distance", 0.5); 
                get_parameter("icp_max_correspondance_distance", icp_max_correspondance_distance_);

                declare_parameter("local_map_width", 20); 
                get_parameter("local_map_width", local_map_width_);

                declare_parameter("use_cloud_scale_for_ds", true); 
                get_parameter("use_cloud_scale_for_ds", use_cloud_scale_for_ds_);

                declare_parameter("use_fusion_with_imu", false); 
                get_parameter("use_fusion_with_imu", use_fusion_with_imu_);

                // RCLCPP_INFO(get_logger(), "ds_voxel_size in constructor is: %f", ds_voxel_size_);

                initializeParameters();
                allocateMemory();

                // setup callback groups

                // publsiher callback groud added?
                subscriber_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // create subscriber callback group
                rclcpp::SubscriptionOptions options; // create subscriver options
                options.callback_group = subscriber_cb_group_; // add callbackgroup to subscriber options

                run_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

                run_timer = this->create_wall_timer(5ms, std::bind(&LidarOdometry::run, this), run_cb_group_); // the process timer 

                pointcloud_sub_ = this->create_subscription<PC_msg>("/preprocessed_point_cloud", 100, std::bind(&LidarOdometry::pointCloudHandler, this, _1), options);
                // pointcloud_sub_ = this->create_subscription<PC_msg>("/surf_features", 100, std::bind(&LidarOdometry::PointCloudHandler, this, _1), options);


                pointcloud_pub = this->create_publisher<PC_msg>("/full_point_cloud_odom", 100);
                keyframe_cloud_pub = this->create_publisher<PC_msg>("/full_point_cloud_keyframe", 100);

                globalcloud_pub = this->create_publisher<PC_msg>("/global_point_cloud", 100);
                localcloud_pub = this->create_publisher<PC_msg>("/local_point_cloud", 100);

                // current odometry publisher
                odometry_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 100);
                path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 100);

                tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);



                
            }
        

            void pointCloudHandler(const PC_msg::SharedPtr lidar_cloud_msg )
            {   
                
                cloud_queue.push_back(*lidar_cloud_msg);


                // time_new_cloud = pcl_msg->header.stamp;
                // cloud_header = pcl_msg->header;
                // fromROSMsg(*pcl_msg, *cloud_in);

                // new_cloud_ready = true;
                // RCLCPP_INFO(get_logger(), "Header: %s,  time of PCL: %f, num points: %i", cloud_header.frame_id, time_new_cloud.nanoseconds()/1e9, cloud_in->points.size());

                // this->run(); // this could be run by a time gate maybe

            }

            void publishCurrentCloud()
            {   
                sensor_msgs::msg::PointCloud2 msgs;
                pcl::PointCloud<PointType> transformed_cloud;
                pcl::transformPointCloudWithNormals<PointType>(*cloud_in_ds, transformed_cloud, last_odometry_pose);
                #ifndef __INTELLISENSE__ 
                pcl::toROSMsg(transformed_cloud, msgs);
                #endif
                msgs.header.stamp = cloud_header.stamp;
                // msgs.header.frame_id = frame_id;
                msgs.header.frame_id = "lidar_odom";

                pointcloud_pub->publish(msgs);
                // RCLCPP_INFO(get_logger(), "Header: %s,  time of PCL: %f, num points: %i", cloud_header.frame_id, time_new_cloud.nanoseconds()/1e9, cloud_in->points.size());
            }

            void publishKeyframeCloud()
            {   
                sensor_msgs::msg::PointCloud2 msgs;
                #ifndef __INTELLISENSE__ 
                pcl::toROSMsg(*cloud_keyframe, msgs);
                #endif
                msgs.header.stamp = cloud_header.stamp;
                // msgs.header.frame_id = frame_id;
                msgs.header.frame_id = "lidar_odom";

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

                registration_transformation = Eigen::Matrix4d::Identity();
                last_odometry_pose = Eigen::Matrix4d::Identity();
                last_odometry_transformation = Eigen::Matrix4d::Identity();
                odometry_transformation_guess = Eigen::Matrix4d::Identity();

                // get_parameter("ds_voxel_size", ds_voxel_size_);

                frame_id = "lidar_odom";
                odom.header.frame_id = frame_id;

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


                
                // *global_cloud += *cloud_in;
                downSampleClouds();

                pushKeyframe(); // first keyframe pose is a identity matrix

                publishCurrentCloud();

                savePose();
                savePointCloud();
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
                local_map_ds.reset(new pcl::PointCloud<PointType>());
                global_cloud.reset(new pcl::PointCloud<PointType>());

                odometry_pose_info.reset(new pcl::PointCloud<PoseInfo>());
                // odometry_pose_positions.reset(new pcl::PointCloud<pcl::PointXYZI>());
            }

            void pushKeyframe()
            {
                RCLCPP_INFO(get_logger(), "New keyframe added! index: %i", latest_keyframe_idx);

                pcl::PointCloud<PointType>::Ptr full = boost::make_shared<pcl::PointCloud<PointType>>();
                // full.reset(new pcl::PointCloud<PointType>());
                pcl::copyPointCloud(*cloud_in, *full);
                *cloud_keyframe = *full;
                keyframe_poses.push_back(last_odometry_pose); 
                // keyframe_index.push_back(latest_frame_idx); // the index to actual frames
                keyframe_index.push_back(latest_keyframe_idx); // the keyframe ordered index
                publishKeyframeCloud();


                odometry_transformation_guess = last_odometry_transformation;
                registration_transformation = Eigen::Matrix4d::Identity(); // this has to be "resest" as it it is used to determine the next guess


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



            void downSampleClouds() {
                // down_size_filter_surf_map.setInputCloud(surf_from_map);
                // down_size_filter_surf_map.filter(*surf_from_map_ds);

                if (use_cloud_scale_for_ds_ == true) {
                    calcCloudScale();
                    RCLCPP_INFO(get_logger(), "cloud scale is: %f", cloud_scale);
                    float temp_leafsize = cloud_scale / 25.0; // 20 points ish from side to side
                    float temp_leafsize_lc = temp_leafsize / 4.0;
                    down_size_filter.setLeafSize(temp_leafsize, temp_leafsize, temp_leafsize);
                    down_size_filter_local_map.setLeafSize(temp_leafsize_lc, temp_leafsize_lc, temp_leafsize_lc);
                }

                cloud_in_ds->clear();
                down_size_filter.setInputCloud(cloud_in);
                down_size_filter.filter(*cloud_in_ds);

                cloud_keyframe_ds->clear();
                down_size_filter.setInputCloud(cloud_keyframe);
                down_size_filter.filter(*cloud_keyframe_ds);
            }

            template <typename PointT> double inline
            getMaxSegment (const pcl::PointCloud<PointT> &cloud)
            {
                double max_dist = std::numeric_limits<double>::min ();
                int i_min = -1, i_max = -1;

                for (size_t i = 0; i < cloud.points.size (); ++i)
                {
                    for (size_t j = i; j < cloud.points.size (); ++j)
                    {
                        // Compute the distance 
                        double dist = (cloud.points[i].getVector4fMap () - 
                                    cloud.points[j].getVector4fMap ()).squaredNorm ();
                        if (dist <= max_dist)
                        continue;

                        max_dist = dist;
                        i_min = i;
                        i_max = j;
                    }
                }

                if (i_min == -1 || i_max == -1)
                return (max_dist = std::numeric_limits<double>::min ());

                // pmin = cloud.points[i_min];
                // pmax = cloud.points[i_max];
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

            void calcCloudScale()
            {
                // cloud_scale = getMaxSegment(*cloud_in); // very slow! for a 10 000 point cloud it does 100 mil iterations ie. O(n²)
                cloud_scale = getMaxLeftRight(*cloud_in); // this only uses O(n) but is not guaranteed to find the largest distance, only a usable distance
            }

        
            void regisrationICP(boost::shared_ptr<pcl::PointCloud<PointType>> source, const boost::shared_ptr<pcl::PointCloud<PointType>> target)
            {
                typedef pcl::registration::TransformationEstimationPointToPlane<PointType, PointType> PointToPlane;
                // maybe not setup the icp on every use
                // pcl::IterativeClosestPoint<PointType, PointType> icp;
                // pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp;
                pcl::IterativeClosestPointWithNormals<PointType, PointType> icp;
                // pcl::IterativeClosestPointNonLinear<PointType, PointType> icp;

                // pcl::NormalDistributionsTransform<PointType, PointType> icp;
                // icp.setResolution(1.0);

                boost::shared_ptr<PointToPlane> p2p(new PointToPlane);

                // double max_correspondance_distance = 0.05;
                double max_correspondance_distance = icp_max_correspondance_distance_;

                icp.setTransformationEstimation(p2p);

                icp.setInputSource(source);
                icp.setInputTarget(target);

                // course estimation first to get a better initial estimate of transformation
                icp.setMaxCorrespondenceDistance(20*max_correspondance_distance);
                icp.setMaximumIterations(10);
                icp.setTransformationEpsilon(1e-2);
                icp.setEuclideanFitnessEpsilon(1e-6);
                // icp.setRANSACIterations(10);

                // icp_nl.setInputSource(source);
                // icp_nl.setInputTarget(target);

                // icp_nl.setMaxCorrespondenceDistance(0.1);
                // icp_nl.setMaximumIterations(50);
                // icp_nl.setTransformationEpsilon(1e-6);
                // icp_nl.setEuclideanFitnessEpsilon(1e-2);

                pcl::PointCloud<PointType>::Ptr aligned_cloud(new pcl::PointCloud<PointType>());
                // Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity(); // should be a better guess anyway
                icp.align(*aligned_cloud, odometry_transformation_guess.cast<float>());

                //get icp transformation and use 
                Eigen::Matrix4f registration_transform;
                registration_transform = icp.getFinalTransformation();//.cast<double>(); // why cast to double??

                // second iteration with finer correspondence limit

                icp.setMaxCorrespondenceDistance(max_correspondance_distance);
                icp.setMaximumIterations(icp_max_iterations_);
                icp.setTransformationEpsilon(1e-6);
                icp.align(*aligned_cloud, registration_transform);

                registration_transform = icp.getFinalTransformation();//.cast<double>(); // why cast to double??

                // icp_nl.align(*aligned_cloud, registration_transform);
                // // Eigen::Matrix4d registration_transform;
                // registration_transform = icp_nl.getFinalTransformation();//.cast<double>(); // why cast to double??
                
                Eigen::Matrix4d registration_transform_double = registration_transform.cast<double>();
                
                registration_transformation = registration_transform_double;

                // make transform matrix into quaternion and vector
                // Eigen::Quaterniond reg_quarternion(registration_transform_double.block<3, 3>(0, 0)); 
                // Eigen::Vector3d reg_translation(registration_transform_double.block<3, 1>(0, 3));

                icp_fitness = icp.getFitnessScore();
                
                RCLCPP_INFO(get_logger(), "ICP has converged: %i,  score: %f", icp.hasConverged(), icp_fitness);

                // std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                // icp.getFitnessScore() << std::endl;
                // std::cout << icp.getFinalTransformation() << std::endl;


                // send found pose to rel_pose

                // // create function to push new_via abs_pose
                // PoseInfo new_pose;
                // rel_pose[0] = reg_quarternion.w();
                // rel_pose[1] = reg_quarternion.x();
                // rel_pose[2] = reg_quarternion.y();
                // rel_pose[3] = reg_quarternion.z();
                // rel_pose[4] = reg_translation.x();
                // rel_pose[5] = reg_translation.y();
                // rel_pose[6] = reg_translation.z();
                // new_pose.idx = odometry_pose_info->points.size();
                // new_pose.time = time_new_cloud.nanoseconds();

                // odometry_pose_info->push_back(new_pose);


                return;
            }



            void updateOdometryPose()
            {   
                Eigen::Matrix4d next_odometry_pose = keyframe_poses.back() * registration_transformation; // the next evaluted odometry pose, not neccesarily a keyframe
                last_odometry_transformation =  last_odometry_pose * next_odometry_pose.inverse(); // transformation between 2 consecutive odometry poses
                odometry_transformation_guess = registration_transformation * last_odometry_transformation; // prediction of the next transformation using the previuosly found transformation - used for ICP initial guess
                last_odometry_pose = next_odometry_pose;

            }

            bool newKeyframeRequired()
            {
                // make transform matrix into quaternion and vector
                Eigen::Quaterniond reg_quarternion(registration_transformation.block<3, 3>(0, 0)); 
                Eigen::Vector3d reg_translation(registration_transformation.block<3, 1>(0, 3));

                // length of translation
                double length = reg_translation.norm();

                // angle of rotation
                // 0.1 rad is approx 5.73 deg
                Eigen::AngleAxisd angleAxis = Eigen::AngleAxisd(reg_quarternion);
                double angle = angleAxis.angle()*180.0/M_PI;

                RCLCPP_DEBUG(get_logger(), "found transform parameters: length: %f, angle: %f, fitness: %f", length, angle, icp_fitness);
                RCLCPP_DEBUG(get_logger(), "threshold parameters: length: %f, angle: %f, fitness: %f", keyframe_threshold_length_, keyframe_threshold_angle_, keyframe_threshold_fitness_);
                RCLCPP_DEBUG(get_logger(), "should this be true: %i", (length > keyframe_threshold_length_ || angle > keyframe_threshold_angle_ || icp_fitness > keyframe_threshold_fitness_ ));


                // and get icp fitness

                // assert if any is beyong thresholds
  
                return (length > keyframe_threshold_length_ || angle > keyframe_threshold_angle_ || icp_fitness > keyframe_threshold_fitness_ );
            }


            void savePose()
            {
                // function to push new_pose to odometries, saved quaternion and translation

                Eigen::Quaterniond reg_quarternion(last_odometry_pose.block<3, 3>(0, 0)); 
                reg_quarternion.normalize();
                Eigen::Vector3d reg_translation(last_odometry_pose.block<3, 1>(0, 3));

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



            void savePointCloud()
            {
                // pcl::PointCloud<PointType>::Ptr full(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr full = boost::make_shared<pcl::PointCloud<PointType>>();
                // full.reset(new pcl::PointCloud<PointType>());

                pcl::copyPointCloud(*cloud_in, *full);
                all_clouds.push_back(full);
            }


            void publishGlobalCloud()
            {
                // build global cloud from odometry transforms and recordeded (keyframe) clouds

                // publish the cloud, maybe with a gate-timer to not hog bandwidth and memory

                // clear the cloud out of RAM.
                // int mapping_interval = 1;
                for (unsigned i = 0; i < odometry_pose_info->points.size(); i++)
                {
                    Eigen::Quaterniond q_po(odometry_pose_info->points[i].qw,
                                            odometry_pose_info->points[i].qx,
                                            odometry_pose_info->points[i].qy,
                                            odometry_pose_info->points[i].qz);

                    Eigen::Vector3d t_po(odometry_pose_info->points[i].x,
                                        odometry_pose_info->points[i].y,
                                        odometry_pose_info->points[i].z);

                    // Eigen::Quaterniond q_tmp = q_po * q_bl;
                    // Eigen::Vector3d t_tmp = q_po * t_bl + t_po;

                    // PoseInfo Ttmp;
                    // // Ttmp.qw = q_tmp.w();
                    // // Ttmp.qx = q_tmp.x();
                    // // Ttmp.qy = q_tmp.y();
                    // // Ttmp.qz = q_tmp.z();
                    // // Ttmp.x = t_tmp.x();
                    // // Ttmp.y = t_tmp.y();
                    // // Ttmp.z = t_tmp.z();

                    // Ttmp.qw = q_po.w();
                    // Ttmp.qx = q_po.x();
                    // Ttmp.qy = q_po.y();
                    // Ttmp.qz = q_po.z();
                    // Ttmp.x = t_po.x();
                    // Ttmp.y = t_po.y();
                    // Ttmp.z = t_po.z();

                    pcl::PointCloud<PointType> transformed_cloud;
                    pcl::transformPointCloudWithNormals<PointType>(*all_clouds[i], transformed_cloud, t_po, q_po);
                    *global_cloud += transformed_cloud;

                    // *global_cloud += *pcl::transformCloud(all_clouds[i], &Ttmp);
                }

                sensor_msgs::msg::PointCloud2 msgs;
                #ifndef __INTELLISENSE__ 
                pcl::toROSMsg(*global_cloud, msgs);
                #endif
                // msgs.header.stamp = ros::Time().fromSec(time_new_cloud);
                msgs.header.stamp = time_new_cloud; 
                msgs.header.frame_id = "global_map";
                globalcloud_pub->publish(msgs);
                global_cloud->clear();
                // global_map_ds->clear();

            }

            // function adapted from lili-om
            void buildLocalMapAroundKeyFrame() 
            {
                
                
                size_t max_frames = local_map_width_;
                // int latest_keyframe_idx = keyframe_index.back();

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
                pcl::PointCloud<PointType>::Ptr transformed_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
                if (recent_frames.size() < max_frames ) {
                    
                    RCLCPP_INFO(get_logger(), "Less than 20 frames in local map, size: %i, odomsize: %i, index:%i", recent_frames.size(),i, latest_frame_idx);
                    pcl::transformPointCloudWithNormals<PointType>(*all_clouds[i], *transformed_cloud, keyframe_poses[i]);
                    recent_frames.push_back(transformed_cloud);
                } 
                else {
                    RCLCPP_INFO(get_logger(), "More than 20 frames in local map, size:%i  index:%i", recent_frames.size(), latest_frame_idx);
                    // if (latest_frame_idx != odometry_pose_info->points.size() - 1) {

                    recent_frames.pop_front();
                    // latest_frame_idx = odometry_pose_info->points.size() - 1;
                    pcl::transformPointCloudWithNormals<PointType>(*all_clouds[i], *transformed_cloud, keyframe_poses[i]);
                    recent_frames.push_back(transformed_cloud);
                }
                
                
                local_map->clear();
                for (size_t i = 0; i < recent_frames.size(); ++i){
                    *local_map += *recent_frames[i];
                }



                local_map_ds->clear();
                down_size_filter_local_map.setInputCloud(local_map);
                down_size_filter_local_map.filter(*local_map_ds);


                sensor_msgs::msg::PointCloud2 msgs;
                #ifndef __INTELLISENSE__ 
                pcl::toROSMsg(*local_map_ds, msgs);
                #endif
                // msgs.header.stamp = ros::Time().fromSec(time_new_cloud);
                msgs.header.stamp = time_new_cloud; 
                msgs.header.frame_id = "global_map";
                localcloud_pub->publish(msgs);
                // global_cloud->clear();

                // transform back to origo
                Eigen::Matrix4d inverse_keyframe_pose = keyframe_poses[keyframe_poses.size() - 1].inverse();
                pcl::transformPointCloudWithNormals<PointType>(*local_map_ds, *local_map_ds, inverse_keyframe_pose );

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
                odometry_pub->publish(odom);

                // odom -> base_link transform
                geometry_msgs::msg::TransformStamped t_;
                // t_.header.stamp = this->get_clock()->now();
                t_.header.stamp = cloud_header.stamp;
                t_.header.frame_id = "odom";
                t_.child_frame_id = "base_link"; // should this be livox frame

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
                path.header.frame_id = frame_id;
                path_pub->publish(path);
            }

            // void publishPath()
            // {
            //     PoseInfo latestPoseInfo = odometry_pose_info->points[latest_keyframe_idx - 1];
            //     odom.header.stamp = cloud_header.stamp;
            //     odom.pose.pose.orientation.w = latestPoseInfo.qw;
            //     odom.pose.pose.orientation.x = latestPoseInfo.qx;
            //     odom.pose.pose.orientation.y = latestPoseInfo.qy;
            //     odom.pose.pose.orientation.z = latestPoseInfo.qz;
            //     odom.pose.pose.position.x = latestPoseInfo.x;
            //     odom.pose.pose.position.y = latestPoseInfo.y;
            //     odom.pose.pose.position.z = latestPoseInfo.z;
            //     // odometry_pub->publish(odom);

            //     geometry_msgs::msg::PoseStamped poseStamped;
            //     poseStamped.header = odom.header;
            //     poseStamped.pose = odom.pose.pose;
            //     poseStamped.header.stamp = odom.header.stamp;
            //     path.header.stamp = odom.header.stamp;
            //     path.poses.push_back(poseStamped);
            //     path.header.frame_id = frame_id;
            //     path_pub->publish(path);
            // }

            bool getNextInBuffer()
            {
                // get next frame in buffer..
                if (cloud_queue.size() <= 0) { 
                    return false; 
                } else {
                    RCLCPP_INFO(get_logger(), "Buffer size is: %i", cloud_queue.size());
                    current_cloud_msg = cloud_queue.front(); // puts the next cloud to variable current
                    cloud_queue.pop_front(); // removes element from the queue

                    // set header id and timestamp
                    cloud_header = current_cloud_msg.header; 
                    cloud_header.frame_id = frame_id;
        
                    // time_scan_next = current_cloud_msg.header.stamp; // why take time from the next? and should it be double?
                    time_new_cloud = current_cloud_msg.header.stamp;

                } 

                // cloud_header = pcl_msg->header;
                fromROSMsg(current_cloud_msg, *cloud_in);
                return true;
            }

            void run()
            {
                if (!getNextInBuffer()){
                    return;
                }


                if (!system_initialized){
                    // save first pose and point cloud and initialize the system

                    initializeSystem();
                    buildLocalMapAroundKeyFrame();
                    RCLCPP_INFO(get_logger(), "LOAM first frame is initialized");
                    return;
                }


                downSampleClouds();

                RCLCPP_INFO(get_logger(), "This is from RUN: frame_id: %s,  time of PCL: %f, num points: %i", cloud_header.frame_id.c_str(), time_new_cloud.nanoseconds()/1e9, cloud_in_ds->points.size());
                
                latest_frame_idx++;
                // buildLocalMap();

                // do icp with last key frame or rather local map
                // regisrationICP(cloud_in_ds, cloud_keyframe_ds); // source, target - such that t = s*T, where T is the transformation. If source is the new frame the transformation found is the inverse of the odometry transformation 
                regisrationICP(cloud_in_ds, local_map_ds); // source, target - such that t = s*T, where T is the transformation. If source is the new frame the transformation found is the inverse of the odometry transformation 

                updateOdometryPose();

                // *cloud_prev_ds = *cloud_in_ds;
                
                
                publishCurrentCloud();

                if (newKeyframeRequired() == true) {
                    pushKeyframe();
                    
                    savePose();
                    savePointCloud();
                    publishOdometry(); 


                    buildLocalMapAroundKeyFrame();


                    // publishGlobalCloud();

                }

                // buildLocalMapAroundKeyFrame();





                // RCLCPP_INFO(get_logger(), "ICP rotation: w: %f x: %f y: %f z: %f, tx %f, ty %f, tz %f", rel_pose[0], rel_pose[1], rel_pose[2], rel_pose[3], rel_pose[4], rel_pose[5], rel_pose[6] );
                





                


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