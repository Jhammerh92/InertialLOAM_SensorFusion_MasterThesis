// #include <memory>
// #include <chrono>

#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"
// #include "utils/common.h"
// #include "common.h"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/PointCloud2.h>

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

// #include <cmath>
// #include <ctime>
// #include <array>
// #include <string>
#include <vector>
// #include <algorithm>
// #include <iostream>
// #include <fstream>
// #include <thread>
// #include <mutex>
// #include <queue>
// #include <assert.h>



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


class LidarOdometry : public rclcpp::Node 
{

    public:
        LidarOdometry()
            : Node("lidar_odometry")
            {

                initializeParameters();
                allocateMemory();

                // pointcloud_sub_ = this->create_subscription<PC_msg::SharedPtr>("/livox/lidar", 100, [this](PC_msg::SharedPtr msg) { PointCloudHandler(msg); });
                pointcloud_sub_ = this->create_subscription<PC_msg>("/full_point_cloud", 100, std::bind(&LidarOdometry::PointCloudHandler, this, _1));
                // pointcloud_sub_ = this->create_subscription<PC_msg>("/edge_features", 100, std::bind(&LidarOdometry::PointCloudHandler, this, _1));
                // pointcloud_sub_ = this->create_subscription<PC_msg>("/surf_features", 100, std::bind(&LidarOdometry::PointCloudHandler, this, _1));

                pointcloud_pub = this->create_publisher<PC_msg>("/full_point_cloud_odom", 100);

                globalcloud_pub = this->create_publisher<PC_msg>("/global_point_cloud", 100);
                localcloud_pub = this->create_publisher<PC_msg>("/local_point_cloud", 100);
            }
        

            void PointCloudHandler(const PC_msg::SharedPtr pcl_msg )
            {   
                // sensor_msgs::msg::PointCloud2 msgs;
                // msgs = *pcl_msg;
                // msgs.header.frame_id = frame_id;
                // pointcloud_pub->publish(msgs);

                time_new_cloud = pcl_msg->header.stamp;
                cloud_header = pcl_msg->header;
                fromROSMsg(*pcl_msg, *cloud_in);

                // pcl::fromPCLPointCloud2(cloud_in, )
                // pcl::PointCloud<PointType>::Ptr cloud_in = boost::make_shared<pcl::PointCloud<PointType>>();

                // pcl::fromROSMsg(*pcl_msg, *cloud_in);
                // pcl_conversions::toPCL(*pointCloudIn, *cloud_in)

                new_cloud_ready = true;
                // RCLCPP_INFO(get_logger(), "Header: %s,  time of PCL: %f, num points: %i", cloud_header.frame_id, time_new_cloud.nanoseconds()/1e9, cloud_in->points.size());

                this->run();

            }

            void publishCurrentCloud()
            {   
                sensor_msgs::msg::PointCloud2 msgs;
              

                pcl::PointCloud<PointType> transformed_cloud;

                Eigen::Quaternionf quaternion(abs_pose[0],
                                                  abs_pose[1],
                                                  abs_pose[2],
                                                  abs_pose[3]);
                Eigen::Vector3f translation(abs_pose[4],
                                               abs_pose[5],
                                               abs_pose[6]);

                pcl::transformPointCloudWithNormals<PointType>(*cloud_in, transformed_cloud, translation, quaternion);

                #ifndef __INTELLISENSE__ 
                pcl::toROSMsg(transformed_cloud, msgs);
                #endif

                msgs.header.stamp = cloud_header.stamp;
                // msgs.header.frame_id = frame_id;
                msgs.header.frame_id = "lidar_odom";

                pointcloud_pub->publish(msgs);


                // RCLCPP_INFO(get_logger(), "Header: %s,  time of PCL: %f, num points: %i", cloud_header.frame_id, time_new_cloud.nanoseconds()/1e9, cloud_in->points.size());


            }

            void initializeParameters()
            {
                RCLCPP_INFO(get_logger(), "Initializing Parameters..");
                abs_pose[0] = 1;
                rel_pose[0] = 1;

                for (int i = 1; i < 7; ++i) {
                    abs_pose[i] = 0;
                    rel_pose[i] = 0;
                }

                frame_id = "lidar_odom";

                double ds_voxel_size = 0.1f;
                down_size_filter.setLeafSize(ds_voxel_size, ds_voxel_size, ds_voxel_size);

                latest_frame_idx = 0;

                system_initialized = false;

            }


            void initializeSystem()
            {   
                RCLCPP_INFO(get_logger(), "Initializing System..");

                downSampleCloud();

                *cloud_prev_ds = *cloud_in_ds;
                *global_cloud += *cloud_in_ds;

                // publish the first cloud to the global map
                sensor_msgs::msg::PointCloud2 msgs;
                #ifndef __INTELLISENSE__ 
                pcl::toROSMsg(*global_cloud, msgs);
                #endif
                msgs.header.stamp = cloud_header.stamp;
                msgs.header.frame_id = "global_map";
                globalcloud_pub->publish(msgs);

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
                global_cloud.reset(new pcl::PointCloud<PointType>());

                odometry_pose_info.reset(new pcl::PointCloud<PoseInfo>());
                odometry_pose_positions.reset(new pcl::PointCloud<pcl::PointXYZI>());
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



            void downSampleCloud() {
                // down_size_filter_surf_map.setInputCloud(surf_from_map);
                // down_size_filter_surf_map.filter(*surf_from_map_ds);

                cloud_in_ds->clear();
                down_size_filter.setInputCloud(cloud_in);
                down_size_filter.filter(*cloud_in_ds);
            }

        
            void regisrationICP(boost::shared_ptr<pcl::PointCloud<PointType>> source, const boost::shared_ptr<pcl::PointCloud<PointType>> target)
            {
                typedef pcl::registration::TransformationEstimationPointToPlane<PointType, PointType> PointToPlane;
                // maybe not setup the icp on every use
                // pcl::IterativeClosestPoint<PointType, PointType> icp;
                // pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp;
                pcl::IterativeClosestPointWithNormals<PointType, PointType> icp;
                // pcl::IterativeClosestPointNonLinear<PointType, PointType> icp;
                boost::shared_ptr<PointToPlane> p2p(new PointToPlane);

                icp.setTransformationEstimation(p2p);

                icp.setInputSource(source);
                icp.setInputTarget(target);

                icp.setMaxCorrespondenceDistance(10*0.1);
                icp.setMaximumIterations(15);
                icp.setTransformationEpsilon(1e-6);
                icp.setEuclideanFitnessEpsilon(1e-6);
                // icp.setRANSACIterations(10);

                // icp_nl.setInputSource(source);
                // icp_nl.setInputTarget(target);

                // icp_nl.setMaxCorrespondenceDistance(0.1);
                // icp_nl.setMaximumIterations(50);
                // icp_nl.setTransformationEpsilon(1e-6);
                // icp_nl.setEuclideanFitnessEpsilon(1e-2);

                pcl::PointCloud<PointType>::Ptr aligned_cloud(new pcl::PointCloud<PointType>());
                Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity(); // should be a better guess anyway
                icp.align(*aligned_cloud, init_guess);

                //get icp transformation and use 
                Eigen::Matrix4f registration_transform;
                registration_transform = icp.getFinalTransformation();//.cast<double>(); // why cast to double??


                // second iteration with finer correspondence limit
                // icp.setInputSource(source);

                icp.setMaxCorrespondenceDistance(0.1);
                icp.setMaximumIterations(100);
                icp.align(*aligned_cloud, registration_transform);

                registration_transform = icp.getFinalTransformation();//.cast<double>(); // why cast to double??

                // icp_nl.align(*aligned_cloud, registration_transform);
                // // Eigen::Matrix4d registration_transform;
                // registration_transform = icp_nl.getFinalTransformation();//.cast<double>(); // why cast to double??
                
                Eigen::Matrix4d registration_transform_double = registration_transform.cast<double>();
                
                // make transform matrix into quaternion and vector
                Eigen::Quaterniond reg_quarternion(registration_transform_double.block<3, 3>(0, 0)); 
                Eigen::Vector3d reg_translation(registration_transform_double.block<3, 1>(0, 3));


                
                RCLCPP_INFO(get_logger(), "ICP has converged: %i,  score: %f", icp.hasConverged(), icp.getFitnessScore());

                // std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                // icp.getFitnessScore() << std::endl;
                // std::cout << icp.getFinalTransformation() << std::endl;


                // send found pose to rel_pose

                // // create function to push new_via abs_pose
                // PoseInfo new_pose;
                rel_pose[0] = reg_quarternion.w();
                rel_pose[1] = reg_quarternion.x();
                rel_pose[2] = reg_quarternion.y();
                rel_pose[3] = reg_quarternion.z();
                rel_pose[4] = reg_translation.x();
                rel_pose[5] = reg_translation.y();
                rel_pose[6] = reg_translation.z();
                // new_pose.idx = odometry_pose_info->points.size();
                // new_pose.time = time_new_cloud.nanoseconds();

                // odometry_pose_info->push_back(new_pose);


                return;
            }

            void updateAbsPose()
            {   
                Eigen::Quaterniond quaternion_rel(rel_pose[0],
                                                  rel_pose[1],
                                                  rel_pose[2],
                                                  rel_pose[3]);
                Eigen::Vector3d translation_rel(rel_pose[4],
                                               rel_pose[5],
                                               rel_pose[6]);


                Eigen::Quaterniond quaternion_abs(abs_pose[0],
                                               abs_pose[1],
                                               abs_pose[2],
                                               abs_pose[3]);
                Eigen::Vector3d translation_abs(abs_pose[4],
                                            abs_pose[5],
                                            abs_pose[6]);

                quaternion_abs =  quaternion_rel * quaternion_abs;
                translation_abs += translation_rel;
                // translation_abs += quaternion_rel * translation_rel;

                abs_pose[0] = quaternion_abs.w();
                abs_pose[1] = quaternion_abs.x();
                abs_pose[2] = quaternion_abs.y();
                abs_pose[3] = quaternion_abs.z();
                abs_pose[4] = translation_abs.x();
                abs_pose[5] = translation_abs.y();
                abs_pose[6] = translation_abs.z();

            }


            void savePose()
            {
                // function to push new_pose via abs_pose
                PoseInfo new_pose;
                new_pose.qw = abs_pose[0];
                new_pose.qx = abs_pose[1];
                new_pose.qy = abs_pose[2];
                new_pose.qz = abs_pose[3];
                new_pose.x = abs_pose[4];
                new_pose.y = abs_pose[5];
                new_pose.z = abs_pose[6];
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
                int mapping_interval = 1;
                for (unsigned i = 0; i < odometry_pose_info->points.size(); i = i + mapping_interval)
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

                    PoseInfo Ttmp;
                    // Ttmp.qw = q_tmp.w();
                    // Ttmp.qx = q_tmp.x();
                    // Ttmp.qy = q_tmp.y();
                    // Ttmp.qz = q_tmp.z();
                    // Ttmp.x = t_tmp.x();
                    // Ttmp.y = t_tmp.y();
                    // Ttmp.z = t_tmp.z();

                    Ttmp.qw = q_po.w();
                    Ttmp.qx = q_po.x();
                    Ttmp.qy = q_po.y();
                    Ttmp.qz = q_po.z();
                    Ttmp.x = t_po.x();
                    Ttmp.y = t_po.y();
                    Ttmp.z = t_po.z();


                    *global_cloud += *transformCloud(all_clouds[i], &Ttmp);
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

            // function taken from lili-om
            void buildLocalMap() 
            {
                // local_map->clear(); // why is this clear here? 
                // Initialization
                if (local_map->points.size() <= 1) { // if there are no point in the local map
                    // ROS_INFO("Initialization for odometry local map");
                    *local_map += *cloud_in;
                    RCLCPP_INFO(get_logger(), "Local Map initialized.");
                    return;
                }

                // If already more then 20 frames, pop the frames at the beginning
                if (recent_frames.size() < 20) {
                    size_t i = odometry_pose_info->points.size() - 1;
                    RCLCPP_INFO(get_logger(), "Less than 20 frames in local map, size: %i, odomsize: %i, index:%i", recent_frames.size(),i, latest_frame_idx);
                    recent_frames.push_back(transformCloud(all_clouds[i], &odometry_pose_info->points[i]));
                    // recent_frames.push_back(all_clouds[i]); // force no transform
                } 
                else {
                    RCLCPP_INFO(get_logger(), "More than 20 frames in local map, size:%i  index:%i", recent_frames.size(), latest_frame_idx);
                    if (latest_frame_idx != odometry_pose_info->points.size() - 1) {
                        recent_frames.pop_front();
                        latest_frame_idx = odometry_pose_info->points.size() - 1;
                        recent_frames.push_back(transformCloud(all_clouds[latest_frame_idx], &odometry_pose_info->points[latest_frame_idx]));
                    }
                }

                RCLCPP_INFO(get_logger(), "Building Local Map.");
                local_map->clear(); // clear should be here
                for (size_t i = 0; i < recent_frames.size(); ++i)
                    *local_map += *recent_frames[i];




                sensor_msgs::msg::PointCloud2 msgs;
                #ifndef __INTELLISENSE__ 
                pcl::toROSMsg(*local_map, msgs);
                #endif
                // msgs.header.stamp = ros::Time().fromSec(time_new_cloud);
                msgs.header.stamp = time_new_cloud; 
                msgs.header.frame_id = "global_map";
                localcloud_pub->publish(msgs);
                // global_cloud->clear();
            }

            // void transformPoint(PointType const *const pi, PointType *const po, Eigen::Quaterniond quaternion, Eigen::Vector3d transition)
            // {
            //     Eigen::Vector3d ptIn(pi->x, pi->y, pi->z);
            //     Eigen::Vector3d ptOut = quaternion * ptIn + transition;

            //     // Eigen::Vector3d normIn(pi->normal_x, pi->normal_y, pi->normal_z);
            //     // Eigen::Vector3d normOut = quaternion * normIn;

            //     po->x = ptOut.x();
            //     po->y = ptOut.y();
            //     po->z = ptOut.z();
            //     po->intensity = pi->intensity;
            //     // po->curvature = pi->curvature;
            //     // po->normal_x = normOut.x();
            //     // po->normal_y = normOut.y();
            //     // po->normal_z = normOut.z();
            // }

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



            void run()
            {
                // check for first frame / initialize
                // check for new frame from timestamp

                if (new_cloud_ready){ 

                    new_cloud_ready = false;

                } else {

                    return;
                }


                downSampleCloud();

                if (!system_initialized){
                    // save first pose and point cloud and initialize the system
                    savePose();
                    savePointCloud();
                    initializeSystem();
                    buildLocalMap();
                    RCLCPP_INFO(get_logger(), "LOAM first frame is initialized");
                    return;
                }


                RCLCPP_INFO(get_logger(), "This is from RUN: frame_id: %s,  time of PCL: %f, num points: %i", cloud_header.frame_id.c_str(), time_new_cloud.nanoseconds()/1e9, cloud_in->points.size());
                buildLocalMap();

                // do icp with previous frame
                regisrationICP(cloud_in_ds, local_map); // source, target - such that t = s*T, where T is the transformation. If source is the new frame the transformation found is the inverse of the odometry transformation 

                updateAbsPose();

                savePose();
                savePointCloud();

                // *cloud_prev_ds = *cloud_in_ds;

                publishCurrentCloud();
                publishGlobalCloud();




                RCLCPP_INFO(get_logger(), "ICP rotation: w: %f x: %f y: %f z: %f", rel_pose[0], rel_pose[1], rel_pose[2], rel_pose[3] );
                





                


            }


    private:
        // pose representation: [quaternion: w, x, y, z | transition: x, y, z]
        double abs_pose[7];   //absolute pose from current frame to the first frame / odometry
        double rel_pose[7];   //relative pose between two frames

        
        bool system_initialized = false;
        bool new_cloud_ready = false;

        size_t latest_frame_idx;

        // strings
        std::string frame_id;

        // headers and header information
        rclcpp::Time time_new_cloud;
        std_msgs::msg::Header cloud_header;

        // pcl filters downsampling
        pcl::VoxelGrid<PointType> down_size_filter;

        //point clouds and vector containing pointclouds
        pcl::PointCloud<PointType>::Ptr cloud_in = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr cloud_in_ds = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr cloud_prev = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr cloud_prev_ds = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr cloud_keyframe = boost::make_shared<pcl::PointCloud<PointType>>();

        pcl::PointCloud<PointType>::Ptr local_map = boost::make_shared<pcl::PointCloud<PointType>>();

        deque<pcl::PointCloud<PointType>::Ptr> recent_frames;


        pcl::PointCloud<PointType>::Ptr global_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
        vector<boost::shared_ptr<pcl::PointCloud<PointType>>> all_clouds;

        // "point cloud" containing a specific type that has the odometry information
        pcl::PointCloud<PoseInfo>::Ptr odometry_pose_info = boost::make_shared<pcl::PointCloud<PoseInfo>>(); // add keyframe pose info?
        pcl::PointCloud<pcl::PointXYZI>::Ptr odometry_pose_positions = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

        // subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

        //publishers
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalcloud_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr localcloud_pub;
    

};


int main(int argc, char ** argv){


    rclcpp::init(argc, argv);



    rclcpp::spin(std::make_shared<LidarOdometry>());
    rclcpp::shutdown();
    return 0;


}