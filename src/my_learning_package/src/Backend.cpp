
#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>


#include <memory>
#include <cstdio>
#include <cmath>
#include <queue>
#include <vector>
#include <fstream>
#include <filesystem>

using namespace std;
using namespace std::filesystem;


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


typedef std::chrono::duration<int,std::milli> milliseconds_type;

// put the following in a genereal header..
// typedef pcl::PointXYZINormal PointType; // need to calculate and assign normals
typedef pcl::PointXYZINormal PointType; // need to calculate and assign normals


//////////////////////////////////////////////////////////////////////////////////////////

class Backend : public rclcpp::Node
{
    private:
        rclcpp::CallbackGroup::SharedPtr sub1_cb_group_;
        rclcpp::CallbackGroup::SharedPtr sub2_cb_group_;

        rclcpp::CallbackGroup::SharedPtr run_cb_group_;
        rclcpp::TimerBase::SharedPtr map_processing_timer;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr full_cloud_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr transformation_sub;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_cloud_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_full_cloud_service;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalcloud_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr long_term_pub;
        // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr localcloud_pub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
       
        pcl::VoxelGrid<PointType> global_map_ds_filter;
        pcl::VoxelGrid<PointType> long_term_ds_filter;

        vector<boost::shared_ptr<pcl::PointCloud<PointType>>> all_clouds;

        pcl::PointCloud<PointType>::Ptr global_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr global_cloud_ds = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr long_term_map = boost::make_shared<pcl::PointCloud<PointType>>();
        // pcl::PointCloud<PointType>::Ptr local_map = boost::make_shared<pcl::PointCloud<PointType>>();
        // pcl::PointCloud<PointType>::Ptr local_map_ds = boost::make_shared<pcl::PointCloud<PointType>>();

        pcl::PointCloud<PointType>::Ptr new_pcl_cloud = boost::make_shared<pcl::PointCloud<PointType>>();

        pcl::PointCloud<PointType>::Ptr pose_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
    
        pcl::PointCloud<PoseInfo>::Ptr odometry_pose_info = boost::make_shared<pcl::PointCloud<PoseInfo>>(); // add keyframe pose info?
        pcl::PointCloud<PoseInfo>::Ptr keyframe_pose_info = boost::make_shared<pcl::PointCloud<PoseInfo>>(); // add keyframe pose info?

        rclcpp::Time time_latest_cloud;


        deque<Eigen::Matrix4d> keyframe_poses; 
        deque<size_t> keyframe_index; 
        deque<Eigen::Matrix4d> all_poses; 

        Eigen::Matrix4d last_odometry_pose;
        Eigen::Matrix4d last_odometry_transformation;
        Eigen::Matrix4d odometry_transformation; // direct transformation from LO registration ie. keyframe to next odometry pose


        size_t index_published{};


        bool new_cloud_ready;
        bool new_pose_ready;
        bool save_pcd_{};

        double global_map_ds_leafsize_;
        double global_map_update_rate_;
        double long_term_ds_leafsize_;

        std::string odometry_topic_;
        std::string cloud_topic_;
        std::string save_path_;

    
    public:
        Backend() // constructer
        : Node("mapping_backend")
        {   
            declare_parameter("global_map_ds_leafsize", 0.01);
            get_parameter("global_map_ds_leafsize", global_map_ds_leafsize_);

            declare_parameter("long_term_ds_leafsize", 1.0);
            get_parameter("long_term_ds_leafsize", long_term_ds_leafsize_);

            declare_parameter("global_map_update_rate", 5.0);
            get_parameter("global_map_update_rate", global_map_update_rate_);    

            declare_parameter("odometry_topic", "/odom_keyframe");
            get_parameter("odometry_topic", odometry_topic_);  

            declare_parameter("cloud_topic", "/full_point_cloud_odom");
            get_parameter("cloud_topic", cloud_topic_);    

            declare_parameter("save_pcd", false);
            get_parameter("save_pcd", save_pcd_);   

            declare_parameter("save_path", "loam_maps");
            get_parameter("save_path", save_path_);    


            allocateMemory();

            milliseconds_type chrono_global_map_update_rate_((int)(global_map_update_rate_ *1000));
            run_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            map_processing_timer = this->create_wall_timer(chrono_global_map_update_rate_, std::bind(&Backend::run, this), run_cb_group_);

            sub1_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options1;
            options1.callback_group = sub1_cb_group_;

            sub2_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options2;
            options2.callback_group = sub2_cb_group_;


            full_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic_, 20, std::bind(&Backend::pointCloudHandler, this, _1), options1);
            // full_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/preprocessed_point_cloud", 100, std::bind(&Backend::pointCloudHandler, this, _1), options1);
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odometry_topic_, 20, std::bind(&Backend::odometryHandler, this, _1), options2);
            // path_sub = this->create_subscription<nav_msgs::msg::Path>("/path", 100, std::bind(&Backend::pathHandler, this, _1));

            // transformation_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/transformation/lidar", 100, std::bind(&Backend::transformationHandler, this, _1));
            // transformation_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/transformation/kalman", 100, std::bind(&Backend::transformationHandler, this, _1));

            globalcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_point_cloud", 1);
            long_term_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/long_term_map", 2);
            // localcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/local_point_cloud", 100);
            odometry_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom_backend", 2);
            path_pub = this->create_publisher<nav_msgs::msg::Path>("/path_backend", 2);

            save_cloud_service = this->create_service<std_srvs::srv::Trigger>("save_cloud_data", std::bind(&Backend::saveCloudService,this, std::placeholders::_1, std::placeholders::_2));
            save_full_cloud_service = this->create_service<std_srvs::srv::Trigger>("save_full_cloud_data", std::bind(&Backend::saveFullCloudService,this, std::placeholders::_1, std::placeholders::_2));


            initializeParameters();

        }
        ~Backend(){}

        void allocateMemory()
        {
            RCLCPP_INFO(get_logger(), "Allocating Point Cloud Memory..");
    
            // local_map.reset(new pcl::PointCloud<PointType>());
            // local_map_ds.reset(new pcl::PointCloud<PointType>());
            
            global_cloud.reset(new pcl::PointCloud<PointType>());
            global_cloud_ds.reset(new pcl::PointCloud<PointType>());
            long_term_map.reset(new pcl::PointCloud<PointType>());
            
            // local_map.reset(new pcl::PointCloud<PointType>());
            // local_map_ds.reset(new pcl::PointCloud<PointType>());

            pose_cloud.reset(new pcl::PointCloud<PointType>());

            new_pcl_cloud.reset(new pcl::PointCloud<PointType>());

            odometry_pose_info.reset(new pcl::PointCloud<PoseInfo>());
            keyframe_pose_info.reset(new pcl::PointCloud<PoseInfo>());
            // odometry_pose_positions.reset(new pcl::PointCloud<pcl::PointXYZI>());

            
        }

        void initializeParameters()
        {
            last_odometry_pose = Eigen::Matrix4d::Identity();
            // keyframe_poses.push_back(first_pose);
            // keyframe_index.push_back(0);
            // all_poses.push_back(last_odometry_pose);

            // float global_map_ds_leafsize_ = 0.01;
            RCLCPP_INFO(get_logger(), "Downsample leafsize is: %f", global_map_ds_leafsize_); 
            global_map_ds_filter.setLeafSize(global_map_ds_leafsize_, global_map_ds_leafsize_, global_map_ds_leafsize_);
            long_term_ds_filter.setLeafSize(long_term_ds_leafsize_, long_term_ds_leafsize_, long_term_ds_leafsize_);

            new_cloud_ready = false;
            new_pose_ready = false;



            // PoseInfo new_pose;
            // rclcpp::Time new_time(0.0); // this time should be something else
            // // odom_message.header.stamp = cloud_header.stamp;
            // new_pose.qw = 1.0;
            // new_pose.qx = 0.0;
            // new_pose.qy = 0.0;
            // new_pose.qz = 0.0;
            // new_pose.x =  0.0;
            // new_pose.y =  0.0;
            // new_pose.z =  0.0;
            // new_pose.idx = odometry_pose_info->points.size();
            // new_pose.time = new_time.nanoseconds() * 1e-9;

            // odometry_pose_info->push_back(new_pose);

            RCLCPP_INFO(get_logger(), "Parameters initialized..");
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

        void pointCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr pc_message)
        {
            time_latest_cloud = pc_message->header.stamp;

            // RCLCPP_INFO(this->get_logger(),"cloud msg recieved");    

            pcl::PointCloud<PointType>::Ptr new_pcl_cloud_copy = boost::make_shared<pcl::PointCloud<PointType>>();

            new_pcl_cloud->clear();
            fromROSMsg(*pc_message, *new_pcl_cloud);

            pcl::copyPointCloud(*new_pcl_cloud, *new_pcl_cloud_copy);

            all_clouds.push_back(new_pcl_cloud_copy);

            new_cloud_ready = true;
            
            // RCLCPP_INFO(get_logger(), "cloud recieved");

            // convert to pcl
            // save the cloud to all clouds            
        }


        void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odom_message)
        {
            // match timestamp with cloud coming in
            // RCLCPP_INFO(this->get_logger(),"odom msg recieved");          

            // save pose to all poses      

            PoseInfo new_pose;
            rclcpp::Time new_time = odom_message->header.stamp;
            // odom_message.header.stamp = cloud_header.stamp;
            new_pose.qw = odom_message->pose.pose.orientation.w;
            new_pose.qx = odom_message->pose.pose.orientation.x;
            new_pose.qy = odom_message->pose.pose.orientation.y;
            new_pose.qz = odom_message->pose.pose.orientation.z;
            new_pose.x =  odom_message->pose.pose.position.x;
            new_pose.y =  odom_message->pose.pose.position.y;
            new_pose.z =  odom_message->pose.pose.position.z;
            new_pose.idx = odometry_pose_info->points.size();
            new_pose.time = new_time.nanoseconds() * 1e-9;

            odometry_pose_info->push_back(new_pose);

            new_pose_ready = true;
            // RCLCPP_INFO(get_logger(), "pose recieved");

        }

        // make function that checks odom and cloud ready bools to run and compares timestamps before saving cloud and pose, as these have to match each other

        void pathHandler(const nav_msgs::msg::Path::SharedPtr message)
        {
            
            RCLCPP_DEBUG(this->get_logger(),"path msg recieved %s", message->header.frame_id.c_str());   
            // save path             
        }

        // void transformationHandler(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
        // {
        //     // make recieved pose into transformation matrix
        //     Eigen::Quaterniond transformation_quaternion;
        //     transformation_quaternion.w() = message->pose.pose.orientation.w;
        //     transformation_quaternion.x() = message->pose.pose.orientation.x;
        //     transformation_quaternion.y() = message->pose.pose.orientation.y;
        //     transformation_quaternion.z() = message->pose.pose.orientation.z;

        //     Eigen::Vector3d transformation_translation(message->pose.pose.position.x,
        //                                                message->pose.pose.position.y,
        //                                                message->pose.pose.position.z);


        //     Eigen::Matrix3d rot_mat(transformation_quaternion);
        //     odometry_transformation.block<3,3>(0,0) = rot_mat;
        //     odometry_transformation.block<3,1>(0,3) = transformation_translation;

        //     updateOdometryPose();

        //     Eigen::Quaterniond reg_quarternion(last_odometry_pose.block<3, 3>(0, 0)); 
        //     reg_quarternion.normalize();
        //     Eigen::Vector3d reg_translation(last_odometry_pose.block<3, 1>(0, 3));


        //     PoseInfo new_pose;
        //     rclcpp::Time new_time = message->header.stamp;
        //     // odom_message.header.stamp = cloud_header.stamp;
        //     new_pose.qw = reg_quarternion.w();
        //     new_pose.qx = reg_quarternion.x();
        //     new_pose.qy = reg_quarternion.y();
        //     new_pose.qz = reg_quarternion.z();
        //     new_pose.x =  reg_translation.x();
        //     new_pose.y =  reg_translation.y();
        //     new_pose.z =  reg_translation.z();
        //     new_pose.idx = odometry_pose_info->points.size();
        //     new_pose.time = new_time.nanoseconds() * 1e-9;

        //     odometry_pose_info->push_back(new_pose);


        //     new_pose_ready = true;
            
        // }

        // void updateOdometryPose()
        // {   
        //     Eigen::Matrix4d next_odometry_pose = last_odometry_pose * odometry_transformation; // the next evaluted odometry pose, not neccesarily a keyframe
        //     // last_odometry_transformation = last_odometry_pose * next_odometry_pose.inverse(); // transformation between 2 consecutive odometry poses
        //     // odometry_transformation_guess = registration_transformation * last_odometry_transformation; // prediction of the next transformation using the previuosly found transformation - used for ICP initial guess
        //     last_odometry_pose = next_odometry_pose;

        //     all_poses.push_back(last_odometry_pose);

        // }

        // void buildLocalMap()
        // {

        // }

        void saveGlobalCloudPCD()
        {
            if (!save_pcd_)
                return;
                
            RCLCPP_INFO(get_logger(), "Saving global cloud..");
            pcl::io::savePCDFileASCII("temp_saved_odometry_data/pcd/global_map.pcd", *global_cloud_ds);

        }

        void saveCloudService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response>  response)
        {
            RCLCPP_INFO(get_logger(), "Request to save data received %s.", request);

            saveGlobalCloudPCDServiceCall(false, response);

            response->success = true;
            // response->message = "End of service" ;

            RCLCPP_INFO(get_logger(), "Data has been saved: %s ", response->success? "true":"false");
        
        }

        void saveFullCloudService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response>  response)
        {
            RCLCPP_INFO(get_logger(), "Request to save data received %s.", request);

            
            saveGlobalCloudPCDServiceCall(true, response);

            response->success = true;
            // response->message = "End of service" ;

            RCLCPP_INFO(get_logger(), "Data has been saved: %s ", response->success? "true":"false");
        
        }


        void saveGlobalCloudPCDServiceCall(bool full, std::shared_ptr<std_srvs::srv::Trigger::Response>  response)
        {
            std::string msg;
            RCLCPP_INFO(get_logger(), "Saving global cloud on service call!");
            RCLCPP_INFO(get_logger(), "Leaf size for downsampling is %f", global_map_ds_leafsize_);


            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);
            std::ostringstream oss;
            oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
            std::string datestr = oss.str();

            std::string full_save_path = save_path_ + "/" + "loam_global_map_" + datestr;
            create_directories(full_save_path);

            bool save_as_bin = true;

            rebuildGlobalMap();
            if (full){
                msg = "Full global map is saved in folder "+ full_save_path;
                pcl::io::savePCDFile(full_save_path + "/loam_map.pcd", *global_cloud, save_as_bin);
                RCLCPP_INFO(get_logger(), msg);
            } else {
                msg = "Downsampled global map is saved in folder "+ full_save_path;
                pcl::io::savePCDFile(full_save_path + "/loam_map_ds.pcd", *global_cloud_ds, save_as_bin);
                RCLCPP_INFO(get_logger(), msg);
            }
            response->message = msg;
            // pcl::io::savePCDFileASCII(full_save_path + "/loam_map_ds.pcd", *global_cloud_ds);

            // global_cloud->clear();
        }
   
        void rebuildGlobalMap()
        {   
            if (all_clouds.empty()){
                return;
            }
            
            // global_cloud->clear();
            // pcl::PointCloud<PointType> transformed_cloud;
            pcl::PointCloud<PointType>::Ptr transformed_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
            // for (unsigned i = 0; i < odometry_pose_info->points.size(); i++)
            // #pragma omp for
            for (size_t i = index_published ; i < all_clouds.size(); i++)
            {
                // Eigen::Quaterniond q_po(odometry_pose_info->points[i].qw,
                //                         odometry_pose_info->points[i].qx,
                //                         odometry_pose_info->points[i].qy,
                //                         odometry_pose_info->points[i].qz);

                // Eigen::Vector3d t_po(odometry_pose_info->points[i].x,
                //                     odometry_pose_info->points[i].y,
                //                     odometry_pose_info->points[i].z);


                // pcl::transformPointCloudWithNormals<PointType>(*all_clouds[i], transformed_cloud, t_po, q_po);
                *transformed_cloud = *all_clouds[i];
                *global_cloud += *transformed_cloud;
                index_published = i+1;
            }
                
            pcl::PointCloud<PointType>::Ptr global_cloud_copy = boost::make_shared<pcl::PointCloud<PointType>>(*global_cloud);

            pcl::copyPointCloud(*global_cloud, *global_cloud_copy);

            global_map_ds_filter.setInputCloud(global_cloud_copy);
            global_map_ds_filter.filter(*global_cloud_ds);

        }

        void publishLongTermMap()
        {
            // Recalculate normals on global cloud?

            long_term_ds_filter.setInputCloud(global_cloud_ds);
            long_term_ds_filter.filter(*long_term_map);

            sensor_msgs::msg::PointCloud2 msgs;
            #ifndef __INTELLISENSE__ 
            pcl::toROSMsg(*long_term_map, msgs);
            #endif
            // msgs.header.stamp = ros::Time().fromSec(time_new_cloud);
            msgs.header.stamp = time_latest_cloud; 
            msgs.header.frame_id = "long_term_map";
            long_term_pub->publish(msgs);

            // clear the the full unsampled cloud out of RAM.
            // global_cloud->clear();
            // global_map_ds->clear();
        }


        void publishGlobalCloud()
        {
            // build global cloud from odometry transforms and recordeded (keyframe) clouds
            // publish the cloud, maybe with a gate-timer to not hog bandwidth and memory
  
            // RCLCPP_INFO(get_logger(),"number of clouds: %i", all_clouds.size());
            // RCLCPP_INFO(get_logger(),"number of poses: %i", odometry_pose_info->points.size());
            if (new_cloud_ready || new_pose_ready) { // if new poses are ready rebuilt map, else just publish current map
                // RCLCPP_INFO(get_logger(),"Rebuilding global map!");
                rebuildGlobalMap();
            }
            // RCLCPP_INFO(get_logger(),"Publishing global map!");

            sensor_msgs::msg::PointCloud2 msgs;
            #ifndef __INTELLISENSE__ 
            pcl::toROSMsg(*global_cloud_ds, msgs);
            #endif
            // msgs.header.stamp = ros::Time().fromSec(time_new_cloud);
            msgs.header.stamp = time_latest_cloud; 
            msgs.header.frame_id = "global_map";
            globalcloud_pub->publish(msgs);

            // clear the the full unsampled cloud out of RAM.
            // global_cloud->clear();
            // global_map_ds->clear();
        }

        void run()
        {
            publishGlobalCloud();
            publishLongTermMap();
            if (!new_cloud_ready || !new_pose_ready) {
                return;
            }
            saveGlobalCloudPCD();
            new_cloud_ready = false;
            new_pose_ready = false;
        }



        
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto backend_node = std::make_shared<Backend>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(backend_node);
    
    executor.spin();
    rclcpp::shutdown();
    return 0;
}