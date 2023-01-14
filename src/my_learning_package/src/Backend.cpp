
#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

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
typedef pcl::PointXYZINormal PointType; // need to calculate and assign normals

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

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalcloud_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr localcloud_pub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
       
        pcl::VoxelGrid<PointType> global_map_ds_filter;

        vector<boost::shared_ptr<pcl::PointCloud<PointType>>> all_clouds;
        pcl::PointCloud<PointType>::Ptr global_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr global_cloud_ds = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr new_pcl_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PoseInfo>::Ptr odometry_pose_info = boost::make_shared<pcl::PointCloud<PoseInfo>>(); // add keyframe pose info?

        rclcpp::Time time_latest_cloud;

        bool new_cloud_ready;
        bool new_pose_ready;
    
    
    public:
        Backend() // constructer
        : Node("mapping_backend")
        {   
            allocateMemory();

            run_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            map_processing_timer = this->create_wall_timer(500ms, std::bind(&Backend::run, this), run_cb_group_);

            sub1_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options1;
            options1.callback_group = sub1_cb_group_;

            sub2_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options2;
            options2.callback_group = sub2_cb_group_;


            full_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/full_point_cloud_keyframe", 100, std::bind(&Backend::pointCloudHandler, this, _1), options1);
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 100, std::bind(&Backend::odometryHandler, this, _1), options2);
            path_sub = this->create_subscription<nav_msgs::msg::Path>("/path", 100, std::bind(&Backend::pathHandler, this, _1));

            globalcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_point_cloud", 100);
            localcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/local_point_cloud", 100);
            odometry_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom_backend", 100);
            path_pub = this->create_publisher<nav_msgs::msg::Path>("/path_backend", 100);

            float global_map_ds_leafsize = 0.001;
            global_map_ds_filter.setLeafSize(global_map_ds_leafsize, global_map_ds_leafsize, global_map_ds_leafsize);

            new_cloud_ready = false;
            new_pose_ready = false;
        }
        ~Backend(){}

        void allocateMemory()
        {
            // RCLCPP_INFO(get_logger(), "Allocating Point Cloud Memory..");
    
            // local_map.reset(new pcl::PointCloud<PointType>());
            // local_map_ds.reset(new pcl::PointCloud<PointType>());
            
            global_cloud.reset(new pcl::PointCloud<PointType>());
            global_cloud_ds.reset(new pcl::PointCloud<PointType>());
            new_pcl_cloud.reset(new pcl::PointCloud<PointType>());

            odometry_pose_info.reset(new pcl::PointCloud<PoseInfo>());
            // odometry_pose_positions.reset(new pcl::PointCloud<pcl::PointXYZI>());
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
            new_pose.time = new_time.nanoseconds();

            odometry_pose_info->push_back(new_pose);

            new_pose_ready = true;
        }

        // make function that checks odom and cloud ready bools to run and compares timestamps before saving cloud and pose, as these have to match each other

        void pathHandler(const nav_msgs::msg::Path::SharedPtr message)
        {
            
            RCLCPP_DEBUG(this->get_logger(),"path msg recieved %s", message->header.frame_id.c_str());   
            // save path             
        }

   

        void publishGlobalCloud()
        {
            // build global cloud from odometry transforms and recordeded (keyframe) clouds
            // publish the cloud, maybe with a gate-timer to not hog bandwidth and memory
  
            RCLCPP_INFO(get_logger(),"Publishing global map!");
            // RCLCPP_INFO(get_logger(),"number of clouds: %i", all_clouds.size());
            // RCLCPP_INFO(get_logger(),"number of poses: %i", odometry_pose_info->points.size());
            
            for (unsigned i = 0; i < odometry_pose_info->points.size(); i++)
            {
                Eigen::Quaterniond q_po(odometry_pose_info->points[i].qw,
                                        odometry_pose_info->points[i].qx,
                                        odometry_pose_info->points[i].qy,
                                        odometry_pose_info->points[i].qz);

                Eigen::Vector3d t_po(odometry_pose_info->points[i].x,
                                    odometry_pose_info->points[i].y,
                                    odometry_pose_info->points[i].z);


                pcl::PointCloud<PointType> transformed_cloud;
                pcl::transformPointCloudWithNormals<PointType>(*all_clouds[i], transformed_cloud, t_po, q_po);
                *global_cloud += transformed_cloud;
            }

            global_map_ds_filter.setInputCloud(global_cloud);
            global_map_ds_filter.filter(*global_cloud_ds);

            sensor_msgs::msg::PointCloud2 msgs;
            #ifndef __INTELLISENSE__ 
            pcl::toROSMsg(*global_cloud_ds, msgs);
            #endif
            // msgs.header.stamp = ros::Time().fromSec(time_new_cloud);
            msgs.header.stamp = time_latest_cloud; 
            msgs.header.frame_id = "global_map";
            globalcloud_pub->publish(msgs);

            // clear the cloud out of RAM.
            global_cloud->clear();
            // global_map_ds->clear();
        }

        void run()
        {
            if (!new_cloud_ready || !new_pose_ready) {
                return;
            }
            publishGlobalCloud();
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