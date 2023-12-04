#ifndef COMMON_H
#define COMMON_H


// #include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
// #include <nav_msgs/Path.h>
// #include <nav_msgs/Odometry.h>
// #include <visualization_msgs/Marker.h>
// #include <eigen_conversions/eigen_msg.h>
// #include <message_filters/subscriber.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/NavSatFix.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/pcl_macros.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>

#include <cstdio>

#include <Eigen/Dense>

struct PointPoseInfo
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


POINT_CLOUD_REGISTER_POINT_STRUCT (PointPoseInfo,
                                   (double, x, x) (double, y, y) (double, z, z)
                                   (double, qw, qw) (double, qx, qx) (double, qy, qy) (double, qz, qz)
                                   (int, idx, idx) (double, time, time)
                                   )



// PCL point types
using pcl::PointXYZI;
using pcl::PointXYZINormal;
typedef pcl::PointXYZINormal PointType;


typedef pcl::PointXYZINormal PointType; // need to calculate and assign normals
typedef sensor_msgs::msg::PointCloud2 PC_msg;


struct Pose
{
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond orientation;
    pcl::PointCloud<PointType>::Ptr pointcloud;
    Eigen::Matrix4d matrix;
    int idx;
    int frame_idx;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct Transformation
{
    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
    Eigen::Matrix4d matrix;
    int idx;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;


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
    Eigen::Vector3d ang;
    Eigen::Quaterniond ori;
    Eigen::Vector3d jerk;
    Eigen::Vector3d alpha;
    double time;
    double dt;
    IMUwrench bias;
};

using namespace std;

using std::placeholders::_1;
using std::placeholders::_2;

double toSec(builtin_interfaces::msg::Time header_stamp)
{
    rclcpp::Time time = header_stamp;
    double nanoseconds = time.nanoseconds();

    return nanoseconds * 1e-9;
}

























#endif // COMMON_H