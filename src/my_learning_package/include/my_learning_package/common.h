#ifndef COMMON_H
#define COMMON_H

#include "rclcpp/rclcpp.hpp"

// #include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <sensor_msgs/msg/imu.hpp>
// #include <nav_msgs/Path.h>
// #include <nav_msgs/Odometry.h>
// #include <visualization_msgs/Marker.h>
// #include <eigen_conversions/eigen_msg.h>
// #include <message_filters/subscriber.h>
// #include <geometry_msgs/Vector3Stamped.h>
// #include <geometry_msgs/QuaternionStamped.h>
// #include <sensor_msgs/NavSatFix.h>
#include "builtin_interfaces/msg/time.hpp"


// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/filter.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/common/common.h>
// #include <pcl/pcl_macros.h>
// #include <pcl/range_image/range_image.h>
// #include <pcl/registration/icp.h>

// #include <cstdio>

// #include <Eigen/Dense>

using namespace std;
using std::placeholders::_1;

// struct PointPoseInfo
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


// POINT_CLOUD_REGISTER_POINT_STRUCT (PointPoseInfo,
//                                    (double, x, x) (double, y, y) (double, z, z)
//                                    (double, qw, qw) (double, qx, qx) (double, qy, qy) (double, qz, qz)
//                                    (int, idx, idx) (double, time, time)
//                                    )



// // PCL point types
// using pcl::PointXYZI;
// using pcl::PointXYZINormal;
// typedef pcl::PointXYZINormal PointType;


double toSec(builtin_interfaces::msg::Time header_stamp)
{
    rclcpp::Time time = header_stamp;
    double nanoseconds = time.nanoseconds();
    return nanoseconds/1e-9;
}
























#endif // COMMON_H