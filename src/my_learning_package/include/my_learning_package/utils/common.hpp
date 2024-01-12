#ifndef COMMON_H
#define COMMON_H

// #include "my_learning_package/point_type.h" // attempt to create a custom point cloud point type

// #include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/NavSatFix.h>


#include <geometry_msgs/msg/wrench_stamped.hpp>


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
#include <cmath>
#include <ctime>
// #include <array>
#include <string>
#include <vector>
// #include <algorithm>
#include <iostream>
#include <fstream>
// #include <thread>
// #include <mutex>
#include <queue>
// #include <assert.h>
#include <iomanip>
// #include <ctime>
#include <sstream>
// #include <format> // can't locate header...

#include <Eigen/Dense>

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



// PCL point types
using pcl::PointXYZI;
using pcl::PointXYZINormal;
// typedef pcl::PointXYZINormal PointType;

typedef pcl::PointXYZI PointTypeNoNormals; // need to calculate and assign normals and the change to PointXYZINormals
typedef pcl::PointXYZINormal PointType; // need to calculate and assign normals
// typedef PointXYZRTLTNormal PointTypeWTime;

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
    double dt;
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


// template <typename timeT>
// double toSec(builtin_interfaces::msg::Time header_stamp)
// {
//     rclcpp::Time time = header_stamp;
//     double nanoseconds = time.nanoseconds();

//     return nanoseconds * 1e-9;
// }

void inline fromROSMsg(const sensor_msgs::msg::PointCloud2 &cloud, pcl::PointCloud<PointType> &pcl_cloud)
{   
    // sensor_msgs::PointCloud2 cloud_PC2 = cloud;
    pcl::PCLPointCloud2 pcl_pc2;
    #ifndef __INTELLISENSE__ // this is to disable an error squiggle from pcl_conversions not having the proper overload for input type.
    pcl_conversions::toPCL(cloud, pcl_pc2);
    #endif
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
}


double inline toSec(builtin_interfaces::msg::Time header_stamp)
{
    rclcpp::Time time = header_stamp;
    double nanoseconds = time.nanoseconds();

    return nanoseconds * 1e-9; // converted to seconds
}

builtin_interfaces::msg::Time inline toHeaderStamp(double time)
{
    builtin_interfaces::msg::Time stamp;
    stamp.set__nanosec((int)(time * 1e9));

    return stamp;
}


// template <typename Derived>
Eigen::Quaterniond inline addQuaternions(const Eigen::Quaterniond q1, const Eigen::Quaterniond q2)
{
    Eigen::Quaterniond q_sum;
    q_sum.w() = q1.w() + q2.w();
    q_sum.vec() = q1.vec() + q2.vec();
    return q_sum;
}

Eigen::Quaterniond inline multQuatByScalar(const Eigen::Quaterniond q, const double scalar)
{
    Eigen::Quaterniond q_prod;
    q_prod.w() = q.w()*scalar;
    q_prod.vec() = q.vec()*scalar;
    return q_prod;
}


template <typename Derived>
Eigen::Quaternion<typename Derived::Scalar> inline deltaQ(const Eigen::MatrixBase<Derived> &theta)
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

// Function to convert quaternion to Euler angles (pitch, yaw, roll)
template <typename Derived>
Eigen::Matrix<typename Eigen::QuaternionBase<Derived>::Scalar, 3, 1> inline quaternionToEulerAngles(const Eigen::QuaternionBase<Derived>& q) 
{
    typedef typename Eigen::QuaternionBase<Derived>::Scalar Scalar_t;;
    Eigen::Matrix<Scalar_t, 3, 1> ypr;

    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    ypr.z() = static_cast<Scalar_t>(std::atan2(sinr_cosp, cosr_cosp));

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        ypr.y() = static_cast<Scalar_t>(std::copysign(M_PI / 2, sinp)); // Use 90 degrees if out of range
    else
        ypr.y() = static_cast<Scalar_t>(std::asin(sinp));

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    ypr.x() = static_cast<Scalar_t>(std::atan2(siny_cosp, cosy_cosp));

    return ypr;
}

// template <typename Derived>
// Eigen::Matrix<typename Derived::Scalar, 3, 1> inline quaternionToEulerAngles(double qw, double qx, double qy, double qz) {
//     return quaternionToEulerAngles(Eigen::Quaterniond(qw, qx, qy, qz));
// }

// void inline printINSstate(const INSstate state, auto logger){
//     RCLCPP_INFO(logger, "-------- State Print -----------");
//     RCLCPP_INFO(logger, "state time: %f", state.time);
//     RCLCPP_INFO(logger, "state dt: %f", state.dt);
//     RCLCPP_INFO(logger, "state pos: %f %f %f", state.pos.x(), state.pos.y(), state.pos.z());
//     RCLCPP_INFO(logger, "state vel: %f %f %f", state.vel.x(), state.vel.y(), state.vel.z());
//     RCLCPP_INFO(logger, "state acc: %f %f %f", state.acc.x(), state.acc.y(), state.acc.z());
//     RCLCPP_INFO(logger, "state jerk: %f %f %f", state.jerk.x(), state.jerk.y(), state.jerk.z());
//     RCLCPP_INFO(logger, "state ang vel: %f %f %f", state.ang.x(), state.ang.y(), state.ang.z());
//     RCLCPP_INFO(logger, "state alpha: %f %f %f", state.alpha.x(), state.alpha.y(), state.alpha.z());
//     RCLCPP_INFO(logger, "state bias acc: %f %f %f", state.bias.acc.x(), state.bias.acc.y(), state.bias.acc.z());
//     RCLCPP_INFO(logger, "state bias ang: %f %f %f", state.bias.ang.x(), state.bias.ang.y(), state.bias.ang.z());
//     RCLCPP_INFO(logger, "--------------------------------");
// }

// virker ikke .... ved ikke hvorfor?
Eigen::Matrix4d inline SE3fromTransformation(const Transformation &transformation)
{
    Eigen::Matrix4d SE3 = Eigen::Matrix4d::Identity();
    SE3.block<3,3>(0,0) = Eigen::Matrix3d(transformation.rotation);
    SE3.block<3,1>(0,3) = transformation.translation;
    return SE3;
}

Eigen::Matrix4d inline SE3fromTransformation(const Pose &pose)
{
    Eigen::Matrix4d SE3 = Eigen::Matrix4d::Identity();
    SE3.block<3,3>(0,0) = Eigen::Matrix3d(pose.orientation);
    SE3.block<3,1>(0,3) = pose.position;
    return SE3;
}

Eigen::Matrix4d inline SE3fromTransformation(const Eigen::Quaterniond q, const Eigen::Vector3d t)
{
    Eigen::Matrix4d SE3 = Eigen::Matrix4d::Identity();
    SE3.block<3,3>(0,0) = Eigen::Matrix3d(q);
    SE3.block<3,1>(0,3) = t;
    return SE3;
}



// template <typename PointT>
        // void lineSampling(const boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_in, boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_out, int numberPerSample)
        // {

        //     RCLCPP_INFO(get_logger(), "Line Sampling?? %i, %i", numberPerSample, cloud_in->points.size());
        //     int n = 0;
        //     PointT point;
        //     for (size_t i = 0; i < cloud_in->points.size(); i++){
        //         if (n < numberPerSample)
        //         {
        //             n++;
        //             // RCLCPP_INFO(get_logger(), "n? %i", n);
        //             continue;
        //         }
        //         // RCLCPP_INFO(get_logger(), "point sampled %i", i);
        //         point = cloud_in->points[i];
        //         cloud_out->points.push_back(point);
        //         n = 0; 
        //     }
        // }

        // template <typename PointT>
        // void lineDensitySampling(const boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_in, boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_out, int numberPerSample)
        // {

        //     // RCLCPP_INFO(get_logger(), "Line Sampling?? %i, %i", numberPerSample, cloud_in->points.size());
        //     double line_distance{};
        //     double threshold = 10.0/(double)numberPerSample;
        //     PointT point;
        //     for (size_t i = 0; i < cloud_in->points.size()-1; i++){
        //         double dist = (cloud_in->points[i].getVector4fMap() - 
        //                     cloud_in->points[i+1].getVector4fMap()).norm ();
        //         line_distance += dist;
        //         if (line_distance < threshold)
        //         {
        //             continue;
        //         }
        //         point = cloud_in->points[i];
        //         cloud_out->points.push_back(point);
        //         line_distance = 0.0;
        //     }
        // }

        // template <typename PointT>
        // void farthestPointSampling(const boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_in, boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_out, int numberSamples)
        // {
        //     vector<int> points_left_idx;
        //     vector<int> points_sampled_idx;
        //     vector<double> dists;
        //     vector<double> dists_to_last_added;

        //     points_sampled_idx.push_back(0);
        //     // cloud_out->points.push_back(cloud_in->points[0]); // push first point to sampled

        //     for (size_t i = 0; i < cloud_in->points.size(); i++){
        //         dists.push_back(std::numeric_limits<double>::max ()); // infinite distances
        //         if (i == 0)
        //             continue;
        //         points_left_idx.push_back(i); // 0 is not added because if statement above

        //     }

        //     PointType point;
        //     // for (size_t i = 0; i < cloud_in->points.size(); i++)
        //     for (int i = 0; i < numberSamples; i++) // number of samples wanted
        //     {
        //         // get the last added index
        //         int last_added_idx = points_sampled_idx.back();

        //         // calculate distances to the last added point
        //         dists_to_last_added.clear();
        //         for (size_t j = 0; j < points_left_idx.size() - i; j++)
        //         {
        //             double dist = (cloud_in->points[points_left_idx[j]].getVector4fMap () - 
        //                     cloud_in->points[last_added_idx].getVector4fMap ()).squaredNorm ();
        //             dists_to_last_added.push_back(dist);
        //         }

        //         // update distances ie. get shortest of dist_to_last_added and dists , mayeb combine this with the next loop to save a loop,
        //         for (size_t k = 0; k < points_left_idx.size(); k++)
        //         {
        //             if (dists_to_last_added[k] < dists[points_left_idx[k]]){
        //                 dists[points_left_idx[k]] = dists_to_last_added[k];
        //             }
        //         }

        //         // find the index of the point in points_left with the largest distance
        //         double max_dist = std::numeric_limits<double>::min ();
        //         int idx_max{};
        //         int selected{};
        //         for (size_t l = 0; l < points_left_idx.size(); l++) {
        //             if (dists[points_left_idx[l]] <= max_dist)
        //                 continue;
        //             max_dist = dists[points_left_idx[l]];
        //             idx_max = l;
        //             selected = points_left_idx[l];
        //         }

        //         // add that index to sampled and remove from points_left
        //         points_sampled_idx.push_back(selected);
        //         points_left_idx.erase(points_left_idx.begin()+idx_max);

        //         // point = lidar_cloud_in->points[i];
        //     }

        //     pcl::PointCloud<PointType>::Ptr sampled = boost::make_shared<pcl::PointCloud<PointType>>();

        //     // assign points to sampled from found indices
        //     for (size_t i = 0; i < points_sampled_idx.size(); i++)
        //     {
        //         sampled->points.push_back(cloud_in->points[points_sampled_idx[i]]);
        //     }

        //     cloud_out = sampled;
        // }


















#endif // COMMON_H