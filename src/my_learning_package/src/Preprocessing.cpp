#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"
// #include "utils/common.h"
// #include "common.h"

#include <sensor_msgs/msg/point_cloud2.hpp>
// #include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/msg/imu.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

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
#include <queue>
// #include <assert.h>

typedef pcl::PointXYZI PointTypeNoNormals; // need to calculate and assign normals and the change to PointXYZINormals
typedef pcl::PointXYZINormal PointType;

using namespace std;
using std::placeholders::_1;



class Preprocessing : public rclcpp::Node
{
private:
    rclcpp::CallbackGroup::SharedPtr subscriber_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_Lidar_cloud;
    rclcpp::TimerBase::SharedPtr process_timer;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;


    pcl::VoxelGrid<PointType> down_size_filter;
    // ros::Subscriber sub_imu;

    // ros::Publisher pub_surf;
    // ros::Publisher pub_edge;
    // ros::Publisher pub_cutted_cloud;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_surf;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_edge;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cutted_cloud;

    // int pre_num = 0;

    pcl::PointCloud<PointType>::Ptr lidar_cloud_in = boost::make_shared<pcl::PointCloud<PointType>>();
    pcl::PointCloud<PointTypeNoNormals>::Ptr lidar_cloud_in_no_normals = boost::make_shared<pcl::PointCloud<PointTypeNoNormals>>();
    pcl::PointCloud<PointType>::Ptr lidar_cloud_cutted = boost::make_shared<pcl::PointCloud<PointType>>();
    std_msgs::msg::Header cloud_header;

    vector<sensor_msgs::msg::Imu::SharedPtr> imu_buffer;
    size_t imu_buffer_max_size = 600;
    int idx_imu = 0;
    int latest_frame_idx = 0;
    int start_idx_ = 0;
    int end_idx_ = 0;
    double current_time_imu = -1;
    double lidar_zero_time = -1;

    Eigen::Vector3d gyr_0;
    Eigen::Quaterniond q_iMU = Eigen::Quaterniond::Identity();
    bool first_imu = true;
    bool first_lidar = true;

    std::deque<sensor_msgs::msg::PointCloud2> cloud_queue;
    sensor_msgs::msg::PointCloud2 current_cloud_msg;

    double time_scan; // convert to rclcpp::Time?
    double time_scan_next; // convert to rclcpp::Time?
    // rclcpp::Time time_scan_next;

    // int N_SCANS = 6;
    // int H_SCANS = 4000;

    string frame_id = "lidar_odom";
    string lidar_source_topic_ = "";
    double runtime = 0;
    // ---- ROSPARAMETERS -----

    double edge_threshold_;
    double surfaces_threshold_;

    // double normal_search_radius = 0.2; // small scale / indoors
    double pc_normal_search_radius_; // large scale / outdoors
    int pc_normal_knn_points_;

    double filter_close_points_distance_m_;
    bool use_gyroscopic_undistortion_{};

    int remove_statistical_outliers_knn_points_;
    double remove_statistical_outliers_std_mul_;
    double remove_transition_outliers_cos_angle_threshold_;
    double normalize_intensities_reference_range_;
    int calculate_point_curvature_kernel_width_;
    int calculate_point_plateau_kernel_width_;

public:
    Preprocessing() : rclcpp::Node("preprocessing")
    {
        declare_parameter("filter_close_points_distance_m", 0.1);
        get_parameter("filter_close_points_distance_m", filter_close_points_distance_m_);

        declare_parameter("use_gyroscopic_undistortion", false);
        get_parameter("use_gyroscopic_undistortion", use_gyroscopic_undistortion_);

        declare_parameter("pc_normal_search_radius", 0.0);
        get_parameter("pc_normal_search_radius", pc_normal_search_radius_);

        declare_parameter("pc_normal_knn_points", 50);
        get_parameter("pc_normal_knn_points", pc_normal_knn_points_);

        declare_parameter("edge_threshold", 0.5);
        get_parameter("edge_threshold", edge_threshold_);

        declare_parameter("surfaces_threshold", 0.01);
        get_parameter("surfaces_threshold", surfaces_threshold_);

        declare_parameter("remove_statistical_outliers_knn_points", 10);
        get_parameter("remove_statistical_outliers_knn_points", remove_statistical_outliers_knn_points_);

        declare_parameter("remove_statistical_outliers_std_mul", 1.5);
        get_parameter("remove_statistical_outliers_std_mul", remove_statistical_outliers_std_mul_);

        declare_parameter("remove_transition_outliers_cos_angle_threshold", 0.9999);
        get_parameter("remove_transition_outliers_cos_angle_threshold", remove_transition_outliers_cos_angle_threshold_);

        declare_parameter("normalize_intensities_reference_range", 0.0);
        get_parameter("normalize_intensities_reference_range", normalize_intensities_reference_range_);

        declare_parameter("calculate_point_curvature_kernel_width", 0);
        get_parameter("calculate_point_curvature_kernel_width", calculate_point_curvature_kernel_width_);

        declare_parameter("calculate_point_plateau_kernel_width", 0);
        get_parameter("calculate_point_plateau_kernel_width", calculate_point_plateau_kernel_width_);

        declare_parameter("lidar_source_topic", "/livox");
        get_parameter("lidar_source_topic", lidar_source_topic_);

        declare_parameter("start_idx", 0); 
        get_parameter("start_idx", start_idx_);

        // declare_parameter("end_idx", std::numeric_limits<int>::max ()); 
        declare_parameter("end_idx", 0); // 0 means endless 
        get_parameter("end_idx", end_idx_);



        RCLCPP_INFO(get_logger(), "\033[1;32m---->\033[0m Preprocessing Started.");
        // if (!getParameter("/preprocessing/surf_thres", surf_thres))
        // {
        //     ROS_WARN("surf_thres not set, use default value: 0.2");
        //     surf_thres = 0.2;
        // }

        // if (!getParameter("/preprocessing/edge_thres", edge_thres))
        // {
        //     ROS_WARN("edge_thres not set, use default value: 4.0");
        //     edge_thres = 4.0;
        // }

        // if (!getParameter("/common/frame_id", frame_id))
        // {
        //     ROS_WARN("frame_id not set, use default value: lili_om");
        //     frame_id = "lili_om";
        // }

        subscriber_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);                                                            // create subscriber callback group
        rclcpp::SubscriptionOptions options;                                                                                                                         // create subscriver options
        options.callback_group = subscriber_cb_group_;                                                                                                               // add callbackgroup to subscriber options
        string lidar_topic = lidar_source_topic_ + "/lidar";
        sub_Lidar_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic, 100, std::bind(&Preprocessing::cloudHandler, this, _1), options); // add subscriber options to the subsriber constructor call..
        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data_raw", 100, std::bind(&Preprocessing::imuHandler, this, _1), options);                  // make separate subscribe callback group?

        timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        process_timer = this->create_wall_timer(1ms, std::bind(&Preprocessing::processNext, this), timer_cb_group_);

        // sub_imu = nh.subscribe<sensor_msgs::Imu>("/livox/imu", 200, &Preprocessing::imuHandler, this);

        pub_surf = this->create_publisher<sensor_msgs::msg::PointCloud2>("/surf_features", 100);
        pub_edge = this->create_publisher<sensor_msgs::msg::PointCloud2>("/edge_features", 100);
        pub_cutted_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/preprocessed_point_cloud", 100);

        down_size_filter.setLeafSize(0.5, 0.5, 0.5);
    }
    ~Preprocessing() {}




    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_cloud_msg)
    {
        // cache point cloud in buffer
        if (first_lidar)
        {
            first_lidar = false; // used by imu_handler to see when the first lidar msg has arrived
        }

        cloud_queue.push_back(*lidar_cloud_msg);
    }

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imu_in)
    {
        imu_buffer.push_back(imu_in);
        if (first_lidar) // when the first lidar msg has arrived this whill no longer be updated and therefore is the time of the first lidar msg in imu time reference
        {
            lidar_zero_time = toSec(imu_in->header.stamp);
            RCLCPP_INFO_ONCE(get_logger(), "First Lidar msg recieved at IMU time: %f", lidar_zero_time);
        }
        // RCLCPP_INFO(get_logger(), "Timestamp of IMU from toSec: %f", toSec(imu_in->header.stamp));



        if (imu_buffer.size() > imu_buffer_max_size)                           // if buffer is "filled" / above max size, clear the entries at the front
            imu_buffer[imu_buffer.size() - imu_buffer_max_size + 1] = nullptr; // why not pop the first element in buffer, if this is to clear old buffer?

        if (current_time_imu < 0) // initialized as -1.0. why not do this in the next if statement??
            // current_time_imu = imu_in->header.stamp.toSec();
            // time_in = imu_in->header.stamp;
            // current_time_imu = time_in.nanoseconds();
            current_time_imu = toSec(imu_in->header.stamp); // current time is the time of the first time timestamp?

        if (first_imu) // this is only done on the first msg
        {
            first_imu = false;
            double rx = 0, ry = 0, rz = 0;
            rx = imu_in->angular_velocity.x;
            ry = imu_in->angular_velocity.y;
            rz = imu_in->angular_velocity.z;
            Eigen::Vector3d angular_velocity(rx, ry, rz);
            gyr_0 = angular_velocity;
        }
    }

    // function that can remove points that are too close to the scanner i.e. auto-scans, weird function name will have to change it when I now how far i goes
    template <typename PointT>
    void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, float thres)
    {

        // if the cloud are not the same, make sure head and size are the same
        if (&cloud_in != &cloud_out)
        {
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize(cloud_in.points.size());
        }

        size_t j = 0;

        for (size_t i = 0; i < cloud_in.points.size(); ++i)
        {
            if (cloud_in.points[i].x * cloud_in.points[i].x +
                    cloud_in.points[i].y * cloud_in.points[i].y +
                    cloud_in.points[i].z * cloud_in.points[i].z <
                thres * thres) // calculating the squared distance of a point and comparing it to a threshold
                continue;
            if (!pcl::isFinite(cloud_in.points[i]) || !pcl::isFinite(cloud_in.points[i]))
            {
                continue;
            }
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

    template <typename PointT>
    double getDepth(PointT pt)
    {
        return sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    }

    template <typename PointT>
    double getSqDepth(PointT pt)
    {
        return pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
    }

    template <typename PointT>
    Eigen::Vector3d getNormalizedPositionVector(PointT pt)
    {
        Eigen::Vector3d position_vector = getPositionVector(pt).normalized();
        return position_vector;
    }

    template <typename PointT>
    Eigen::Vector3d getPositionVector(PointT pt)
    {
        Eigen::Vector3d position_vector(pt.x, pt.y, pt.z);
        return position_vector;
    }

    template <typename PointT>
    Eigen::Vector3d getXNormalizedVector(PointT pt)
    {
        Eigen::Vector3d position_vector(1.0, pt.y/pt.x, pt.z/pt.x);
        return position_vector;
    }

    // template <typename PointT>
    // Eigen::Vector3d getDepthNormalizedVector(PointT pt)
    // {
    //     double depth = getDepth(pt);
    //     Eigen::Vector3d position_vector(pt.x/depth, pt.y/depth, pt.z/depth);
    //     // position_vector.normalize();
    //     return position_vector;
    // }

    template <typename PointT>
    Eigen::Vector3d getSurfaceNormal(PointT pt)
    {
        Eigen::Vector3d normal_vector(pt.normal_x, pt.normal_y, pt.normal_z);
        return normal_vector;
    }


    double toSec(builtin_interfaces::msg::Time header_stamp)
    {
        rclcpp::Time time = header_stamp;
        double nanoseconds = time.nanoseconds();

        return nanoseconds * 1e-9;
    }

    // get delta quaternion from angular rates
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

    void solveRotation(double dt, Eigen::Vector3d angular_velocity)
    {
        Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity); // mean gyrorate
        q_iMU *= deltaQ(un_gyr * dt);
        gyr_0 = angular_velocity;
    }

    void processIMU(double t_cur)
    {
        double rx = 0, ry = 0, rz = 0;
        size_t i = idx_imu;
        if (i >= imu_buffer.size())
            i--;
        while (toSec(imu_buffer[i]->header.stamp) < t_cur)
        {

            double t = toSec(imu_buffer[i]->header.stamp);
            if (current_time_imu < 0) // if not initialized?
                current_time_imu = t;
            double dt = t - current_time_imu; // dt should be got from time_ref or set static to 1/f
            current_time_imu = toSec(imu_buffer[i]->header.stamp);

            rx = imu_buffer[i]->angular_velocity.x;
            ry = imu_buffer[i]->angular_velocity.y;
            rz = imu_buffer[i]->angular_velocity.z;
            solveRotation(dt, Eigen::Vector3d(rx, ry, rz));
            i++;
            if (i >= imu_buffer.size())
                break;
        }

        if (i < imu_buffer.size())
        {
            double dt1 = t_cur - current_time_imu;
            double dt2 = toSec(imu_buffer[i]->header.stamp) - t_cur;

            double w1 = dt2 / (dt1 + dt2);
            double w2 = dt1 / (dt1 + dt2);

            rx = w1 * rx + w2 * imu_buffer[i]->angular_velocity.x;
            ry = w1 * ry + w2 * imu_buffer[i]->angular_velocity.y;
            rz = w1 * rz + w2 * imu_buffer[i]->angular_velocity.z;
            solveRotation(dt1, Eigen::Vector3d(rx, ry, rz));
        }
        current_time_imu = t_cur;
        idx_imu = i;


    }

    template <typename PointT>
    void calculatePointCurvature(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointType> &cloud_out)
    {
        size_t m = calculate_point_curvature_kernel_width_; // the "width" around a point, so the section size becomes 2m + 1
        size_t n = 2 * m + 1;
        size_t N = cloud_in.points.size();
        // double curvature_sum = 0.0; // to get mean of curvature
        for (size_t i = m; i < N - m; i++)
        {
            double point_norm = getDepth(cloud_in.points[i]);
            // double point_norm = sqrt(cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z);

            // Eigen::Vector3d diff(0.0,0.0,0.0);
            PointT diff_sum;
            diff_sum.x = 0;
            diff_sum.y = 0;
            diff_sum.z = 0;
            for (size_t j = i - m; j < i + m + 2; j++)
            {
                diff_sum.x += (cloud_in.points[i].x - cloud_in.points[j].x);
                diff_sum.y += (cloud_in.points[i].y - cloud_in.points[j].y);
                diff_sum.z += (cloud_in.points[i].z - cloud_in.points[j].z);
            }
            double section_norm = getDepth(diff_sum);
            double curvature = section_norm / (n * point_norm);
            // curvature_sum += curvature;

            cloud_out.points[i].curvature = curvature;
            // cloud_out.points[i].curvature = (curvature + 1) * (curvature + 1) - 1;
        }

        cloud_out.points.resize(N - 2 * m); // m amount of points have been removed on each end of the scan

        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(N - 2 * m);
        // cloud_out.is_dense = true;
    }

    template <typename PointT>
    void calculatePointPlateau(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointType> &cloud_out)
    {
        size_t m = calculate_point_plateau_kernel_width_; // the "width" around a point, so the section size becomes 2m + 1
        size_t n = 2 * m + 1;
        size_t N = cloud_in.points.size();

        // create kernels
        vector<double> kernel(n);
        vector<double> kernel_r(n);
        for (size_t i = 0; i < n; i++)
        {
            if (i < m)
            {
                kernel[i] = -1.0;
                kernel_r[i] = 0.0;
            }
            else if (i == m)
            {
                kernel[i] = (double)m;
                kernel_r[i] = (double)m;
            }
            else if (i > m)
            {
                kernel[i] = 0.0;
                kernel_r[i] = -1.0;
            }
        }

        double out = 0.0;
        double out_r = 0.0;
        for (size_t i = m; i < N - m; i++)
        {
            out = 0.0;
            out_r = 0.0;
            for (size_t j = i - m; j < i + n; j++)
            {
                out += cloud_in.points[j].x * kernel[-i + m + j];
                out_r += cloud_in.points[j].x * kernel_r[-i + m + j];
            }
            if (out > 0.0)
            {
                out = 0.0;
            }
            if (out_r > 0.0)
            {
                out_r = 0.0;
            }

            cloud_out.points[i].curvature = -(out + out_r);
        }
        cloud_out.points.resize(N - 2 * m); // m amount of points have been removed on each side of the scan
        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(N - 2 * m);
        cloud_out.is_dense = true;
    }

    template <typename PointT>
    void calculatePointNormals(const boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_in, pcl::PointCloud<PointType> &cloud_out)
    {
        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimationOMP<PointType, PointType> normal_estimator;
        normal_estimator.useSensorOriginAsViewPoint();
        normal_estimator.setInputCloud(cloud_in);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>()); // boost shared ptr?
        normal_estimator.setSearchMethod(tree);

        // // Output datasets
        // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

        if (pc_normal_search_radius_ > 0.0)
        {
            normal_estimator.setRadiusSearch(pc_normal_search_radius_); // Use all neighbors in a sphere of radius x meters
        }
        else
        {
            normal_estimator.setKSearch(pc_normal_knn_points_); // use x nearest points, more robust to cloud scale variation
        }

        // Compute the features
        normal_estimator.compute(cloud_out);

        // cloud_normals->size () should have the same size as the input cloud->size ()*
    }

    template <typename PointT>
    void removeStatisticalOutliers(const boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_in, pcl::PointCloud<PointT> &cloud_out)
    {
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(cloud_in);
        sor.setMeanK(remove_statistical_outliers_knn_points_);        // n amount of points to use in mean distance estimation
        sor.setStddevMulThresh(remove_statistical_outliers_std_mul_); // x*std outlier rejection threshold - 2.0 should cover 98% percent of gaussin dist points
        sor.filter(cloud_out);
    }

    template <typename PointT>
    void removeTransitionOutliers(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out)
    {
        // PointType point;
        size_t size_in = cloud_in.points.size();
        cloud_out.points[0] = cloud_in.points[0];
        size_t j = 1;
        // size_t section_start = 1;
        // size_t section_end = 0;
        // double nominal_step_length = 10.0;
        for (size_t i = 1; i < size_in-1; ++i)
        {
            Eigen::Vector3d shooting_vector_fore = getPositionVector(cloud_in.points[i-1]); // P
            Eigen::Vector3d shooting_vector_query = getPositionVector(cloud_in.points[i]); // Q
            Eigen::Vector3d shooting_vector_aft = getPositionVector(cloud_in.points[i+1]); 

            Eigen::Vector3d step_vector_to = (shooting_vector_query - shooting_vector_fore ); //
            // Eigen::Vector3d step_vector_from = (shooting_vector_aft - shooting_vector_query ); // 
            Eigen::Vector3d step_vector_across = (shooting_vector_fore - shooting_vector_aft ); // 
            // Eigen::Vector3d center_vector(1.0,0.0,0.0);
            // double step_length = (step_vector_to.norm()*step_vector_to.norm() + step_vector_from.norm()*step_vector_from.norm());
            // double step_length = (step_vector_to.norm() + step_vector_from.norm()) * (step_vector_to.norm() + step_vector_from.norm());
            double step_length = step_vector_to.norm();
            // double depth = getDepth(cloud_in.points[i]);
            // double cos_angle = abs(step_vector_to.normalized().dot(shooting_vector_fore.normalized()));
            // double cos_angle_from_center = abs(center_vector.dot(shooting_vector_query.normalized()));
            // double scan_wander = (shooting_vector_fore.normalized() - shooting_vector_aft.normalized()).norm()*depth;
            // scan_wander *= scan_wander;
            // double distance_to_shooting_vector = step_vector_to.cross(shooting_vector_fore).norm() / shooting_vector_fore.norm();
            
            double intensity = (double)floor(cloud_in.points[i].intensity);



            double cos_angle = abs(step_vector_across.normalized().dot(shooting_vector_query.normalized()));
            // if (distance_to_shooting_vector < remove_transition_outliers_cos_angle_threshold_ && cos_angle >= 0.9986){ 
            // if (step_length * (1.0/ (cos_angle_from_center*cos_angle_from_center))/(scan_wander) > remove_transition_outliers_cos_angle_threshold_  ) {//&& cos_angle >= 0.9986){ 
            bool incidence = cos_angle > remove_transition_outliers_cos_angle_threshold_ ;
            // bool hidden = step_length >= 0.1 * shooting_vector_query.norm() && shooting_vector_query.norm() > shooting_vector_fore.norm();
            bool hidden = false;
            bool intensity_thresh = intensity < 0.0  || intensity > 260.0;
            if (incidence || hidden || intensity_thresh){ 
            // if (distance_to_shooting_vector < remove_transition_outliers_cos_angle_threshold_){ 
                // RCLCPP_INFO(get_logger(), "point %i removed cos angle = %f", i, cos_angle);
                // RCLCPP_INFO(get_logger(), "scan_wander %f", scan_wander);
                // RCLCPP_INFO(get_logger(), "scan_wander/depth %f", scan_wander/depth);
                // RCLCPP_INFO(get_logger(), "distance_to_shooting_vector %f", distance_to_shooting_vector);
                // RCLCPP_INFO(get_logger(), "step_length %f", step_length);
                // RCLCPP_INFO(get_logger(), "step_length/depth %f", step_length/depth);
                // RCLCPP_INFO(get_logger(), "threshold %f", step_length/(scan_wander) /depth);
                continue;
            }
            // section_start = i;
            // RCLCPP_INFO(get_logger(), "point %i added %i cos angle = %f", i, j,cos_angle);
            cloud_out.points[j] = cloud_in.points[i]; 
            j++;                                      // count exchanged points
        }

        if (j != cloud_in.points.size())
        {
            cloud_out.points.resize(j);
        }

        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(j);
        cloud_out.is_dense = true;
        // RCLCPP_INFO(get_logger(), "points removed %i ", size_in - cloud_out.points.size());
    }
    

    // template <typename PointT>
    void assignEdgeAndSurface(boost::shared_ptr<pcl::PointCloud<PointType>> &edges, boost::shared_ptr<pcl::PointCloud<PointType>> &surfs)
    {

        size_t cloud_size = lidar_cloud_in->points.size();
        // this is in meters when using plateau
        //  double edge_threshold_ = 0.5; // edge treshold, in meters when using plateau
        //  double surfaces_threshold_ = 0.01; // surfaces threshold

        PointType point;
        for (size_t i = 0; i < cloud_size; i++)
        {
            point = lidar_cloud_in->points[i];


            if (point.curvature > surfaces_threshold_ && point.curvature < edge_threshold_)
            {
                continue;
            }
            else if (point.curvature > edge_threshold_)
            {
                edges->points.push_back(point);
                // RCLCPP_INFO(get_logger(), "Point should be added to edge");
            }
            else if (point.curvature < surfaces_threshold_)
            {
                surfs->points.push_back(point);
                // RCLCPP_INFO(get_logger(), "Point should be added to surf");
            }
        }
    }

    template <typename PointT>
    void normalizeIntensities(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointType> &cloud_out)
    {
        double range_reference = normalize_intensities_reference_range_; // could be actively determined from cloudscale..
        double intensity_normalized = 0.0;
        Eigen::Vector3d range_vector(0.0, 0.0, 0.0);
        Eigen::Vector3d normal_vector(0.0, 0.0, 0.0);
        // double max_intesity = std::numeric_limits<double>::min ();
        for (size_t i = 0; i < cloud_in.points.size(); ++i)
        {
            double range = getDepth(cloud_in.points[i]);
            normal_vector = getSurfaceNormal(cloud_in.points[i]);
            range_vector = -getNormalizedPositionVector(cloud_in.points[i]);
            double cos_incidence_angle = abs(range_vector.transpose() * normal_vector);

            double intensity = (double)cloud_in.points[i].intensity;
            intensity_normalized = intensity * (range / range_reference) * (range / range_reference) * (1.0 / cos_incidence_angle);
            // intensity_normalized = intensity * (range / range_reference) * (1.0 / cos_incidence_angle);
            // intensity_normalized = intensity * (1.0 / cos_incidence_angle);

            if (intensity_normalized > 255.0)
            {
                intensity_normalized = 255.0;
            }

            // if (intensity_normalized > max_intesity) {
            //     max_intesity = intensity_normalized;
            // }

            cloud_out.points[i].intensity = (int)round(intensity_normalized);
        }

        // for (size_t i = 0; i < cloud_in.points.size(); ++i) {
        //     cloud_out.points[i].intensity = (int)round(cloud_out.points[i].intensity/ max_intesity * 255.0);
        // }
    }

    template <typename PointT>
    PointT undistortPoint(PointT pt, const Eigen::Quaterniond quat) {
        
        Eigen::Vector3d pt_i(pt.x, pt.y, pt.z);
        Eigen::Vector3d pt_s = quat * pt_i; // should in theory also have to account for the offset between lidar and IMU here?

        PointT p_out;
        p_out.x = pt_s.x();
        p_out.y = pt_s.y();
        p_out.z = pt_s.z();
        p_out.intensity = pt.intensity;
        // p_out.curvature = pt.curvature;
        return p_out;
    }

    template <typename PointT>
    void undistortCloud(const boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_in, boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_out)
    {
        double scan_dt = cloud_out->points[cloud_in->points.size()-1].intensity - (int)cloud_out->points[cloud_in->points.size()-1].intensity;
        // RCLCPP_INFO(get_logger(), "scan dt undistortion: %f", scan_dt);
        // double point_dt = scan_time / cloud_in->points.size(); // should be the first thing of the preprocessing for this reason

        Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();


        for (size_t i=0; i < cloud_in->points.size(); i++){
            //  double dt_i = point_dt * i;
            double intensity = cloud_out->points[i].intensity;
            double dt_i = intensity - (int)intensity;
            double ratio_i = dt_i / scan_dt;
            if(ratio_i >= 1.0) {
                ratio_i = 1.0;
            }

            // Eigen::Quaterniond quat_spheric_interp = q0.slerp(1 - ratio_i, q_iMU.inverse()); // 1-ratio because the last points are the most distorted
            Eigen::Quaterniond quat_spheric_interp = q0.slerp(ratio_i, q_iMU); // 1-ratio because the last points are the most distorted

            cloud_out->points[i] = undistortPoint(cloud_in->points[i], quat_spheric_interp);

        }
    }


    template <typename PointT>
    void embedPointTime(const boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_in, boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_out, const double scan_dt){
        double scan_time = scan_dt;
        // double scan_time = 0.1;
        double point_dt = scan_time / cloud_in->points.size();

        for (size_t i=0; i < cloud_in->points.size(); i++){

            double dt_i = point_dt * (i+1); // +1 because the last point in the previous scan is considered t=0 therefore first(zeroth) point in this scan is at t=1*dt
            // this also makes sure that the full scan_time is imbedded in the final point, and can be retrieved for later use. 
            cloud_out->points[i].intensity += dt_i;
        }
    }

    template <typename PointT>
    void embedPointTimeVelodyne(const boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_in, boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_out,  const double scan_dt){
        double scan_time = scan_dt;
        // double scan_time = 0.1;
        double point_dt = scan_time / cloud_in->points.size();

        for (size_t i=0; i < cloud_in->points.size(); i++){
            double azimuth = std::atan2(cloud_out->points[i].x, cloud_out->points[i].y) + cloud_out->points[i].x < 0 ? 2.0 * M_PI: 0;
            double c = azimuth / (2.0 * M_PI);
            double dt_i = point_dt * c; // +1 because the last point in the previous scan is considered t=0 therefore first(zeroth) point in this scan is at t=1*dt
            // this also makes sure that the full scan_time is imbedded in the final point, and can be retrieved for later use. 
            cloud_out->points[i].intensity += dt_i;
        }
    }

    void processNext()
    {

        // get next frame in buffer..
        if (cloud_queue.size() <= 1) // leave atleast 1, to get the time of the next cloud.. (or set next time to + 0.1 (scanning time) staticly)
        {
            return;
        }
        else
        {
            time_scan = toSec(cloud_queue.front().header.stamp);
            current_cloud_msg = cloud_queue.front(); // puts the next cloud to variable current
            cloud_queue.pop_front();                 // removes element from the queue

            // set header id and timestamp
            cloud_header = current_cloud_msg.header;
            cloud_header.frame_id = frame_id;

            time_scan_next = toSec(cloud_queue.front().header.stamp); // why take time from the next? and should it be double?
            // !answer: the start-time of the next scan is the end of the current, this is needed to integrate the imu, and thats why the buffer needs to leave 1 frame

            if (cloud_queue.size() > 5)
            {
                RCLCPP_WARN(get_logger(), "Buffer size is: %i", cloud_queue.size());
            }
        }

        latest_frame_idx++;
        if (latest_frame_idx < start_idx_ || (latest_frame_idx > end_idx_ && end_idx_ > 0) ){
            RCLCPP_INFO(get_logger(), "Skipping frames %i", latest_frame_idx);
            return;
        }

        #ifndef __INTELLISENSE__ // to ignore error from missing overload of function, should still work on the new sensor_msg::msg::PointCloud2
        pcl::fromROSMsg(current_cloud_msg, *lidar_cloud_in_no_normals);
        #endif

        // imbed the scan time of the individual point to the intesity value which is then the integer part, and the time is the fractional part
        if ( lidar_source_topic_.compare("/velodyne") ==0) {
            // embedPointTimeVelodyne(lidar_cloud_in_no_normals, lidar_cloud_in_no_normals, time_scan_next - time_scan);
            embedPointTime(lidar_cloud_in_no_normals, lidar_cloud_in_no_normals, time_scan_next - time_scan);
        } else {
            embedPointTime(lidar_cloud_in_no_normals, lidar_cloud_in_no_normals, time_scan_next - time_scan);
        }


        // UNDISTORTION SHOULD BE THE FIRST PROCESSING STEP
        if (use_gyroscopic_undistortion_){
            // imu_stuff..
            int tmp_idx = 0;
            if(idx_imu > 0) 
                tmp_idx = idx_imu - 1; // to not get bad indexing outside array?
            if (imu_buffer.empty()) // if no imu data or time is ahead of lidar?
            {
                // ROS_WARN("Waiting for IMU data ...");
                RCLCPP_INFO(get_logger(), "Waiting for IMU data..");
                return; // skips the lidar scan
            } else if (toSec(imu_buffer[tmp_idx]->header.stamp) > time_scan_next)
            {
                RCLCPP_INFO(get_logger(), "IMU data is ahead..");
                return;
            }
        
       
            // if(imu_buffer.size() > 0) // this was just checked above
            processIMU(time_scan_next);
            RCLCPP_DEBUG(get_logger(), "IMU qauternion: w: %f, x: %f, y: %f, z: %f", q_iMU.w(), q_iMU.x(), q_iMU.y(), q_iMU.z());

            if(isnan(q_iMU.w()) || isnan(q_iMU.x()) || isnan(q_iMU.y()) || isnan(q_iMU.z())) { // reset the imu orientation if values becomes nan
                q_iMU = Eigen::Quaterniond::Identity();
            }

            undistortCloud(lidar_cloud_in_no_normals, lidar_cloud_in_no_normals);
            q_iMU = Eigen::Quaterniond::Identity();
        }

        // ESSENTIAL PREPROCESSING
        removeClosedPointCloud(*lidar_cloud_in_no_normals, *lidar_cloud_in_no_normals, filter_close_points_distance_m_); // removes invalid points within a distance of x m from the center of the lidar
        removeTransitionOutliers(*lidar_cloud_in_no_normals, *lidar_cloud_in_no_normals);
        removeStatisticalOutliers(lidar_cloud_in_no_normals, *lidar_cloud_in_no_normals);
        pcl::copyPointCloud(*lidar_cloud_in_no_normals, *lidar_cloud_in); // change pointtype to point cloud with normal data
        if ( lidar_source_topic_.compare("/velodyne") ==0) { // downsample if cloud is from velodyne
            down_size_filter.setInputCloud(lidar_cloud_in);
            down_size_filter.filter(*lidar_cloud_in);
        } else {
        }
        calculatePointNormals(lidar_cloud_in, *lidar_cloud_in); // openMP multi-processing accelerated

        // NON-ESSENTIAL PREPROCESSING. The if statements are slow and therefore uncommented. This needs a complex solution to determine bools at compile time 
        if (normalize_intensities_reference_range_ > 0.0)
        {
            normalizeIntensities(*lidar_cloud_in, *lidar_cloud_in);
        }

        if (calculate_point_curvature_kernel_width_ > 0)
        {
            calculatePointCurvature(*lidar_cloud_in, *lidar_cloud_in); // this is done in normal calculation anyway, so never needed again
        }

        if (calculate_point_plateau_kernel_width_ > 0)
        {
            calculatePointPlateau(*lidar_cloud_in, *lidar_cloud_in); // disabled because i'm not yet using the edges
        }



        pcl::PointCloud<PointType>::Ptr surf_features = boost::make_shared<pcl::PointCloud<PointType>>(); // (new pcl::PointCloud<PointType>()); // boost shared ptr??
        pcl::PointCloud<PointType>::Ptr edge_features = boost::make_shared<pcl::PointCloud<PointType>>(); // (new pcl::PointCloud<PointType>());

        // TODO separte these into two function such that edge points can be extracted without using processing power for surf points? -> doesn't matter as it loops once..
        assignEdgeAndSurface(edge_features, surf_features);


        // TODO make a toROSMsg overload in a header using the intellisense block out!
        // publish clouds created by the preprocessor
        sensor_msgs::msg::PointCloud2 surf_features_msg;
#ifndef __INTELLISENSE__
        pcl::toROSMsg(*surf_features, surf_features_msg);
#endif
        surf_features_msg.header.stamp = cloud_header.stamp;
        surf_features_msg.header.frame_id = frame_id;
        pub_surf->publish(surf_features_msg);

        sensor_msgs::msg::PointCloud2 edge_features_msg;
#ifndef __INTELLISENSE__
        pcl::toROSMsg(*edge_features, edge_features_msg);
#endif
        edge_features_msg.header.stamp = cloud_header.stamp;
        edge_features_msg.header.frame_id = frame_id;
        pub_edge->publish(edge_features_msg);

        sensor_msgs::msg::PointCloud2 cloud_cutted_msg;
#ifndef __INTELLISENSE__
        pcl::toROSMsg(*lidar_cloud_in, cloud_cutted_msg);
#endif
        cloud_cutted_msg.header.stamp = cloud_header.stamp;
        cloud_cutted_msg.header.frame_id = frame_id;
        pub_cutted_cloud->publish(cloud_cutted_msg);

        // q_iMU = Eigen::Quaterniond::Identity();
        // t_pre.tic_toc();
        // runtime += t_pre.toc();
        // cout<<"pre_num: "<<++pre_num<<endl;
        // cout<<"Preprocessing average run time: "<<runtime / pre_num<<endl;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto processing_node = std::make_shared<Preprocessing>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(processing_node);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}