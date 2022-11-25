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
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/statistical_outlier_removal.h>

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
typedef pcl::PointXYZINormal PointType; // need to calculate and assign normals and the change to PointXYZINormals

using namespace std;
using std::placeholders::_1;




class Preprocessing : public rclcpp::Node {
private:

    rclcpp::CallbackGroup::SharedPtr subscriber_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_Lidar_cloud;
    rclcpp::TimerBase::SharedPtr process_timer;
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

    // vector<sensor_msgs::ImuConstPtr> imu_buf;
    // int idx_imu = 0;
    // double current_time_imu = -1;

    // Eigen::Vector3d gyr_0;
    // Eigen::Quaterniond q_iMU = Eigen::Quaterniond::Identity();
    // bool first_imu = false;

    std::deque<sensor_msgs::msg::PointCloud2> cloud_queue;
    sensor_msgs::msg::PointCloud2 current_cloud_msg;

    // double time_scan_next; // convert to rclcpp::Time?
    rclcpp::Time time_scan_next;

    // int N_SCANS = 6;
    // int H_SCANS = 4000;

    string frame_id = "lidar_preproc";
    double edge_thres, surf_thres;
    double runtime = 0;
    

    // double normal_search_radius = 0.2; // small scale / indoors
    double normal_search_radius = 4.0; // large scale / outdoors

public:
    Preprocessing(): 
        rclcpp::Node("preprocessing_velodyne") {
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

        subscriber_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // create subscriber callback group
        rclcpp::SubscriptionOptions options; // create subscriver options
        options.callback_group = subscriber_cb_group_; // add callbackgroup to subscriber options
        sub_Lidar_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_points", 100, std::bind(&Preprocessing::cloudHandler, this, _1), options); // add subscriber options to the subsriber constructor call..

        timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        process_timer = this->create_wall_timer(5ms, std::bind(&Preprocessing::processNext, this), timer_cb_group_);  

        // sub_imu = nh.subscribe<sensor_msgs::Imu>("/livox/imu", 200, &Preprocessing::imuHandler, this);


        pub_surf = this->create_publisher<sensor_msgs::msg::PointCloud2>("/surf_features", 100);
        pub_edge = this->create_publisher<sensor_msgs::msg::PointCloud2>("/edge_features", 100);
        pub_cutted_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/preprocessed_point_cloud", 100);



    }

    ~Preprocessing(){}

    // function that can remove points that are too close to the scanner i.e. auto-scans, weird function name will have to change it when I now how far i goes
    template <typename PointT>
    void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, float thres) {

        // if the cloud are not the same, make sure head and size are the same
        if (&cloud_in != &cloud_out) {
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize(cloud_in.points.size());
        }

        size_t j = 0;

        for (size_t i = 0; i < cloud_in.points.size(); ++i) {
            if (cloud_in.points[i].x * cloud_in.points[i].x +
                    cloud_in.points[i].y * cloud_in.points[i].y +
                    cloud_in.points[i].z * cloud_in.points[i].z < thres * thres) // calculating the squared distance of a point and comparing it to a threshold
                continue;
            if (!pcl::isFinite(cloud_in.points[i]) || !pcl::isFinite(cloud_in.points[i])){
                continue;
            }
            cloud_out.points[j] = cloud_in.points[i];  // if they are beyond the threshold assign the point to cloud out
            j++; // count exchanged points
        }
        if (j != cloud_in.points.size()) {
            cloud_out.points.resize(j);
        }

        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(j);
        cloud_out.is_dense = true;
    }

    template <typename PointT>
    double getDepth(PointT pt) {
        return sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
    }

    // PointXYZINormal undistortion(PointXYZINormal pt, const Eigen::Quaterniond quat) {
    //     double dt = 0.1;
    //     int line = int(pt.intensity);
    //     double dt_i = pt.intensity - line;

    //     double ratio_i = dt_i / dt;
    //     if(ratio_i >= 1.0) {
    //         ratio_i = 1.0;
    //     }

    //     Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
    //     Eigen::Quaterniond q_si = q0.slerp(ratio_i, q_iMU);

    //     Eigen::Vector3d pt_i(pt.x, pt.y, pt.z);
    //     Eigen::Vector3d pt_s = q_si * pt_i;

    //     PointXYZINormal p_out;
    //     p_out.x = pt_s.x();
    //     p_out.y = pt_s.y();
    //     p_out.z = pt_s.z();
    //     p_out.intensity = pt.intensity;
    //     p_out.curvature = pt.curvature;
    //     return p_out;
    // }

    // void solveRotation(double dt, Eigen::Vector3d angular_velocity) {
    //     Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity);
    //     q_iMU *= deltaQ(un_gyr * dt);
    //     gyr_0 = angular_velocity;
    // }

    // void processIMU(double t_cur) {
    //     double rx = 0, ry = 0, rz = 0;
    //     int i = idx_imu;
    //     if(i >= imu_buf.size())
    //         i--;
    //     while(imu_buf[i]->header.stamp.toSec() < t_cur) {

    //         double t = imu_buf[i]->header.stamp.toSec();
    //         if (current_time_imu < 0)
    //             current_time_imu = t;
    //         double dt = t - current_time_imu;
    //         current_time_imu = imu_buf[i]->header.stamp.toSec();

    //         rx = imu_buf[i]->angular_velocity.x;
    //         ry = imu_buf[i]->angular_velocity.y;
    //         rz = imu_buf[i]->angular_velocity.z;
    //         solveRotation(dt, Eigen::Vector3d(rx, ry, rz));
    //         i++;
    //         if(i >= imu_buf.size())
    //             break;
    //     }

    //     if(i < imu_buf.size()) {
    //         double dt1 = t_cur - current_time_imu;
    //         double dt2 = imu_buf[i]->header.stamp.toSec() - t_cur;

    //         double w1 = dt2 / (dt1 + dt2);
    //         double w2 = dt1 / (dt1 + dt2);

    //         rx = w1 * rx + w2 * imu_buf[i]->angular_velocity.x;
    //         ry = w1 * ry + w2 * imu_buf[i]->angular_velocity.y;
    //         rz = w1 * rz + w2 * imu_buf[i]->angular_velocity.z;
    //         solveRotation(dt1, Eigen::Vector3d(rx, ry, rz));
    //     }
    //     current_time_imu = t_cur;
    //     idx_imu = i;
    // }

    // void imuHandler(const sensor_msgs::ImuConstPtr& imu_in) {
    //     imu_buf.push_back(imu_in);

    //     if(imu_buf.size() > 600)
    //         imu_buf[imu_buf.size() - 601] = nullptr;

    //     if (current_time_imu < 0)
    //         current_time_imu = imu_in->header.stamp.toSec();

    //     if (!first_imu)
    //     {
    //         first_imu = true;
    //         double rx = 0, ry = 0, rz = 0;
    //         rx = imu_in->angular_velocity.x;
    //         ry = imu_in->angular_velocity.y;
    //         rz = imu_in->angular_velocity.z;
    //         Eigen::Vector3d angular_velocity(rx, ry, rz);
    //         gyr_0 = angular_velocity;
    //     }
    // }


    template <typename PointT>
    void calculatePointCurvature(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointType> &cloud_out)
    {
        size_t m = 1; // the "width" around a point, so the section size becomes 2m + 1
        size_t n = 2*m + 1;
        size_t N = cloud_in.points.size();
        // double curvature_sum = 0.0; // to get mean of curvature
        for (size_t i = m; i < N - m; i++) {
            double point_norm = getDepth(cloud_in.points[i]);
            // double point_norm = sqrt(cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z);

            // Eigen::Vector3d diff(0.0,0.0,0.0);
            PointT diff_sum;
            diff_sum.x = 0;
            diff_sum.y = 0;
            diff_sum.z = 0;
            for (size_t j = i-m; j < i+m+2; j++) {     
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

        
        cloud_out.points.resize(N - 2*m); // m amount of points have been removed on each side of the scan
        
        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(N - 2*m);
        // cloud_out.is_dense = true;
    }

    template <typename PointT>
    void calculatePointPlateau(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointType> &cloud_out, int m)
    {
        // size_t m = ; // the "width" around a point, so the section size becomes 2m + 1
        int n = 2*m + 1;
        size_t N = cloud_in.points.size();

        // create kernels
        vector<double> kernel(n);
        vector<double> kernel_r(n);
        for (int i=0; i< n ; i++){
            if ( i < m) {
                kernel[i] = -1.0;
                kernel_r[i] = 0.0;
            } else if (i == m){
                kernel[i] = (double)m;
                kernel_r[i] = (double)m;
            }else if (i > m)
            {
                kernel[i] = 0.0;
                kernel_r[i] = -1.0;
            }
        }

        double out = 0.0;
        double out_r = 0.0;
        for (size_t i = m; i < N - m; i++) {
            out = 0.0;
            out_r = 0.0;
            for (size_t j = i-m; j < i+n ; j++){
                out += cloud_in.points[j].x * kernel[-i+m+j];
                out_r += cloud_in.points[j].x * kernel_r[-i+m+j];
            }
            if (out > 0.0){
                out = 0.0;
            }
            if (out_r > 0.0){
                out_r = 0.0;
            }
    
            cloud_out.points[i].curvature = -(out + out_r);
        }
        cloud_out.points.resize(N - 2*m); // m amount of points have been removed on each side of the scan
        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(N - 2*m);
        cloud_out.is_dense = true;
    }


    
    template <typename PointT>
    void calculatePointNormals(const boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_in, pcl::PointCloud<PointType> &cloud_out)
    {
        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimationOMP<PointType, PointType> normal_estimator;
        normal_estimator.setInputCloud(cloud_in);
        
        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ()); // boost shared ptr?
        normal_estimator.setSearchMethod(tree);
        
        // // Output datasets
        // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        
        // Use all neighbors in a sphere of radius 3cm
        normal_estimator.setRadiusSearch(normal_search_radius);
        
        // Compute the features
        normal_estimator.compute(cloud_out);
        
        // cloud_normals->size () should have the same size as the input cloud->size ()*
    }



    template <typename PointT>
    void removeStatisticalOutliers(const boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_in, pcl::PointCloud<PointTypeNoNormals> &cloud_out)
    {
        pcl::StatisticalOutlierRemoval<PointTypeNoNormals> sor;
        sor.setInputCloud(cloud_in);
        sor.setMeanK(10); // n amount of points to use in mean distance estimation
        sor.setStddevMulThresh(1.5); // x*std outlier rejection threshold - 2.0 should cover 98% percent of gaussin dist points
        sor.filter(cloud_out);
    }


    // template <typename PointT>
    void assignEdgeAndSurface( boost::shared_ptr<pcl::PointCloud<PointType>> &edges, boost::shared_ptr<pcl::PointCloud<PointType>> &surfs) 
    {

        size_t cloud_size = lidar_cloud_in->points.size();
        //in meters when using plateau
        double T_edge = 0.2; // edge treshold, in meters when using plateau
        double T_surf = 0.01; // surfaces threshold

        PointType point;
        for (size_t i = 0; i < cloud_size; i++) {
            point = lidar_cloud_in->points[i];
            // point.x = lidar_cloud_in->points[i].x;
            // point.y = lidar_cloud_in->points[i].y;
            // point.z = lidar_cloud_in->points[i].z;
            // point.normal_x = lidar_cloud_in->points[i].normal_x;
            // point.normal_y = lidar_cloud_in->points[i].normal_y;
            // point.normal_z = lidar_cloud_in->points[i].normal_z;
            // point.intensity = lidar_cloud_in->points[i].intensity;
            // point.curvature = lidar_cloud_in->points[i].curvature;


            if (point.curvature > T_surf && point.curvature < T_edge){
                continue;
            }
            else if (point.curvature > T_edge){
                edges->points.push_back(point);
                // RCLCPP_INFO(get_logger(), "Point should be added to edge");
            } 
            else if (point.curvature < T_surf){
                surfs->points.push_back(point);
                // RCLCPP_INFO(get_logger(), "Point should be added to surf");
            }
        }
    }



    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_cloud_msg) 
    {
        // cache point cloud
        cloud_queue.push_back(*lidar_cloud_msg);
    }


    void processNext()
    {

        // get next frame in buffer..
        if (cloud_queue.size() <= 0) { 
            return; 
        } else {
            RCLCPP_INFO(get_logger(), "Buffer size is: %i", cloud_queue.size());
            current_cloud_msg = cloud_queue.front(); // puts the next cloud to variable current
            cloud_queue.pop_front(); // removes element from the queue

            // set header id and timestamp
            cloud_header = current_cloud_msg.header; 
            cloud_header.frame_id = frame_id;
 
            time_scan_next = current_cloud_msg.header.stamp; // why take time from the next? and should it be double?

        } 

        // imu_stuff..
        // int tmp_idx = 0;
        // if(idx_imu > 0) // initialized as -1
        //     tmp_idx = idx_imu - 1;
        // if (imu_buf.empty() || imu_buf[tmp_idx]->header.stamp.toSec() > time_scan_next)
        // {
        //     ROS_WARN("Waiting for IMU data ...");
        //     return;
        // }
        // timer is just a matlab tic toc equivalent and is found in timer.h of the original project "lili-om" on github
        // Timer t_pre("Preprocessing"); 

        #ifndef __INTELLISENSE__  // to ignore error from missing overload of function, should still work on the new sensor_msg::msg::PointCloud2
        pcl::fromROSMsg(current_cloud_msg, *lidar_cloud_in_no_normals);
        #endif

        // lidar_cloud_cutted->clear();

        // RCLCPP_INFO(get_logger(), "num of points before: %i", lidar_cloud_in->points.size());

        // std::vector<int> indices;
        // pcl::removeNaNFromPointCloud(*lidar_cloud_in, *lidar_cloud_in, indices);

        removeClosedPointCloud(*lidar_cloud_in_no_normals, *lidar_cloud_in_no_normals, 2.5); // removes invalid points within a distance of x m from the center of the lidar
        removeStatisticalOutliers(lidar_cloud_in_no_normals, *lidar_cloud_in_no_normals);
        pcl::copyPointCloud(*lidar_cloud_in_no_normals, *lidar_cloud_in);
        calculatePointNormals(lidar_cloud_in, *lidar_cloud_in); // openMP multi-processing accelerated 
        // calculatePointCurvature(*lidar_cloud_in, *lidar_cloud_in); // this is done in normal calculation anyway, so never needed again

        // calculatePointPlateau(*lidar_cloud_in, *lidar_cloud_in, 2); // disabled because i'm not yet using the edges

        pcl::PointCloud<PointType>::Ptr surf_features = boost::make_shared<pcl::PointCloud<PointType>>(); // (new pcl::PointCloud<PointType>()); // boost shared ptr??
        pcl::PointCloud<PointType>::Ptr edge_features = boost::make_shared<pcl::PointCloud<PointType>>(); // (new pcl::PointCloud<PointType>());

        // assignEdgeAndSurface(edge_features, surf_features);


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
        //t_pre.tic_toc();
        // runtime += t_pre.toc();
        //cout<<"pre_num: "<<++pre_num<<endl;
        //cout<<"Preprocessing average run time: "<<runtime / pre_num<<endl;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto processing_node = std::make_shared<Preprocessing>();
    rclcpp::executors::MultiThreadedExecutor executor;    
    executor.add_node(processing_node);
    
    executor.spin();
    rclcpp::shutdown();
    return 0;

}