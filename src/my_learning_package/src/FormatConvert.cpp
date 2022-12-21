#define BOOST_BIND_NO_PLACEHOLDERS

// #include "include/my_learning_package/common.h"
#include "rclcpp/rclcpp.hpp"
// #include "livox_ros_driver/custom_msg.h"
// #include "livox_interfaces/msg"
// #include "livox_interfaces/msg/custom_point.hpp"
// #include "livox_interfaces/msg/custom_msg.hpp"
// #include "livox_ros2_driver/include/livox_ros2_driver.h"
// #include "utils/common.h"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>


#define  PI  3.1415926535

using namespace std;
using std::placeholders::_1;


typedef pcl::PointXYZI PointTypeNoNormals; // need to calculate and assign normals and the change to PointXYZINormals
typedef pcl::PointXYZINormal PointType; // need to calculate and assign normals and the change to PointXYZINormals

// ros::Publisher pub_ros_points;

// livox_sdk::Custommsg
class FormatConvert : public rclcpp::Node
{
    private:
        string frame_id = "lidar_link";    
        /* data */
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ros_pc2_msg;
        rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr sub_lidar_msg;
    public:
        FormatConvert():
        rclcpp::Node("lidar_format_converter")
        {
            sub_lidar_msg = this->create_subscription<livox_interfaces::msg::CustomMsg>("/livox/lidar", 100, std::bind(&FormatConvert::livoxLidarHandler, this, _1)); 

            pub_ros_pc2_msg = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar", 100);

        }
        ~FormatConvert(){}

        void livoxLidarHandler(const livox_interfaces::msg::CustomMsg::ConstPtr livox_msg_in) 
        {
            pcl::PointCloud<PointType> pcl_in;
            // auto time_end = livox_msg_in->points.back().offset_time;
            for (unsigned int i = 0; i < livox_msg_in->point_num; ++i) {
                PointType pt;
                pt.x = livox_msg_in->points[i].x;
                pt.y = livox_msg_in->points[i].y;
                pt.z = livox_msg_in->points[i].z;
                // float s = float(livox_msg_in->points[i].offset_time / (float)time_end); // decimal 0 to 1 relative point offset time.
                pt.intensity = livox_msg_in->points[i].reflectivity; // TODO: what is this? integer part: line number; decimal /part: timestamp
                // pt.intensity = livox_msg_in->points[i].line + s*0.1; // TODO: what is this? integer part: line number; decimal /part: timestamp
                // pt.curvature = 0.1 * livox_msg_in->points[i].reflectivity;
                pcl_in.push_back(pt);
            }

            /// timebase 5ms ~ 50000000, so 10 ~ 1ns

            rclcpp::Time timestamp(livox_msg_in->header.stamp);

            sensor_msgs::msg::PointCloud2 pc2_msg;
            #ifndef __INTELLISENSE__
            pcl::toROSMsg(pcl_in, pcl_ros_msg);
            #endif
            pc2_msg.header.stamp = timestamp;
            pc2_msg.header.frame_id = frame_id;

            pub_ros_pc2_msg->publish(pc2_msg);
        }
};




int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FormatConvert>());
    rclcpp::shutdown();
    return 0;
}