#include <memory>
#include <cstdio>


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"




using std::placeholders::_1;

class ImuBroadcaster : public rclcpp::Node{
    public:
        ImuBroadcaster()
        : Node("imu_tf_broadcaster")
        {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            data_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, std::bind(&ImuBroadcaster::data_subscriber, this, _1));
            q_subscriber_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>("/filter/quaternion", 10, std::bind(&ImuBroadcaster::quat_subscriber, this, _1));
            p_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>("/filter/positionxyz", 10, std::bind(&ImuBroadcaster::position_subscriber, this, _1));

            RCLCPP_INFO(get_logger(), "Listening for update to broadcast..");
        }

    private:

        void data_subscriber(const std::shared_ptr<sensor_msgs::msg::Imu> data)
        {
            // rclcpp::Time now = this->get_clock()->now();
            geometry_msgs::msg::TransformStamped t;
            t_.header.stamp = this->get_clock()->now();;
            t_.header.frame_id = "odom";
            t_.child_frame_id = "imu_link";

            t_.transform.rotation.w = data->orientation.w;
            t_.transform.rotation.x = data->orientation.x;
            t_.transform.rotation.y = data->orientation.y;
            t_.transform.rotation.z = data->orientation.z;

            broadcast();
        }

        void position_subscriber( const std::shared_ptr<geometry_msgs::msg::Point> point)
        {
             // lock to origo at the moment..
            t_.transform.translation.x = point->x;            
            t_.transform.translation.y = point->y;  
            t_.transform.translation.z = point->z;
        }

        void quat_subscriber(const std::shared_ptr<geometry_msgs::msg::QuaternionStamped> quaternion_stamped)
        {
            rclcpp::Time now = this->get_clock()->now();
            geometry_msgs::msg::TransformStamped t;
            t_.header.stamp = now;
            t_.header.frame_id = "odom";
            t_.child_frame_id = "imu_link";


            t_.transform.rotation.w = quaternion_stamped->quaternion.w;
            t_.transform.rotation.x = quaternion_stamped->quaternion.x;
            t_.transform.rotation.y = quaternion_stamped->quaternion.y;
            t_.transform.rotation.z = quaternion_stamped->quaternion.z;

            // broadcast();
        }
        
        void broadcast()
        {
            tf_broadcaster_->sendTransform(t_);
        }


        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr data_subscriber_;

        rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr q_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr p_subscriber_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        geometry_msgs::msg::TransformStamped t_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuBroadcaster>());
    rclcpp::shutdown();
    return 0;

}