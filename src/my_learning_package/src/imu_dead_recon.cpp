#include <memory>
#include <cstdio>

// #include <queue>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/msg/imu.hpp"


using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
    public:
        MinimalSubscriber() // constructer
        : Node("imu_dead_recon")
        {
            RCLCPP_INFO(get_logger(), "Starting IMU dead reckoning..");


            // init_time = this->get_clock()->now();

            position_.x = 0.0;
            position_.y = 0.0;
            position_.z = 0.0;
            velocity_.vector.x = 0.0;
            velocity_.vector.y = 0.0;
            velocity_.vector.z = 0.0;

            tf2::Quaternion init_att;
            init_att.setRPY(0.0, 0.0, 0.0);
            attitude_.quaternion = tf2::toMsg(init_att);



            data_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("imu/data_raw", 1, std::bind(&MinimalSubscriber::subscribe_data, this, _1));
            // imu/data is the output topic of the madgwick filter which processes the ouput from the imu directly via imu/data_raw

            // dv_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("imu/dv", 1, std::bind(&MinimalSubscriber::subscribe_dv, this, _1));
            // dq_subscriber_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>("imu/dq", 1, std::bind(&MinimalSubscriber::subscribe_dq, this, _1));

            // q_publisher_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>("/filter/quaternion", 1);
            p_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/filter/positionxyz", 1); // TODO: try with pose-type
            v_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/filter/velocity", 1);
           
            // following should be loaded from a yaml created from a calibration script! or found at initilasation
            dv_static_bias.x = 0.001390;
            dv_static_bias.y = 0.00019;
            // dv_static_bias.z = -0.0245;
            dv_static_bias.z = -0.09785;
        }

    private:
        void subscribe_data(const sensor_msgs::msg::Imu::SharedPtr data)
        {
            velocity_.header.stamp = data->header.stamp;

            
            geometry_msgs::msg::Vector3 linear_acceleration_w;
            linear_acceleration_w = data->linear_acceleration;
            // apply bias in the imu-frame axes
            linear_acceleration_w.x += dv_static_bias.x;
            linear_acceleration_w.y += dv_static_bias.y;
            linear_acceleration_w.z += dv_static_bias.z;

            // rotate into world or odom frame
            tf2::Vector3 linear_acceleration_tf2;
            tf2::Vector3 linear_acceleration_tf2_prime;
            tf2::Quaternion orientation_tf2;
            tf2::Vector3 gravity;
            setTF2vector(gravity, 0.0, 0.0, 9.815);
            
            // tf2::fromMsg(linear_acceleration_w, linear_acceleration_tf2);
            linear_acceleration_tf2 = tf2::Vector3(linear_acceleration_w.x, linear_acceleration_w.y, linear_acceleration_w.z);
            tf2::fromMsg(data->orientation, orientation_tf2);

            // inverse(orientation_tf2) * linear_acceleration_tf2; // this gives a quaternion?

            rotate_vector_by_quaternion(linear_acceleration_tf2, inverse(orientation_tf2), linear_acceleration_tf2_prime);
            // linear_acceleration_tf2_prime -= gravity;

            // linear_acceleration_w.vector = tf2::toMsg(linear_acceleration_tf2_prime);  doesn't work in foxy.. reestablished in galactic..
            linear_acceleration_w.x = linear_acceleration_tf2_prime.getX();
            linear_acceleration_w.y = linear_acceleration_tf2_prime.getY();
            linear_acceleration_w.z = linear_acceleration_tf2_prime.getZ();
            

            // add the rotated acceleration to velocity
            // velocity_.vector.x += (linear_acceleration_w.x * 1.0/400.0);
            // velocity_.vector.y += (linear_acceleration_w.y * 1.0/400.0);
            // velocity_.vector.z += (linear_acceleration_w.z * 1.0/400.0);
            add_velocity(velocity_.vector, linear_acceleration_w, 0.1);
            // velocity_.vector.x += 0.0;
            // velocity_.vector.y += 0.0;
            // velocity_.vector.z += 0.0;
            v_publisher_->publish(velocity_);

            position_.x += velocity_.vector.x * 1.0/400.0 ; 
            position_.y += velocity_.vector.y * 1.0/400.0 ; 
            position_.z += velocity_.vector.z * 1.0/400.0 ; 
            p_publisher_->publish(position_);
        }


        void add_velocity(geometry_msgs::msg::Vector3 &velocity, geometry_msgs::msg::Vector3 &dv, double noise_gate)
        {
            if (abs(dv.x) > noise_gate){
                velocity.x += (dv.x * 1.0/400.0);
            }
            if (abs(dv.y) > noise_gate){
                velocity.y += (dv.y * 1.0/400.0);
            }
            if (abs(dv.z) > noise_gate){
                velocity.z += (dv.z * 1.0/400.0);
            }
        }


        void rotate_vector_by_quaternion(const tf2::Vector3& v, const tf2::Quaternion& q, tf2::Vector3& vprime)
        {
            tf2::Vector3 u;
            // u.setX(q.x());
            // u.setY(q.y());
            // u.setZ(q.z());
            setTF2vector(u, q.getX(), q.getY(), q.getZ());

            double s = q.w();

            vprime = 2.0f * u.dot(v) * u + (s*s - u.dot(v)) * v + 2.0f * s * u.cross(v);
        }


        void setTF2vector(tf2::Vector3 &v, double x, double y, double z)
        {
            v.setX(x);
            v.setY(y);
            v.setZ(z);
        }
            
        // void subscribe_dv(const geometry_msgs::msg::Vector3Stamped::SharedPtr dv)
        //     {
        //         // velocity_.x += (dv->vector.x + dv_static_bias.x);
        //         // velocity_.y += (dv->vector.y + dv_static_bias.y);
        //         // velocity_.z += (dv->vector.z + dv_static_bias.z);
        //         velocity_.x += 0.0;
        //         velocity_.y += 0.0;
        //         velocity_.z += 0.0;
        //         v_publisher_->publish(velocity_);

        //         position_.x += velocity_.x * 1.0/100.0 ; 
        //         position_.y += velocity_.y * 1.0/100.0 ; 
        //         position_.z += velocity_.z * 1.0/100.0 ; 
        //         p_publisher_->publish(position_);
        //         RCLCPP_INFO(get_logger(), "x: %f, v.x: %f, dv.x %f", position_.x, velocity_.x, dv->vector.x + dv_static_bias.x);
        //     }

        // void subscribe_dq(const geometry_msgs::msg::QuaternionStamped::SharedPtr dq)
        //     {
        //         // update attitude ( need to implement timewise mean of rotation later) 
        //         tf2::Quaternion dq_tf2;
        //         tf2::Quaternion attitude_tf2;
        //         tf2::fromMsg(dq->quaternion, dq_tf2);
                
        //         tf2::fromMsg(attitude_.quaternion, attitude_tf2);
        //         attitude_tf2  = dq_tf2 * attitude_tf2;
        //         attitude_tf2.normalize();
        //         attitude_.quaternion = tf2::toMsg(attitude_tf2);

        //         RCLCPP_INFO(get_logger(), " attitude: w: %f, x: %f, y: %f, z: %f ", attitude_.quaternion.w , attitude_.quaternion.x, attitude_.quaternion.y, attitude_.quaternion.z);
                
        //         q_publisher_->publish(attitude_);
        //     }


    
        // subscribers
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr data_subscriber_;
        // rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr dv_subscriber_;
        // rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr dq_subscriber_;

        // publishers
        // rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr q_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr p_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr v_publisher_;

        geometry_msgs::msg::Point position_;
        geometry_msgs::msg::Vector3Stamped velocity_;
        geometry_msgs::msg::Vector3 dv_static_bias;
        geometry_msgs::msg::QuaternionStamped attitude_;
        // geometry_msgs::msg::Quaternion dq_static_bias;



        
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}