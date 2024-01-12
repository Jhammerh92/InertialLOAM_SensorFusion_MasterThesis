#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"

// #include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/msg/point_cloud2.hpp>


#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <memory>
#include <cstdio>
#include <filesystem>
#include <queue>

// typedef pcl::PointXYZINormal PointType; // need to calculate and assign normals
typedef pcl::PointXYZI PointType; // need to calculate and assign normals
typedef sensor_msgs::msg::PointCloud2 PC_msg;


using namespace std::chrono_literals;
using namespace std::chrono;
using namespace std::filesystem;

typedef std::chrono::duration<int,std::milli> milliseconds_type;

using std::placeholders::_1;


class StaticRecorder : public rclcpp::Node // inheritance here  from ROS RCLCPP node type
{
    public:
        StaticRecorder() // constructor
        : Node("static_cloud_recorder")
        {

            declare_parameter("record_time", 1.0f);
            get_parameter("record_time", record_time_);

            declare_parameter("interval_time", 10.0f);
            get_parameter("interval_time", interval_time_);

            declare_parameter("save_path", "static_recorded_clouds"); 
            get_parameter("save_path", save_path_);
            

            RCLCPP_INFO(get_logger(), "Static recorder has started. Record time is %f s, interval time is %f s", record_time_, interval_time_);

            // check that interval time is bigger than record time, else display warning and set them equal


            // milliseconds_type chrono_interval_time = duration<int, std::milli>((int)interval_time_ *1000.0);
            milliseconds_type chrono_interval_time((int)(interval_time_ *1000));
            timer_ = this->create_wall_timer(chrono_interval_time, std::bind(&StaticRecorder::startInterval, this));   


            first_lidar = true;
            buffer_frames = true;

            // i = 0;
            // timer_ = this->create_wall_timer(1000ms, std::bind(&StaticRecorder::publish_message, this));   

            cloud_in.reset(new pcl::PointCloud<PointType>());
            collected_cloud.reset(new pcl::PointCloud<PointType>());

            pointcloud_sub_ = this->create_subscription<PC_msg>("/livox/lidar", 100, std::bind(&StaticRecorder::cloudHandler, this, _1));
        
            // create folder for recording
            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);
            std::ostringstream oss;
            oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
            std::string datestr = oss.str();

            full_save_path = save_path_ + "/" + "static_recording_" + datestr;

            create_directories(full_save_path);

        }

    private:

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        pcl::PointCloud<PointType>::Ptr cloud_in = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr collected_cloud = boost::make_shared<pcl::PointCloud<PointType>>();


        size_t frame_index{};

        float record_time_;
        float interval_time_;
        double first_timestamp;
        double ref_timestamp;
        double time_since_start;
        double latest_timestamp;
        bool first_lidar;
        bool buffer_frames;

        std::string save_path_;
        std::string full_save_path;

        // first lidar time is needed at some point

        std::deque<sensor_msgs::msg::PointCloud2> cloud_queue;

        void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_cloud_msg)
        {
            latest_timestamp = toSec(lidar_cloud_msg->header.stamp);

            if (first_lidar)
            {
                first_lidar = false; // used by imu_handler to see when the first lidar msg has arrived
                first_timestamp = toSec(lidar_cloud_msg->header.stamp);
                ref_timestamp = toSec(lidar_cloud_msg->header.stamp);
            }


            if (!buffer_frames)
                return;


            // cache point cloud in buffer
            cloud_queue.push_back(*lidar_cloud_msg);
            

            if (recordTimeIsfull(first_timestamp, toSec(cloud_queue.back().header.stamp)) )
            {
                // RCLCPP_INFO(get_logger(), "this was true??");
                stopInterval();
                saveRecording();
            }

        }

        void startInterval(){
            // set timer of last cloud in queue, this will be the first time of the clouds to be recorded
            if (cloud_queue.empty()) // return if buffer is empty
                return;
            // first_timestamp = toSec(cloud_queue.back().header.stamp);
            first_timestamp = latest_timestamp;
            // then clear buffer to remove unwanted scans in the buffer 
            cloud_queue.clear();
            // set flag that allows buffering
            buffer_frames = true & !first_lidar;

            time_since_start =  first_timestamp - ref_timestamp;

            RCLCPP_INFO(get_logger(), "Main interval has started");
        }

        void stopInterval(){
            RCLCPP_INFO(get_logger(), "End of record interval");
            buffer_frames = false; // stop buffering while saving and waiting for the next interval start.
        }

        void saveRecording(){
            
            RCLCPP_INFO(get_logger(), "Size of buffer: %i", cloud_queue.size());

            // join all frames to single plc::pcd and save 
            for (size_t i=0; i < cloud_queue.size(); i++){
                // convert cloud msg to pcl cloud
                fromROSMsg(cloud_queue[i] , *cloud_in);
                // join to single cloud
                *collected_cloud += *cloud_in;
            } 

            
            
            char index_buffer[5];
            sprintf(index_buffer, "%04d", (int)frame_index);
            std::string index_str(index_buffer);
            
            std::string filename = "/static_cloud_" + index_str +"_"+ std::to_string((int)round(time_since_start))+ ".pcd";

            // save the accumulated cloud as pcd
            // TODO get actual timestap since start and index number on name of cloud for traceability
            // timestamp is put on a parent folder, index and time since start shall be put on filename
            bool save_as_binary = true;
            pcl::io::savePCDFile(full_save_path + filename, *collected_cloud, save_as_binary);
            // clear created cloud
            collected_cloud->clear();

            frame_index++;

            RCLCPP_INFO(get_logger(), "Cloud has been saved!");
        }

        bool recordTimeIsfull(double t_start, double t_end){
            // RCLCPP_INFO(get_logger(), "Tstart is %f, Tend is %f", t_start, t_end);
            return ((t_end - t_start) > record_time_);
        }

        void fromROSMsg(const sensor_msgs::msg::PointCloud2 &cloud, pcl::PointCloud<PointType> &pcl_cloud)
        {   
            // sensor_msgs::PointCloud2 cloud_PC2 = cloud;
            pcl::PCLPointCloud2 pcl_pc2;
            #ifndef __INTELLISENSE__ // this is to disable an error squiggle from pcl_conversions not having the proper overload for input type.
            pcl_conversions::toPCL(cloud, pcl_pc2);
            #endif
            pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
        }

        double toSec(builtin_interfaces::msg::Time header_stamp)
        {
            rclcpp::Time time = header_stamp;
            double nanoseconds = time.nanoseconds();

            return nanoseconds * 1e-9;
        }





        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_Lidar_cloud;
        // std_msgs::msg::String msg_;
        rclcpp::TimerBase::SharedPtr timer_;
        
        int i;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticRecorder>());
    rclcpp::shutdown();
    return 0;

}