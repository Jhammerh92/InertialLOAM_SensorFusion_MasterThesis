// #include <memory>
// #include <chrono>


#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"
#include "my_learning_package/utils/common.hpp"
// #include "my_learning_package/ins.hpp"



// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <std_msgs/msg/int64.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
// #include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

// #include "example_interfaces/srv/add_two_ints.hpp"
// #include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>


#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/ndt.h>
// #include <fast_pcl/registration/ndt.h>
// #include <fast_pcl/registration/ndt.h>

#include <pclomp/gicp_omp.h>
#include <pclomp/gicp_omp_impl.hpp>
#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>
// #include <pclomp/gicp_omp.h>

#include <cmath>
#include <ctime>
// #include <array>
// #include <string>
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

#define _USE_MATH_DEFINES


///////////////////////////////////////////////////////////////////////////////////////////

class INS //: public rclcpp::Node
{
public:
    INS()
    // : Node("ins")
    {  
        preint_state.ori = Eigen::Quaterniond::Identity();
        preint_state.pos = Eigen::Vector3d::Zero();
        preint_state.vel = Eigen::Vector3d::Zero();
        preint_state.acc = Eigen::Vector3d::Zero();
        preint_state.time = 0.0;

        current_bias.acc = Eigen::Vector3d::Zero();
        current_bias.ang = Eigen::Vector3d::Zero();
        current_bias.time = 0.0;

        preint_state.bias = current_bias;

        preint_transformation_guess.rotation = Eigen::Quaterniond::Identity();
        preint_transformation_guess.translation = Eigen::Vector3d::Zero();
        preint_transformation_guess.matrix = Eigen::Matrix4d::Identity();
        preint_transformation_guess.time = 0.0;
    }
    ~INS(){}

private:
    Pose initial_pose;
    Pose current_pose;

    // double expected_imu_dt = 0.005;

    INSstate preint_anchor;
    INSstate preint_state;
    INSstate predict_state;
    INSstate lidar_synced_state;

    deque<INSstate> preint_states;
    deque<INSstate> predict_states;

    Transformation preint_transformation_guess;

    IMUwrench current_bias;


    deque<double> imu_dts;
    double mean_imu_dt = 0.005;
    double std_imu_dt = 0.02;
    double last_imu_stamp{};

    deque<sensor_msgs::msg::Imu::SharedPtr> imu_msg_buffer;

    Eigen::Quaterniond madgwickOrientation = Eigen::Quaterniond::Identity();
    
    
public:
    void addImuMsgToBuffer(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    const sensor_msgs::msg::Imu::SharedPtr getNextMsgFromBuffer(void);
    void resetPreintAnchor();
    void setPreintAnchor(const INSstate state);
    void getPreintAnchor(INSstate &state); 
    INSstate getPreintAnchor(); 
    void setBias(const IMUwrench new_bias);

    void initializeInitialPose(Eigen::Quaterniond);

    Transformation& getPreintegratedTransformationGuess(); 

    void updateInitialPose(const Pose initial_pose);
    
    double getIMUdt(int buffer_index);
    
    void updateIMUdtAverage(double new_dt);
    
    double validateIMUdt(double new_dt);
    
    void integrateImuStep(Eigen::Vector3d acc, Eigen::Vector3d ang_vel, double dt);

    void integrateConstVelocityStep(double dt);

    void integrateJerkAlpha(INSstate *this_state, const INSstate next_state, double dt);

    IMUwrench createIMUwrench(Eigen::Vector3d acc, Eigen::Vector3d ang, double dt, double t);

    void preintegrateIMU(const double last_scan_start_time, const double end_time);

    void predictConstVelocityStep(double dt);
    void predictionIntegrate(double deltaT, int steps);

    void calcLidarSyncedState(const double frame_start_time);

    INSstate calcKinematicInterpolatedState(INSstate this_state, INSstate next_state, double interpolate_time);

    void undistortCloud(const pcl::PointCloud<PointType> &cloud_in, pcl::PointCloud<PointType> &cloud_out);

    const INSstate& getPreintState(int index);
    const INSstate& getPredictState(int index);


    Eigen::Quaterniond getMadgwickOrientation(double timestamp);

};

void INS::addImuMsgToBuffer(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{   
    this->imu_msg_buffer.push_back(imu_msg);
    updateIMUdtAverage(toSec(imu_msg->header.stamp) - last_imu_stamp);
    last_imu_stamp = toSec(imu_msg->header.stamp);
}

const sensor_msgs::msg::Imu::SharedPtr INS::getNextMsgFromBuffer(void)
{
    auto msg = this->imu_msg_buffer.front();
    this->imu_msg_buffer.pop_front();
    return msg;
}

void INS::resetPreintAnchor()
{
    preint_anchor.pos   = Eigen::Vector3d::Zero(); // initial position in the future
    preint_anchor.vel   = Eigen::Vector3d::Zero(); // saved in local frame
    preint_anchor.ori   = Eigen::Quaterniond(initial_pose.orientation);
    preint_anchor.ang   = Eigen::Vector3d::Zero();
    // preint_anchor.bias = Eigen::Vector3d::Zero();
    preint_anchor.acc   = Eigen::Vector3d::Zero();
    preint_anchor.jerk  = Eigen::Vector3d::Zero();
    preint_anchor.alpha = Eigen::Vector3d::Zero();
    // preint_anchor.bias  = IMUwrench();

}

void INS::setPreintAnchor(const INSstate state)
{
    
    preint_anchor.time  = state.time ;
    preint_anchor.pos   = state.pos ;
    preint_anchor.vel  = state.vel ;
    // preint_anchor.vel   = state.ori.matrix().inverse() * state.vel ; // velocity saved in local frame, so it can rotated into a corrected frame
    preint_anchor.ori   = state.ori ;
    preint_anchor.ang   = state.ang ;
    preint_anchor.bias  = state.bias;
    preint_anchor.acc   = state.acc ;
    preint_anchor.jerk  = state.jerk ;
    preint_anchor.alpha = state.alpha ;
    preint_anchor.dt    = state.dt;
}

// set to return anchor by reference?
void INS::getPreintAnchor(INSstate &state) // passed by reference to make it mutable
{
   
    state.time  = preint_anchor.time;
    state.pos   = preint_anchor.pos ;
    state.vel   = preint_anchor.vel; 
    // state.vel   = state.ori.matrix() * preint_anchor.vel; // is rotated into new frame
    state.ori   = preint_anchor.ori ;
    state.ang   = preint_anchor.ang ;
    state.bias  = preint_anchor.bias;
    state.acc   = preint_anchor.acc ;
    state.jerk  = preint_anchor.jerk ;
    state.alpha = preint_anchor.alpha ;
    state.dt    = preint_anchor.dt;
}

INSstate INS::getPreintAnchor() 
{
    INSstate state;
    state.time  = preint_anchor.time;
    state.pos   = preint_anchor.pos ;
    // state.vel   = state.ori.matrix() * preint_anchor.vel; // is rotated into new frame
    state.vel   = preint_anchor.vel; 
    state.ori   = preint_anchor.ori ;
    state.ang   = preint_anchor.ang ;
    state.bias  = preint_anchor.bias;
    state.acc   = preint_anchor.acc ;
    state.jerk  = preint_anchor.jerk ;
    state.alpha = preint_anchor.alpha ;
    state.dt    = preint_anchor.dt;
    return state;
}

void INS::setBias(const IMUwrench new_bias)
{
    this->current_bias = new_bias;
}

void INS::initializeInitialPose(Eigen::Quaterniond orientation)
{
    initial_pose.orientation             =  Eigen::Quaterniond(orientation);
    preint_state.ori                     =  Eigen::Quaterniond(orientation);
    preint_anchor.ori                    =  Eigen::Quaterniond(orientation);
    preint_transformation_guess.rotation =  Eigen::Quaterniond(orientation);
    preint_transformation_guess.matrix.block<3,3>(0,0) = Eigen::Matrix3d(orientation);
    cout << "FROM INS: Initial orientation :\n" << orientation.w() <<"\n"<< orientation.vec() << "\n";

}

Transformation& INS::getPreintegratedTransformationGuess(){
    // Transformation preint_transformation_guess;
    // preint_transformation_guess.translation = lidar_synced_state.pos;
    // preint_transformation_guess.rotation = lidar_synced_state.ori;
    return this->preint_transformation_guess;
}

void INS::updateInitialPose(const Pose initial_pose)
{
    // make something that averages the incoming poses??
    this->initial_pose = initial_pose;
    this->current_pose = initial_pose;
    // std::cout << "FROM INS: initial pose updated q: " << std::to_string(this->initial_pose.orientation.w()) << " " << std::to_string(this->initial_pose.orientation.x()) <<" " << std::to_string(this->initial_pose.orientation.y()) <<" " << std::to_string(this->initial_pose.orientation.z()) << "\n";
}

double INS::getIMUdt(int buffer_index)
{
    double dt = toSec(imu_msg_buffer[buffer_index+1]->header.stamp) - toSec(imu_msg_buffer[buffer_index]->header.stamp);
    return dt;
    
}

void INS::updateIMUdtAverage(double new_dt)
{   
    // cout << "new imu dt "<< to_string(new_dt) << " mean is: "<< to_string(mean_imu_dt) <<" std is: "<< to_string(std_imu_dt) << "\n";
    // if (abs(new_dt - mean_imu_dt) < 3.0*std_imu_dt && std_imu_dt > mean_imu_dt/5.0){
    if (abs(new_dt - mean_imu_dt) < 3.0*std_imu_dt){
        // cout << "imu dt sample added"<< "\n";
        if (this->imu_dts.size() > 1000){ // limit the size of buffer else mean calculations would slowly increase in time
            imu_dts.pop_back();
        }
        imu_dts.push_front(new_dt);
    } else {
        return;
    }

    // cout << "FROM INS: imu msg dt: " << to_string(new_dt) << "\n";
    // TODO make a running mean calculation instead
    size_t n = imu_dts.size();
    if (n >= 10){
        double sum = 0.0;
        double sum_sq = 0.0;
        for (size_t i=0; i < n; i++){
            sum += imu_dts[i];
            sum_sq += imu_dts[i]*imu_dts[i];
        }
        mean_imu_dt = sum / (double)n;
        std_imu_dt = sqrt((sum_sq)/(double)n - mean_imu_dt*mean_imu_dt );
        // cout << "new mean is: "<< to_string(mean_imu_dt) <<" new std is: "<< to_string(std_imu_dt) << "\n";
    } 
    return;

}

// should add an expected dt?
double INS::validateIMUdt(double new_dt)
{   
    // RCLCPP_INFO(get_logger(),"IMU dt: %f", new_dt);
    // return expected_imu_dt;
    if (this->imu_dts.size() < 10){ // need atleast a little sample to do mean calculations
        // cout << "not enough imu dt samples to validate imu"<< "\n";
        return new_dt;
    }

    // if (abs(new_dt - mean_dt) > 2.0*std_dt){ // if the time deviation is by too big it is replaced by the mean of the good samples
    if (new_dt - mean_imu_dt > 3.0*std_imu_dt){ // if the time deviation is by too big it is replaced by the mean of the good samples
        // cout << "FROM INS: Bad dt detected! " << to_string(new_dt) << " replaced with mean " << to_string(mean_imu_dt) << " std " << to_string(std_imu_dt) <<"\n";
        return mean_imu_dt;
    }
    
    // past here, the dt values is good and is added to good samples and is returned as it is
    // cout << "good imu dt"<< "\n";
    return new_dt;
}

// OBS!: This function assumes that gravity has been removed from the incoming acceleration.
void INS::integrateImuStep(Eigen::Vector3d acc, Eigen::Vector3d ang_vel, double dt)
{
    // save the previous state to be used 
    INSstate previous_state = preint_state;

    // Integrate the orientation from the angular rate and save the average ori between this and the previous state
    preint_state.ang = ang_vel - current_bias.ang;
    // Eigen::Quaterniond dq_vel = deltaQ(0.5*(preint_state.ang + previous_state.ang)); // the 0.5 is to find the average
    // dq_vel = multQuatByScalar(dq_vel, dt);
    Eigen::Quaterniond dq_vel = deltaQ(0.5*(preint_state.ang + previous_state.ang)*dt); // the 0.5 is to find the average
    // dq_vel = multQuatByScalar(dq_vel, dt);

    Eigen::Quaterniond dq = preint_state.ori*dq_vel; // 1/2 * q * dq * dt
    preint_state.ori = addQuaternions(dq, preint_state.ori); 
    preint_state.ori.normalize(); // orientation normalized
    Eigen::Quaterniond ori_avg = previous_state.ori.slerp(0.5, preint_state.ori);

    // need to think about how bias is applied and saved..
    // acceleration and bias is in the local imu frame
    // preint_state.acc = (acc - observer_state.bias.acc); // save the acc in local body frame - with subtracted bias?
    preint_state.acc = (acc - current_bias.acc); // save the acc in local body frame - with subtracted bias
    preint_state.vel = previous_state.vel + ori_avg.matrix() * preint_state.acc * dt;
    preint_state.pos = previous_state.pos + preint_state.vel * dt - ori_avg.matrix() * preint_state.acc * dt*dt *0.5;
    
    ///velocity in local frame:
    // preint_state.vel = previous_state.vel +  preint_state.acc * dt; 
    // preint_state.pos = previous_state.pos + ori_avg.matrix() * preint_state.vel * dt - ori_avg.matrix() * preint_state.acc * dt*dt *0.5;
}

void INS::integrateConstVelocityStep(double dt)
{
    // save the previous state to be used 
    INSstate previous_state = preint_state;

    // Integrate the orientation from the angular rate and save the average ori between this and the previous state
    preint_state.ang = Eigen::Vector3d::Zero(); //previous_state.ang; // the angular rate is assumed constant and the previous measurement is used.
    Eigen::Quaterniond dq_vel = deltaQ(0.5*(preint_state.ang + previous_state.ang)); 
 
    dq_vel = multQuatByScalar(dq_vel, dt);

    Eigen::Quaterniond dq = preint_state.ori*dq_vel;
    preint_state.ori = addQuaternions(dq, preint_state.ori);
    preint_state.ori.normalize(); // orientation normalized
    // Eigen::Quaterniond ori_avg = previous_state.ori.slerp(0.5, preint_state.ori);

    preint_state.acc = Eigen::Vector3d::Zero(); // net acceleration is assumed zero - ofc not true while moving in a curve..
    preint_state.vel = previous_state.vel; 
    preint_state.pos = previous_state.pos + preint_state.vel * dt;

    // velocity in local frame
    // preint_state.pos = previous_state.pos + ori_avg.matrix() * preint_state.vel * dt;  //- ori_avg.matrix() * preint_state.acc * dt*dt *0.5;

    // jerk and alpha is zero in a constant velocity model
    preint_state.jerk = Eigen::Vector3d::Zero(); 
    preint_state.alpha = Eigen::Vector3d::Zero();
}

void INS::integrateJerkAlpha(INSstate *this_state, const INSstate next_state, double dt)
{
    // Eigen::Quaterniond ori_avg = this_state.ori.slerp(0.5, next_state.ori);
    this_state->alpha = (next_state.ang - this_state->ang) / dt;
    this_state->jerk =  (next_state.ori*next_state.acc - this_state->ori*this_state->acc) / dt; // calculate and save jerk in world frame
    // this_state->jerk =  this_state->ori.inverse() * (next_state.ori*next_state.acc - this_state->ori*this_state->acc) / dt;
    
}

IMUwrench INS::createIMUwrench(Eigen::Vector3d acc, Eigen::Vector3d ang, double dt, double t)
{
    IMUwrench new_imu_wrench;
    new_imu_wrench.acc = acc;
    new_imu_wrench.ang = ang;
    new_imu_wrench.time = t;
    new_imu_wrench.dt = dt;
    return new_imu_wrench;
}

void INS::preintegrateIMU(const double last_scan_start_time, const double next_scan_end_time)
{

    // deque<sensor_msgs::msg::Imu::SharedPtr> preintegrate_buffer;
    deque<IMUwrench> preintegrate_buffer;
    // the total time delta that the preintegration should cover
    double integration_delta = next_scan_end_time - last_scan_start_time;

    cout << "FROM INS: Integration delta: " << integration_delta << ", start: " << to_string(last_scan_start_time) << " end: " << to_string(next_scan_end_time) << "\n";
    double integration_time = 0.0;
    // clear out buffer up till the first imu msg before the scan begins, the anchor state should be just before the start of the scan
    double imu_msg_dt{};
    double imu_dt{};
    double lag_discrepancy{};

    // removes past imu msg 
    while(last_scan_start_time > toSec(imu_msg_buffer[1]->header.stamp) ) // >=?
    {
        imu_msg_buffer.pop_front();
    }

    int j = 0;
    double msg_time = toSec(imu_msg_buffer[0]->header.stamp);
    double delay = last_scan_start_time - msg_time;
    if (delay > mean_imu_dt){ // if the time difference is bigger than the expected imu delta then add a msg at the start time of the frame
        // cout << "delay!: "<< delay << "\n";
        j++;
        Eigen::Vector3d acc_in(0.0,0.0,0.0);
        Eigen::Vector3d ang_vel_in(0.0,0.0,0.0);
        double fill_time = toSec(imu_msg_buffer[1]->header.stamp) - last_scan_start_time ;
        IMUwrench new_imu_wrench = createIMUwrench(acc_in, ang_vel_in, fill_time, last_scan_start_time);    
        preintegrate_buffer.push_back(new_imu_wrench);
        msg_time = last_scan_start_time;
        // cout << "FROM INS: !const vel! msg added " << j << "\ttimestamp: " << to_string(last_scan_start_time) << "\tmsg dt: "<< fill_time << "\tdt: " << mean_imu_dt << "\n";
    }

    sensor_msgs::msg::Imu imu_msg;
    // while (true){
    while (msg_time <= next_scan_end_time){
        imu_msg = *imu_msg_buffer[j];
        msg_time = toSec(imu_msg.header.stamp);
        imu_msg_dt = getIMUdt(j); 
        imu_dt = validateIMUdt(imu_msg_dt); 
        // lag_discrepancy = imu_msg_dt - imu_dt;


        // extract acc and ang vel from the imu msg
        Eigen::Vector3d acc_in(imu_msg.linear_acceleration.x,
                               imu_msg.linear_acceleration.y,
                               imu_msg.linear_acceleration.z);
        Eigen::Vector3d ang_vel_in(imu_msg.angular_velocity.x,
                                   imu_msg.angular_velocity.y,
                                   imu_msg.angular_velocity.z);
        
        IMUwrench new_imu_wrench = createIMUwrench(acc_in, ang_vel_in, imu_msg_dt, msg_time);    
        preintegrate_buffer.push_back(new_imu_wrench);
        j++;
        // cout << "FROM INS: new msg added" << j << "\ttimestamp: " << to_string(msg_time) << "\tmsg dt:"<< imu_msg_dt << "\tdt: " << imu_dt << "\n";
        // cout << " acc: " << acc_in.transpose() << "\n ang:  " << ang_vel_in.transpose() << "\n"; 
    }

    cout << "FROM INS: preintegrate buffer length: " << preintegrate_buffer.size() << "\n";
    
    // get the preintegration achor state, this is the state that the integration iterates from
    getPreintAnchor(preint_state);
    // overwrite timestamp of the anchor to the correct time of the next msg
    // preint_state.time = toSec(imu_msg.header.stamp);
    preint_states.clear(); // clear preint_states to make sure it is empty

    INSstate this_preint_state = preint_state;
    INSstate new_preint_state;
    
    preint_states.push_back(this_preint_state); // push the anchor state
    
    // cout << "FROM INS: before integration preint state rotation:\n" << preint_state.ori.w() <<"\n"<< preint_state.ori.vec() << "\n";
    // integrate all the collected imu msg in the integration delta time
    for (size_t i = 0; i < preintegrate_buffer.size(); i++) {
        IMUwrench imu_input = preintegrate_buffer[i];
        double msg_time = imu_input.time;
        // imu_dt = validateIMUdt(imu_input.dt);
        imu_dt = imu_input.dt;


        integrateImuStep(imu_input.acc, imu_input.ang, imu_dt); // this writes to preint_state, but does not calc jerk and alpha
        // cout << "FROM INS: after integration preint state rotation:\n" << preint_state.ori.w() <<"\n"<< preint_state.ori.vec() << "\n";
        new_preint_state = preint_state;
        integrateJerkAlpha(&this_preint_state, new_preint_state, imu_dt); // calcs jerk and alpha between this and next and saves to this_state
        // this_preint_state.time = toSec(imu_msg.header.stamp);
        this_preint_state.time = imu_input.time;
        this_preint_state.dt = imu_dt;
        preint_states.push_back(this_preint_state);
        preint_state = new_preint_state;
        this_preint_state = new_preint_state;

        integration_time += imu_dt;

        Eigen::Quaterniond rotation = preint_state.ori.inverse() * preint_anchor.ori; 
        Eigen::Vector3d translation = preint_state.pos - preint_anchor.pos;

        // double preint_norm = translation.norm();
        Eigen::AngleAxisd angleAxis = Eigen::AngleAxisd(rotation);
        double preint_angle = angleAxis.angle()*180.0/M_PI;

        // cout << "idx: "<< i << " timestamp: " << to_string(msg_time) << " msg dt: "<< imu_input.dt << " dt: " << imu_dt << "\n";
        // cout << "acc: " << imu_input.acc.transpose() << " ang:  " << imu_input.ang.transpose() << "\n";
        // cout << "preintegrated:"<< i << "\t translation:" << translation.transpose() << " m,\tangle: " << preint_angle << " degrees \n";

        // if (msg_time > next_scan_end_time){ // not really need anymore
        // // if (integration_time > integration_delta){ 
        //     return;
        // }
    }
} 

void INS::predictConstVelocityStep(double dt)
{
    // save the previous state to be used 
    INSstate previous_state = predict_state;

    // Integrate the orientation from the angular rate and save the average ori between this and the previous state
    predict_state.ang = previous_state.ang;
    Eigen::Quaterniond dq_vel = deltaQ(0.5*(predict_state.ang + previous_state.ang)); 
 
    dq_vel = multQuatByScalar(dq_vel, dt);

    Eigen::Quaterniond dq = predict_state.ori*dq_vel;
    predict_state.ori = addQuaternions(dq, predict_state.ori); // save new orientation normalized!
    predict_state.ori.normalize(); // orientation normalized!
    Eigen::Quaterniond ori_avg = previous_state.ori.slerp(0.5, predict_state.ori);

    // need to think about how bias is applied and saved..
    // acceleration and bias is in the local imu frame
    predict_state.acc = previous_state.acc; // maybe assume constant acceleration? this is only used for vizualisation of the future trajectory 
    // predict_state.acc = Eigen::Vector3d::Zero();
    predict_state.vel = previous_state.vel + ori_avg.matrix() * predict_state.acc * dt; 
    // predict_state.vel = previous_state.vel; // + predict_state.acc * dt; 
    predict_state.pos = previous_state.pos + predict_state.vel * dt - ori_avg.matrix() * predict_state.acc * dt*dt *0.5;
    // predict_state.pos = previous_state.pos + ori_avg.matrix() * predict_state.vel * dt;

    predict_state.jerk = Eigen::Vector3d::Zero();
    predict_state.alpha = Eigen::Vector3d::Zero();
}

void INS::predictionIntegrate(double dt, int steps)
{

    predict_state = preint_anchor;
    INSstate this_predict_state = preint_anchor;
    INSstate new_predict_state;

    predict_states.clear(); // clear predict_states to make sure it is empty
    for (int i = 0; i < steps; i++){

            predictConstVelocityStep(dt); // this writes to preint_state, but does not calc jerk and alpha
            new_predict_state = predict_state;
            this_predict_state.time = predict_state.time + i*dt;
            this_predict_state.dt = dt;
            predict_states.push_back(this_predict_state);
            predict_state = new_predict_state;
            this_predict_state = new_predict_state;

    }
} 

void INS::calcLidarSyncedState(const double frame_start_time)
{
    // get the latest integrated state just before the start of the new cloud scan
    // size_t i = 0;
    while (preint_states[1].time <= frame_start_time) // comparing to 1 to get the state just before frame_start
    {
        preint_states.pop_front();
    }

    // set state anchor for next preintegration - this is synced with imu steps
    // setPreintAnchor(preint_states[i]);
    // setPreintAnchor(preint_states.front()); THIS SHOULD STILL BE THE CORRECT THING TO USE

    // calculates the imu syncronised interpolated state at time of the start of the cloud
    double sync_time = frame_start_time - preint_states.front().time;
   
    lidar_synced_state = calcKinematicInterpolatedState(preint_states[0], preint_states[1], sync_time);
    setPreintAnchor(lidar_synced_state);

    // this synced state is also the prior for the scan registration 
    // it is saved so it can be used in the observer
    preint_transformation_guess.translation = lidar_synced_state.pos;
    preint_transformation_guess.rotation = lidar_synced_state.ori;
    preint_transformation_guess.matrix = Eigen::Matrix4d::Identity();
    preint_transformation_guess.matrix.block<3,3>(0,0) = Eigen::Matrix3d(lidar_synced_state.ori);
    preint_transformation_guess.matrix.block<3,1>(0,3) = Eigen::Vector3d(lidar_synced_state.pos);

}

INSstate INS::calcKinematicInterpolatedState(INSstate this_state, INSstate next_state, double interpolate_time)
{
    // calculates the imu interpolated between 2 given states with and a sync time
    double sync_time = interpolate_time;
    double half_sync_time_sq = 1.0/2.0 * sync_time*sync_time;
    double sixth_sync_time_cb = 1.0/3.0 * half_sync_time_sq*sync_time;
    double imu_dt = this_state.dt;

    INSstate interpolated_state;

    // interpolated_state.pos = this_state.pos + this_state.vel * sync_time + (preint_states[0].ori *preint_states[0].acc) * half_sync_time_sq  +  preint_states[0].jerk * sixth_sync_time_cb;
    // interpolated_state.vel = this_state.vel + (preint_states[0].ori.matrix() *preint_states[0].acc) *sync_time + preint_states[0].jerk * half_sync_time_sq;
    // interpolated_state.acc = this_state.acc + preint_states[0].ori.inverse().matrix() * preint_states[0].jerk * sync_time;
    // interpolated_state.ang = this_state.ang + preint_states[0].alpha * sync_time;
    interpolated_state.pos = this_state.pos + this_state.vel * sync_time + (this_state.ori * this_state.acc) * half_sync_time_sq  + this_state.jerk * sixth_sync_time_cb;
    interpolated_state.vel = this_state.vel + (this_state.ori * this_state.acc) *sync_time + this_state.jerk * half_sync_time_sq;
    interpolated_state.acc = this_state.acc + this_state.ori.inverse() * this_state.jerk * sync_time;
    interpolated_state.ang = this_state.ang + this_state.alpha * sync_time;


    interpolated_state.ori =  (deltaQ(this_state.alpha * half_sync_time_sq) * (deltaQ(this_state.ang * sync_time) * this_state.ori)).normalized();
    
    // Eigen::Quaterniond dq_vel = multQuatByScalar(deltaQ(this_state.ang), sync_time);
    // (deltaQ(this_state.alpha * half_sync_time_sq) *
    // interpolated_state.ori =   addQuaternions(this_state.ori * dq_vel, this_state.ori).normalized();
    // interpolated_state.ori =   addQuaternions(this_state.ori, this_state.ori).normalized();
    
    // linear interpolated jerk and alpha, interpolated from the sync_time to imu_dt
    double interpolate_ratio = sync_time / imu_dt; // sync_time should be between 0 and imu_dt, if not something is wrong
    Eigen::Vector3d jerk  = this_state.jerk * (1 - interpolate_ratio)   +  next_state.jerk * interpolate_ratio; 
    Eigen::Vector3d alpha = this_state.alpha * (1 - interpolate_ratio)  +  next_state.alpha * interpolate_ratio; 
    interpolated_state.jerk = jerk;
    interpolated_state.alpha = alpha;
    
    interpolated_state.time =  this_state.time + sync_time;
    interpolated_state.dt =  imu_dt; // not sure this should be the full dt, maybe imu_dt - sync_time?

    return interpolated_state;
}

void INS::undistortCloud(const pcl::PointCloud<PointType> &cloud_in, pcl::PointCloud<PointType> &cloud_out)
{   
    // create state sync look-up index
    size_t cloud_size = cloud_in.points.size();

    int *state_sync_index = new int[cloud_size];
    double *point_frame_dts = new double[cloud_size];
    #pragma omp parallel for shared(state_sync_index, point_frame_dts)
    for (size_t k = 0; k < cloud_size; k++){
        int i = 0;
        PointType point = cloud_in.points[k];
        double intensity = point.intensity;
        double point_frame_dt = intensity - (int)intensity; // point timestamps are saved as the decimal part of the intensity values that are ints
        while ((point_frame_dt + lidar_synced_state.time) >= preint_states[i+1].time){
            i++;
            // RCLCPP_INFO(get_logger(),"state sync index %i to point index k %i", i, k); 
        }
        state_sync_index[k] = i;
        point_frame_dts[k] = point_frame_dt ;
    }


    auto u1 = std::chrono::high_resolution_clock::now();
    double calc_time{};
    // INSstate stateX;
    INSstate stateX;
    INSstate state_next;
    Eigen::Matrix4d T_star = Eigen::Matrix4d::Identity();
    // double point_state_dt;
    #pragma omp parallel for reduction(+:calc_time) private(stateX, state_next, T_star) shared(state_sync_index, point_frame_dts) num_threads(6)
    // #pragma omp parallel for private(stateX, state_next) shared(state_sync_index, point_frame_dts) num_threads(8)
    for (size_t k = 0; k < cloud_size; k++){
        // rclcpp::Time time_calc_start = system_clock.now();
        PointType point = cloud_in.points[k];
        int i = state_sync_index[k];
        double point_frame_dt = point_frame_dts[k];
        double sync_time = 0.0; // sync_time should be negative, time between scan start and preceding imu measurement
        if (i == 0) {
            // RCLCPP_INFO(get_logger(), "i == 0, k = %i", k);
            stateX = lidar_synced_state;
            state_next = preint_states[1];
        } else {
            stateX = preint_states[i];
            state_next = preint_states[i+1]; // WHAT IF ARRAY IS ONLY 'i' BIG?! -> apparantly this is not a problem
            sync_time = lidar_synced_state.time - stateX.time ;
        }
        double point_state_dt = point_frame_dt + sync_time; 

        if (point_state_dt < 0.0){  // error detection.. only really happens if the dt is "negative zero" - beyond machine precision
            // RCLCPP_WARN(get_logger(), "Negative point dt detected state dt %f, frame dt %f, sync time %f, index %i", point_state_dt, point_frame_dt, sync_time, k);
            // cout << "Negative point dt detected state dt "<< to_string(point_state_dt) <<", frame dt  "<< to_string(point_frame_dt) <<", sync time  "<< to_string(sync_time) <<", index  "<< to_string(k) << "\n";
            point_state_dt = +0.0;
        }


        // calculate point specific transform..
        // Eigen::Vector3d point_translation_distortion = stateX.pos + stateX.vel * point_state_dt +  (stateX.ori.matrix() * (stateX.acc * 0.5*point_state_dt*point_state_dt)) + 1.0/6.0 * stateX.jerk *point_state_dt*point_state_dt*point_state_dt;
        
        // Eigen::Quaterniond point_rotation_distortion_alpha = deltaQ( 0.5 * point_state_dt*point_state_dt * stateX.alpha);
        // Eigen::Quaterniond point_rotation_distortion_rate = deltaQ(stateX.ang * point_state_dt);
            
        // Eigen::Quaterniond point_rotation_distortion = (point_rotation_distortion_alpha * point_rotation_distortion_rate) * stateX.ori;
        // point_rotation_distortion.normalize();

        // if (!point_rotation_distortion_rate.matrix().allFinite()){
        //     RCLCPP_WARN(get_logger(), "Nan Quaternion detected!");
        //     point_rotation_distortion_rate = Eigen::Quaterniond::Identity();
        // }
        auto t1 = std::chrono::high_resolution_clock::now();
        INSstate point_undistort_state = calcKinematicInterpolatedState(stateX, state_next, point_state_dt); // returns the interpolated kinematic state at the point time
        auto t2 = std::chrono::high_resolution_clock::now();
        
        Eigen::Vector3d point_translation_distortion = point_undistort_state.pos;
        Eigen::Quaterniond point_rotation_distortion = point_undistort_state.ori;

        T_star.block<3,3>(0,0) = point_rotation_distortion.matrix();
        T_star.block<3,1>(0,3) = point_translation_distortion;

        // Eigen::Matrix4d T_star_normal = Eigen::Matrix4d::Identity();
        // T_star_normal.block<3,3>(0,0) = Eigen::Matrix3d(point_rotation_distortion);

        // if (!pcl::isFinite(point))
        // {
            // RCLCPP_INFO_ONCE(get_logger(), "point NAN! before undistortion");
            // cout << "point NAN! before undistortion" << "\n";
        // 
        // apply transformation to point
        // Eigen::Vector3d point_vector(point.x, point.y, point.z);
        // point_vector = point_rotation_distortion * point_vector + point_translation_distortion;

        Eigen::Vector4d point_vector(point.x, point.y, point.z, 1.0); // create homogenous representatation of point
        point_vector = T_star*point_vector;

        Eigen::Vector3d point_normal_vector(point.normal_x, point.normal_y, point.normal_z);
        point_normal_vector = T_star.block<3,3>(0,0)*point_normal_vector;

        point.x = point_vector.x();
        point.y = point_vector.y();
        point.z = point_vector.z();

        point.normal_x = point_normal_vector.x();
        point.normal_y = point_normal_vector.y();
        point.normal_z = point_normal_vector.z();
        cloud_out.points[k] = point;

        // if (!pcl::isFinite(*point))
        if (!pcl::isFinite(point))
        {
            // cout << "point NAN After undistortion!" << "\n";
            // RCLCPP_WARN_ONCE(get_logger(), "point NAN After undistortion!");
            // RCLCPP_WARN_ONCE(get_logger(), "omp thread %i", omp_get_thread_num());
            // RCLCPP_WARN_ONCE(get_logger(), "point number %i", k);
            // RCLCPP_WARN_ONCE(get_logger(), "state to point dt: %f", point_state_dt);
            // RCLCPP_WARN_ONCE(get_logger(), "stateX time: %f", stateX.time);
            // RCLCPP_WARN_ONCE(get_logger(), "stateX pos: %f %f %f", stateX.pos.x(), stateX.pos.y(), stateX.pos.z());
            // RCLCPP_WARN_ONCE(get_logger(), "stateX vel: %f %f %f", stateX.vel.x(), stateX.vel.y(), stateX.vel.z());
            // RCLCPP_WARN_ONCE(get_logger(), "stateX acc: %f %f %f", stateX.acc.x(), stateX.acc.y(), stateX.acc.z());
            // RCLCPP_WARN_ONCE(get_logger(), "stateX ang vel: %f %f %f", stateX.ang.x(), stateX.ang.y(), stateX.ang.z());
            // RCLCPP_WARN_ONCE(get_logger(), "stateX jerk: %f %f %f", stateX.jerk.x(), stateX.jerk.y(), stateX.jerk.z());
            // RCLCPP_WARN_ONCE(get_logger(), "stateX alpha: %f %f %f", stateX.alpha.x(), stateX.alpha.y(), stateX.alpha.z());
            // RCLCPP_WARN_ONCE(get_logger(), "point translation: %f %f %f", point_translation_distortion.x(), point_translation_distortion.y(), point_translation_distortion.z());
            // // RCLCPP_WARN_ONCE(get_logger(), "point rotation vel: %f %f %f %f", dq_vel.w(), dq_vel.x(), dq_vel.y(), dq_vel.z());
            // // RCLCPP_WARN_ONCE(get_logger(), "point rotation rate: %f %f %f %f", dq_rate.w(), dq_rate.x(), dq_rate.y(), dq_rate.z());
            // RCLCPP_WARN_ONCE(get_logger(), "point rotation: %f %f %f %f", point_rotation_distortion.w(), point_rotation_distortion.x(), point_rotation_distortion.y(), point_rotation_distortion.z());
            // // printINSstate(stateX);
        }

        // rclcpp::Time time_calc_end = system_clock.now();
        // calc_time += (time_calc_end.seconds() - time_calc_start.seconds()) * 1000.0;
        
        calc_time += getTimeDouble(t2 - t1) *1000;
    }
    auto u2 = std::chrono::high_resolution_clock::now();
    double undistort_time = getTimeDouble(u2 - u1) *1000;

    // rclcpp::Time time_undistort_end = system_clock.now();
    // undistort_time = (time_undistort_end.seconds() - time_undistort_start.seconds()) * 1000.0;

    delete[] state_sync_index; // clear the memory usage
    delete[] point_frame_dts;

    // RCLCPP_INFO(get_logger(), "---- UNDISTORTION ---- ");
    // RCLCPP_INFO(get_logger(), "Undistortion of %i points, MP cpu time: %fms, avg sub-calc time: %fms", cloud_in.points.size(), undistort_time,  calc_time/(float)cloud_in.points.size() );
    // RCLCPP_INFO(get_logger(), "Total non-MP process time: %fms", calc_time );
    cout << "Undistortion of " << cloud_in.points.size()<< " points, MP cpu time: " << undistort_time  << " ms, avg sub-calc time: " <<  calc_time/(float)cloud_in.points.size() << " ms, total non-MP time: "<< calc_time <<" ms\n";
}

const INSstate& INS::getPreintState(int index)
{
    return this->preint_states[index];
}

const INSstate& INS::getPredictState(int index)
{
    return this->predict_states[index];
}

Eigen::Quaterniond getMadgwickOrientation(double timestamp)
{

}

///////////////////////////////////////////////////////////////////////////////////////////



class LidarOdometry : public rclcpp::Node 
{
    private:
        rclcpp::Clock run_clock;

        //callback groups
        rclcpp::CallbackGroup::SharedPtr subscriber_cb_group_;
        rclcpp::CallbackGroup::SharedPtr subscriber_cb_group2_;
        rclcpp::CallbackGroup::SharedPtr run_cb_group_;

        rclcpp::TimerBase::SharedPtr run_timer;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_data_service;

        // pose representation: [quaternion: w, x, y, z | transition: x, y, z]
        // double abs_pose[7];   //absolute pose from current frame to the first frame / odometry
        // double rel_pose[7];   //relative pose between two frames

        
        boost::shared_ptr<pcl::Registration < PointType, PointType >> registration_;
        boost::shared_ptr<pcl::Registration < PointType, PointType >> registration_fuse_;

        rclcpp::Clock system_clock;
        double registration_process_time;

        double system_start_time{};


        bool system_initialized = false;
        bool new_cloud_ready = false;
        bool init_map_built = false;

        size_t latest_frame_idx;
        size_t latest_keyframe_idx;

        double icp_fitness = 0.0;
        deque<double> fitnesses;

        deque<double> imu_dts;
        double last_imu_time{};

        double first_lidar_msg_time{};
        double scan_dt{};
        double current_scan_time{};
        // double next_scan_time{};

        double cloud_scale;
        double prev_cloud_scale;

        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg;

        Eigen::Matrix4d initial_pose_matrix;
        Pose initial_pose;

        // transformation matrices
        Transformation registration_transformation;
        Transformation last_odometry_transformation;
        Transformation preint_transformation_guess;
        Transformation keyframe_to_odometry_transformation;
        Transformation preint_residual;
        Pose last_odometry_pose;

        Pose ins_pose_new;
        Pose ins_pose;
        Transformation ins_relative;

        // Eigen::Matrix4d registration_transformation;
        // Eigen::Matrix4d last_odometry_transformation;
        // Eigen::Matrix4d last_odometry_pose;
        Eigen::Matrix4d odometry_transformation_guess;

        Eigen::Matrix4f init_guess;


        // deque<Eigen::Matrix4d> keyframe_poses; // keyframes class instead?
        Pose keyframe_pose;
        deque<Pose> keyframe_poses; // keyframes class instead?
        // deque<size_t> keyframe_index; // keyframes class instead?



        // headers and header information
        rclcpp::Time time_new_cloud;
        std_msgs::msg::Header cloud_header;

        nav_msgs::msg::Odometry odom;
        nav_msgs::msg::Path path;
        nav_msgs::msg::Path path_ins;
        nav_msgs::msg::Path path_predicted_ins;

        geometry_msgs::msg::PoseWithCovarianceStamped transformation_geommsg;

        // pcl filters downsampling
        pcl::VoxelGrid<PointType> down_size_filter;
        pcl::VoxelGrid<PointType> down_size_filter_local_map;
        pcl::VoxelGrid<PointType> down_size_filter_keyframe_map;

        //point clouds and vector containing pointclouds
        pcl::PointCloud<PointType>::Ptr cloud_in = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr cloud_in_ds = boost::make_shared<pcl::PointCloud<PointType>>();
        // pcl::PointCloud<PointType>::Ptr cloud_prev = boost::make_shared<pcl::PointCloud<PointType>>();
        // pcl::PointCloud<PointType>::Ptr cloud_prev_ds = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr cloud_keyframe = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr cloud_keyframe_ds = boost::make_shared<pcl::PointCloud<PointType>>();

        deque<sensor_msgs::msg::PointCloud2> cloud_queue;
        sensor_msgs::msg::PointCloud2 current_cloud_msg;

        pcl::PointCloud<PointType>::Ptr keyframe_map = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr keyframe_map_ds = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr local_map = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr local_map_ds = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr long_term_map = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr target_map = boost::make_shared<pcl::PointCloud<PointType>>();

        pcl::PointCloud<PointType>::Ptr reduced_global_map = boost::make_shared<pcl::PointCloud<PointType>>();

        deque<pcl::PointCloud<PointType>::Ptr> recent_frames;


        vector<boost::shared_ptr<pcl::PointCloud<PointType>>> all_clouds;

        // "point cloud" containing a specific type that has the odometry information
        pcl::PointCloud<PoseInfo>::Ptr odometry_pose_info = boost::make_shared<pcl::PointCloud<PoseInfo>>(); // add keyframe pose info?
        pcl::PointCloud<PoseInfo>::Ptr keyframe_pose_info = boost::make_shared<pcl::PointCloud<PoseInfo>>(); // add keyframe pose info?
        // pcl::PointCloud<pcl::PointXYZI>::Ptr odometry_pose_positions = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

        // subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr long_term_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub;
        // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ins_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr bias_sub;

        //publishers
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fullpointcloud_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloudprior_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_cloud_pub;
        // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalcloud_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr localcloud_pub;

        // rclcpp::Publisher<std_msgs::msg::Int64 >::SharedPtr keyframe_idx_pub;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ins_pub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr keyframe_odometry_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_ins_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_predicted_ins_pub;

        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr lidar_odometry_transformation_pub;

        deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        double translation_std_x;
        double translation_std_y;
        double translation_std_z;

        double translation_std_min_x_;
        double translation_std_min_y_;
        double translation_std_min_z_;


        // Eigen::Quaterniond preint_quat ;
        // Eigen::Vector3d preint_position;
        // Eigen::Vector3d preint_velocity;

        INS ins; // the new ins class that will handle INS and preintegration
        // rclcpp::Node ins; // the new ins class that will handle INS and preintegration

        INSstate state0;

        INSstate preint_state;
        INSstate observer_state;
        INSstate preint_anchor;
        IMUwrench preint_bias;
        // IMUwrench observer_bias;
        // IMUwrench kalman_bias;
        deque<INSstate> preint_states;


        Eigen::Vector3d average_translation;
        deque<Eigen::Vector3d> recent_translations;

        bool initial_pose_recieved{};
        bool first_lidar_received{};

        // parameters
        // ds_voxelsize
        // ds_voxelsize_lc
        float ds_voxel_size_;
        float ds_voxel_size_lc_;
        float ds_voxel_size_kf_;
        // strings
        std::string frame_id = "lidar_odom";
        // this->declare_parameter("my_parameter", "world");
        double keyframe_threshold_angle_;
        double keyframe_threshold_length_;
        double keyframe_threshold_fitness_;
        int keyframe_threshold_index_; // max frames between keyframes
        int icp_max_iterations_;
        int icp_max_coarse_iterations_;
        double icp_max_correspondence_distance_;
        int coarse_correspondence_factor_;
        int local_map_width_;
        int start_idx_;
        int end_idx_;

        size_t max_frames;


        int points_per_cloud_scale_;
        int scan_matching_method_;

        int local_map_init_frames_count_;

        bool use_cloud_scale_for_ds_{};
        bool use_wheel_constraint_{};
        bool use_ins_guess_{};
        bool use_preint_imu_guess_{};
        bool use_lidar_odometry_guess_{};
        bool use_preint_undistortion_{};

        bool save_running_data_{};

        double ds_lc_voxel_size_ratio_;
        double cloud_scale_previuos_cloud_weight_;

        double start_delay_s_;
        double sync_offset_ms_;


        double gamma_1_;
        double gamma_2_;
        double gamma_3_;
        double gamma_4_;
        double gamma_5_;


        std::string imu_topic_;

        std::string save_path_;

        // publish topics as parameters so they can be changed?

        std::ofstream data_file;
    

    public:
        LidarOdometry()
        : Node("lidar_odometry")
        {

            declare_parameter("start_delay_s", 0.0f);
            get_parameter("start_delay_s", start_delay_s_);

            declare_parameter("sync_offset_ms", 0.1f);
            get_parameter("sync_offset_ms", sync_offset_ms_);

            declare_parameter("ds_voxel_size", 0.1f);
            get_parameter("ds_voxel_size", ds_voxel_size_);

            declare_parameter("ds_voxel_size_lc", 0.1f);
            get_parameter("ds_voxel_size_lc", ds_voxel_size_lc_);

            declare_parameter("ds_voxel_size_kf", 0.5f);
            get_parameter("ds_voxel_size_kf", ds_voxel_size_kf_);

            declare_parameter("keyframe_threshold_length", 0.3);
            get_parameter("keyframe_threshold_length", keyframe_threshold_length_);

            declare_parameter("keyframe_threshold_angle", 2.0);
            get_parameter("keyframe_threshold_angle", keyframe_threshold_angle_);

            declare_parameter("keyframe_threshold_fitness", 0.1); // 0.5 * icp_correspondance threshold
            get_parameter("keyframe_threshold_fitness", keyframe_threshold_fitness_);

            declare_parameter("keyframe_threshold_index", 0); 
            get_parameter("keyframe_threshold_index", keyframe_threshold_index_);

            declare_parameter("scan_matching_method", 0); 
            get_parameter("scan_matching_method", scan_matching_method_);

            declare_parameter("icp_max_iterations", 25); 
            get_parameter("icp_max_iterations", icp_max_iterations_);

            declare_parameter("icp_max_coarse_iterations", 5); 
            get_parameter("icp_max_coarse_iterations", icp_max_coarse_iterations_);

            declare_parameter("icp_max_correspondance_distance", 0.5); 
            get_parameter("icp_max_correspondance_distance", icp_max_correspondence_distance_);

            declare_parameter("course_correspondence_factor", 10); 
            get_parameter("course_correspondence_factor", coarse_correspondence_factor_);

            declare_parameter("local_map_width", 20); 
            get_parameter("local_map_width", local_map_width_);

            declare_parameter("local_map_init_frames_count", 0); 
            get_parameter("local_map_init_frames_count", local_map_init_frames_count_);

            declare_parameter("use_cloud_scale_for_ds", true); 
            get_parameter("use_cloud_scale_for_ds", use_cloud_scale_for_ds_);

            declare_parameter("points_per_cloud_scale", 25); 
            get_parameter("points_per_cloud_scale", points_per_cloud_scale_);

            declare_parameter("use_wheel_constraint", false); 
            get_parameter("use_wheel_constraint", use_wheel_constraint_);

            declare_parameter("imu_topic", "/imu/data"); 
            get_parameter("imu_topic", imu_topic_);

            declare_parameter("use_ins_guess", false); 
            get_parameter("use_ins_guess", use_ins_guess_);

            declare_parameter("use_preint_imu_guess", false); 
            get_parameter("use_preint_imu_guess", use_preint_imu_guess_);

            declare_parameter("use_lidar_odometry_guess", true); 
            get_parameter("use_lidar_odometry_guess", use_lidar_odometry_guess_);

            declare_parameter("use_preint_undistortion", true); 
            get_parameter("use_preint_undistortion", use_preint_undistortion_);

            declare_parameter("cloud_scale_previuos_cloud_weight", 0.5); 
            get_parameter("cloud_scale_previuos_cloud_weight", cloud_scale_previuos_cloud_weight_);

            declare_parameter("ds_lc_voxel_size_ratio", 3.0); 
            get_parameter("ds_lc_voxel_size_ratio", ds_lc_voxel_size_ratio_);

            declare_parameter("translation_std_min_x", 0.01); 
            get_parameter("translation_std_min_x", translation_std_min_x_);
            declare_parameter("translation_std_min_y", 0.01); 
            get_parameter("translation_std_min_y", translation_std_min_y_);
            declare_parameter("translation_std_min_z", 0.01); 
            get_parameter("translation_std_min_z", translation_std_min_z_);

            declare_parameter("gamma_1", 10.0); 
            get_parameter("gamma_1", gamma_1_);
            declare_parameter("gamma_2", 10.0); 
            get_parameter("gamma_2", gamma_2_);
            declare_parameter("gamma_3", 10.0); 
            get_parameter("gamma_3", gamma_3_);
            declare_parameter("gamma_4", 10.0); 
            get_parameter("gamma_4", gamma_4_);
            declare_parameter("gamma_5", 10.0); 
            get_parameter("gamma_5", gamma_5_);

            declare_parameter("save_running_data", true); 
            get_parameter("save_running_data", save_running_data_);
            
            declare_parameter("save_path", "temp_saved_odometry_data/run_data.csv"); 
            get_parameter("save_path", save_path_);


            declare_parameter("start_idx", 0); 
            get_parameter("start_idx", start_idx_);

            // declare_parameter("end_idx", std::numeric_limits<int>::max ()); 
            declare_parameter("end_idx", 0); // 0 means endless 
            get_parameter("end_idx", end_idx_);

            // RCLCPP_INFO(get_logger(), "ds_voxel_size in constructor is: %f", ds_voxel_size_);

            initializeParameters();
            initializeScanmatcher();
            initializeFusematcher();
            allocateMemory();

            // setup callback groups
            run_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            // publsiher callback groud added?
            subscriber_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // create subscriber callback group
            rclcpp::SubscriptionOptions options; // create subscribver options
            options.callback_group = subscriber_cb_group_; // add callbackgroup to subscriber options

            subscriber_cb_group2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // create subscriber callback group
            rclcpp::SubscriptionOptions options2; // create subscribver options
            options2.callback_group = subscriber_cb_group2_; // add callbackgroup to subscriber options

            run_timer = this->create_wall_timer(20ms, std::bind(&LidarOdometry::run, this), run_cb_group_); // the process timer 

            // save_data_service = this->create_service<std_srvs::srv::Trigger>("save_odometry_data", &LidarOdometry::save_data, rmw_qos_profile_services_default , subscriber_cb_group_);
            save_data_service = this->create_service<std_srvs::srv::Trigger>("save_odometry_data", std::bind(&LidarOdometry::saveDataService,this, std::placeholders::_1, std::placeholders::_2));


            pointcloud_sub_ = this->create_subscription<PC_msg>("/preprocessed_point_cloud", 100, std::bind(&LidarOdometry::pointCloudHandler, this, _1), options);
            long_term_sub_ = this->create_subscription<PC_msg>("/long_term_map", 10, std::bind(&LidarOdometry::longTermMapHandler, this, _1), options);
            // pointcloud_sub_ = this->create_subscription<PC_msg>("/surf_features", 100, std::bind(&LidarOdometry::pointCloudHandler, this, _1), options);
            // pointcloud_sub_ = this->create_subscription<PC_msg>("/edge_features", 100, std::bind(&LidarOdometry::pointCloudHandler, this, _1), options);
            // initial_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initial_pose", 100, std::bind(&LidarOdometry::initialPoseHandler, this, _1), options);
            // ins_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom_ins", 100, std::bind(&LidarOdometry::insHandler, this, _1), options);
            imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_, 1000, std::bind(&LidarOdometry::imuDataHandler, this, _1), options2);
            bias_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>("/kalman_bias", 100, std::bind(&LidarOdometry::biasHandler, this, _1), options2);

            pointcloud_pub = this->create_publisher<PC_msg>("/ds_point_cloud_odom", 10);
            fullpointcloud_pub = this->create_publisher<PC_msg>("/full_point_cloud_odom", 10);
            pointcloudprior_pub = this->create_publisher<PC_msg>("/full_point_cloud_transform_guess", 10);
            keyframe_cloud_pub = this->create_publisher<PC_msg>("/full_point_cloud_keyframe", 2);

            // globalcloud_pub = this->create_publisher<PC_msg>("/global_point_cloud", 100);
            localcloud_pub = this->create_publisher<PC_msg>("/local_point_cloud", 10);

            // current odometry publisher
            odometry_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 25);
            keyframe_odometry_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom_keyframe", 25);
            ins_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom_ins", 10);

            path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
            path_ins_pub = this->create_publisher<nav_msgs::msg::Path>("/path_ins", 10);
            path_predicted_ins_pub = this->create_publisher<nav_msgs::msg::Path>("/path_predicted_ins", 10);

            lidar_odometry_transformation_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/transformation/lidar", 100);

            // keyframe_idx_pub = this->create_publisher<std_msgs::msg::Int64>("keyframe_idx", 100);

            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  
            initial_pose_matrix = Eigen::Matrix4d::Identity();
            initial_pose.orientation = Eigen::Quaterniond::Identity();
            initial_pose.position = Eigen::Vector3d::Zero();

            preint_state.ori = Eigen::Quaterniond::Identity();
            preint_state.pos = Eigen::Vector3d::Zero();
            preint_state.vel = Eigen::Vector3d::Zero();
            preint_state.acc = Eigen::Vector3d::Zero();
            preint_state.time = 0.0;

            preint_bias.acc = Eigen::Vector3d::Zero();
            preint_bias.ang = Eigen::Vector3d::Zero();
            preint_bias.time = 0.0;

            preint_state.bias = preint_bias;

            observer_state = preint_state;

            // observer_bias = preint_bias;
            // kalman_bias = preint_bias;

            ins = INS(); // maybe add an expected dt from rosparameter input 
        
            initDataFile();
        }
        ~LidarOdometry(){}

        void saveDataService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response>  response){
        // void save_data(){
            RCLCPP_INFO(get_logger(), "Request to save data received %d.", request);
            bool data_to_be_saved = fitnesses.size() > 0;
            size_t elements_to_save = fitnesses.size();

            if (data_to_be_saved){

                std::ofstream file;
                file.open ("temp_saved_odometry_data/fitness.csv");
                for(size_t i=0; i < elements_to_save; i++){
                    double fitn = fitnesses.front();
                    file << std::to_string(fitn) + "\n"; 
                    fitnesses.pop_front();
                }

                // file << "This is the first cell in the first column.\n";
                // file << "a,b,c,\n";
                // file << "c,s,v,\n";
                // file << "1,2,3.456\n";
                // file << "semi;colon";
                file.close();
            } 

            bool data_emptied_to_file = fitnesses.size() == 0;

            response->success = data_to_be_saved && data_emptied_to_file;
            response->message = "End of service" ;

            RCLCPP_INFO(get_logger(), "Data has been saved, Data: %s, Emptied: %s ", data_to_be_saved? "true":"false", data_emptied_to_file?"true":"false");
        
        }

        void initDataFile()
        {   
            if (!save_running_data_)
                return;

            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);
            std::ostringstream oss;
            oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
            std::string datestr = oss.str();


            data_file.open (save_path_ + datestr + "_run_data.csv");

            // write the header to the save file

            data_file << "ds_voxel_size: " + std::to_string(ds_voxel_size_)+"\n";
            data_file << "ds_voxel_size_lc: "+ std::to_string(ds_voxel_size_lc_)+"\n";
            data_file << "use_cloud_scale_for_ds: "+ std::to_string(use_cloud_scale_for_ds_)+"\n";
            data_file << "cloud_scale_previuos_cloud_weight: "+ std::to_string(cloud_scale_previuos_cloud_weight_)+"\n";
            data_file << "ds_lc_voxel_size_ratio: "+ std::to_string(ds_lc_voxel_size_ratio_)+"\n";
            data_file << "points_per_cloud_scale: "+ std::to_string(points_per_cloud_scale_)+"\n";
            data_file << "keyframe_threshold_length: "+ std::to_string(keyframe_threshold_length_)+"\n";
            data_file << "keyframe_threshold_angle: "+ std::to_string(keyframe_threshold_angle_)+"\n";
            data_file << "keyframe_threshold_fitness: "+ std::to_string(keyframe_threshold_fitness_)+"\n";
            data_file << "keyframe_threshold_index: "+ std::to_string(keyframe_threshold_index_)+"\n";
            data_file << "scan_matching_method: "+ std::to_string(scan_matching_method_)+"\n";
            data_file << "icp_max_iterations: "+ std::to_string(icp_max_iterations_)+"\n";
            data_file << "icp_max_coarse_iterations: "+ std::to_string(icp_max_coarse_iterations_)+"\n";
            data_file << "icp_max_correspondence_distance: "+ std::to_string(icp_max_correspondence_distance_)+"\n";
            data_file << "coarse_correspondence_factor: "+ std::to_string(coarse_correspondence_factor_)+"\n";
            data_file << "local_map_width: "+ std::to_string(local_map_width_)+"\n";
            data_file << "local_map_init_frames_count: "+ std::to_string(local_map_init_frames_count_)+"\n";
            data_file << "imu_topic:"+ imu_topic_+"\n";
            data_file << "use_ins_guess: "+ std::to_string(use_ins_guess_)+"\n";
            data_file << "use_preint_imu_guess: "+ std::to_string(use_preint_imu_guess_)+"\n";
            data_file << "use_lidar_odometry_guess: "+ std::to_string(use_lidar_odometry_guess_)+"\n";
            data_file << "use_preint_undistortion: "+ std::to_string(use_preint_undistortion_)+"\n";
            data_file << "sync_offset_ms: "+ std::to_string(sync_offset_ms_)+"\n";
            data_file << "translation_std_min_x: "+ std::to_string(translation_std_min_x_)+"\n";
            data_file << "translation_std_min_y: "+ std::to_string(translation_std_min_y_)+"\n";
            data_file << "translation_std_min_z: "+ std::to_string(translation_std_min_z_)+"\n";
            data_file << "gamma_1: "+ std::to_string(gamma_1_)+"\n";
            data_file << "gamma_2: "+ std::to_string(gamma_2_)+"\n";
            data_file << "gamma_3: "+ std::to_string(gamma_3_)+"\n";
            data_file << "gamma_4: "+ std::to_string(gamma_4_)+"\n";
            data_file << "gamma_5: "+ std::to_string(gamma_5_)+"\n";

            data_file << "time, x, y, z, vx, vy, vz, obs_vx, obs_vy, obs_vz, qw, qx, qy, qz, fitness, residual_x, residual_y, residual_z, residual_qw, residual_qx, residual_qy, residual_qz, cov_x, cov_y, cov_z, bias_acc_x, bias_acc_y, bias_acc_z, bias_ang_x, bias_ang_y, bias_ang_z";

        }

        void saveRunningData()
        {
            if (!save_running_data_)
                return;

            RCLCPP_DEBUG(get_logger(), "---- SAVING DATA ----");
            rclcpp::Time time_save_start = system_clock.now();
            // order of data: "time, x, y, z, vx, vy, vz, obs_vx, obs_vy, obs_vz, qw, qx, qy, qz, fitness, residual_x, residual_y, residual_z, residual_qw, residual_qx, residual_qy, residual_qz, cov_x, cov_y, cov_z, bias_acc_x, bias_acc_y, bias_acc_z, bias_ang_x, bias_ang_y, bias_ang_z "
            deque<double> print_qeue;
            print_qeue.push_back(current_scan_time);                    // time
            print_qeue.push_back(last_odometry_pose.position.x());      // x          
            print_qeue.push_back(last_odometry_pose.position.y());      // y          
            print_qeue.push_back(last_odometry_pose.position.z());      // z  
            print_qeue.push_back(last_odometry_pose.velocity.x());      // vx          
            print_qeue.push_back(last_odometry_pose.velocity.y());      // vy          
            print_qeue.push_back(last_odometry_pose.velocity.z());      // vz  
            print_qeue.push_back(observer_state.vel.x());               // obs_vx          
            print_qeue.push_back(observer_state.vel.y());               // obs_vy          
            print_qeue.push_back(observer_state.vel.z());               // obs_vz          
            print_qeue.push_back(last_odometry_pose.orientation.w());   // qw         
            print_qeue.push_back(last_odometry_pose.orientation.x());   // qx         
            print_qeue.push_back(last_odometry_pose.orientation.y());   // qy         
            print_qeue.push_back(last_odometry_pose.orientation.z());   // qz         
            print_qeue.push_back(icp_fitness);                          // fitness 
            print_qeue.push_back(preint_residual.translation.x());      // residual_x
            print_qeue.push_back(preint_residual.translation.y());      // residual_y
            print_qeue.push_back(preint_residual.translation.z());      // residual_z
            print_qeue.push_back(preint_residual.rotation.w());         // residual_qw 
            print_qeue.push_back(preint_residual.rotation.x());         // residual_qx 
            print_qeue.push_back(preint_residual.rotation.y());         // residual_qy 
            print_qeue.push_back(preint_residual.rotation.z());         // residual_qz 
            print_qeue.push_back(translation_std_x*translation_std_x);  // cov_x 
            print_qeue.push_back(translation_std_y*translation_std_y);  // cov_y 
            print_qeue.push_back(translation_std_z*translation_std_z);  // cov_z 
            print_qeue.push_back(observer_state.bias.acc.x());          // bias_acc_x          
            print_qeue.push_back(observer_state.bias.acc.y());          // bias_acc_y         
            print_qeue.push_back(observer_state.bias.acc.z());          // bias_acc_z          
            print_qeue.push_back(observer_state.bias.ang.x());          // bias_ang_x          
            print_qeue.push_back(observer_state.bias.ang.y());          // bias_ang_y         
            print_qeue.push_back(observer_state.bias.ang.z());          // bias_ang_z          

            int length =  (int)print_qeue.size();

            data_file << "\n";
            for(int i=0; i < length; i++){
                data_file << std::to_string(print_qeue.front()); 
                if (i < length-1){
                    data_file << ","; 
                }
                print_qeue.pop_front();
            }
            print_qeue.clear(); // just to make sure it is empty

            rclcpp::Time time_save_end = system_clock.now();

            RCLCPP_DEBUG(get_logger(), "Odometry data saved: %fms", time_save_end.seconds()*1000.0 - time_save_start.seconds()*1000.0);
        }
    

        void pointCloudHandler(const PC_msg::SharedPtr lidar_cloud_msg )
        {   
            if (!first_lidar_received) {
                first_lidar_msg_time = toSec(lidar_cloud_msg->header.stamp);
            }

            RCLCPP_INFO_ONCE(get_logger(), "First Lidar cloud received - timestamp: %f..", first_lidar_msg_time);
            
            first_lidar_received = true;
            cloud_queue.push_back(*lidar_cloud_msg);

        }

        void longTermMapHandler(const PC_msg::SharedPtr lidar_cloud_msg )
        {   
            RCLCPP_INFO_ONCE(get_logger(), "Long Term Map Updated..");
            // first_lidar_received = true;
            // cloud_queue.push_back(*lidar_cloud_msg);
            fromROSMsg(*lidar_cloud_msg, *long_term_map);
        }

        // not used atm
        // void initialPoseHandler(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initial_pose_msg)
        // {
        //     double qw = initial_pose_msg->pose.pose.orientation.w;
        //     double qx = initial_pose_msg->pose.pose.orientation.x;
        //     double qy = initial_pose_msg->pose.pose.orientation.y;
        //     double qz = initial_pose_msg->pose.pose.orientation.z;
        //     double x = initial_pose_msg->pose.pose.position.x;
        //     double y = initial_pose_msg->pose.pose.position.y;
        //     double z = initial_pose_msg->pose.pose.position.z;

        //     Eigen::Quaterniond quat( qx, qy, qz, qw );
            
        //     initial_pose_matrix.block<3,3>(0,0) = quat.toRotationMatrix();

        //     // also include a position offset at some point
        //     initial_pose_matrix.block<3,1>(0,3) = Eigen::Vector3d(x, y, z);

        //     RCLCPP_INFO_ONCE(get_logger(), "Initial pose recieved q: %f %f %f %f p: %f %f %f", qx, qy, qz, qw, x, y, z);
        //     // RCLCPP_INFO_ONCE(get_logger(), "Initial attitude recieved q: %f %f %f %f", initial_pose.orientation.w(), initial_pose.orientation.x(), initial_pose.orientation.y(), initial_pose.orientation.z());

        //     initial_pose_recieved = true;

        // }

        // // not used INS is internal in Loam node atm
        // void insHandler(const nav_msgs::msg::Odometry::SharedPtr ins_odom_msg){
        //     // the INS propagates to the time of the next frame so this pose is a "guess"
        //     ins_pose_new.orientation.w() = ins_odom_msg->pose.pose.orientation.w;
        //     ins_pose_new.orientation.x() = ins_odom_msg->pose.pose.orientation.x;
        //     ins_pose_new.orientation.y() = ins_odom_msg->pose.pose.orientation.y;
        //     ins_pose_new.orientation.z() = ins_odom_msg->pose.pose.orientation.z;

        //     ins_pose_new.position.x() = ins_odom_msg->pose.pose.position.x;
        //     ins_pose_new.position.y() = ins_odom_msg->pose.pose.position.y;
        //     ins_pose_new.position.z() = ins_odom_msg->pose.pose.position.z;

        //     ins_pose_new.velocity.x() = ins_odom_msg->twist.twist.linear.x;
        //     ins_pose_new.velocity.y() = ins_odom_msg->twist.twist.linear.y;
        //     ins_pose_new.velocity.z() = ins_odom_msg->twist.twist.linear.z;

        //     if (!initial_pose_recieved) {
        //     // if (!first_lidar_received ) {
        //         ins_pose = ins_pose_new;
        //         initial_pose = ins_pose_new;
        //         initial_pose_recieved = true;
        //         RCLCPP_INFO_ONCE(get_logger(), "INS Initial attitude recieved q: %f %f %f %f", initial_pose.orientation.w(), initial_pose.orientation.x(), initial_pose.orientation.y(), initial_pose.orientation.z());
        //     }
        // }

        void imuDataHandler(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
        {
            // put data into buffer back
            RCLCPP_INFO_ONCE(get_logger(),"First IMU message recieved..");
            // apply offset
            // double time = toSec(imu_msg->header.stamp);
            // time += sync_offset_ms_ *1e-3;
            // imu_msg->header.stamp = toStamp(time);

            // imu_buffer.push_back(imu_msg);
            double imu_msg_time = toSec(imu_msg->header.stamp);
            if (last_imu_time < imu_msg_time){
                // RCLCPP_INFO(get_logger(), "imu msg added to buffer timestamp: %f", imu_msg_time);
                ins.addImuMsgToBuffer(imu_msg);
                last_imu_time = imu_msg_time;
            }


            // if (!initial_pose_recieved) {
            if (!first_lidar_received ) { // makes it keep update the pose until a lidar msg is recieved
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for first LiDAR msg..");
                initial_pose.orientation.w() = imu_msg->orientation.w;
                initial_pose.orientation.x() = imu_msg->orientation.x;
                initial_pose.orientation.y() = imu_msg->orientation.y;
                initial_pose.orientation.z() = imu_msg->orientation.z;
                initial_pose_recieved = true;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Initial attitude updated q: %f %f %f %f", initial_pose.orientation.w(), initial_pose.orientation.x(), initial_pose.orientation.y(), initial_pose.orientation.z());
                // INS note: send initial pose to external INS
                // ins.updateInitialPose(initial_pose);
            }

        }

        void biasHandler(const geometry_msgs::msg::WrenchStamped::SharedPtr bias_msg)
        {
            preint_bias.acc = Eigen::Vector3d(bias_msg->wrench.force.x, bias_msg->wrench.force.y, bias_msg->wrench.force.z );
            preint_bias.ang = Eigen::Vector3d(bias_msg->wrench.torque.x, bias_msg->wrench.torque.y, bias_msg->wrench.torque.z );
            preint_bias.time = toSec(bias_msg->header.stamp);
        }


        void publishCurrentCloud()
        {   
            sensor_msgs::msg::PointCloud2 msgs;
            pcl::PointCloud<PointType> transformed_cloud;
            // transformed_cloud = 
            // pcl::transformPointCloudWithNormals<PointType>(*cloud_in_ds, transformed_cloud, preint_residual.translation, preint_residual.rotation);
            // pcl::transformPointCloudWithNormals<PointType>(*cloud_in_ds, transformed_cloud, last_odometry_pose.position, last_odometry_pose.orientation);
            
            transformed_cloud = *cloud_in_ds;
            #ifndef __INTELLISENSE__ 
            pcl::toROSMsg(transformed_cloud, msgs);
            #endif
            msgs.header.stamp = cloud_header.stamp;
            msgs.header.frame_id = frame_id;
            pointcloud_pub->publish(msgs);
            // RCLCPP_INFO(get_logger(), "Header: %s,  time of PCL: %f, num points: %i", cloud_header.frame_id, time_new_cloud.nanoseconds()/1e9, cloud_in->points.size());
        }

        void publishCurrentFullCloud()
        {   
            sensor_msgs::msg::PointCloud2 msgs;
            pcl::PointCloud<PointType> transformed_cloud;
            // transformed_cloud = 
            // pcl::transformPointCloudWithNormals<PointType>(*cloud_in_ds, transformed_cloud, preint_residual.translation, preint_residual.rotation);
            // pcl::transformPointCloudWithNormals<PointType>(*cloud_in_ds, transformed_cloud, last_odometry_pose.position, last_odometry_pose.orientation);
            
            transformed_cloud = *cloud_in;
            #ifndef __INTELLISENSE__ 
            pcl::toROSMsg(transformed_cloud, msgs);
            #endif
            msgs.header.stamp = cloud_header.stamp;
            msgs.header.frame_id = frame_id;
            fullpointcloud_pub->publish(msgs);
            // RCLCPP_INFO(get_logger(), "Header: %s,  time of PCL: %f, num points: %i", cloud_header.frame_id, time_new_cloud.nanoseconds()/1e9, cloud_in->points.size());
        }

        void publishPriorCloud(Eigen::Matrix4f transformation)
        {   
            sensor_msgs::msg::PointCloud2 msgs;
            pcl::PointCloud<PointType> transformed_cloud;
            pcl::transformPointCloudWithNormals<PointType>(*cloud_in_ds, transformed_cloud, transformation);
            #ifndef __INTELLISENSE__ 
            pcl::toROSMsg(transformed_cloud, msgs);
            #endif
            msgs.header.stamp = cloud_header.stamp;
            msgs.header.frame_id = frame_id;

            pointcloudprior_pub->publish(msgs);
        }
        
        void publishKeyframeCloud()
        {   
            sensor_msgs::msg::PointCloud2 msgs;
            #ifndef __INTELLISENSE__ 
            pcl::toROSMsg(*cloud_keyframe, msgs);
            #endif
            msgs.header.stamp = cloud_header.stamp;
            // msgs.header.frame_id = frame_id;
            msgs.header.frame_id = frame_id;

            keyframe_cloud_pub->publish(msgs);
            // RCLCPP_INFO(get_logger(), "Header: %s,  time of PCL: %f, num points: %i", cloud_header.frame_id, time_new_cloud.nanoseconds()/1e9, cloud_in->points.size());
        }

        void initializeFusematcher()
        {
            // typedef pcl::registration::TransformationEstimationPointToPlane<PointType, PointType> PointToPlane;
                // typedef pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<PointType, PointType> PointToPlane;
                // maybe not setup the icp on every use <-- this!          
                // boost::shared_ptr<pcl::IterativeClosestPointWithNormals<PointType, PointType>> icp(new pcl::IterativeClosestPointWithNormals<PointType, PointType>());
                // boost::shared_ptr<PointToPlane> p2pl(new PointToPlane);
                // icp->setTransformationEstimation(p2pl);
                // icp->setMaxCorrespondenceDistance(0.5);
                // icp->setUseReciprocalCorrespondences(true);
                // icp->setUseSymmetricObjective(true);
                // icp->setEnforceSameDirectionNormals(true);
                // icp->setMaximumIterations(10);

              
                boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>>
                icp(new pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>());
                
                // boost::shared_ptr<PointToPlane> p2pl(new PointToPlane);
                // gicp->setTransformationEstimation(p2pl);

                icp->setMaxCorrespondenceDistance(0.5);
                icp->setMaximumOptimizerIterations(0);
                icp->setMaximumIterations(30);
                icp->setCorrespondenceRandomness(25);
                icp->setUseReciprocalCorrespondences(true);
                icp->setTransformationEpsilon(1e-3);


                
                registration_fuse_ = icp;
                // method_name = "ICP p2pl";
        }

        void initializeScanmatcher()
        {
            RCLCPP_INFO(get_logger(), "Initializing Scanmatcher..");

            string method_name;
            if (scan_matching_method_ == 3) {

                // pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr
                // ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
                pclomp::NormalDistributionsTransform<PointType, PointType>::Ptr
                ndt(new pclomp::NormalDistributionsTransform<PointType, PointType>());
                ndt->setResolution(icp_max_correspondence_distance_);
                // ndt->setTransformationEpsilon(1e-5);
                ndt->setStepSize(0.2);
                // ndt_omp
                ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
                // if (ndt_num_threads > 0) {ndt->setNumThreads(ndt_num_threads);}
                ndt->setTransformationEpsilon(1e-4);
                ndt->setMaximumIterations(icp_max_coarse_iterations_);

                registration_ = ndt;
                method_name = "NDT OMP";

            } else if (scan_matching_method_ == 1 || scan_matching_method_ == 2) {
                // typedef pcl::registration::TransformationEstimationPointToPlane<PointType, PointType> PointToPlane;

                boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>>
                gicp(new pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>());
                method_name = "GICP OMP";
                if (scan_matching_method_ == 2) {
                    boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<PointType, PointType>>
                    gicp(new pcl::GeneralizedIterativeClosestPoint<PointType, PointType>());
                    method_name = "GICP";
                } 
                // boost::shared_ptr<PointToPlane> p2pl(new PointToPlane);
                // gicp->setTransformationEstimation(p2pl);

                gicp->setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
                gicp->setMaximumOptimizerIterations(500);
                gicp->setMaximumIterations(icp_max_iterations_);
                gicp->setCorrespondenceRandomness(25);
                gicp->setUseReciprocalCorrespondences(true);
                gicp->setTransformationEpsilon(1e-4);
                // gicp->setTransformationRotationEpsilon(cos(0.001));
                // max iterations... other settings?

                // gicp->search

                registration_ = gicp;

            } else {
                typedef pcl::registration::TransformationEstimationPointToPlane<PointType, PointType> PointToPlane;
                // typedef pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<PointType, PointType> PointToPlane;
                // maybe not setup the icp on every use <-- this!          
                boost::shared_ptr<pcl::IterativeClosestPointWithNormals<PointType, PointType>> icp(new pcl::IterativeClosestPointWithNormals<PointType, PointType>());
                boost::shared_ptr<PointToPlane> p2pl(new PointToPlane);
                icp->setTransformationEstimation(p2pl);
                icp->setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
                icp->setUseReciprocalCorrespondences(true);
                icp->setUseSymmetricObjective(true);
                icp->setEnforceSameDirectionNormals(true);
                icp->setMaximumIterations(icp_max_iterations_);
                icp->setTransformationEpsilon(1e-4);


                registration_ = icp;
                method_name = "ICP p2pl";
            }

            RCLCPP_INFO(get_logger(), "Scan Match Registration method is: %s", method_name.c_str()); // put here the algorithm that is chosen, maybe in red color
        }



        void initializeParameters()
        {
            RCLCPP_INFO(get_logger(), "Initializing Parameters..");
            // abs_pose[0] = 1;
            // rel_pose[0] = 1;

            // for (int i = 1; i < 7; ++i) {
            //     abs_pose[i] = 0;
            //     rel_pose[i] = 0;
            // }

            // if (initial_pose_recieved){
            //     RCLCPP_INFO(get_logger(), "Using Initial Attitude quaternion from Madgwick:  q_init: %f, %f, %f, %f",initial_pose.orientation.w(), initial_pose.orientation.x(), initial_pose.orientation.y(), initial_pose.orientation.z() );
            //     last_odometry_pose.orientation = initial_pose.orientation;
            //     observer_state.ori = initial_pose.orientation;
            //     preint_state.ori = initial_pose.orientation;
            // } else {
            //     RCLCPP_INFO(get_logger(), "No Initial attitude determined, assuming identity..");
            // }

            last_odometry_pose.orientation = Eigen::Quaterniond::Identity();
            observer_state.ori = Eigen::Quaterniond::Identity();
            preint_state.ori = Eigen::Quaterniond::Identity();

            last_odometry_pose.position = Eigen::Vector3d::Zero();
            last_odometry_pose.matrix = Eigen::Matrix4d::Identity();

            registration_transformation.matrix = Eigen::Matrix4d::Identity();
            registration_transformation.rotation = Eigen::Quaterniond::Identity();
            registration_transformation.translation = Eigen::Vector3d::Zero();

            last_odometry_transformation.rotation = Eigen::Quaterniond::Identity();
            last_odometry_transformation.translation = Eigen::Vector3d::Zero();

            keyframe_to_odometry_transformation.rotation = Eigen::Quaterniond::Identity();
            keyframe_to_odometry_transformation.translation = Eigen::Vector3d::Zero();
            
            preint_transformation_guess.rotation = Eigen::Quaterniond::Identity();
            preint_transformation_guess.translation = Eigen::Vector3d::Zero();
            preint_residual.rotation = Eigen::Quaterniond::Identity();
            preint_residual.translation = Eigen::Vector3d::Zero();

            ins_pose_new.orientation = Eigen::Quaterniond::Identity();
            ins_pose_new.position = Eigen::Vector3d::Zero();
            ins_pose.orientation = Eigen::Quaterniond::Identity();
            ins_pose.position = Eigen::Vector3d::Zero();
            ins_relative.rotation = Eigen::Quaterniond::Identity();
            ins_relative.translation = Eigen::Vector3d::Zero();

            // last_odometry_transformation = Eigen::Matrix4d::Identity();
            odometry_transformation_guess = Eigen::Matrix4d::Identity();
            init_guess = Eigen::Matrix4f::Identity();

            average_translation = Eigen::Vector3d::Zero();
            recent_translations.push_back(average_translation);

            // get_parameter("ds_voxel_size", ds_voxel_size_);

            
            odom.header.frame_id = "odom";
            odom.child_frame_id = frame_id;

            transformation_geommsg.header.frame_id = "odom";

            // // ds_voxel_size = 0.1f;
            // ds_voxel_size = 0.2f;
            // RCLCPP_INFO(get_logger(), "ds_voxel_size in function is: %f", ds_voxel_size_);
            down_size_filter.setLeafSize(ds_voxel_size_, ds_voxel_size_, ds_voxel_size_);
            down_size_filter_local_map.setLeafSize(ds_voxel_size_lc_, ds_voxel_size_lc_, ds_voxel_size_lc_);
            down_size_filter_keyframe_map.setLeafSize(ds_voxel_size_kf_, ds_voxel_size_kf_, ds_voxel_size_kf_);

            latest_frame_idx = 0;
            latest_keyframe_idx = 0;

            max_frames = local_map_init_frames_count_;
            if (local_map_width_ > 0){
                max_frames = local_map_width_;
            }

            system_initialized = false;
            system_start_time = run_clock.now().seconds();

        }


        void initializeSystem()
        {   
            RCLCPP_INFO(get_logger(), "Initializing System..");

            // make ESTIMATE INITIAL ORIENTATION function
            if (initial_pose_recieved){
                RCLCPP_INFO(get_logger(), "Using Initial Attitude quaternion from Madgwick:  q_init: %f, %f, %f, %f",initial_pose.orientation.w(), initial_pose.orientation.x(), initial_pose.orientation.y(), initial_pose.orientation.z() );
                last_odometry_pose.orientation =            Eigen::Quaterniond(initial_pose.orientation);
                last_odometry_pose.matrix = Eigen::Matrix4d::Identity();
                last_odometry_pose.matrix.block<3,3>(0,0) = Eigen::Matrix3d(last_odometry_pose.orientation);
                observer_state.ori  =                       Eigen::Quaterniond(initial_pose.orientation);
                preint_state.ori    =                       Eigen::Quaterniond(initial_pose.orientation);
                preint_anchor.ori   =                       Eigen::Quaterniond(initial_pose.orientation);
                preint_transformation_guess.rotation =      Eigen::Quaterniond(initial_pose.orientation);
                keyframe_pose.orientation =                 Eigen::Quaterniond(initial_pose.orientation);

                ins.initializeInitialPose(initial_pose.orientation);
            } else {
                RCLCPP_INFO(get_logger(), "No Initial attitude determined, assuming identity pose..");
                // last_odometry_pose.orientation = Eigen::Quaterniond::Identity();
                // observer_state.ori = Eigen::Quaterniond::Identity();
                // preint_state.ori = Eigen::Quaterniond::Identity();
                // preint_anchor.ori = Eigen::Quaterniond::Identity();
            }

            

            RCLCPP_INFO(get_logger(), "Max frames in local map is %i", max_frames);

            updateTransformationErrorFromConstVelocityModel();
            calcCloudScale();
            prev_cloud_scale = cloud_scale;
            last_odometry_pose.time = current_scan_time;


            pcl::transformPointCloudWithNormals<PointType>(*cloud_in, *cloud_in, last_odometry_pose.matrix);

            // *global_cloud += *cloud_in;
            downSampleClouds();
            pushKeyframe(); 

            publishCurrentCloud();

            savePose();
            // savePointCloud();
            publishOdometry();
            
            // addToLocalMap();

            publishLocalMap();
            // publishGlobalCloud();

            saveRunningData();

            system_initialized = true;
            RCLCPP_INFO(get_logger(), "First frame initialized...");
        }


        void allocateMemory()
        {
            RCLCPP_INFO(get_logger(), "Allocating Point Cloud Memory..");

            cloud_in.reset(new pcl::PointCloud<PointType>());
            cloud_in_ds.reset(new pcl::PointCloud<PointType>());
            // cloud_prev.reset(new pcl::PointCloud<PointType>());
            // cloud_prev_ds.reset(new pcl::PointCloud<PointType>());

            keyframe_map.reset(new pcl::PointCloud<PointType>());
            keyframe_map_ds.reset(new pcl::PointCloud<PointType>());
            local_map.reset(new pcl::PointCloud<PointType>());
            local_map_ds.reset(new pcl::PointCloud<PointType>());
            reduced_global_map.reset(new pcl::PointCloud<PointType>());
            long_term_map.reset(new pcl::PointCloud<PointType>());
            target_map.reset(new pcl::PointCloud<PointType>());

            odometry_pose_info.reset(new pcl::PointCloud<PoseInfo>()); // point of the odometry as clouds, enables fast knn search for nearby locations
            keyframe_pose_info.reset(new pcl::PointCloud<PoseInfo>());
            // odometry_pose_positions.reset(new pcl::PointCloud<pcl::PointXYZI>());
        }

        void pushKeyframe()
        {

            // pcl::PointCloud<PointType>::Ptr full = boost::make_shared<pcl::PointCloud<PointType>>();
            // full.reset(new pcl::PointCloud<PointType>());
            // pcl::copyPointCloud(*cloud_in, *full);
            // *cloud_keyframe = *full;

            Pose new_keyframe_pose;
            // new_keyframe_pose.orientation = Eigen::Quaterniond(last_odometry_pose.block<3,3>(0,0)); // this needs to change at some point
            new_keyframe_pose.orientation = last_odometry_pose.orientation; // this needs to change at some point
            new_keyframe_pose.position = last_odometry_pose.position;
            new_keyframe_pose.velocity = last_odometry_pose.velocity;
            new_keyframe_pose.idx = keyframe_poses.size();
            new_keyframe_pose.frame_idx = latest_frame_idx;
            // new_keyframe_pose.pointcloud = cloud_keyframe; // omitting this to save RAM
            new_keyframe_pose.time = current_scan_time; 
            new_keyframe_pose.matrix = Eigen::Matrix4d::Identity();
            new_keyframe_pose.matrix.block<3, 3>(0, 0) = new_keyframe_pose.orientation.matrix();
            new_keyframe_pose.matrix.block<3, 1>(0, 3) = new_keyframe_pose.position;

            // RCLCPP_INFO(get_logger(), "Keyframe Attitude quaternion:  q_init: %f, %f, %f, %f", new_keyframe_pose.orientation.w(), new_keyframe_pose.orientation.x(), new_keyframe_pose.orientation.y(), new_keyframe_pose.orientation.z() );
            
            keyframe_pose = new_keyframe_pose;
            // keyframe_poses.push_back(last_odometry_pose); 
            keyframe_poses.push_back(new_keyframe_pose); 


            // Eigen::Matrix4d K_to_O = Eigen::Matrix4d::Identity();
            // Eigen::Matrix4d Anchor_T = Eigen::Matrix4d::Identity();
            // K_to_O.block<3,3>(0,0) = Eigen::Matrix3d( keyframe_to_odometry_transformation.rotation);
            // K_to_O.block<3,1>(0,3) = keyframe_to_odometry_transformation.translation;
            // Anchor_T.block<3,3>(0,0) = Eigen::Matrix3d( preint_anchor.ori);
            // Anchor_T.block<3,1>(0,3) = preint_anchor.pos;

            // Anchor_T = K_to_O.inverse() * Anchor_T;

            // preint_anchor.pos = Anchor_T.block<3,1>(0,3);
            // preint_anchor.ori =  Eigen::Quaterniond(Anchor_T.block<3,3>(0,0));

            // preint_anchor.pos -= keyframe_to_odometry_transformation.translation;
            // preint_anchor.ori =  preint_anchor.ori.inverse() * keyframe_to_odometry_transformation.rotation ;

            // keyframe_index.push_back(latest_frame_idx); // the index to actual frames
            // keyframe_index.push_back(latest_frame_idx); // the keyframe index to all frames

            // publishKeyframeCloud();

            savePointCloud();
            saveKeyframePose();
            publishKeyframeOdometry();

            // odometry_transformation_guess = Eigen::Matrix4d::Identity(); //last_odometry_transformation;
            // odometry_transformation_guess = last_odometry_transformation;
            // registration_transformation = Eigen::Matrix4d::Identity(); // this has to be "reset" as it it is used to determine the next guess
            
            
            registration_transformation.rotation.setIdentity(); // this has to be "reset" as it it is used to determine the next guess
            registration_transformation.translation.setZero(); // this has to be "reset" as it it is used to determine the next guess
            registration_transformation.matrix.setIdentity(); // this has to be "reset" as it it is used to determine the next guess


            // RCLCPP_INFO(get_logger(), "New keyframe added! index: %i", latest_keyframe_idx);

            latest_keyframe_idx++; // the count of keyframes
        }   

        

        void downSampleClouds() 
        {
            // RCLCPP_INFO(get_logger(), "Downsampling clouds..");
            setCloudScale();
            if (ds_voxel_size_ > 0.0 || use_cloud_scale_for_ds_) {
                cloud_in_ds->clear();
                down_size_filter.setInputCloud(cloud_in);
                down_size_filter.filter(*cloud_in_ds);
            } else {
                *cloud_in_ds = *cloud_in;
            }
            // calculatePointNormals(cloud_in_ds, *cloud_in_ds);
            // farthestPointSampling(cloud_in, cloud_in_ds, 1000);

            // lineSampling(cloud_in, cloud_in_ds, points_per_cloud_scale_);
            // lineDensitySampling(cloud_in, cloud_in_ds, points_per_cloud_scale_);


            // cloud_keyframe_ds->clear();
            // if (ds_voxel_size_ > 0.0) {
            //     down_size_filter.setInputCloud(cloud_keyframe);
            //     down_size_filter.filter(*cloud_keyframe_ds);
            // } else {
            //     cloud_keyframe_ds = cloud_keyframe;
            // }

            // if (ds_voxel_size_lc_ > 0.0){
            //     // RCLCPP_INFO(get_logger(), "Downsample much..?");
            //     local_map_ds->clear();
            //     down_size_filter_local_map.setInputCloud(local_map);
            //     down_size_filter_local_map.filter(*local_map_ds);
            // } else {
            //     *local_map_ds = *local_map;
            // }
        }

        void setCloudScale()
        {
            calcCloudScale();
            if (use_cloud_scale_for_ds_) {
                double prev_cloud_weight = cloud_scale_previuos_cloud_weight_;
                double new_cloud_scale = prev_cloud_scale * prev_cloud_weight + cloud_scale * (1 - prev_cloud_weight); // calculate new scale as weighted average between old and new
                RCLCPP_INFO(get_logger(), "cloud scale is: %f, prev: %f, new scale is: %f", cloud_scale, prev_cloud_scale, new_cloud_scale);
                float temp_leafsize = new_cloud_scale / float(points_per_cloud_scale_); // 25 points from side to side by default
                float temp_leafsize_lc = temp_leafsize / float(ds_lc_voxel_size_ratio_); // best have as uneven number to have center point ?
                RCLCPP_INFO(get_logger(), "leaf size: %f", temp_leafsize);

                down_size_filter.setLeafSize(temp_leafsize, temp_leafsize, temp_leafsize);
                down_size_filter_local_map.setLeafSize(temp_leafsize_lc, temp_leafsize_lc, temp_leafsize_lc);
                // down_size_filter_global_map.setLeafSize(temp_leafsize, temp_leafsize, temp_leafsize);
                prev_cloud_scale = new_cloud_scale;
            }
        }

        void calcCloudScale()
        {
            // cloud_scale = getMaxSegment(*cloud_in); // was! very slow! for a 10 000 point cloud it does 100 mil iterations ie. O(n²)
            cloud_scale = getMaxLeftRight(*cloud_in); // this only uses O(n) but is not guaranteed to find the largest distance, only a usable distance
        }

        template <typename PointT> double inline
        getMaxSegment (const pcl::PointCloud<PointT> &cloud)
        {
            double max_dist = std::numeric_limits<double>::min ();
            double max_left = std::numeric_limits<double>::min ();
            int i_min = -1, i_max = -1;
            int i_left = -1;

            // new method N*2 iterations
            // find leftmost point
            for (size_t i = 0; i < cloud.points.size (); ++i)
            {        
                double y_val = cloud.points[i].y;
                // Compute the distance 
                // double dist = (cloud.points[i].getVector4fMap () - 
                //             cloud.points[j].getVector4fMap ()).squaredNorm ();
                if (y_val <= max_left) {
                    max_left = y_val;
                    i_left = i;
                }
            }
            i_min = i_left;

            // need find the distance to farhest point from this.
            for (size_t i = 0; i < cloud.points.size (); ++i)
            {   
                // Compute the distance 
                double dist = (cloud.points[i_left].getVector4fMap () - 
                            cloud.points[i].getVector4fMap ()).squaredNorm ();
                if (dist <= max_dist)
                    continue;

                max_dist = dist;
                // i_min = i_left;
                i_max = i;
                // max_right = y_val;
                // i_right = i;
            }
            
            // Old method -> N² iterations, very slow
            // for (size_t i = 0; i < cloud.points.size (); ++i)
            // {
            //     for (size_t j = i; j < cloud.points.size (); ++j)
            //     {
            //         // Compute the distance 
            //         double dist = (cloud.points[i].getVector4fMap () - 
            //                     cloud.points[j].getVector4fMap ()).squaredNorm ();
            //         if (dist <= max_dist)
            //         continue;

            //         max_dist = dist;
            //         i_min = i;
            //         i_max = j;
            //     }
            // }

            if (i_min == -1 || i_max == -1)
            return (max_dist = std::numeric_limits<double>::min ());

            return (std::sqrt (max_dist));
        }

        template <typename PointT> double inline
        getMaxLeftRight(const pcl::PointCloud<PointT> &cloud)
        {
            double max_left = std::numeric_limits<double>::max ();
            double max_right = std::numeric_limits<double>::min ();
            int i_left = -1, i_right = -1;

            for (size_t i = 0; i < cloud.points.size (); ++i)
            {        
                double y_val = cloud.points[i].y;
                // Compute the distance 
                // double dist = (cloud.points[i].getVector4fMap () - 
                //             cloud.points[j].getVector4fMap ()).squaredNorm ();
                if (y_val <= max_left) {
                    max_left = y_val;
                    i_left = i;

                } else if (y_val >= max_right) {
                    max_right = y_val;
                    i_right = i;
                }
            }

            if (i_left == -1 || i_right == -1)
            return (max_left = std::numeric_limits<double>::min ());

            // double dist = (cloud.points[i_left].getVector4fMap () - 
            //                 cloud.points[i_right].getVector4fMap ()).squaredNorm ();


            // double dist = cloud.points[i_right].y - cloud.points[i_left].y;
            double dist = max_right - max_left;
            // RCLCPP_INFO(get_logger(), "max_left is: %f - max_right is: %f, difference is %f", max_left, max_right, dist);

            // pmin = cloud.points[i_left];
            // pmax = cloud.points[i_right];
            return dist;
        }

        void resetPreintAnchor()
        {
            preint_anchor.pos  = Eigen::Vector3d::Zero(); // initial position in the future
            preint_anchor.vel  = Eigen::Vector3d::Zero(); // saved in local frame
            preint_anchor.ori  = Eigen::Quaterniond(initial_pose.orientation);
            preint_anchor.ang  = Eigen::Vector3d::Zero();
            // preint_anchor.bias = Eigen::Vector3d::Zero();
            preint_anchor.acc  = Eigen::Vector3d::Zero();
            preint_anchor.jerk  = Eigen::Vector3d::Zero();
            preint_anchor.alpha  = Eigen::Vector3d::Zero();

        }

        void setPreintAnchor(const INSstate state)
        {
            
            preint_anchor.time  = state.time ;
            preint_anchor.pos  = state.pos ;
            // preint_anchor.vel  = state.vel ;
            preint_anchor.vel  = state.ori.matrix().inverse() * state.vel ; // saved in local frame
            preint_anchor.ori  = state.ori ;
            preint_anchor.ang  = state.ang ;
            preint_anchor.bias = state.bias;
            preint_anchor.acc  = state.acc ;
            preint_anchor.jerk  = state.jerk ;
            preint_anchor.alpha  = state.alpha ;
            preint_anchor.dt = state.dt;
        }

        void getPreintAnchor(INSstate &state) // passed by reference to make it mutable
        {
            state.time = preint_anchor.time;
            state.pos  = preint_anchor.pos ;
            state.vel  = state.ori.matrix() * preint_anchor.vel; // is rotated into new frame
            state.ori  = preint_anchor.ori ;
            state.ang  = preint_anchor.ang ;
            state.bias = preint_anchor.bias;
            state.acc  = preint_anchor.acc ;
            state.jerk  = preint_anchor.jerk ;
            state.alpha  = preint_anchor.alpha ;
            state.dt = preint_anchor.dt;
        }

        void publishINSstates( )
        {
            size_t i = 0;
            // while (preint_states[i+1].time < current_scan_time) // comparing to i+1 to get the state just before frame_start
            // {
            //     publishINS(preint_states[i]);
            // }
            double ins_time  = ins.getPreintState(i+1).time;
            // RCLCPP_INFO(get_logger(),"ins time: %f scan time: %f, difference %f  ", ins_time, current_scan_time, current_scan_time - ins_time);
            while (ins_time < current_scan_time) // comparing to i+1 to get the state just before frame_start
            {
                // RCLCPP_INFO(get_logger(),"publishing ins i: %i", i);
                publishINS(ins.getPreintState(i));
                i++;
                ins_time  = ins.getPreintState(i+1).time;
            }
        }

        void publishPredictedINSstates( )
        {
            path_predicted_ins.poses.clear();
            for (int i =0; i< 10 ; i++ ){
                RCLCPP_DEBUG(get_logger(),"publishing predicted ins i: %i", i);
                publishPredictedINS(ins.getPredictState(i));
            }
        }

        void calcState0()
        {
            
            // double calc_time = 0.0;
            // double undistort_time = 0.0;

            double frame_start_time = current_scan_time;

            // get the latest integrated state just before the start of the new cloud scan
            size_t i = 0;
            RCLCPP_INFO(get_logger(),"frame start time %f", frame_start_time);
            while (preint_states[i+1].time < frame_start_time) // comparing to i+1 to get the state just before frame_start
            {
                publishINS(preint_states[i]);
                preint_states.pop_front();
                // RCLCPP_INFO(get_logger(), "sync state%i ori %f %f %f %f",i,preint_states[i].ori.w(), preint_states[i].ori.x(), preint_states[i].ori.y(), preint_states[i].ori.z() );
                // i++;
                // RCLCPP_INFO(get_logger(), "undistort sync i %i",i );
            }
            // int i_base = i;
            // RCLCPP_INFO(get_logger(), "final sync state%i ori %f %f %f %f",i,preint_states[i].ori.w(), preint_states[i].ori.x(), preint_states[i].ori.y(), preint_states[i].ori.z() );
            // RCLCPP_INFO(get_logger(), "undistort sync final i %i",i );

            // preint_state.vel = preint_states[i].vel; // the initial velocity of the next preintegration sweep
            // set state anchor for next preintegration - this is synced with imu steps
            setPreintAnchor(preint_states[i]);

            // calculates state0, the syncronised interpolated state at time of the start of the cloud
            double sync_time = frame_start_time - preint_states[i].time;
            // double imu_dt = preint_states[i+1].time - preint_states[i].time;
            double imu_dt = preint_states[i].dt;

            RCLCPP_INFO(get_logger(), "state0  dt %f", imu_dt );
            RCLCPP_INFO(get_logger(), "state0 sync time %f", sync_time );
            // Eigen::Vector3d jerk = 1.0 / sync_time * (preint_states[i+1].ori.matrix() * preint_states[i+1].acc  - preint_states[i].ori.matrix() * preint_states[i].acc);
            // Eigen::Vector3d alpha = 1.0 / sync_time * (preint_states[i+1].ang - preint_states[i].ang);
            Eigen::Vector3d jerk = (preint_states[i+1].jerk + preint_states[i].jerk) * sync_time/imu_dt ; // interpolated jerk and alpha, interpolated from the sync_time to imu_dt, above is wrong
            Eigen::Vector3d alpha = (preint_states[i+1].alpha + preint_states[i].alpha) * sync_time/imu_dt ;
            // Eigen::Vector3d jerk = Eigen::Vector3d::Zero(); //preint_states[i].jerk; THIS CAUSE ALOT OF ERROR IN UNDISTORT IF NOT ZERO?!
            // Eigen::Vector3d alpha = Eigen::Vector3d::Zero();//preint_states[i].alpha;
            state0.pos = preint_states[i].pos + preint_states[i].vel * sync_time + 0.5 * (preint_states[i].ori *preint_states[i].acc) *sync_time*sync_time + 1.0/6.0 * jerk * sync_time*sync_time*sync_time;
            state0.vel = preint_states[i].vel + (preint_states[i].ori.matrix() *preint_states[i].acc) *sync_time;
            state0.acc = preint_states[i].acc + jerk * sync_time;
            state0.ang = preint_states[i].ang + alpha * sync_time;
            state0.ori =  deltaQ(0.5 * alpha * sync_time*sync_time) * (deltaQ(preint_states[i].ang * sync_time) * preint_states[i].ori).normalized();
            state0.jerk = jerk;
            state0.alpha = alpha;
            // state0.time = preint_states[i].time + sync_time; // current_scan_time
            state0.time =  current_scan_time;
            state0.dt =  imu_dt;

            // printINSstate(state0);
            
            

            // RCLCPP_INFO(get_logger(), "Set Anchor: pos %f %f %f vel %f %f %f", preint_anchor.pos.x(), preint_anchor.pos.y(), preint_anchor.pos.z(), preint_anchor.vel.x(),preint_anchor.vel.y(), preint_anchor.vel.z());
            // RCLCPP_INFO(get_logger(), "Set Anchor: ori %f %f %f %f", preint_anchor.ori.w(), preint_anchor.ori.x(), preint_anchor.ori.y(), preint_anchor.ori.z());

            // preint_anchor = state0;

            // this state0 is also the prior for the scan registration !!
            // it is saved so it can be removed from the cloud again to match the cloud with the odometry position
            preint_transformation_guess.translation = state0.pos;
            preint_transformation_guess.rotation = state0.ori;


            RCLCPP_INFO(get_logger(), "state0 pose: pos %f %f %f ", preint_transformation_guess.translation.x(), preint_transformation_guess.translation.y(), preint_transformation_guess.translation.z());
            RCLCPP_INFO(get_logger(), "state0 pose: ori %f %f %f %f", preint_transformation_guess.rotation.w(), preint_transformation_guess.rotation.x(), preint_transformation_guess.rotation.y(), preint_transformation_guess.rotation.z());


        }

        void undistortCloud(const pcl::PointCloud<PointType> &cloud_in, pcl::PointCloud<PointType> &cloud_out)
        {   

            calcState0(); // syncs states to lidar frame
            rclcpp::Clock system_clock;
            rclcpp::Time time_undistort_start = system_clock.now();

            double calc_time = 0.0;
            double undistort_time = 0.0;

            // create state sync look-up index
            int *state_sync_index = new int[cloud_in.points.size()];
            // double point_frame_dts[cloud_in.points.size()];
            double *point_frame_dts = new double[cloud_in.points.size()];
            #pragma omp parallel for shared(state_sync_index, point_frame_dts)
            for (size_t k = 0; k < cloud_in.points.size(); k++){
                int i = 0;
                PointType point = cloud_in.points[k];
                double intensity = point.intensity;
                double point_frame_dt = intensity - (int)intensity;
                while ((point_frame_dt + current_scan_time) >= preint_states[i+1].time){
                    i++;
                    // RCLCPP_INFO(get_logger(),"state sync index %i to point index k %i", i, k); 
                }
                state_sync_index[k] = i;
                point_frame_dts[k] = point_frame_dt ;
            }


            // INSstate stateX;
            INSstate stateX;
            // double point_state_dt;
            #pragma omp parallel for reduction(+:calc_time) private(stateX) shared(state_sync_index, point_frame_dts) num_threads(8)
            for (size_t k = 0; k < cloud_in.points.size(); k++){
                rclcpp::Time time_calc_start = system_clock.now();
                PointType point = cloud_in.points[k];
                int i = state_sync_index[k];
                double point_frame_dt = point_frame_dts[k];

                // validate that sync_time and state is correct
                // get next imu preintegration state when state gets too old
                // if ((point_frame_dt + current_scan_time) >= preint_states[i+1].time && preint_states.size() >= i+2){
                    // while ((point_frame_dt + current_scan_time) >= preint_states[i+1].time && preint_states.size() >= i+2){
                    //     i++;
                    //     // RCLCPP_INFO(get_logger(),"next undistort state! %i", i);
                    // }
                    // publishINS(preint_states[i]);

                    // set new stateX..
                    // double state_dt = preint_states[i+1].time - preint_states[i].time;
                    // Eigen::Vector3d jerk = 1.0 / state_dt * (preint_states[i+1].ori * preint_states[i+1].acc - preint_states[i].ori * preint_states[i].acc);
                    // Eigen::Vector3d alpha = 1.0 / state_dt * (preint_states[i+1].ang - preint_states[i].ang);
                    // stateX.pos      = preint_states[i].pos; //+ preint_states[i].vel * sync_time + 0.5 * (preint_states[i].ori *preint_states[i].acc) *sync_time*sync_time + 1.0/6.0 * jerk * sync_time*sync_time*sync_time;
                    // stateX.vel      = preint_states[i].vel; //+ (preint_states[i].ori *preint_states[i].acc) *sync_time;
                    // stateX.acc      = preint_states[i].acc; //+ jerk *sync_time;
                    // stateX.ori      = preint_states[i].ori; //* deltaQ(preint_states[i].ang * sync_time) * deltaQ(0.5 * alpha * sync_time*sync_time);
                    // stateX.jerk     = preint_states[i].jerk; //
                    // stateX.alpha    = preint_states[i].alpha; //
                    // double sync_time = stateX.time - current_scan_time;
                    double sync_time = 0.0; // sync_time should be negative, time between scan start and preceding imu measurement
                    if (i == 0) {
                        // RCLCPP_INFO(get_logger(), "i == 0, k = %i", k);
                        stateX = state0;
                    } else {
                        stateX = preint_states[i];
                        sync_time = current_scan_time - stateX.time ;
                    }


                    // if (state_dt > 1.0 || state_dt < 0.0){
                    //     RCLCPP_INFO(get_logger(), "bad state_dt! %f = %f - %f ", state_dt, preint_states[i+1].time,  preint_states[i].time);
                    //     state_dt = 0.01;
                    //     RCLCPP_INFO(get_logger(), "bad state undistort state%i dt %f pos %f %f %f, vel %f %f %f acc %f %f %f jerk %f %f %f",i-1,state_dt, stateX.pos.x(), stateX.pos.y(), stateX.pos.z(), stateX.vel.x(), stateX.vel.y(), stateX.vel.z(), stateX.acc.x(), stateX.acc.y(), stateX.acc.z() , jerk.x(), jerk.y(), jerk.z() );
                    // }
                    
                    
                    // RCLCPP_INFO(get_logger(), "undistort state%i pos %f %f %f, vel %f %f %f acc %f %f %f jerk %f %f %f",i, stateX.pos.x(), stateX.pos.y(), stateX.pos.z(), stateX.vel.x(), stateX.vel.y(), stateX.vel.z(), stateX.acc.x(), stateX.acc.y(), stateX.acc.z() , jerk.x(), jerk.y(), jerk.z() );
                    // RCLCPP_INFO(get_logger(), "undistort state%i ori %f %f %f %f",i,stateX.ori.w(), stateX.ori.x(), stateX.ori.y(), stateX.ori.z() );

                    // RCLCPP_INFO(get_logger(), "undistort state_dt %f", );
                // }

                double point_state_dt = point_frame_dt + sync_time; 
                if (point_state_dt < 0.0){  // error detection..
                    RCLCPP_WARN(get_logger(), "Negative point dt detected state dt %f, frame dt %f, sync time %f, index %i", point_state_dt, point_frame_dt, sync_time, k);
                    point_state_dt = +0.0;
                }

                // calculate point specific transform..
                Eigen::Vector3d point_translation_distortion = stateX.pos + stateX.vel * point_state_dt +  (stateX.ori.matrix() * (stateX.acc * 0.5*point_state_dt*point_state_dt)) + 1.0/6.0 * stateX.jerk *point_state_dt*point_state_dt*point_state_dt;
                
                // Eigen::Quaterniond dq_vel = deltaQ(stateX.ang); 
                // dq_vel = multQuatByScalar(dq_vel, point_state_dt);
                // // Eigen::Quaterniond dq_rate = deltaQ(stateX.alpha); 
                // // dq_rate = multQuatByScalar(dq_rate, 0.5*point_state_dt*point_state_dt);
                // Eigen::Quaterniond dq_rate = Eigen::Quaterniond::Identity();
                // Eigen::Quaterniond dq = addQuaternions(stateX.ori*dq_vel, stateX.ori*dq_rate);
                // Eigen::Quaterniond point_rotation_distortion = addQuaternions(dq, stateX.ori); 
                // point_rotation_distortion.normalize();// save new orientation normalized!

                // Eigen::Quaterniond point_rotation_distortion_alpha = deltaQ(stateX.alpha);
                // point_rotation_distortion_alpha = multQuatByScalar(point_rotation_distortion_alpha, 0.5 * point_state_dt*point_state_dt);
                // Eigen::Quaterniond point_rotation_distortion_rate = deltaQ(stateX.ang);
                // point_rotation_distortion_rate = multQuatByScalar(point_rotation_distortion_rate, point_state_dt);

                Eigen::Quaterniond point_rotation_distortion_alpha = deltaQ(stateX.alpha *0.5 * point_state_dt*point_state_dt);
                Eigen::Quaterniond point_rotation_distortion_rate = deltaQ(stateX.ang*point_state_dt);
                
                // if (!point_rotation_distortion_rate.matrix().allFinite()){
                //     RCLCPP_WARN(get_logger(), "Nan Quaternion detected!");
                //     point_rotation_distortion_rate = Eigen::Quaterniond::Identity();
                // }

                // Eigen::Quaterniond point_rotation_distortion = (deltaQ(0.5 * stateX.alpha * point_state_dt*point_state_dt) * deltaQ(stateX.ang * point_state_dt) * stateX.ori).normalized();
                // Eigen::Quaterniond point_rotation_distortion = addQuaternions(point_rotation_distortion_alpha, point_rotation_distortion_rate) * stateX.ori;
                Eigen::Quaterniond point_rotation_distortion = (point_rotation_distortion_alpha * point_rotation_distortion_rate) * stateX.ori;
                point_rotation_distortion.normalize();


                Eigen::Matrix4d T_star = Eigen::Matrix4d::Identity();
                // Eigen::Matrix4d T_star_normal = Eigen::Matrix4d::Identity();
                T_star.block<3,3>(0,0) = point_rotation_distortion.matrix();
                // T_star_normal.block<3,3>(0,0) = Eigen::Matrix3d(point_rotation_distortion);
                T_star.block<3,1>(0,3) = point_translation_distortion;


                // if (!pcl::isFinite(*point))
                if (!pcl::isFinite(point))
                {
                    RCLCPP_INFO_ONCE(get_logger(), "point NAN! before undistortion");
                }
                // apply transformation to point
                // Eigen::Vector3d point_vector(point.x, point.y, point.z);
                // point_vector = point_rotation_distortion * point_vector + point_translation_distortion;

                Eigen::Vector4d point_vector(point.x, point.y, point.z, 1.0);
                // Eigen::Vector3d point_normal_vector(point.normal_x, point.normal_y, point.normal_z, 1.0);
                point_vector = T_star*point_vector;
                // point_normal_vector = T_star.block<3,3>(0,0)*point_normal_vector;

                point.x = point_vector.x();
                point.y = point_vector.y();
                point.z = point_vector.z();

                // point.normal_x = point_normal_vector.x();
                // point.normal_y = point_normal_vector.y();
                // point.normal_z = point_normal_vector.z();
                cloud_out.points[k] = point;

                // if (!pcl::isFinite(*point))
                if (!pcl::isFinite(point))
                {
                    RCLCPP_WARN_ONCE(get_logger(), "point NAN After undistortion!");
                    RCLCPP_WARN_ONCE(get_logger(), "omp thread %i", omp_get_thread_num());
                    RCLCPP_WARN_ONCE(get_logger(), "point number %i", k);
                    RCLCPP_WARN_ONCE(get_logger(), "state to point dt: %f", point_state_dt);
                    RCLCPP_WARN_ONCE(get_logger(), "stateX time: %f", stateX.time);
                    RCLCPP_WARN_ONCE(get_logger(), "stateX pos: %f %f %f", stateX.pos.x(), stateX.pos.y(), stateX.pos.z());
                    RCLCPP_WARN_ONCE(get_logger(), "stateX vel: %f %f %f", stateX.vel.x(), stateX.vel.y(), stateX.vel.z());
                    RCLCPP_WARN_ONCE(get_logger(), "stateX acc: %f %f %f", stateX.acc.x(), stateX.acc.y(), stateX.acc.z());
                    RCLCPP_WARN_ONCE(get_logger(), "stateX ang vel: %f %f %f", stateX.ang.x(), stateX.ang.y(), stateX.ang.z());
                    RCLCPP_WARN_ONCE(get_logger(), "stateX jerk: %f %f %f", stateX.jerk.x(), stateX.jerk.y(), stateX.jerk.z());
                    RCLCPP_WARN_ONCE(get_logger(), "stateX alpha: %f %f %f", stateX.alpha.x(), stateX.alpha.y(), stateX.alpha.z());
                    RCLCPP_WARN_ONCE(get_logger(), "point translation: %f %f %f", point_translation_distortion.x(), point_translation_distortion.y(), point_translation_distortion.z());
                    // RCLCPP_WARN_ONCE(get_logger(), "point rotation vel: %f %f %f %f", dq_vel.w(), dq_vel.x(), dq_vel.y(), dq_vel.z());
                    // RCLCPP_WARN_ONCE(get_logger(), "point rotation rate: %f %f %f %f", dq_rate.w(), dq_rate.x(), dq_rate.y(), dq_rate.z());
                    RCLCPP_WARN_ONCE(get_logger(), "point rotation: %f %f %f %f", point_rotation_distortion.w(), point_rotation_distortion.x(), point_rotation_distortion.y(), point_rotation_distortion.z());
                    // printINSstate(stateX);
                }

                rclcpp::Time time_calc_end = system_clock.now();
                calc_time += (time_calc_end.seconds() - time_calc_start.seconds()) * 1000.0;

                // k++;
            }
            rclcpp::Time time_undistort_end = system_clock.now();
            undistort_time = (time_undistort_end.seconds() - time_undistort_start.seconds()) * 1000.0;

            // preint_states.pop_front(); // removes the interpolated state0 from imu states.
            // preint_states.push_front(temp_state); // readd the nonsynced state

            delete[] state_sync_index;
            delete[] point_frame_dts;

            RCLCPP_INFO(get_logger(), "---- UNDISTORTION ---- ");
            RCLCPP_INFO(get_logger(), "Undistortion of %i points, MP cpu time: %fms, avg sub-calc time: %fms", cloud_in.points.size(), undistort_time,  calc_time/(float)cloud_in.points.size() );
            RCLCPP_INFO(get_logger(), "Total non-mp process time: %fms", calc_time );
        
        }

        void undistortCloud_OLD()
        {   
            double frame_start_time = current_scan_time;

            // get the latest integrated state just before the start of the new cloud scan
            size_t i = 0;
            while (preint_states[i+1].time < frame_start_time)
            {
                publishINS(preint_states[i]);
                i++;
                // RCLCPP_INFO(get_logger(), "undistort sync i %i",i );
            }
            // RCLCPP_INFO(get_logger(), "undistort sync i %i",i );

            
            setPreintAnchor(preint_states[i]);


            // calculate state at start of the cloud
            double sync_time = frame_start_time - preint_states[i].time;
            // RCLCPP_INFO(get_logger(), "undistort sync time %f", sync_time );
            INSstate state0;
            Eigen::Vector3d jerk = 1.0 / sync_time * (preint_states[i+1].ori.matrix() * preint_states[i+1].acc  - preint_states[i].ori.matrix() * preint_states[i].acc);
            Eigen::Vector3d alpha = 1.0 / sync_time * (preint_states[i+1].ang - preint_states[i].ang);
            state0.pos = preint_states[i].pos + preint_states[i].vel * sync_time + 0.5 * (preint_states[i].ori *preint_states[i].acc) *sync_time*sync_time + 1.0/6.0 * jerk * sync_time*sync_time*sync_time;
            state0.vel = preint_states[i].vel + (preint_states[i].ori.matrix() *preint_states[i].acc) *sync_time;
            state0.acc = preint_states[i].acc + jerk *sync_time;
            state0.ori =  deltaQ(0.5 * alpha * sync_time*sync_time) * ( deltaQ(preint_states[i].ang * sync_time) * preint_states[i].ori).normalized();
            

            // RCLCPP_INFO(get_logger(), "Set Anchor: pos %f %f %f vel %f %f %f", preint_anchor.pos.x(), preint_anchor.pos.y(), preint_anchor.pos.z(), preint_anchor.vel.x(),preint_anchor.vel.y(), preint_anchor.vel.z());
            // RCLCPP_INFO(get_logger(), "Set Anchor: ori %f %f %f %f", preint_anchor.ori.w(), preint_anchor.ori.x(), preint_anchor.ori.y(), preint_anchor.ori.z());

            // this state0 is also the prior for the scan registration !!
            // it is saved so it can be removed from the cloud again to match the cloud with the odometry position
            preint_transformation_guess.translation = state0.pos;
            preint_transformation_guess.rotation = state0.ori;
            // preint_state.vel = state0.vel;

            // RCLCPP_INFO(get_logger(), "undistort state0 pos %f %f %f, vel %f %f %f", state0.pos.x(), state0.pos.y(), state0.pos.z(), state0.vel.x(), state0.vel.y(), state0.vel.z()  );
            
            // get the total scan dt
            // double scan_dt = cloud_in->points[cloud_in->points.size()-1].intensity - (int)cloud_in->points[cloud_in->points.size()-1].intensity;
            sync_time = 0.0;
            for (auto &point : cloud_in->points) {
                // get point frame dt
                double intensity = point.intensity;
                double dt_i = intensity - (int)intensity;
                double point_dt = dt_i + sync_time; // sync_time should be negative, time between scan an preceding imu measurement
                if (point_dt < 0.0)
                    point_dt = 0.0;

                // calculate point specific transform..
                Eigen::Vector3d point_translation_distortion = state0.pos + state0.vel * point_dt + 0.5 * (state0.ori.matrix() * state0.acc) *point_dt*point_dt + 1.0/6.0 * jerk *point_dt*point_dt*point_dt;
                Eigen::Quaterniond point_rotation_distortion = (deltaQ(0.5 * alpha * point_dt*point_dt) * deltaQ(state0.ang * point_dt) * state0.ori ).normalized();

                Eigen::Matrix4d T_star = Eigen::Matrix4d::Identity();
                T_star.block<3,3>(0,0) = Eigen::Matrix3d(point_rotation_distortion);
                T_star.block<3,1>(0,3) = point_translation_distortion;


                if (!pcl::isFinite(point))
                {
                    RCLCPP_INFO_ONCE(get_logger(), "point NAN! before undistortion");
                }
                // apply to point
                // Eigen::Vector3d point_vector(point.x, point.y, point.z);
                // point_vector = point_rotation_distortion * point_vector + point_translation_distortion;

                Eigen::Vector4d point_vector(point.x, point.y, point.z, 1.0);
                point_vector = T_star *point_vector;

                point.x = point_vector.x();
                point.y = point_vector.y();
                point.z = point_vector.z();
                if (!pcl::isFinite(point))
                {
                    RCLCPP_INFO_ONCE(get_logger(), "point NAN!");
                    RCLCPP_INFO_ONCE(get_logger(), "point dt: %f", point_dt);
                    RCLCPP_INFO_ONCE(get_logger(), "point translation: %f %f %f", point_translation_distortion.x(), point_translation_distortion.y(), point_translation_distortion.z());
                    RCLCPP_INFO_ONCE(get_logger(), "point rotation: %f %f %f %f", point_rotation_distortion.w(), point_rotation_distortion.x(), point_rotation_distortion.y(), point_rotation_distortion.z());
                    RCLCPP_INFO_ONCE(get_logger(), "point jerk: %f %f %f", jerk.x(), jerk.y(), jerk.z());
                    RCLCPP_INFO_ONCE(get_logger(), "point alpha: %f %f %f", alpha.x(), alpha.y(), alpha.z());
                }

                // get next imu preintegration state when state gets to old
                if ((dt_i + frame_start_time) >= preint_states[i].time && preint_states.size() >= i+2){
                    i++;
                    sync_time = frame_start_time - preint_states[i].time;
                    // set new state0..
                    double state_dt = preint_states[i+1].time - preint_states[i].time;
                    if (state_dt > 1.0 || state_dt < 0.0){
                        RCLCPP_INFO(get_logger(), "bad state_dt! %f = %f - %f ", state_dt, preint_states[i+1].time,  preint_states[i].time);
                        state_dt = 0.01;
                        RCLCPP_INFO(get_logger(), "bad state undistort state%i dt %f pos %f %f %f, vel %f %f %f acc %f %f %f jerk %f %f %f",i-1,state_dt, state0.pos.x(), state0.pos.y(), state0.pos.z(), state0.vel.x(), state0.vel.y(), state0.vel.z(), state0.acc.x(), state0.acc.y(), state0.acc.z() , jerk.x(), jerk.y(), jerk.z() );
                    }
                    jerk = 1.0 / state_dt * (preint_states[i+1].ori * preint_states[i+1].acc  - preint_states[i].ori * preint_states[i].acc);
                    alpha = 1.0 / state_dt * (preint_states[i+1].ang - preint_states[i].ang);
                    state0.pos = preint_states[i].pos; //+ preint_states[i].vel * sync_time + 0.5 * (preint_states[i].ori *preint_states[i].acc) *sync_time*sync_time + 1.0/6.0 * jerk * sync_time*sync_time*sync_time;
                    state0.vel = preint_states[i].vel; //+ (preint_states[i].ori *preint_states[i].acc) *sync_time;
                    state0.acc = preint_states[i].acc; //+ jerk *sync_time;
                    state0.ori = preint_states[i].ori; //* deltaQ(preint_states[i].ang * sync_time) * deltaQ(0.5 * alpha * sync_time*sync_time);
                    // RCLCPP_INFO(get_logger(), "undistort state%i pos %f %f %f, vel %f %f %f acc %f %f %f jerk %f %f %f",i, state0.pos.x(), state0.pos.y(), state0.pos.z(), state0.vel.x(), state0.vel.y(), state0.vel.z(), state0.acc.x(), state0.acc.y(), state0.acc.z() , jerk.x(), jerk.y(), jerk.z() );

                }
            }
        }

        template <typename PointT>
        void undistortCloudInterpolated(const boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_in, boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_out,const Eigen::Quaterniond q, const Eigen::Vector3d t)
        {
            double scan_dt = cloud_out->points[cloud_in->points.size()-1].intensity - (int)cloud_out->points[cloud_in->points.size()-1].intensity;
            // RCLCPP_INFO(get_logger(), "scan dt undistortion: %f", scan_dt);
            // double point_dt = scan_time / cloud_in->points.size(); // should be the first thing of the preprocessing for this reason

            Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();


            // RCLCPP_INFO(get_logger(), "keyframe pose ? %f, %f ", keyframe_pose.matrix(0,3), keyframe_pose.position[0]);
            for (size_t i=0; i < cloud_in->points.size(); i++){
                //  double dt_i = point_dt * i;
                double intensity = cloud_out->points[i].intensity;
                double dt_i = intensity - (int)intensity;
                double ratio_i = dt_i / scan_dt;
                if(ratio_i >= 1.0) {
                    ratio_i = 1.0;
                }

                Eigen::Quaterniond quat_spheric_interp = q0.slerp(1 - ratio_i, q.inverse()); //
                // Eigen::Quaterniond quat_spheric_interp = q0.slerp(ratio_i, q); // 
                Eigen::Vector3d t_interp = t* (1-ratio_i); // 
                Eigen::Matrix4d undistort_T = Eigen::Matrix4d::Identity();
                undistort_T.block<3,3>(0,0) = quat_spheric_interp.matrix();
                undistort_T.block<3,1>(0,3) = t_interp;
                // undistort_T = keyframe_pose.matrix * undistort_T;
                // undistort_T = undistort_T * keyframe_pose.matrix ;
                // undistort_T = keyframe_pose.matrix ;

                // Eigen::Vector3d point_vector(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
                // point_vector = quat_spheric_interp * point_vector + t_interp;
                Eigen::Vector4d point_vector(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z, 1.0);
                point_vector = undistort_T * point_vector;

                cloud_in->points[i].x = point_vector.x();
                cloud_in->points[i].y = point_vector.y();
                cloud_in->points[i].z = point_vector.z();
                // cloud_out->points[i] = undistortPoint(cloud_in->points[i], quat_spheric_interp);

            }
        }

        // void updateINSRelative()
        // {
        //     ins_relative.rotation = ins_pose.orientation.inverse() * ins_pose_new.orientation; 
        //     ins_pose.orientation = ins_pose_new.orientation;

        //     ins_relative.translation = (ins_pose_new.position - ins_pose.position);
        // }

        

        void printINSstate(const INSstate state){
            RCLCPP_INFO(get_logger(), "-------- State Print -----------");
            RCLCPP_INFO(get_logger(), "state time: %f", state.time);
            RCLCPP_INFO(get_logger(), "state dt: %f", state.dt);
            RCLCPP_INFO(get_logger(), "state pos: %f %f %f", state.pos.x(), state.pos.y(), state.pos.z());
            RCLCPP_INFO(get_logger(), "state vel: %f %f %f", state.vel.x(), state.vel.y(), state.vel.z());
            RCLCPP_INFO(get_logger(), "state acc: %f %f %f", state.acc.x(), state.acc.y(), state.acc.z());
            RCLCPP_INFO(get_logger(), "state jerk: %f %f %f", state.jerk.x(), state.jerk.y(), state.jerk.z());
            RCLCPP_INFO(get_logger(), "state ang vel: %f %f %f", state.ang.x(), state.ang.y(), state.ang.z());
            RCLCPP_INFO(get_logger(), "state alpha: %f %f %f", state.alpha.x(), state.alpha.y(), state.alpha.z());
            RCLCPP_INFO(get_logger(), "state bias acc: %f %f %f", state.bias.acc.x(), state.bias.acc.y(), state.bias.acc.z());
            RCLCPP_INFO(get_logger(), "state bias ang: %f %f %f", state.bias.ang.x(), state.bias.ang.y(), state.bias.ang.z());
            RCLCPP_INFO(get_logger(), "--------------------------------");
        }


        // // OBS!: This function assumes that gravity has been removed from the incoming acceleration.
        // void integrateImuStep(Eigen::Vector3d acc, Eigen::Vector3d ang_vel, double dt)
        // {
        //     // save the previous state to be used 
        //     INSstate previous_state = preint_state;

        //     // Integrate the orientation from the angular rate and save the average ori between this and the previous state
        //     // preint_state.ori *= deltaQ((0.5*(ang_vel - preint_bias.ang + preint_state.ang)  ) * dt) ;
        //     preint_state.ang = ang_vel - observer_state.bias.ang;
        //     // Eigen::Quaterniond dq = deltaQ(0.5*(ang_vel - observer_state.bias.ang + preint_state.ang)); // why is this 0.5 factor in here? because it is an average
        //     Eigen::Quaterniond dq_vel = deltaQ(0.5*(preint_state.ang + previous_state.ang)); 
        //     // Eigen::Quaterniond dq_vel = deltaQ(preint_state.ang); 
        //     dq_vel = multQuatByScalar(dq_vel, dt);
        //     // Eigen::Quaterniond dq_rate = deltaQ(previous_state.alpha); 
        //     // dq_rate = multQuatByScalar(dq_rate, 0.5*dt*dt);
        //     // Eigen::Quaterniond dq = addQuaternions(previous_state.ori*dq_vel, previous_state.ori*dq_rate);
        //     Eigen::Quaterniond dq = preint_state.ori*dq_vel;
        //     preint_state.ori = addQuaternions(dq, preint_state.ori); // save new orientation normalized!
        //     preint_state.ori.normalize(); // orientation normalized!
        //     Eigen::Quaterniond ori_avg = previous_state.ori.slerp(0.5, preint_state.ori);

        //     // need to think about how bias is applied and saved..
        //     //acceleration and bias is in the local imu frame
        //     preint_state.acc = (acc - observer_state.bias.acc); // save the acc in local body frame - with subtracted bias?
        //     preint_state.vel = previous_state.vel + ori_avg.matrix() * preint_state.acc * dt;
        //     preint_state.pos = previous_state.pos + preint_state.vel * dt - ori_avg.matrix() * preint_state.acc * dt*dt *0.5;
        //     // preint_state.pos = previous_state.pos + (preint_state.vel + previous_state.vel)/2.0 * dt;
            
        //     // preint_state.ang = ang_vel - preint_bias.ang;
        // }

        // void integrateJerkAlpha(INSstate *this_state, const INSstate next_state, double dt)
        // {
        //     // Eigen::Quaterniond ori_avg = this_state.ori.slerp(0.5, next_state.ori);
        //     this_state->alpha = (next_state.ang - this_state->ang) / dt; // these should be rotated into world frame right?
        //     this_state->jerk =  (next_state.ori*next_state.acc - this_state->ori*this_state->acc) / dt; // calculate and save jerk in world frame
        // }
        
        // double getIMUdt(int buffer_index)
        // {
        //     double dt = toSec(imu_buffer[buffer_index+1]->header.stamp) - toSec(imu_buffer[buffer_index]->header.stamp);
        //     // if x standard deviation from a running avg -> use avg instead
        //     return dt;
        // }


        // double validateIMUdt(double new_dt)
        // {   
        //     RCLCPP_INFO(get_logger(),"IMU dt: %f", new_dt);

        //     if (imu_dts.size() < 30){
        //         imu_dts.push_front(new_dt);
        //         // RCLCPP_INFO(get_logger(), "scan time %f", new_dt);
        //         return new_dt;
        //     }

        //     size_t n = imu_dts.size();
        //     double sum = 0.0;
        //     double sum_sq = 0.0;
        //     for (size_t i=0; i < n; i++){
        //         sum += imu_dts[i];
        //         sum_sq += imu_dts[i]*imu_dts[i];
        //     }
        //     double mean_dt = sum / (double)n;
        //     double std_dt = sqrt((sum_sq)/(double)n - mean_dt*mean_dt );

        //     if (abs(new_dt - mean_dt) > 3.0*std_dt){
        //         RCLCPP_WARN(get_logger(), "Bad dt detected!! %f,  using mean instead %f.", new_dt, mean_dt );
        //         return mean_dt;
        //     }
        //     // RCLCPP_INFO(get_logger(), "scan time %f,  mean %f, std %f.", new_dt, mean_dt, std_dt);

        //     imu_dts.pop_back();
        //     imu_dts.push_front(new_dt);

        //     return new_dt;
        // }

        // void preintegrateIMU()
        // {
        //     // rclcpp::Clock system_clock;
            
        //     rclcpp::Time time_preint_start = system_clock.now();
        //     // clear buffer up till last scan time stamp
        //     // RCLCPP_INFO(get_logger(), "last time %f", last_odometry_pose.time);
        //     // RCLCPP_INFO(get_logger(), "imu buffer size %i", imu_buffer.size());
        //     // while(keyframe_pose.time >= toSec(imu_buffer.front()->header.stamp) )
        //     while(last_odometry_pose.time > toSec(imu_buffer.front()->header.stamp) ) // >=?
        //     {
        //         // RCLCPP_INFO(get_logger(), "imu msg removed");
        //         imu_buffer.pop_front();
        //     }
        //     // RCLCPP_INFO(get_logger(), "OH NO!");

        //     // double dt = 0.01;
            
        //     sensor_msgs::msg::Imu imu_msg;
        //     imu_msg = *imu_buffer[0];
        //     double next_scan_end_time = current_scan_time + scan_dt; // preintegrating two frames, first for prior embedding transform second for undistortion
        //     // get the preintegration achor state 
        //     getPreintAnchor(preint_state);
        //     // overwrite timestamp of the anchor
        //     preint_state.time = toSec(imu_msg.header.stamp);
        //     INSstate this_preint_state = preint_state;
        //     INSstate new_preint_state;

        //     // RCLCPP_INFO(get_logger(), "Get Anchor: pos %f %f %f vel %f %f %f", preint_anchor.pos.x(), preint_anchor.pos.y(), preint_anchor.pos.z(), preint_anchor.vel.x(),preint_anchor.vel.y(), preint_anchor.vel.z());
        //     // RCLCPP_INFO(get_logger(), "Get Anchor: ori %f %f %f %f", preint_anchor.ori.w(), preint_anchor.ori.x(), preint_anchor.ori.y(), preint_anchor.ori.z());

        //     // RCLCPP_INFO(get_logger(), "residual vel %f %f %f", preint_residual.translation.x()/ scan_dt, preint_residual.translation.y()/ scan_dt, preint_residual.translation.z()/ scan_dt);
            
        //     // RCLCPP_INFO(get_logger(), "preint state%i ori %f %f %f %f",0, new_preint_state.ori.w(), new_preint_state.ori.x(), new_preint_state.ori.y(), new_preint_state.ori.z()  );
        //     // RCLCPP_INFO(get_logger(), "last odometry  pos %f %f %f", last_odometry_pose.position.x(), last_odometry_pose.position.y(), last_odometry_pose.position.z()  );

        //     preint_states.clear(); // clear preint_states to make sure it is empty
        //     // preint_states.push_back(this_preint_state); // push the anchor state

        //     // run buffer til next scan time
        //     // for (size_t i=0 ; i < imu_buffer.size(); i++ )
        //     size_t i = 0;
        //     // RCLCPP_INFO(get_logger(), "STARTING INTEGRATION..");
        //     while(true)
        //     {
        //         // RCLCPP_INFO(get_logger(), "preint state%i pos %f %f %f, vel %f %f %f  ori %f %f %f %f time %f",i, new_preint_state.pos.x(), new_preint_state.pos.y(), new_preint_state.pos.z(), new_preint_state.vel.x(), new_preint_state.vel.y(), new_preint_state.vel.z(), new_preint_state.ori.w(),new_preint_state.ori.x(),new_preint_state.ori.y(),new_preint_state.ori.z() , new_preint_state.time);
        //         imu_msg = *imu_buffer[i];
        //         double msg_time = toSec(imu_msg.header.stamp);
        //         // double imu_dt = toSec(imu_buffer[i+1]->header.stamp) - msg_time;
        //         double imu_dt = getIMUdt(i);
        //         // imu_dt = validateIMUdt(imu_dt);
                
                
        //         i++;
        //         // if (preint_state.time < 10000.0 ){
        //         //     RCLCPP_WARN(get_logger(), "BAD ins state time %f",preint_state.time );
        //         // }
                
        //         Eigen::Vector3d acc_in(imu_msg.linear_acceleration.x,
        //                                imu_msg.linear_acceleration.y,
        //                                imu_msg.linear_acceleration.z);
        //         Eigen::Vector3d ang_vel_in(imu_msg.angular_velocity.x,
        //                                    imu_msg.angular_velocity.y,
        //                                    imu_msg.angular_velocity.z);
        //         integrateImuStep(acc_in, ang_vel_in, imu_dt); // this writes to preint_state, but does not calc jerk and alpha
        //         new_preint_state = preint_state;
        //         integrateJerkAlpha(&this_preint_state, new_preint_state, imu_dt); // calcs jerk and alpha and saves to this state
        //         this_preint_state.time = toSec(imu_msg.header.stamp);
        //         this_preint_state.dt = imu_dt;
        //         preint_states.push_back(this_preint_state);
        //         // printINSstate(this_preint_state);
        //         preint_state = new_preint_state;
        //         this_preint_state = new_preint_state;
        //         // RCLCPP_INFO(get_logger(), "preint state%i pos %f %f %f, vel %f %f %f time %f",i, new_preint_state.pos.x(), new_preint_state.pos.y(), new_preint_state.pos.z(), new_preint_state.vel.x(), new_preint_state.vel.y(), new_preint_state.vel.z(), new_preint_state.time  );
        //         // this loops should run until enough imu frames are acquired for preintegration
        //         if (msg_time > next_scan_end_time + imu_dt*1.0){ // we wish to add atleast one frame beyond to calc jerk etc. so when the first imu frame that is later is added we break/return, the 1.10 is to add 10% to account for variation in timestamps precision
        //             rclcpp::Time time_preint_end = system_clock.now();
        //             RCLCPP_INFO(get_logger(), "--- PREINTERGRATION ----");
        //             RCLCPP_INFO(get_logger(), "IMU frames integrated %i, preint time: %fms", preint_states.size(), time_preint_end.seconds()*1000.0 - time_preint_start.seconds()*1000.0);
        //             return;
        //         }
        //     }

        //     RCLCPP_INFO(get_logger(), "FAILED INTEGRATION..  (this shouldn't happen)");

        // }   


        void calculateInitialTransformationGuess(){

            init_guess = Eigen::Matrix4f::Identity(); 
            

            // Eigen::Matrix3d last_odometry_transformation_rot = last_odometry_transformation.block<3,3>(0,0);

            // odometry_transformation_guess = registration_transformation * last_odometry_transformation; // prediction of the next transformation using the previuosly found transformation - used for ICP initial guess
            // odometry_transformation_guess = registration_transformation; // prediction of the next transformation using the previuosly found transformation - used for ICP initial guess
            // odometry_transformation_guess = registration_transformation * last_odometry_transformation_rot; // prediction of the next transformation using the previuosly found transformation - used for ICP initial guess
            // odometry_transformation_guess = Eigen::Matrix4d::Identity(); // prediction of the next transformation using the previuosly found transformation - used for ICP initial guess

            Eigen::Matrix4f K;
            K.block<3, 3>(0, 0) = Eigen::Matrix3f(Eigen::Matrix3d(keyframe_pose.orientation).cast<float>());
            K.block<3, 1>(0, 3) = keyframe_pose.position.cast<float>();
            // K.block<3, 1>(0, 3) = Eigen::Vector3f::Zero();

            if (use_preint_imu_guess_ && !imu_buffer.empty()){  
                // preintegrateIMU();
                // preint_state.pos.z() = 0.0;
                init_guess.block<3, 3>(0, 0) = Eigen::Matrix3f((preint_state.ori).cast<float>());
                // init_guess.block<3, 3>(0, 0) = Eigen::Matrix3f((ins_relative.rotation).cast<float>());
                // init_guess.block<3, 1>(0, 3) = (registration_transformation.translation + preint_position).cast<float>();
                // init_guess.block<3, 1>(0, 3) = (preint_position).cast<float>();
                init_guess.block<3, 1>(0, 3) = (registration_transformation.translation).cast<float>();
                // init_guess.block<3, 1>(0, 3) = (average_translation).cast<float>();
                init_guess(1,3) = 0.0;
                init_guess(2,3) = - keyframe_pose.position.z();
            }  else if (use_ins_guess_ ){  // and with "ins_is_updated" -- need also to make guarentee that last ins msg has been recieved.
                // RCLCPP_INFO(get_logger(), "INS guess used");
                // updateINSRelative();

                Eigen::Matrix3f ins_rot(Eigen::Matrix3d(ins_pose.orientation).cast<float>());
                Eigen::Matrix3f rot_guess(( K.block<3, 3>(0, 0).inverse() * ins_rot));

                init_guess.block<3, 3>(0, 0) = rot_guess;
                // init_guess.block<3, 1>(0, 3) = (ins_relative.translation).cast<float>();
                init_guess.block<3, 1>(0, 3) = (ins_relative.translation).cast<float>();
                // init_guess.block<3, 1>(0, 3) = (registration_transformation.translation + rot_guess*last_odometry_transformation.translation).cast<float>();
                // init_guess.block<3, 1>(0, 3) = Eigen::Vector3f::Zero();
                // init_guess.block<3, 1>(0, 3) = (average_translation).cast<float>();
                init_guess(2,3) = - keyframe_pose.position.z();
            } else if (use_lidar_odometry_guess_ ) {
                // RCLCPP_INFO(get_logger(), "Odometry guess used");
                init_guess.block<3, 3>(0, 0) = Eigen::Matrix3f((registration_transformation.rotation * last_odometry_transformation.rotation).cast<float>());
                init_guess.block<3, 1>(0, 3) = (registration_transformation.translation + last_odometry_transformation.translation).cast<float>();
                // init_guess.block<3, 3>(0, 0) = Eigen::Matrix3f((registration_transformation.rotation).cast<float>());
                // init_guess.block<3, 1>(0, 3) = (registration_transformation.translation).cast<float>();
                // init_guess.block<3, 1>(0, 3) = average_translation.cast<float>();
                // init_guess(1,3) = 0.0;
                // init_guess(2,3) = 0.0;
            } 
            // else just identity guess..
            // RCLCPP_INFO(get_logger(), "Problem in here?");

            // publishPriorCloud(keyframe_pose.matrix.inverse().cast<float>());
            publishPriorCloud(init_guess);
        }


        // Eigen::Matrix4d regisrationICP(boost::shared_ptr<pcl::PointCloud<PointType>> source, const boost::shared_ptr<pcl::PointCloud<PointType>> target)
        // {
        //     typedef pcl::registration::TransformationEstimationPointToPlane<PointType, PointType> PointToPlane;
        //     // typedef pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<PointType, PointType> PointToPlane;
        //     // maybe not setup the icp on every use
        //     // pcl::IterativeClosestPoint<PointType, PointType> icp;
         
        //     pcl::IterativeClosestPointWithNormals<PointType, PointType> icp;
            
        //     boost::shared_ptr<PointToPlane> p2pl(new PointToPlane);
        //     icp.setTransformationEstimation(p2pl);
        //     icp.setUseReciprocalCorrespondences(true);

        //     // double max_correspondance_distance = cloud_scale * icp_max_correspondence_distance_  ;
        //     double max_correspondance_distance = icp_max_correspondence_distance_  ;

        //     icp.setUseSymmetricObjective(true);
        //     icp.setEnforceSameDirectionNormals(true);
            
        //     // RCLCPP_INFO(get_logger(),"source size %i target size %i", source->points.size(), target->points.size());

        //     icp.setInputSource(source);
        //     icp.setInputTarget(target);
        //     icp.setEuclideanFitnessEpsilon(1e-9);
        //     icp.setTransformationRotationEpsilon(0.999999); // cos(angle)

        //     // icp.setRANSACIterations(10);
        //     // icp.setRANSACOutlierRejectionThreshold(1.5);

        //     pcl::PointCloud<PointType>::Ptr aligned_cloud(new pcl::PointCloud<PointType>());
        //     // Eigen::Matrix4f init_guess = odometry_transformation_guess.cast<float>(); // should be a better guess anyway

            

        //     // course estimation first to get a better initial estimate of transformation
        //     icp.setMaxCorrespondenceDistance(coarse_correspondence_factor_*max_correspondance_distance); // 10*
        //     icp.setMaximumIterations(icp_max_coarse_iterations_);
        //     icp.setTransformationEpsilon(1e-1);

        //     icp.align(*aligned_cloud, init_guess);

        //     //get icp transformation 
        //     init_guess = icp.getFinalTransformation();//.cast<double>(); // why cast to double??
            

        //     // second iteration with finer correspondence limit
        //     // icp.setTransformationEpsilon(1e-5);
        //     icp.setMaxCorrespondenceDistance(max_correspondance_distance);
        //     icp.setMaximumIterations(icp_max_iterations_);
        //     icp.setTransformationEpsilon(1e-9);
        //     icp.align(*aligned_cloud, init_guess);

        //     // Eigen::Matrix4f registration_transform;
        //     // registration_transform = icp.getFinalTransformation();//.cast<double>(); // why cast to double??

            
        //     Eigen::Matrix4d registration_transform_double = icp.getFinalTransformation().cast<double>();


        //     icp_fitness = icp.getFitnessScore();
            
        //     RCLCPP_INFO(get_logger(), "ICP %i,  fitness: %f", icp.hasConverged(), icp_fitness);


        //     return registration_transform_double;
        // }

        // Eigen::Matrix4d regisrationICP_gcip(boost::shared_ptr<pcl::PointCloud<PointType>> source, const boost::shared_ptr<pcl::PointCloud<PointType>> target)
        // {

        //     // maybe not setup the icp on every use
     
        //     // pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp;
        //     pclomp::GeneralizedIterativeClosestPoint<PointType, PointType> icp;

        //     // double max_correspondance_distance = icp_max_correspondence_distance_;
        //     double max_correspondance_distance = cloud_scale * icp_max_correspondence_distance_  ;

        //     icp.setInputSource(source);
        //     icp.setInputTarget(target);
        //     icp.setEuclideanFitnessEpsilon(1e-5);
        //     // icp.setRANSACIterations(10);
        //     // icp.setRANSACOutlierRejectionThreshold(1.5);
        //     // icp.setCorrespondenceRandomness(20);
        //     // icp.setMaximumOptimizerIterations(50);

        //     // course estimation first to get a better initial estimate of transformation
        //     icp.setMaxCorrespondenceDistance(coarse_correspondence_factor_*max_correspondance_distance);
        //     icp.setMaximumIterations(icp_max_coarse_iterations_);
        //     icp.setTransformationEpsilon(1e-1);

        //     pcl::PointCloud<PointType>::Ptr aligned_cloud(new pcl::PointCloud<PointType>());
        //     icp.align(*aligned_cloud, init_guess);

        //     //get icp transformation and use 
        //     Eigen::Matrix4f registration_transform;
        //     registration_transform = icp.getFinalTransformation();//.cast<double>(); // why cast to double??

        //     // second iteration with finer correspondence limit
        //     icp.setMaxCorrespondenceDistance(max_correspondance_distance);
        //     icp.setMaximumIterations(icp_max_iterations_);
        //     icp.setTransformationEpsilon(1e-8);
        //     icp.align(*aligned_cloud, registration_transform);

        //     registration_transform = icp.getFinalTransformation();//.cast<double>(); // why cast to double??

        //     // icp_nl.align(*aligned_cloud, registration_transform);
        //     // // Eigen::Matrix4d registration_transform;
        //     // registration_transform = icp_nl.getFinalTransformation();//.cast<double>(); // why cast to double??
            
        //     Eigen::Matrix4d registration_transform_double = registration_transform.cast<double>();
            
        //     // registration_transformation = registration_transform_double;

        //     // make transform matrix into quaternion and vector
        //     // Eigen::Quaterniond reg_quarternion(registration_transform_double.block<3, 3>(0, 0)); 
        //     // Eigen::Vector3d reg_translation(registration_transform_double.block<3, 1>(0, 3));

        //     icp_fitness = icp.getFitnessScore();
            
        //     RCLCPP_INFO(get_logger(), "ICP: %i,  fitness: %f", icp.hasConverged(), icp_fitness);


        //     return registration_transform_double;
        // }

        // Eigen::Matrix4d regisrationNDT(boost::shared_ptr<pcl::PointCloud<PointType>> source, const boost::shared_ptr<pcl::PointCloud<PointType>> target)
        // {
        //     // Initializing Normal Distributions Transform (NDT).
        //     // pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        //     pclomp::NormalDistributionsTransform<PointType, PointType> ndt;
        //     // fast_pcl::NormalDistributionsTransform<PointType, PointType> ndt;

        //     // Setting scale dependent NDT parameters
        
        //     ndt.setResolution (0.5);
        //     // Setting point cloud to be aligned.
        //     ndt.setInputSource(source);
        //     // Setting point cloud to be aligned to.
        //     ndt.setInputTarget(target);

        //     ndt.setNeighborhoodSearchMethod(pclomp::DIRECT1);

        //     // ndt.setOulierRatio(0.01);
        
        //     pcl::PointCloud<PointType>::Ptr aligned_cloud(new pcl::PointCloud<PointType>());
        //     // Eigen::Matrix4f init_guess = odometry_transformation_guess.cast<float>(); // should be a better guess anyway

        //     // ndt.setStepSize (coarse_correspondence_factor_* icp_max_correspondence_distance_);
        //     if (false) {

        //         // course estimation first to get a better initial estimate of transformation

        //         ndt.setStepSize (coarse_correspondence_factor_* icp_max_correspondence_distance_);
        //         //Setting Resolution of NDT grid structure (VoxelGridCovariance).
        //         // ndt.setResolution (0.1);
        //         ndt.setMaximumIterations(icp_max_coarse_iterations_);
        //         ndt.setTransformationEpsilon(1e-1);

        //         ndt.align(*aligned_cloud, init_guess);
        //         // RCLCPP_INFO(get_logger(),"Where's the poop Robin?");

        //         //get icp transformation 
        //         init_guess = ndt.getFinalTransformation();//.cast<double>(); // why cast to double??
        //     }

        //     ndt.setTransformationEpsilon (1e-2);
        //     // Setting max number of registration iterations final.
        //     ndt.setMaximumIterations(icp_max_iterations_);
        //     // Setting maximum step size for More-Thuente line search.
        //     ndt.setStepSize(icp_max_correspondence_distance_);
        //     //Setting Resolution of NDT grid structure (VoxelGridCovariance).

        //     // Setting minimum transformation difference for termination condition.


        //     ndt.align(*aligned_cloud, init_guess);
        
        //     // double max_correspondance_distance = cloud_scale * icp_max_correspondence_distance_  ;

        //     // RCLCPP_INFO(get_logger(),"source size %i target size %i", source->points.size(), target->points.size());

        //     Eigen::Matrix4f registration_transform;
        //     registration_transform = ndt.getFinalTransformation();//.cast<double>(); // why cast to double??

            
        //     Eigen::Matrix4d registration_transform_double = registration_transform.cast<double>();
            
        //     // registration_transformation = registration_transform_double;

        //     // make transform matrix into quaternion and vector
        //     // Eigen::Quaterniond reg_quarternion(registration_transform_double.block<3, 3>(0, 0)); 
        //     // Eigen::Vector3d reg_translation(registration_transform_double.block<3, 1>(0, 3));

        //     icp_fitness = ndt.getFitnessScore();
            
        //     RCLCPP_INFO(get_logger(), "NDT %i,  fitness: %f", ndt.hasConverged(), icp_fitness);

        //     return registration_transform_double;
        // }

        // Eigen::Matrix4d cloudRegisration(boost::shared_ptr<pcl::PointCloud<PointType>> source, const boost::shared_ptr<pcl::PointCloud<PointType>> target)
        Eigen::Matrix4d cloudRegisration(boost::shared_ptr<pcl::PointCloud<PointType>> source)
        {
            // rclcpp::Clock system_clock;
            

            registration_->setInputSource(source);
            // rclcpp::Time time_covcalc_start = system_clock.now();
            // registration_->setInputTarget(target);
            // rclcpp::Time time_covcalc_end = system_clock.now();
            // RCLCPP_INFO(get_logger(), "Covariance calculation time: %fms", time_covcalc_end.seconds()*1000.0 - time_covcalc_start.seconds()*1000.0);
            
            pcl::PointCloud<PointType>::Ptr aligned_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
            rclcpp::Time time_align_start = system_clock.now();
            // RCLCPP_INFO(get_logger(), "Problemin align?");
            registration_->align(*aligned_cloud, init_guess);
            // RCLCPP_INFO(get_logger(), "No..");
            rclcpp::Time time_align_end = system_clock.now();

            Eigen::Matrix4d registration_transform_double = registration_->getFinalTransformation().cast<double>();

            

            icp_fitness = registration_->getFitnessScore();

            registration_process_time = time_align_end.seconds()*1000.0 - time_align_start.seconds()*1000.0;
            RCLCPP_INFO(get_logger(), "----  REGISTRATION ----");
            RCLCPP_INFO(get_logger(), "Registration converged: %i,  fitness: %f, time: %fms", registration_->hasConverged(), icp_fitness, registration_process_time);
            RCLCPP_INFO(get_logger(), "size of source %i, size of target %i ", source->points.size(), registration_->getInputTarget()->points.size());
            return registration_transform_double;
        }

        // void scanMatchRegistration(boost::shared_ptr<pcl::PointCloud<PointType>> src, const boost::shared_ptr<pcl::PointCloud<PointType>> tgt)
        void scanMatchRegistration(boost::shared_ptr<pcl::PointCloud<PointType>> src)
        {
            
            pcl::PointCloud<PointType>::Ptr source = boost::make_shared<pcl::PointCloud<PointType>>(*src);
            // pcl::PointCloud<PointType>::Ptr target = boost::make_shared<pcl::PointCloud<PointType>>(*tgt);
            // pcl::copyPointCloud(*src, *source);

            // Eigen::Vector4d centroid;
            // pcl::compute3DCentroid(*source, centroid);
            // Eigen::Vector3d centroid3(-centroid[0], -centroid[1], -centroid[2]);
            // RCLCPP_INFO(get_logger(), "centroid %f %f %f", centroid3[0], centroid3[1], centroid3[2] );
            // Eigen::Matrix4d demean_T = Eigen::Matrix4d::Identity();
            // // demean_T.block<3,3>(0,0) = last_odometry_pose.matrix.block<3,3>(0,0).inverse();
            // demean_T.block<3,1>(0,3) = centroid3;
            // // demean_T.block<3,3>(0,0) = Eigen::Matrix3d(preint_transformation_guess.rotation).inverse();
            // // demean_T.block<3,1>(0,3) = - preint_transformation_guess.translation;
  
            // // subtract the centroid from source and target copies, should still be aligned as before but source be will centered in origo
            // pcl::transformPointCloudWithNormals(*source, *source, demean_T);
            // pcl::transformPointCloudWithNormals(*target, *target, demean_T);
            // the output transformation is the only thing that matters
            
            Eigen::Matrix4d T;
            T = keyframe_pose.matrix.inverse();
            // pcl::transformPointCloudWithNormals<PointType>(*source, *source, T);

            // RCLCPP_INFO(get_logger(), "Starting scan matching..");
            Eigen::Matrix4d registration_transform_matrix;
            Eigen::Matrix4d R;
            R = T.inverse();
            R.block<3,1>(0,3).setZero();
            registration_transform_matrix = cloudRegisration(source);
            // registration_transform_matrix = R * registration_transform_matrix;
            // cout << registration_transform_matrix << "\n";

            // registration_transformation.rotation = Eigen::Quaterniond(registration_transform_matrix.block<3,3>(0,0)).normalized();
            registration_transformation.matrix = registration_transform_matrix;
            // registration_transformation.matrix.block<3,1>(0,3) += centroid3;
            registration_transformation.rotation = Eigen::Quaterniond(registration_transform_matrix.block<3,3>(0,0));
            registration_transformation.translation = Eigen::Vector3d(registration_transform_matrix.block<3,1>(0,3));

            saveFitness();
        }

        void setRegistrationTarget(boost::shared_ptr<pcl::PointCloud<PointType>> tgt)
        {
            pcl::PointCloud<PointType>::Ptr target = boost::make_shared<pcl::PointCloud<PointType>>(*tgt);
            // pcl::copyPointCloud(*tgt, *target);
            Eigen::Matrix4d T;
            T = keyframe_pose.matrix.inverse();
            // cout <<last_odometry_pose.matrix <<"\n" << T << "\n";
            // pcl::transformPointCloudWithNormals<PointType>(*target, *target, T);
            registration_->setInputTarget(target);
        }

        void saveFitness(){
            fitnesses.push_back(icp_fitness);
        }


        void updateTransformationErrorFromConstVelocityModel()
        {
            // make transform matrix into quaternion and vector
            // Eigen::Quaterniond quarternion_pose(last_odometry_pose.block<3, 3>(0, 0)); 
            // Eigen::Matrix3d rot_mat(last_odometry_transformation.block<3, 3>(0, 0)); 
            Eigen::Vector3d translation = last_odometry_transformation.translation;

            Eigen::Vector3d translation_deviation;
            translation_deviation = average_translation - translation;
            // translation = rot_mat * translation;
            // RCLCPP_INFO(get_logger(), "translation %f %f %f", translation[0], translation[1], translation[2]);

            translation_std_x = ( abs(translation_deviation[0]))  ;//+ translation_std_min_x_;
            translation_std_y = ( abs(translation_deviation[1]))  ;//+ translation_std_min_y_;
            translation_std_z = ( abs(translation_deviation[2]))  ;//+ translation_std_min_z_;

            translation_std_x = max(translation_std_x, translation_std_min_x_);
            translation_std_y = max(translation_std_y, translation_std_min_y_);
            translation_std_z = max(translation_std_z, translation_std_min_z_);

            // RCLCPP_INFO(get_logger(), "trans deviation %f %f %f", translation_std_x, translation_std_y, translation_std_z);

            // length of translation
            // double distance = translation.norm();

            // angle of rotation
            // 0.1 rad is approx 57.3 deg
            // Eigen::AngleAxisd angleAxis = Eigen::AngleAxisd(quarternion);
            // double angle = angleAxis.angle()*180.0/M_PI;
        }

        void calculateConstantVelocityModelPrediction(){

            average_translation << 0.0,0.0,0.0;
            for (size_t i=0; i < recent_translations.size(); i++){
                average_translation += recent_translations[i];
            }
            average_translation = average_translation/ ((float)recent_translations.size());
    
        }

        void pushLastTransformationToConstVelocityModel()
        {
            // RCLCPP_INFO(get_logger(), "reg_translation %f %f %f", reg_translation[0], reg_translation[1], reg_translation[2]);    
            // Eigen::Vector3d reg_translation(registration_transformation.block<3, 1>(0, 3));
            Eigen::Vector3d recent_translation = last_odometry_transformation.translation;
            recent_translations.push_back(recent_translation);
            if (recent_translations.size() > 3){
                recent_translations.pop_front();
            }

        }

        
        void simpleConstraningObserverUpdate(const Transformation preint_transformation_guess, INSstate &preint_anchor)
        {
            double gamma_1, gamma_2, gamma_3, gamma_4, gamma_5;
            gamma_1 = gamma_1_ * scan_dt;
            gamma_2 = gamma_2_ * scan_dt;
            gamma_3 = gamma_3_ * scan_dt;
            gamma_4 = gamma_4_ * scan_dt;
            gamma_5 = gamma_5_ * scan_dt;

            
            observer_state = preint_anchor; // get the anchor in the local frame
            // RCLCPP_INFO(get_logger(), "Anchor before: pos %f %f %f vel %f %f %f", observer_state.pos.x(), observer_state.pos.y(), observer_state.pos.z(), observer_state.vel.x(),observer_state.vel.y(), observer_state.vel.z());
            // RCLCPP_INFO(get_logger(), "Anchor before : ori %f %f %f %f", observer_state.ori.w(), observer_state.ori.x(), observer_state.ori.y(), observer_state.ori.z());

            // the scan match residual are in local frame
            // Eigen::Quaterniond q_error = preint_residual.rotation;
            // Eigen::Vector3d p_error = preint_residual.translation;

            // calculated in world frame - here last odometry is the current one just establsihed by ICP
            Eigen::Quaterniond q_error = preint_transformation_guess.rotation.inverse() * last_odometry_pose.orientation; 
            Eigen::Vector3d p_error = (last_odometry_pose.position - preint_transformation_guess.translation); 

            preint_residual.rotation = q_error;
            preint_residual.translation = last_odometry_pose.orientation.matrix() * p_error; // saved in the body frame

            RCLCPP_INFO(get_logger(), "Observer: Translation residual: %f %f %f norm: %f", p_error.x(), p_error.y(), p_error.z(), p_error.norm());
            RCLCPP_INFO(get_logger(), "Observer: Rotational residual: %f %f %f %f", q_error.w(), q_error.x(), q_error.y(), q_error.z());

            Eigen::Vector3d q_e_vec = q_error.vec(); // the imaginary part of the quaternion is made to a vector
            // RCLCPP_INFO(get_logger(), "q_error calc: %f %f %f %f", q_error_calc.w(), q_error_calc.x(), q_error_calc.y(), q_error_calc.z());
            // Eigen::Quaterniond q_error = observer_state.ori.inverse() * last_odometry_pose.orientation;
            // Eigen::Vector3d p_error = last_odometry_pose.position - observer_state.pos;

            double qw = q_error.w();
            double abs_qw = abs(qw);
            int sgn_qw = 1;
            if (abs_qw != 0.0) // to avoid division with zero, if abs_qw = 0.0 sign is set positive 
                sgn_qw = qw/ abs_qw;

            Eigen::Quaterniond q_e(1.0 - abs_qw, sgn_qw * q_error.x(), sgn_qw * q_error.y(), sgn_qw * q_error.z());
            // RCLCPP_INFO(get_logger(), "q_e: %f %f %f %f", q_e.w(), q_e.x(), q_e.y(), q_e.z());

            // Eigen::Vector3d dynamic_model_deweight(1.0, 0.3, 0.1); // a ground based vehicle is not expected to move sideways or laterally
            Eigen::Vector3d dynamic_model_deweight(1.0, 1.0, 1.0); 
            Eigen::Vector3d z_deweight(1.0, 1.0, 0.1);
            // Eigen::Vector3d eps(0.0001,0.0001,0.0001);

            observer_state.ori           = addQuaternions(observer_state.ori,  multQuatByScalar(observer_state.ori * q_e, gamma_1)).normalized();
            observer_state.bias.ang     -=  gamma_2 * qw * q_e_vec;  
            // Eigen::Vector3d velocity_deweight(observer_state.vel);
            // velocity_deweight = (velocity_deweight.cwiseAbs()) / (velocity_deweight.cwiseAbs().maxCoeff() + 1e-5);
            // observer_state.pos          +=  gamma_3 * (p_error).cwiseProduct(observer_state.ori.matrix() * dynamic_model_deweight);
            observer_state.pos          +=  gamma_3 * p_error;
            observer_state.vel          +=  gamma_4 * (p_error).cwiseProduct(dynamic_model_deweight); // here it is corrected in world frame
            // observer_state.vel          +=  gamma_4 * (observer_state.ori.matrix().inverse() * p_error).cwiseProduct(dynamic_model_deweight); // here it is corrected in the local frame
            observer_state.bias.acc     -=  gamma_5 * observer_state.ori.matrix().inverse() * p_error;  // the error is rotated into local frame to update bias
            // observer_state.bias.acc     -=  gamma_5 *  p_error;  

            if (abs(observer_state.bias.ang[1])  > 1.0) { // attempt at debugging, this should not happen
                RCLCPP_INFO(get_logger(), "What is wrong?");
                RCLCPP_INFO(get_logger(), "q_res: %f %f %f %f", q_error.w(), q_error.x(), q_error.y(), q_error.z());
                RCLCPP_INFO(get_logger(), "q_e: %f %f %f %f", q_e.w(), q_e.x(), q_e.y(), q_e.z());
                observer_state.bias.ang[1] = 0.0;
            }   
            // RCLCPP_INFO(get_logger(), "Anchor after: pos %f %f %f vel %f %f %f", observer_state.pos.x(), observer_state.pos.y(), observer_state.pos.z(), observer_state.vel.x(),observer_state.vel.y(), observer_state.vel.z());
            // RCLCPP_INFO(get_logger(), "Anchor after : ori %f %f %f %f", observer_state.ori.w(), observer_state.ori.x(), observer_state.ori.y(), observer_state.ori.z());

            preint_anchor = observer_state;
            Eigen::Vector3d local_velocity(observer_state.ori.matrix().inverse() * observer_state.vel);
            RCLCPP_INFO(get_logger(), "Observer: local frame vel: %f %f %f", local_velocity.x(), local_velocity.y(), local_velocity.z());
            RCLCPP_INFO(get_logger(), "Observer: bias acc: %f %f %f", observer_state.bias.acc.x(), observer_state.bias.acc.y(), observer_state.bias.acc.z());
            RCLCPP_INFO(get_logger(), "Observer: bias ang: %f %f %f", observer_state.bias.ang.x(), observer_state.bias.ang.y(), observer_state.bias.ang.z());

        }

        void updateOdometryPose(Transformation update_transformation)
        {   
            Eigen::Matrix4d update_T, Last_T, Last_P, Key_P, next_P, DeltaT;
            update_T = Eigen::Matrix4d::Identity();
            Last_T = Eigen::Matrix4d::Identity();
            Last_P = Eigen::Matrix4d::Identity();
            Key_P = Eigen::Matrix4d::Identity();
            next_P = Eigen::Matrix4d::Identity();
            DeltaT = Eigen::Matrix4d::Identity();
            Last_P.block<3,1>(0,3) = last_odometry_pose.position;
            Last_P.block<3,3>(0,0) = Eigen::Matrix3d(last_odometry_pose.orientation);
            Key_P.block<3,1>(0,3) = keyframe_poses.back().position;
            Key_P.block<3,3>(0,0) = Eigen::Matrix3d(keyframe_poses.back().orientation);

            // update_T.block<3,1>(0,3) = update_transformation.translation;
            // update_T.block<3,3>(0,0) = Eigen::Matrix3d(update_transformation.rotation);
            update_T = update_transformation.matrix;
            DeltaT = update_transformation.matrix;
            // RCLCPP_INFO(get_logger(), "prior pose: pos %f %f %f ", preint_transformation_guess.translation.x(), preint_transformation_guess.translation.y(), preint_transformation_guess.translation.z());
            if (use_preint_undistortion_){

                RCLCPP_INFO(get_logger(), "prior INS pose: pos %f %f %f ", preint_transformation_guess.translation.x(), preint_transformation_guess.translation.y(), preint_transformation_guess.translation.z());
                RCLCPP_INFO(get_logger(), "prior INS pose: ori %f %f %f %f", preint_transformation_guess.rotation.w(), preint_transformation_guess.rotation.x(), preint_transformation_guess.rotation.y(), preint_transformation_guess.rotation.z());
                
                Eigen::Matrix4d T_M;
                T_M = Eigen::Matrix4d::Identity();
                T_M.block<3,3>(0,0) = Eigen::Matrix3d(preint_transformation_guess.rotation);
                T_M.block<3,1>(0,3) = preint_transformation_guess.translation;
                // cloud_zeroing_transform = Eigen::Matrix4d::Identity();
                
                // this value is only used to save to file

                update_T = DeltaT * T_M; // DeltaT is the refining ICP transformation, T_M is the preintegrated guess
                // update_T = T_M * DeltaT ; // DeltaT is the refining ICP transformation, T_M is the preintegrated guess
                update_transformation.translation = update_T.block<3,1>(0,3);
                update_transformation.rotation = Eigen::Quaterniond(update_T.block<3,3>(0,0));

                // registration_transformation = update_transformation; 

                // preint_residual.matrix = DeltaT;
                // preint_residual.translation = preint_residual.matrix.block<3,1>(0,3);
                // preint_residual.rotation = Eigen::Quaterniond(preint_residual.matrix.block<3,3>(0,0)); 

                // RCLCPP_INFO(get_logger(), "Translation residual: %f %f %f norm: %f", preint_residual.translation.x(), preint_residual.translation.y(), preint_residual.translation.z(), preint_residual.translation.norm());
                // RCLCPP_INFO(get_logger(), "Rotational residual:%f %f %f %f",preint_residual.rotation.w(),  preint_residual.rotation.x(), preint_residual.rotation.y(), preint_residual.rotation.z());


                next_P = update_T;
            } else {
                // next_P = Key_P * update_T;
                // next_P = keyframe_pose.matrix * update_T ;
                // next_P = update_T * keyframe_pose.matrix;
                update_T =  DeltaT * Key_P;
                // update_T =  Key_P * DeltaT;
                next_P = update_T;


            }

            // correct the cloud by the scan match delta
            pcl::transformPointCloudWithNormals<PointType>(*cloud_in, *cloud_in, DeltaT);
            pcl::transformPointCloudWithNormals<PointType>(*cloud_in_ds, *cloud_in_ds, DeltaT);
            
            // update_T.block<3,1>(0,3) = update_transformation.translation;
            // update_T.block<3,3>(0,0) = Eigen::Matrix3d(update_transformation.rotation);

            // RCLCPP_INFO(get_logger(), "key %f %f %f", translation_std_x, translation_std_y, translation_std_z);
            
            // Eigen::Vector3d rotated_translation = keyframe_poses.back().orientation * update_transformation.translation;
        
            Pose next_odometry_pose;
            // next_odometry_pose.position =   keyframe_poses.back().position  + rotated_translation;
            // next_odometry_pose.orientation = (update_transformation.rotation * keyframe_poses.back().orientation).normalized();
            next_odometry_pose.position =    next_P.block<3,1>(0,3);
            next_odometry_pose.orientation = Eigen::Quaterniond(next_P.block<3,3>(0,0));
            
            Last_T = Last_P.inverse() * next_P;
            last_odometry_transformation.translation = Last_T.block<3,1>(0,3);  // transformation between 2 consecutive odometry poses
            last_odometry_transformation.rotation = Eigen::Quaterniond(Last_T.block<3,3>(0,0)); // transformation between 2 consecutive odometry poses
            
            // RCLCPP_INFO(get_logger(), "translation %f %f %f", update_transformation.translation[0] , registration_transformation.translation[1], update_transformation.translation[2]);
            // RCLCPP_INFO(get_logger(), "rotated translation %f %f %f", rotated_translation[0] , rotated_translation[1], rotated_translation[2]);
            // RCLCPP_INFO(get_logger(), "keyframe position %f %f %f", keyframe_pose.position[0] , keyframe_pose.position[1], keyframe_pose.position[2]);

            // RCLCPP_INFO(get_logger(), "next position %f %f %f", next_odometry_pose.position[0] , next_odometry_pose.position[1], next_odometry_pose.position[2]);

            // next_odometry_pose.position = keyframe_pose.orientation * registration_transformation.translation;
            // next_odometry_pose.orientation = (keyframe_pose.orientation * registration_transformation.rotation);

            // last_odometry_transformation.rotation = last_odometry_pose.orientation.inverse() * next_odometry_pose.orientation; // transformation between 2 consecutive odometry poses
            // last_odometry_transformation.translation = last_odometry_pose.orientation.inverse() * (next_odometry_pose.position - last_odometry_pose.position) ; // transformation between 2 consecutive odometry poses

            updateTransformationErrorFromConstVelocityModel();

            pushLastTransformationToConstVelocityModel();
            calculateConstantVelocityModelPrediction();

            // RCLCPP_INFO(get_logger(), "average translation %f %f %f", average_translation[0], average_translation[1], average_translation[2]);
            // calculateAverageRotation();


            last_odometry_pose.position = Eigen::Vector3d(next_odometry_pose.position);
            last_odometry_pose.orientation = Eigen::Quaterniond(next_odometry_pose.orientation.normalized());
            last_odometry_pose.velocity = last_odometry_transformation.translation / scan_dt; // translation is on local frame therefore this speed is in the local frame
            last_odometry_pose.time = current_scan_time;
            last_odometry_pose.matrix = next_P;
            last_odometry_transformation.matrix = Last_T;

            

            RCLCPP_INFO(get_logger(), "Posterior LO pose: pos %f %f %f ", last_odometry_pose.position.x(), last_odometry_pose.position.y(), last_odometry_pose.position.z());
            RCLCPP_INFO(get_logger(), "Posterior LO pose: vel %f %f %f ", last_odometry_pose.velocity.x(), last_odometry_pose.velocity.y(), last_odometry_pose.velocity.z());
            RCLCPP_INFO(get_logger(), "Posterior LO pose: ori %f %f %f %f", last_odometry_pose.orientation.w(), last_odometry_pose.orientation.x(), last_odometry_pose.orientation.y(), last_odometry_pose.orientation.z());
            
            Eigen::Vector3d ypr = quaternionToEulerAngles(last_odometry_pose.orientation) * 180.0 / M_PI;
            RCLCPP_INFO(get_logger(), "Posterior LO pose: ypr %f %f %f", ypr.x(), ypr.y(), ypr.z());


            // keyframe_to_odometry_transformation.rotation = keyframe_pose.orientation.inverse() * last_odometry_pose.orientation;
            // keyframe_to_odometry_transformation.translation = keyframe_pose.orientation.inverse() * (last_odometry_pose.position - keyframe_pose.position);
        }

        

        bool newKeyframeRequired()
        {
            // get transformation between pose and last keyframe
            keyframe_to_odometry_transformation.rotation = keyframe_pose.orientation.inverse() * last_odometry_pose.orientation;
            keyframe_to_odometry_transformation.translation = keyframe_pose.orientation.inverse() * (last_odometry_pose.position - keyframe_pose.position);
            // make transform matrix into quaternion and vector
            Eigen::Quaterniond reg_quarternion = keyframe_to_odometry_transformation.rotation; 
            Eigen::Vector3d reg_translation = keyframe_to_odometry_transformation.translation;
            // Eigen::Quaterniond reg_quarternion = registration_transformation.rotation; 
            // Eigen::Vector3d reg_translation = registration_transformation.translation;

            // length of translation
            double distance = reg_translation.norm();

            // angle of rotation
            // 0.1 rad is approx 5.73 deg
            Eigen::AngleAxisd angleAxis = Eigen::AngleAxisd(reg_quarternion);
            double angle = angleAxis.angle()*180.0/M_PI;

            // RCLCPP_INFO(get_logger(), "found transform parameters: length: %f, angle: %f, fitness: %f", distance, angle, icp_fitness);
            // RCLCPP_INFO(get_logger(), "threshold parameters: length: %f, angle: %f, fitness: %f", keyframe_threshold_length_, keyframe_threshold_angle_, keyframe_threshold_fitness_);
            // RCLCPP_INFO(get_logger(), "should this be true: %i", (distance > keyframe_threshold_length_ || angle > keyframe_threshold_angle_ || icp_fitness > keyframe_threshold_fitness_ ));


            // and get icp fitness

            // assert if any is beyond thresholds

            bool dist = distance > keyframe_threshold_length_ && keyframe_threshold_length_ > 0.0;
            bool ang = angle > keyframe_threshold_angle_ && keyframe_threshold_angle_ > 0.0;
            bool fitness = icp_fitness > keyframe_threshold_fitness_ && keyframe_threshold_fitness_ > 0.0;
            bool index = (int)latest_frame_idx > (keyframe_threshold_index_ + keyframe_poses.back().frame_idx -1 ) && keyframe_threshold_index_ > 0;

            // RCLCPP_INFO(get_logger(), "Index bool %i, frame index: %i, index threshold: %i", index, latest_frame_idx, keyframe_threshold_index_ + keyframe_poses.back().frame_idx -1 );
            
            bool covariance_x = translation_std_x < 0.1;
            bool covariance_y = translation_std_y < 0.1;
            bool covariance_z = translation_std_z < 0.05;

            // if transformation has big covaraince it is not used as keyframe!
            bool good_covariance = (covariance_x && covariance_y && covariance_z);

            good_covariance = true; // this line is an override!


            // if (!good_covariance){ // get the prior transformation guess?
            //     registration_transformation.rotation = Eigen::Quaterniond::Identity();
            //     registration_transformation.translation = Eigen::Vector3d::Zero();
            // }


            return ((dist || ang || fitness ) && good_covariance) || index;
        }


        void savePose()
        {
            // function to push new_pose to odometries, saved as quaternion and translation

            Eigen::Quaterniond reg_quarternion = last_odometry_pose.orientation; 
            // reg_quarternion.normalize();
            Eigen::Vector3d reg_translation = last_odometry_pose.position;

            PoseInfo new_pose;
            new_pose.qw = reg_quarternion.w();
            new_pose.qx = reg_quarternion.x();
            new_pose.qy = reg_quarternion.y();
            new_pose.qz = reg_quarternion.z();
            new_pose.x = reg_translation.x();
            new_pose.y = reg_translation.y();
            new_pose.z = reg_translation.z();
            new_pose.idx = odometry_pose_info->points.size();
            new_pose.time = time_new_cloud.nanoseconds();

            odometry_pose_info->push_back(new_pose);

        }

        void saveKeyframePose()
        {
            // function to push new_pose to odometries, saved as quaternion and translation
            Eigen::Quaterniond reg_quarternion = keyframe_pose.orientation; 
            // reg_quarternion.normalize();
            Eigen::Vector3d reg_translation = keyframe_pose.position;



            PoseInfo new_pose;
            new_pose.qw = reg_quarternion.w();
            new_pose.qx = reg_quarternion.x();
            new_pose.qy = reg_quarternion.y();
            new_pose.qz = reg_quarternion.z();
            new_pose.x = reg_translation.x();
            new_pose.y = reg_translation.y();
            new_pose.z = reg_translation.z();
            new_pose.idx = keyframe_pose.frame_idx;
            new_pose.time = time_new_cloud.nanoseconds();

            keyframe_pose_info->push_back(new_pose);

        }



        void savePointCloud()
        {
            // pcl::PointCloud<PointType>::Ptr full(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr full = boost::make_shared<pcl::PointCloud<PointType>>();
            // full.reset(new pcl::PointCloud<PointType>());

            pcl::copyPointCloud(*cloud_in, *full);
            all_clouds.push_back(full);
        }

        
        


        template <typename PointT>
        void cropLocalMap(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out)
        {
            size_t j = 0;
            for (size_t i = 0; i < cloud_in.points.size(); ++i)
            {
                if (cloud_in.points[i].x < 0.0) // remove points with negative x
                    continue;
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
        Eigen::Vector3d getSurfaceNormal(PointT pt)
        {
            Eigen::Vector3d normal_vector(pt.normal_x, pt.normal_y, pt.normal_z);
            return normal_vector;
        }

        template <typename PointT>
        void calculatePointNormals(const boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_in, pcl::PointCloud<PointType> &cloud_out)
        {
            // Create the normal estimation class, and pass the input dataset to it
            pcl::NormalEstimationOMP<PointType, PointType> normal_estimator;
            // normal_estimator.useSensorOriginAsViewPoint();
            normal_estimator.setInputCloud(cloud_in);

            // Create an empty kdtree representation, and pass it to the normal estimation object.
            // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
            pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>()); // boost shared ptr?
            normal_estimator.setSearchMethod(tree);

            // // Output datasets
            // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

            // if (pc_normal_search_radius_ > 0.0)
            // {
            //     normal_estimator.setRadiusSearch(pc_normal_search_radius_); // Use all neighbors in a sphere of radius x meters
            // }
            // else
            // {
            normal_estimator.setKSearch(25); // use x nearest points, more robust to cloud scale variation
            // }

            // Compute the features
            normal_estimator.compute(cloud_out);

            // cloud_normals->size () should have the same size as the input cloud->size ()*
        }

        template <typename PointT>
        void normalFilterLocalMap(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out)
        {
            
            size_t j = 0;
            Eigen::Vector3d direction_vector(1.0,0.0,0.0);
            for (size_t i = 0; i < cloud_in.points.size(); ++i)
            {
                PointT pt = cloud_in.points[i];
                Eigen::Vector3d shooting_vector = getNormalizedPositionVector(pt);
                Eigen::Vector3d normal_vector = getSurfaceNormal(pt);
                
                if (shooting_vector.dot(direction_vector) < 0.707) // remove points behind the origo or beyond and angle of cos(theta) 
                    continue;
                if (shooting_vector.dot(normal_vector) > 0.0) // remove points where normal vector is pointing away from origo
                    continue;
                cloud_out.points[j] = cloud_in.points[i]; 
                j++;                                      
            }

            if (j != cloud_in.points.size())
            {
                cloud_out.points.resize(j);
            }

            cloud_out.height = 1;
            cloud_out.width = static_cast<uint32_t>(j);
            cloud_out.is_dense = true;
        }

        void fuseLocalLongTermMap()
        {
            target_map->clear();
            RCLCPP_INFO(get_logger(), "----  FUSE LONG TERM AND LOCAL MAP ----");

            if (long_term_map->points.size() > 0 && false) {
                RCLCPP_INFO(get_logger(), "----  FUSE registration ----");

                pcl::PointCloud<PointType>::Ptr aligned_cloud = boost::make_shared<pcl::PointCloud<PointType>>();

                registration_fuse_->setInputSource(local_map_ds);
                registration_fuse_->setInputTarget(long_term_map);
                
                Eigen::Matrix4f init_guess_fuse = Eigen::Matrix4f::Identity();
                rclcpp::Time time_align_start = system_clock.now();
                // RCLCPP_INFO(get_logger(), "Problemin align?");
                registration_fuse_->align(*aligned_cloud, init_guess_fuse);
                // RCLCPP_INFO(get_logger(), "No..");
                rclcpp::Time time_align_end = system_clock.now();

                // Eigen::Matrix4d registration_transform_double = registration_fuse_->getFinalTransformation().cast<double>();

                double icp_fuse_fitness = registration_fuse_->getFitnessScore();

                double registration_fuse_process_time = time_align_end.seconds()*1000.0 - time_align_start.seconds()*1000.0;
                RCLCPP_INFO(get_logger(), "Fuse registration converged: %i, fitness: %f, time: %fms", registration_fuse_->hasConverged(), icp_fuse_fitness, registration_fuse_process_time);

                *local_map_ds = *aligned_cloud;
            }
            
            *target_map += *long_term_map;
            *target_map += *local_map_ds;
        }

        void trimLocalMap()
        {
            
            // int latest_keyframe_idx = keyframe_index.back();
            // IDEA: get closest frames from as KD knn search as the odometry points are saved as a point cloud, and use the neighbourhood clouds as local map. -> this requires and is close to loop closure 

            // If already more than max frames, pop the frames at the beginning
            while (recent_frames.size() >= max_frames ) 
            {
                // RCLCPP_INFO(get_logger(), "Less than limit (%i) frames in local map, size: %i, odomsize: %i, index:%i", local_map_width_ ,recent_frames.size(),i, latest_frame_idx);
                recent_frames.pop_front();
            }
        }

        // function adapted from lili-om
        void buildLocalMapAroundKeyFrame() 
        {
            pcl::PointCloud<PointType>::Ptr transformed_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
            // pcl::PointCloud<PointType>::Ptr transformed_cloud_ds = boost::make_shared<pcl::PointCloud<PointType>>();
            // pcl::transformPointCloudWithNormals<PointType>(*all_clouds[keyframe_poses.size()-1], *transformed_cloud,  keyframe_pose.position, keyframe_pose.orientation);
            pcl::copyPointCloud(*cloud_in, *transformed_cloud);
            // pcl::copyPointCloud(*cloud_in, *transformed_cloud_ds);
            // transformed_cloud = cloud_in;
            recent_frames.push_back(transformed_cloud);

            // down_size_filter_keyframe_map.setInputCloud(transformed_cloud);
            // down_size_filter_keyframe_map.filter(*transformed_cloud_ds);
            // *reduced_global_map += *transformed_cloud_ds;
            // *reduced_global_map += *transformed_cloud;
            
            // down_size_filter_global_map.setInputCloud(reduced_global_map);
            // down_size_filter_global_map.filter(*reduced_global_map);
            
            // // try with downsampling each keyframe in the local map instead of 
            // local_map->clear();
            // local_map_ds->clear();
            // pcl::PointCloud<PointType>::Ptr downsampled_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
            // for (size_t i = 0; i < recent_frames.size(); ++i){

            //     down_size_filter_local_map.setInputCloud(recent_frames[i]);
            //     down_size_filter_local_map.filter(*downsampled_cloud);

            //     // *local_map += *recent_frames[i];
            //     *local_map_ds += *downsampled_cloud;
            // }


            local_map->clear();
            for (size_t i = 0; i < recent_frames.size(); ++i){
                *local_map += *recent_frames[i];
            }
           
            // calculatePointNormals(local_map, *local_map);
            downsampleLocalMap();
            // *local_map_ds += *reduced_global_map;

            // transform the local map back to latest keyframe/"origo" for it to be matchable with the most recent cloud
            // Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            // T.block<3,3>(0,0) = Eigen::Matrix3d(keyframe_pose.orientation);
            // T.block<3,1>(0,3) = (keyframe_pose.position);
            // T = T.inverse().eval();
            // pcl::transformPointCloudWithNormals<PointType>(*local_map_ds, *local_map_ds, T);
            

            // normalFilterLocalMap(*local_map_ds, *local_map_ds);
            // cropLocalMap(*local_map_ds, *local_map_ds);

            

        }

        void addToLocalMap()
        {   

            // Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            // T.block<3,3>(0,0) = Eigen::Matrix3d(keyframe_pose.orientation);
            // T.block<3,1>(0,3) = (keyframe_pose.position);
            // T.block<3,3>(0,0) = Eigen::Matrix3d(preint_residual.rotation);
            // T.block<3,1>(0,3) = (preint_residual.translation);
            // T = T.inverse().eval();


            pcl::PointCloud<PointType>::Ptr transformed_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
            // pcl::transformPointCloudWithNormals<PointType>(*all_clouds[keyframe_poses.size()-1], *transformed_cloud,  T);
            pcl::copyPointCloud(*cloud_in, *transformed_cloud);
            recent_frames.push_back(transformed_cloud);
            // pcl::transformPointCloudWithNormals<PointType>(*local_map, *local_map, T.inverse().eval());
            
            *reduced_global_map += *transformed_cloud;

            local_map->clear();
            // pcl::transformPointCloudWithNormals<PointType>(*reduced_global_map, *local_map,  T.inverse().eval());
            *local_map = *reduced_global_map;
            downsampleLocalMap();
            // calculatePointNormals(local_map_ds, *local_map_ds);
            // normalFilterLocalMap(*local_map_ds, *local_map_ds);

            // cropLocalMap(*local_map_ds, *local_map_ds);

            // pcl::transformPointCloudWithNormals<PointType>(*local_map_ds, *local_map_ds, T);
            

            // publishLocalMap();
        }

        void downsampleLocalMap()
        {
            // try and downsample each keyframe in the local map instead of 
            local_map_ds->clear();
            if (ds_voxel_size_lc_ > 0.0){
                down_size_filter_local_map.setInputCloud(local_map);
                down_size_filter_local_map.filter(*local_map_ds);
            } else {
                *local_map_ds = *local_map;
            }
        }



        void publishLocalMap()
        {
            
            // Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            // T.block<3,3>(0,0) = Eigen::Matrix3d(keyframe_pose.orientation);
            // T.block<3,1>(0,3) = (keyframe_pose.position);

            // pcl::PointCloud<PointType>::Ptr cropped_local_map = boost::make_shared<pcl::PointCloud<PointType>>();
            // pcl::copyPointCloud(*local_map_ds, *cropped_local_map);
            // pcl::transformPointCloudWithNormals<PointType>(*cropped_local_map, *cropped_local_map, T);
            
            sensor_msgs::msg::PointCloud2 msgs;
            #ifndef __INTELLISENSE__ 
            pcl::toROSMsg(*target_map, msgs);
            #endif
            // msgs.header.stamp = ros::Time().fromSec(time_new_cloud);
            msgs.header.stamp = time_new_cloud; 
            // msgs.header.frame_id = frame_id;
            msgs.header.frame_id = "lidar_odom";
            localcloud_pub->publish(msgs);
            // RCLCPP_INFO(get_logger(), "Local map published!");
            // global_cloud->clear();
        }

        void publishLocalMap(Eigen::Matrix4d transformation)
        {
            // Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            // T.block<3,3>(0,0) = Eigen::Matrix3d(keyframe_pose.orientation);
            // T.block<3,1>(0,3) = (keyframe_pose.position);

            pcl::PointCloud<PointType>::Ptr transformed_local_map = boost::make_shared<pcl::PointCloud<PointType>>();
            // pcl::copyPointCloud(*local_map_ds, *cropped_local_map);
            pcl::transformPointCloudWithNormals<PointType>(*target_map, *transformed_local_map, transformation);

            sensor_msgs::msg::PointCloud2 msgs;
            // #ifndef __INTELLISENSE__ 
            pcl::toROSMsg(*transformed_local_map, msgs);
            // #endif
            // msgs.header.stamp = ros::Time().fromSec(time_new_cloud);
            msgs.header.stamp = time_new_cloud; 
            // msgs.header.frame_id = frame_id;
            msgs.header.frame_id = "lidar_odom";
            localcloud_pub->publish(msgs);
            // RCLCPP_INFO(get_logger(), "Local map published!");
            // global_cloud->clear();
        }




        pcl::PointCloud<PointType>::Ptr transformCloud(const pcl::PointCloud<PointType>::Ptr &cloudIn, PoseInfo * PointInfoIn)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut = boost::make_shared<pcl::PointCloud<PointType>>();;

            Eigen::Quaterniond quaternion(PointInfoIn->qw,
                                        PointInfoIn->qx,
                                        PointInfoIn->qy,
                                        PointInfoIn->qz);
            Eigen::Vector3d transition(PointInfoIn->x,
                                    PointInfoIn->y,
                                    PointInfoIn->z);

            int numPts = cloudIn->points.size();
            cloudOut->resize(numPts);

            for (int i = 0; i < numPts; ++i)
            {
                Eigen::Vector3d ptIn(cloudIn->points[i].x, cloudIn->points[i].y, cloudIn->points[i].z);
                Eigen::Vector3d ptOut = quaternion * ptIn + transition;

                Eigen::Vector3d normIn(cloudIn->points[i].normal_x, cloudIn->points[i].normal_y, cloudIn->points[i].normal_z);
                Eigen::Vector3d normOut = quaternion * normIn;

                PointType pt;
                pt.x = ptOut.x();
                pt.y = ptOut.y();
                pt.z = ptOut.z();
                pt.intensity = cloudIn->points[i].intensity;
                pt.curvature = cloudIn->points[i].curvature;
                pt.normal_x = normOut.x();
                pt.normal_y = normOut.y();
                pt.normal_z = normOut.z();

                cloudOut->points[i] = pt;
            }

            return cloudOut;
        }

        void publishTransformation()
        {
            Eigen::Quaterniond quarternion = last_odometry_transformation.rotation; 
            quarternion.normalize();
            Eigen::Vector3d translation = last_odometry_transformation.translation;


            transformation_geommsg.header.stamp = cloud_header.stamp;

            transformation_geommsg.pose.pose.orientation.w = quarternion.w();
            transformation_geommsg.pose.pose.orientation.x = quarternion.x();
            transformation_geommsg.pose.pose.orientation.y = quarternion.y();
            transformation_geommsg.pose.pose.orientation.z = quarternion.z();


            transformation_geommsg.pose.pose.position.x = translation.x();
            transformation_geommsg.pose.pose.position.y = translation.y();
            transformation_geommsg.pose.pose.position.z = translation.z();

            vector<double> cov_diag{translation_std_x*translation_std_x, translation_std_y* translation_std_y, translation_std_z * translation_std_z, 0.0,0.0,0.0};
            Eigen::MatrixXd cov_mat = createCovarianceEigen(cov_diag);
            Eigen::Matrix3d rot_mat(last_odometry_pose.orientation);
            rotateCovMatrix(cov_mat, rot_mat);
            setCovariance(transformation_geommsg.pose.covariance, cov_mat);

            // handle covariance depending on registration, eg. use deviation from guess 

            lidar_odometry_transformation_pub->publish(transformation_geommsg);

        }

        void rotateCovMatrix(Eigen::MatrixXd &cov_mat, Eigen::Matrix3d rot_mat)
        {
            Eigen::MatrixXd R(6,6);
            R.fill(0.0);
            R.block<3,3>(0,0) = rot_mat;
            R.block<3,3>(3,3) = rot_mat;

            // cov_mat = R.transpose() * cov_mat * R;
            cov_mat = R * cov_mat * R.transpose();
            // cov_mat.block<3,3>(0,0) = rot_mat.transpose() * cov_mat.block<3,3>(0,0) * rot_mat;
            // cov_mat.block<3,3>(3,3) = rot_mat * cov_mat.block<3,3>(3,3) * rot_mat.transpose();

            // for (int i =0; i<36; i++){
            //     if (cov_mat(i) < 1e-8){
            //         cov_mat(i) = 0.0;
            //     }
            // }
        }

        Eigen::MatrixXd createCovarianceEigen(vector<double> cov_diag)
        {
            Eigen::MatrixXd cov_mat(6,6);
            cov_mat.fill(0.0);
            for (int i=0 ; i < 6; i++){
                cov_mat(i*6 + i) = cov_diag[i];
            }

            return cov_mat;
        }

        void setCovariance(array<double, 36> &cov, vector<double> cov_diag)
        // assumes a 6x6 covariance array ie. size 36
        {
            for (int i=0 ; i < 6; i++){
                cov[i*6 + i] = cov_diag[i];
            }

        }

        void setCovariance(array<double, 36> &cov, Eigen::MatrixXd cov_mat)
        // assumes a 6x6 covariance array ie. size 36
        {
            for (int i=0 ; i < 36; i++){
                double value = cov_mat(i);
                if (value < 0.0) {
                    value = 0.0;
                }
                cov[i] = value; // no negative values
            }

        }

        void publishOdometry()
        {
            // PoseInfo latestPoseInfo = odometry_pose_info->points[latest_keyframe_idx - 1];
            PoseInfo latestPoseInfo = odometry_pose_info->points[odometry_pose_info->points.size() -1 ];
            odom.header.stamp = cloud_header.stamp;
            odom.pose.pose.orientation.w = latestPoseInfo.qw;
            odom.pose.pose.orientation.x = latestPoseInfo.qx;
            odom.pose.pose.orientation.y = latestPoseInfo.qy;
            odom.pose.pose.orientation.z = latestPoseInfo.qz;
            odom.pose.pose.position.x = latestPoseInfo.x;
            odom.pose.pose.position.y = latestPoseInfo.y;
            odom.pose.pose.position.z = latestPoseInfo.z;

            odom.twist.twist.linear.x = last_odometry_pose.velocity.x();
            odom.twist.twist.linear.y = last_odometry_pose.velocity.y();
            odom.twist.twist.linear.z = last_odometry_pose.velocity.z();

            vector<double> cov_diag{translation_std_x*translation_std_x, translation_std_y* translation_std_y, translation_std_z * translation_std_z, 0.001, 0.001, 0.001};
            Eigen::MatrixXd cov_mat = createCovarianceEigen(cov_diag);
            // Eigen::Matrix3d rot_mat = last_odometry_pose.block<3,3>(0,0);
            Eigen::Matrix3d rot_mat(Eigen::Quaterniond(latestPoseInfo.qw, latestPoseInfo.qx, latestPoseInfo.qy, latestPoseInfo.qz ));
            rotateCovMatrix(cov_mat, rot_mat);
            setCovariance(odom.pose.covariance, cov_mat);
            // set scan dt in the top right position (idx 5) of 6x6 covariance matrix
            odom.pose.covariance[5] = scan_dt;

            // odom.twist.twist.linear.x // add the velocities in twist
            odometry_pub->publish(odom);


            // odom -> base_link transform
            geometry_msgs::msg::TransformStamped t_;
            // t_.header.stamp = this->get_clock()->now();
            t_.header.stamp = cloud_header.stamp;
            t_.header.frame_id = "odom";//"lidar_odom";
            t_.child_frame_id = "base_link"; // "livox_frame"

            t_.transform.rotation.w = odom.pose.pose.orientation.w;
            t_.transform.rotation.x = odom.pose.pose.orientation.x;
            t_.transform.rotation.y = odom.pose.pose.orientation.y;
            t_.transform.rotation.z = odom.pose.pose.orientation.z;
            t_.transform.translation.x = odom.pose.pose.position.x;            
            t_.transform.translation.y = odom.pose.pose.position.y;  
            t_.transform.translation.z = odom.pose.pose.position.z;
            tf_broadcaster_->sendTransform(t_);


            geometry_msgs::msg::PoseStamped poseStamped;
            poseStamped.header = odom.header;
            poseStamped.pose = odom.pose.pose;
            poseStamped.header.stamp = odom.header.stamp;
            path.header.stamp = odom.header.stamp;
            path.poses.push_back(poseStamped);
            // path.header.frame_id = frame_id;
            path.header.frame_id = "odom"; // "livox_frame"
            path_pub->publish(path);
        }

        void publishINS(INSstate state)
        {
            // header etc has been set earlier
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.child_frame_id = "ins";
            odom_msg.header.frame_id = "odom";
            // data
            odom_msg.pose.pose.position.x = state.pos.x();
            odom_msg.pose.pose.position.y = state.pos.y();
            odom_msg.pose.pose.position.z = state.pos.z();

            odom_msg.pose.pose.orientation.w = state.ori.w();
            odom_msg.pose.pose.orientation.x = state.ori.x();
            odom_msg.pose.pose.orientation.y = state.ori.y();
            odom_msg.pose.pose.orientation.z = state.ori.z();

            odom_msg.twist.twist.linear.x = state.vel.x();
            odom_msg.twist.twist.linear.y = state.vel.y();
            odom_msg.twist.twist.linear.z = state.vel.z();

            odom_msg.twist.twist.angular.x = state.ang.x();
            odom_msg.twist.twist.angular.y = state.ang.y();
            odom_msg.twist.twist.angular.z = state.ang.z();

            
            // vector<double> cov_diag{0.02,0.02,0.02, 0.0001, 0.0001, 0.0001};
            // // vector<double> cov_diag{0.0000001,0.000001,0.000001, 0.000001, 0.000001, 0.0000001};
            // vector<double> cov_diag_twist{0.0025,0.0025,0.0025, 0.001, 0.001, 0.001};
            // Eigen::MatrixXd cov_mat = createCovarianceEigen(cov_diag);
            // Eigen::MatrixXd cov_twist = createCovarianceEigen(cov_diag_twist);
            // Eigen::Matrix3d rot_mat = last_odometry_pose.block<3,3>(0,0);
            // Eigen::Matrix3d rot_mat(Eigen::Quaterniond(latestPoseInfo.qw, latestPoseInfo.qx, latestPoseInfo.qy, latestPoseInfo.qz ));
            // rotateCovMatrix(cov_mat, rot_mat);
            // setCovariance(odom_msg.pose.covariance, cov_mat);
            // setCovariance(odom_msg.twist.covariance, cov_twist);
            ins_pub->publish(odom_msg);

            geometry_msgs::msg::PoseStamped poseStamped;
            poseStamped.header = odom_msg.header;
            poseStamped.pose = odom_msg.pose.pose;
            poseStamped.header.stamp = odom_msg.header.stamp;
            path_ins.header.stamp = odom_msg.header.stamp;
            path_ins.poses.push_back(poseStamped);
            // path.header.frame_id = frame_id;
            path_ins.header.frame_id = "odom";
            path_ins_pub->publish(path_ins);

        }

        void publishPredictedINS(INSstate state)
        {
            // header etc has been set earlier
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.child_frame_id = "ins";
            odom_msg.header.frame_id = "odom";
            // data
            odom_msg.pose.pose.position.x = state.pos.x();
            odom_msg.pose.pose.position.y = state.pos.y();
            odom_msg.pose.pose.position.z = state.pos.z();

            odom_msg.pose.pose.orientation.w = state.ori.w();
            odom_msg.pose.pose.orientation.x = state.ori.x();
            odom_msg.pose.pose.orientation.y = state.ori.y();
            odom_msg.pose.pose.orientation.z = state.ori.z();

            odom_msg.twist.twist.linear.x = state.vel.x();
            odom_msg.twist.twist.linear.y = state.vel.y();
            odom_msg.twist.twist.linear.z = state.vel.z();

            odom_msg.twist.twist.angular.x = state.ang.x();
            odom_msg.twist.twist.angular.y = state.ang.y();
            odom_msg.twist.twist.angular.z = state.ang.z();

            
            // vector<double> cov_diag{0.02,0.02,0.02, 0.0001, 0.0001, 0.0001};
            // // vector<double> cov_diag{0.0000001,0.000001,0.000001, 0.000001, 0.000001, 0.0000001};
            // vector<double> cov_diag_twist{0.0025,0.0025,0.0025, 0.001, 0.001, 0.001};
            // Eigen::MatrixXd cov_mat = createCovarianceEigen(cov_diag);
            // Eigen::MatrixXd cov_twist = createCovarianceEigen(cov_diag_twist);
            // Eigen::Matrix3d rot_mat = last_odometry_pose.block<3,3>(0,0);
            // Eigen::Matrix3d rot_mat(Eigen::Quaterniond(latestPoseInfo.qw, latestPoseInfo.qx, latestPoseInfo.qy, latestPoseInfo.qz ));
            // rotateCovMatrix(cov_mat, rot_mat);
            // setCovariance(odom_msg.pose.covariance, cov_mat);
            // setCovariance(odom_msg.twist.covariance, cov_twist);
            // ins_pub->publish(odom_msg);

            geometry_msgs::msg::PoseStamped poseStamped;
            poseStamped.header = odom_msg.header;
            poseStamped.pose = odom_msg.pose.pose;
            poseStamped.header.stamp = odom_msg.header.stamp;
            path_predicted_ins.header.stamp = odom_msg.header.stamp;
            path_predicted_ins.poses.push_back(poseStamped);
            // path.header.frame_id = frame_id;
            path_predicted_ins.header.frame_id = "odom";

            path_predicted_ins_pub->publish(path_predicted_ins);

        }

        void publishKeyframeOdometry()
        {
            // PoseInfo latestPoseInfo = odometry_pose_info->points[latest_keyframe_idx - 1];
            PoseInfo latestPoseInfo = keyframe_pose_info->points[keyframe_pose_info->points.size() - 1 ];
            // odom.header.stamp = keyframe_pose_info.time;
            odom.pose.pose.orientation.w = latestPoseInfo.qw;
            odom.pose.pose.orientation.x = latestPoseInfo.qx;
            odom.pose.pose.orientation.y = latestPoseInfo.qy;
            odom.pose.pose.orientation.z = latestPoseInfo.qz;
            odom.pose.pose.position.x = latestPoseInfo.x;
            odom.pose.pose.position.y = latestPoseInfo.y;
            odom.pose.pose.position.z = latestPoseInfo.z;

            odom.twist.twist.linear.x = keyframe_pose.velocity.x();
            odom.twist.twist.linear.y = keyframe_pose.velocity.y();
            odom.twist.twist.linear.z = keyframe_pose.velocity.z();

            

            // odom.twist.twist.linear.x // add the velocities in twist

            keyframe_odometry_pub->publish(odom);


            // // odom -> base_link transform
            // geometry_msgs::msg::TransformStamped t_;
            // // t_.header.stamp = this->get_clock()->now();
            // t_.header.stamp = cloud_header.stamp;
            // t_.header.frame_id = "odom";//"lidar_odom";
            // t_.child_frame_id = "base_link"; // "livox_frame"

            // t_.transform.rotation.w = odom.pose.pose.orientation.w;
            // t_.transform.rotation.x = odom.pose.pose.orientation.x;
            // t_.transform.rotation.y = odom.pose.pose.orientation.y;
            // t_.transform.rotation.z = odom.pose.pose.orientation.z;
            // t_.transform.translation.x = odom.pose.pose.position.x;            
            // t_.transform.translation.y = odom.pose.pose.position.y;  
            // t_.transform.translation.z = odom.pose.pose.position.z;
            // tf_broadcaster_->sendTransform(t_);


            // geometry_msgs::msg::PoseStamped poseStamped;
            // poseStamped.header = odom.header;
            // poseStamped.pose = odom.pose.pose;
            // poseStamped.header.stamp = odom.header.stamp;
            // path.header.stamp = odom.header.stamp;
            // path.poses.push_back(poseStamped);
            // // path.header.frame_id = frame_id;
            // path.header.frame_id = "odom"; // "livox_frame"
            // path_pub->publish(path);
        }

        // double toSec(builtin_interfaces::msg::Time header_stamp)
        // {
        //     rclcpp::Time time = header_stamp;
        //     double nanoseconds = time.nanoseconds();

        //     return nanoseconds * 1e-9;
        // }

        builtin_interfaces::msg::Time toStamp(double double_stamp)
        {   
            builtin_interfaces::msg::Time header_stamp;
            header_stamp.set__sec((int)double_stamp);
            header_stamp.set__nanosec((int)(double_stamp - (int)double_stamp)*1e9);
            return header_stamp;
        }

        bool getNextInBuffer()
        {
            // get next frame in buffer..
            if (cloud_queue.size() <= 0) { 
                return false; 
            } else {
                current_cloud_msg = cloud_queue.front(); // puts the next cloud to variable current
                cloud_queue.pop_front(); // removes element from the queue

                // set header id and timestamp
                cloud_header = current_cloud_msg.header; 
                cloud_header.frame_id = frame_id;
    
                time_new_cloud = current_cloud_msg.header.stamp;
                current_scan_time = toSec(time_new_cloud) + sync_offset_ms_ *1e-3;
                // next_scan_time = toSec(cloud_queue.front().header.stamp) + sync_offset_ms_ *1e-3; // obs this is not necassarily just current time + scan_dt due to holes in recordings

                if ( start_delay_s_ > (current_scan_time - first_lidar_msg_time)){
                    RCLCPP_INFO(get_logger(), "Buffer time: %f", current_scan_time - first_lidar_msg_time);
                    return false;
                }

                if (cloud_queue.size() > 5) {
                    RCLCPP_WARN(get_logger(), "Buffer size is: %i", cloud_queue.size());
                }
            } 

            // cloud_header = pcl_msg->header;
            fromROSMsg(current_cloud_msg, *cloud_in);
            
            // this scan dt is validated in the preprocesser why it is pulled from the last points intensity value
            scan_dt = cloud_in->points[cloud_in->points.size() -1].intensity - (int)cloud_in->points[cloud_in->points.size() -1].intensity;
            
            RCLCPP_INFO(get_logger(), "Processing frame index: %i, scan timestamp: %f dt: %f", latest_frame_idx, current_scan_time, scan_dt);
            
            // rotate to body frame
            // pcl::transformPointCloudWithNormals<PointType>(*cloud_in, *cloud_in, last_odometry_pose.position, last_odometry_pose.orientation);

            return true;
        }




        void run()
        {
            if (!getNextInBuffer()){
                return;
            }


            rclcpp::Clock frame_clock;
            rclcpp::Clock missing_time_clock;
            rclcpp::Time time_frame_start = frame_clock.now();

            double undistort_time{};

            latest_frame_idx++;
            // RCLCPP_INFO(get_logger(), "Frame idx: %i", latest_frame_idx);

            if (latest_frame_idx < (size_t)start_idx_ || (latest_frame_idx > (size_t)end_idx_ && end_idx_ > 0) ){
                RCLCPP_INFO(get_logger(), "Skipping frames not in interval %i to %i", start_idx_, end_idx_);
                return;
            }

            if (!system_initialized){
                // save first pose and point cloud and initialize the system
                initializeSystem();
                // buildLocalMapAroundKeyFrame(); // only uses the first cloud for map.
                RCLCPP_INFO(get_logger(), "LOAM system is initialized.");
                return;
            }

            if (local_map_init_frames_count_ > -1 && !init_map_built){
                if (keyframe_poses.size() < size_t(local_map_init_frames_count_)+1 ) {
                    RCLCPP_INFO(get_logger(), "Adding frame %i of %i to initial local map by assuming no movement", keyframe_poses.size(), local_map_init_frames_count_);
                    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock() , 1000, "Building initial local map by assuming no movement: frame %i", keyframe_poses.size());
                    // last_odometry_pose.time = current_scan_time;
                    

                    if (use_preint_undistortion_){
                        // updateOdometryPose(registration_transformation);
                        // preintegrateIMU();
                        // calcState0();


                        ins.preintegrateIMU(last_odometry_pose.time, current_scan_time + scan_dt);
                        // ins.preintegrateIMU(last_odometry_pose.time, next_scan_time);
                        publishINSstates();
                        ins.calcLidarSyncedState(current_scan_time);

                        ins.predictionIntegrate(0.1, 10); // input is step dt and # steps TODO: makes these ros params
                        publishPredictedINSstates();

                        preint_transformation_guess = ins.getPreintegratedTransformationGuess();
                        RCLCPP_INFO(get_logger(), "preint_transformation_guess rotation: %f %f %f %f", preint_transformation_guess.rotation.w(), preint_transformation_guess.rotation.x(), preint_transformation_guess.rotation.y(), preint_transformation_guess.rotation.z());
                        Transformation calib_transformation;

                        calib_transformation.translation = -(preint_transformation_guess.translation); // might need to rotate this vector
                        // calib_transformation.translation = Eigen::Vector3d::Zero();
                        calib_transformation.rotation = preint_transformation_guess.rotation.inverse()* initial_pose.orientation;
                        // calib_transformation.rotation = Eigen::Quaterniond::Identity();
                        calib_transformation.matrix.block<3,3>(0,0) = calib_transformation.rotation.normalized().matrix();
                        calib_transformation.matrix.block<3,1>(0,3) = calib_transformation.translation;
                        
                        // updateOdometryPose(calib_transformation);  // registration is just an identity transform here, so no actual transform but it calculates a residual
                        last_odometry_pose.time = current_scan_time;

                        preint_anchor = ins.getPreintAnchor();
                        simpleConstraningObserverUpdate(preint_transformation_guess, preint_anchor);
                        ins.setBias(observer_state.bias);
                        ins.resetPreintAnchor(); // zeroes the preintergration position as we are assuming no movement
                    }

                    pcl::transformPointCloudWithNormals<PointType>(*cloud_in, *cloud_in, keyframe_pose.matrix);
                    savePose();
                    // pcl::transformPointCloudWithNormals(*cloud_in, *cloud_in, keyframe_pose.position.cast<float>(), keyframe_pose.orientation.cast<float>());
                    downSampleClouds();
                    publishCurrentCloud();
                    pushKeyframe();

                    
                    publishOdometry(); 
                    // buildLocalMapAroundKeyFrame();

                    addToLocalMap();
                    
                    saveRunningData();
                    *target_map = *local_map_ds;
                    publishLocalMap();
                    return;
                }
                // if (keyframe_poses.size() >= size_t(local_map_init_frames_count_)) {
                RCLCPP_INFO(get_logger(), "Initial local map built!");
                // buildLocalMapAroundKeyFrame();
                publishLocalMap();
                // registration_->setInputTarget(local_map_ds);
                setRegistrationTarget(local_map_ds);
                init_map_built = true;
            }

            rclcpp::Time time_missing_start = frame_clock.now();

            if (use_preint_undistortion_){
                ins.preintegrateIMU(last_odometry_pose.time, current_scan_time + scan_dt); 

                publishINSstates();
                ins.calcLidarSyncedState(current_scan_time); 

                ins.predictionIntegrate(0.1, 10); // input is step dt and # steps
                publishPredictedINSstates();
                
                preint_transformation_guess = ins.getPreintegratedTransformationGuess();
                rclcpp::Time undistort_start_time = frame_clock.now();
                ins.undistortCloud(*cloud_in, *cloud_in); // this also transforms the cloud into the guess of the transformation ie. the imu preintegrated pose
                rclcpp::Time undistort_end_time = frame_clock.now();
                undistort_time = undistort_end_time.seconds()*1000.0 - undistort_start_time.seconds()*1000.0; 
                // preint_transformation_guess = ins.getPreintegratedTransformationGuess(); // testing this
            
            } else {
                // undistortCloudInterpolated(cloud_in, cloud_in, last_odometry_transformation.rotation, last_odometry_transformation.translation);
                
                pcl::transformPointCloudWithNormals<PointType>(*cloud_in, *cloud_in, keyframe_pose.matrix);
                // undistortCloudInterpolated(cloud_in, cloud_in, last_odometry_transformation.rotation, Eigen::Vector3d::Zero());
            }
            

            downSampleClouds();

            calculateInitialTransformationGuess();
            rclcpp::Time time_missing_end = frame_clock.now();
            // RCLCPP_INFO(get_logger(), "what about here?");

            // do icp with last key frame or  local map
            scanMatchRegistration(cloud_in_ds); // TODO: make a check for degenerate estimated transformation 
            // RCLCPP_INFO(get_logger(), "did ICP happen?!...");
            
            updateOdometryPose(registration_transformation); // if using preintegrated undistortion the transformation found here is the delta between the guess and the "actual" pose
            savePose(); 

            publishCurrentCloud();
            publishCurrentFullCloud(); // this is sent to map backend
            
            // savePointCloud(); // testing this here
            publishOdometry(); 
            // publishTransformation();


            if (use_preint_undistortion_){
                
                preint_anchor = ins.getPreintAnchor();
                simpleConstraningObserverUpdate(preint_transformation_guess, preint_anchor);
                ins.setPreintAnchor(preint_anchor);
                ins.setBias(observer_state.bias);
            }

            
            if (newKeyframeRequired()) { // need a check that the new solution is not degenerate or divergent and therefore triggers the geometric conditions

                pushKeyframe();
                // pushLastTransformationToConstVelocityModel(); // is not needed?
                // savePointCloud();


                if (local_map_width_ > 0) {
                    trimLocalMap();  // if the local map is too "wide"
                    buildLocalMapAroundKeyFrame();
                    // fuseLocalLongTermMap();

                    *target_map = *local_map_ds;
                } else { // if width is set zero it means infinite length
                    addToLocalMap();
                    *target_map = *local_map_ds;
                }

                
                // registration_->setInputTarget(target_map);
                setRegistrationTarget(target_map);
                publishLocalMap();
                // publishLocalMap(keyframe_pose.matrix.inverse());
                    
            }
            // } else { // keyframe rejection ei. 
            // }
            
            saveRunningData();

            rclcpp::Time time_frame_end = frame_clock.now();
            double frame_process_time = time_frame_end.seconds()*1000.0 - time_frame_start.seconds()*1000.0; 
            double mystery_time = time_missing_end.seconds()*1000.0 - time_missing_start.seconds()*1000.0; 

            double total_run_time = run_clock.now().seconds() - system_start_time;

            RCLCPP_INFO(get_logger(), " -- Frame process time: This: %f ms Average: %f ms ---  Reg. difference: %f ms", frame_process_time, total_run_time/(double)latest_frame_idx *1000.0 ,  frame_process_time - registration_process_time );
            // RCLCPP_INFO(get_logger(), " -- Frame process time ---  %f ms --- difference --- %f ms", frame_process_time, frame_process_time - registration_process_time );
            RCLCPP_INFO(get_logger(), " -- mystery time:  %f ms --- Undistortion time: %f ms" , mystery_time, undistort_time);
            RCLCPP_INFO(get_logger(), " -- Processed time: %f s -- Total run Time: %f s" , current_scan_time - first_lidar_msg_time ,total_run_time);


        }



};


int main(int argc, char ** argv){

    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto lidar_odometry_node = std::make_shared<LidarOdometry>();
    executor.add_node(lidar_odometry_node);


    // rclcpp::spin(std::make_shared<LidarOdometry>());
    executor.spin();
    rclcpp::shutdown();
    return 0;


}