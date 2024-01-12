
#include "my_learning_package/ins.hpp"





INS::INS(){
    
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
    // preint_anchor.vel  = state.vel ;
    preint_anchor.vel   = state.ori.matrix().inverse() * state.vel ; // saved in local frame
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

void INS::updateInitialPose(const Pose initial_pose)
{
    // make something that averages the incoming poses??
    this->initial_pose = initial_pose;
}





// INS::set