#pragma once
#ifndef INS_H
#define INS_H


#include "my_learning_package/utils/common.hpp"
#include <Eigen/Dense>



class INS{
    public:
    INS();
    ~INS();

    

    private:
    Pose initial_pose;
    Pose current_pose;
    Pose previous_pose;

    INSstate preint_anchor;
    INSstate preint_state;
    deque<INSstate> preint_states;
    
    
    
    deque<double> imu_dts;
    
    
    
    
    
    
    public:
    void resetPreintAnchor();
    void setPreintAnchor(const INSstate state);
    void getPreintAnchor(INSstate &state); 

    void updateInitialPose(const Pose initial_pose);

};








#endif // INS_H