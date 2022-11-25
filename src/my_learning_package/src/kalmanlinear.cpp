#include <cstdio>
#include <cmath>
#include <Eigen>

// #include "pcl_conversions/pcl_conversions.h"
// #include "pcl/registration/icp.h"



// typedef pcl::PointXYZINormal PointType;

// pcl::PointCloud<PointType>::Ptr pcl;


// attempt at a IMU jerk filter.
using namespace Eigen;

class KalmanLinear{

    public: 
        KalmanLinear()
        {
            dt = 1.0/400;
            x << 0,0,0,0,0,0;
            x_post << 0,0,0,0,0,0;
            
            updateF();
            z << 0,0,0;


            R = MatrixXd::Identity(3,3) * 1;


            H << 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0,
                 1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0;

            double q = 0.01;
            Q << q, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, q, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, q, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, q, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, q, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, q;

            p = 1.0;
            P << p, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, p, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, p, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, p, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, p, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, p;

            MatrixXd P_post = P;
            
            K = MatrixXd::Zero(3,6);
                 

        }

        void setdt(double dt)
        {
            this->dt = dt;
        }

        void update()
        {   
            // update kalman gain
            updateK();
            // x = 
            // posterior

            // z = 0,0,0 reset measurement 
            this->z << 0.0, 0.0, 0.0;
        }

        void addMeasurement(double x, double y, double z){
            this->z << x, y, z;
        }

    private:
        void updateK()
        {
            MatrixXd PHT = P * H.transpose();
            K = PHT * (H * PHT + R).inverse();
        }
        
        void updateP()
        {
            P = F * P * F.transpose() + Q;
        }

        void updateF() // update according to dt
        {
            F << 1.0, 0.0, 0.0, dt, 0.0, 0.0,
                 0.0, 1.0, 0.0, 0.0, dt, 0.0,
                 0.0, 0.0, 1.0, 0.0, 0.0, dt,
                 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
        }

        
        double dt;
        double p;
        VectorXd x;  // state vector [ax,ay,az, jx, jy, jz]
        VectorXd x_post;  // state vector [ax,ay,az, jx, jy, jz]
        MatrixXd F; // state transition matrix dim.x x dim.x
        MatrixXd R; // measurement noise dim.z x dim.z
        MatrixXd Q; // process noise dim.x x dim.x
        MatrixXd H; // measurement sensitivity amtrix dim.z x dim.x
        MatrixXd P; // covariance of state process i.e. error of prediction 
        MatrixXd P_post; // covariance of state process i.e. error of prediction 
        MatrixXd K; // Kalman gain matrix 
        Vector3d z; // measurement
};


