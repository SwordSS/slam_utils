#ifndef _POSEEXTRAPOLATOR_H
#define _POSEEXTRAPOLATOR_H

#include <Eigen/Eigen>
#include <iostream>

class PoseExtrapolator
{
private:
    enum PoseExtrapolatorStatus
    {
        Initialzing,
        Initialzed
    };    
public:
    PoseExtrapolator(){
        last_velocity = Eigen::Vector3d::Zero();
        pose_extrapolator_status = Initialzing;
        last_time = 0;
        
    };
    ~PoseExtrapolator(){};
    void Prediction(double cur_time,Eigen::Vector3d& v3_predict_motion);
    void UpdateVelocity(double cur_time,const Eigen::Vector3d& v3_regis_motion);

private:
    PoseExtrapolatorStatus pose_extrapolator_status;
    Eigen::Vector3d last_velocity;
    double last_time;
}; 

#endif