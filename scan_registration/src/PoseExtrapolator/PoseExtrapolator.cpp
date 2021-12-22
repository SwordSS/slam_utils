#include "PoseExtrapolator/PoseExtrapolator.h"


void PoseExtrapolator::Prediction(double cur_time,Eigen::Vector3d& v3_predict_motion)
{
    if(pose_extrapolator_status==Initialzing)
    {
        v3_predict_motion = Eigen::Vector3d::Zero();
        return;
    }
    else if(pose_extrapolator_status==Initialzed)
    {
        v3_predict_motion = last_velocity*(cur_time - last_time);
        return;
    }
}

void PoseExtrapolator::UpdateVelocity(double cur_time,const Eigen::Vector3d& v3_regis_motion)
{
    if(pose_extrapolator_status==Initialzing)
    {
        last_velocity = Eigen::Vector3d::Zero();
        last_time = cur_time;
        pose_extrapolator_status = Initialzed;
        return;
    }
    else if(pose_extrapolator_status==Initialzed)
    {
        last_velocity = v3_regis_motion/(cur_time - last_time);
        last_time = cur_time;
        return;
    }
}