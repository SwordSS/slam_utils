
#include "ScanOdom/ScanOdom.h"

namespace
{
    void VectorToTranform(const Eigen::Vector3d& input,Eigen::Matrix4d& output)
    {
        output = Eigen::Matrix4d::Identity();
        Eigen::AngleAxisd rotation_vector(input[2],Eigen::Vector3d(0,0,1));
        output.block<3,3>(0,0) = rotation_vector.matrix();
        output(3,0) = input(0);
        output(3,1) = input(1);
    }
}


ScanOdom::ScanOdom()
{
    ros::NodeHandle node_handle;
    ros::NodeHandle pnode_handle("~");
    m_scan_sub = node_handle.subscribe(
        "/scan", 1, &ScanOdom::ScanCallback, this);
    scan_regis_base = ScanRegisFactory::CreateScanRegisMethod(ScanRegisFactory::PLICP);
    m_base_in_odom = Eigen::Matrix4d::Identity();
    scan_odom_status = Initialzing;
    running_flag = 1;
}

void ScanOdom::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    if(scan_odom_status==Initialzing)
    {
        last_scan_msg = scan_msg;
        scan_regis_base->Init(scan_msg);
        scan_regis_base->UpdateRefScan(scan_msg);
        scan_odom_status = Initialzed;
        return ;
    }
    else if(scan_odom_status == Initialzed)
    {
        Eigen::Vector3d predict_motion;
        Prediction(scan_msg,predict_motion);

        Eigen::Vector3d real_motion;
        bool result_flag = scan_regis_base->ScanMatch(scan_msg,predict_motion,real_motion);

        if (result_flag)
        {
            UpdateVelocity(scan_msg,real_motion);
            Eigen::Matrix4d robot_motion;
            VectorToTranform(real_motion,robot_motion);
            m_base_in_odom = m_base_in_odom * robot_motion;
            PublishMsg(m_base_in_odom,scan_msg->header.stamp);
            scan_regis_base->UpdateRefScan(scan_msg);
        }
        return ;
    }
}

void ScanOdom::PublishMsg(const Eigen::Matrix4d& real_motion,ros::Time scan_time)
{
    geometry_msgs::TransformStamped tf_odom_to_base;
    Eigen::Quaterniond q_v = Eigen::Quaterniond(real_motion.block<3,3>(0,0));
    tf_odom_to_base.header.stamp = scan_time;
    tf_odom_to_base.header.frame_id = "odom";
    tf_odom_to_base.child_frame_id = "laser";
    tf_odom_to_base.transform.translation.x = real_motion(3,0);
    tf_odom_to_base.transform.translation.y = real_motion(3,1);
    tf_odom_to_base.transform.translation.z = 0;
    tf_odom_to_base.transform.rotation.x = q_v.x();
    tf_odom_to_base.transform.rotation.y = q_v.y();
    tf_odom_to_base.transform.rotation.z = q_v.z();
    tf_odom_to_base.transform.rotation.w = q_v.w();

    tf_broadcaster.sendTransform(tf_odom_to_base);

}

void ScanOdom::Prediction(const sensor_msgs::LaserScan::ConstPtr &scan_msg,Eigen::Vector3d& predict_motion)
{
    double cur_time = scan_msg->header.stamp.toSec();
    if(running_flag==0)
    {
        predict_motion = Eigen::Vector3d::Zero();
        return;
    }
    else if(running_flag==1)
    {
        predict_motion = last_velocity*(cur_time - last_time);
    }
}

void ScanOdom::UpdateVelocity(const sensor_msgs::LaserScan::ConstPtr &scan_msg,const Eigen::Vector3d& real_motion)
{
    double cur_time = scan_msg->header.stamp.toSec();
    if(running_flag==0)
    {
        last_velocity = Eigen::Vector3d::Zero();
        last_time = scan_msg->header.stamp.toSec();
        running_flag = 1;
    }
    else if(running_flag==1)
    {
        last_velocity = real_motion/(cur_time - last_time);
        last_time = scan_msg->header.stamp.toSec();
    }
    

}