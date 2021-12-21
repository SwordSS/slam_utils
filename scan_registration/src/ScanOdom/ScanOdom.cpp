
#include "ScanOdom/ScanOdom.h"

namespace
{
    void VectorToTranform(const Eigen::Vector3d& v3_input,Eigen::Matrix4d& T_output)
    {
        T_output = Eigen::Matrix4d::Identity();
        Eigen::AngleAxisd rotation_vector(v3_input[2],Eigen::Vector3d(0,0,1));
        T_output.block<3,3>(0,0) = rotation_vector.matrix();
        T_output(0,3) = v3_input(0);
        T_output(1,3) = v3_input(1);
    }
}


ScanOdom::ScanOdom()
{
    ros::NodeHandle node_handle;
    ros::NodeHandle pnode_handle("~");
    m_scan_sub = node_handle.subscribe(
        "/scan", 1, &ScanOdom::ScanCallback, this);
    scan_regis_base = ScanRegisFactory::CreateScanRegisMethod(ScanRegisFactory::PLICP);
    m_T_base_in_odom = Eigen::Matrix4d::Identity();
    scan_odom_status = Initialzing;
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
        double cur_time = scan_msg->header.stamp.toSec();
        Eigen::Vector3d v3_predict_motion;
        pose_extrapolator.Prediction(cur_time,v3_predict_motion);

        Eigen::Vector3d v3_regis_motion;
        bool regis_successed = scan_regis_base->ScanMatch(scan_msg,v3_predict_motion,v3_regis_motion);

        if (regis_successed)
        {
            pose_extrapolator.UpdateVelocity(cur_time,v3_regis_motion);
            Eigen::Matrix4d T_regis_motion;
            VectorToTranform(v3_regis_motion,T_regis_motion);
            m_T_base_in_odom = m_T_base_in_odom * T_regis_motion;                 
            PublishMsg(m_T_base_in_odom,scan_msg->header.stamp);
            scan_regis_base->UpdateRefScan(scan_msg);
        }
        return ;
    }
}

void ScanOdom::PublishMsg(const Eigen::Matrix4d& T_base_in_odom,ros::Time scan_time)
{
    geometry_msgs::TransformStamped tf_odom_to_base;
    Eigen::Quaterniond q_v = Eigen::Quaterniond(T_base_in_odom.block<3,3>(0,0));
    tf_odom_to_base.header.stamp = scan_time;
    tf_odom_to_base.header.frame_id = "odom";
    tf_odom_to_base.child_frame_id = "laser";
    tf_odom_to_base.transform.translation.x = T_base_in_odom(0,3);
    tf_odom_to_base.transform.translation.y = T_base_in_odom(1,3);
    tf_odom_to_base.transform.translation.z = 0;
    tf_odom_to_base.transform.rotation.x = q_v.x();
    tf_odom_to_base.transform.rotation.y = q_v.y();
    tf_odom_to_base.transform.rotation.z = q_v.z();
    tf_odom_to_base.transform.rotation.w = q_v.w();

    tf_broadcaster.sendTransform(tf_odom_to_base);

}