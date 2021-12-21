#include "DebugTools/DebugTools.h"
#include "ScanOdom/ScanOdom.h"

//debug
DebugTools debug_tools;
////

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

    //debug
    debug_tools.Init();
    ////
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

            //debug                 
            debug_tools.PublishTF(m_T_base_in_odom,scan_msg->header.stamp,"odom","base_footprint");
            debug_tools.PublishPath(m_T_base_in_odom,scan_msg->header.stamp,"odom");
            debug_tools.WritePath(m_T_base_in_odom,cur_time);
            ////

            scan_regis_base->UpdateRefScan(scan_msg);
        }
        return ;
    }
}