#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "DebugTools/DebugTools.h"//这个必须放这里
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

    bool GetLaserInBase(const sensor_msgs::LaserScan::ConstPtr &scan_msg,Eigen::Matrix4d& T_laser_in_base)
    {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        try
        {
            geometry_msgs::TransformStamped tf_laser_in_base = 
                    tfBuffer.lookupTransform("base_footprint", "laser", scan_msg->header.stamp,ros::Duration(1.0));
            Eigen::Quaterniond q(tf_laser_in_base.transform.rotation.w,
                                 tf_laser_in_base.transform.rotation.x,
                                 tf_laser_in_base.transform.rotation.y,
                                 tf_laser_in_base.transform.rotation.z) ;
            T_laser_in_base.block<3,3>(0,0) = q.toRotationMatrix();
            T_laser_in_base(0,3) = tf_laser_in_base.transform.translation.x;
            T_laser_in_base(1,3) = tf_laser_in_base.transform.translation.y;
            T_laser_in_base(2,3) = tf_laser_in_base.transform.translation.z;
            return true;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("%s",
                    ex.what());
            return false;
        }
    }
}


ScanOdom::ScanOdom()
{
    ros::NodeHandle node_handle;
    ros::NodeHandle pnode_handle("~");
    m_scan_sub = node_handle.subscribe(
        "/scan", 1, &ScanOdom::ScanCallback, this);
    scan_regis_base = ScanRegisFactory::CreateScanRegisMethod(ScanRegisFactory::PLICP);
    m_T_laser_in_base = Eigen::Matrix4d::Identity();
    m_T_baseKF_in_odom = Eigen::Matrix4d::Identity();
    m_T_baseNKF_in_odom = Eigen::Matrix4d::Identity();
    m_T_laserG_in_base = m_T_laser_in_base;
    m_T_laserG_in_base.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
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
        scan_regis_base->Init(scan_msg);//这些行为后续可能也会出现判断情况
        scan_regis_base->UpdateRefScan(scan_msg);//这些行为后续可能也会出现判断情况
        bool b_tf_flag = GetLaserInBase(scan_msg,m_T_laser_in_base);//这些行为后续可能也会出现判断情况


        if(b_tf_flag)
        {
            scan_odom_status = Initialzed;
        }
        return ;
    }
    else if(scan_odom_status == Initialzed)
    {
        double cur_time = scan_msg->header.stamp.toSec();
        Eigen::Vector3d v3_predict_motion;
        pose_extrapolator.Prediction(cur_time,v3_predict_motion);//这部分与关键帧机制相关

        Eigen::Vector3d v3_regis_motion;
        bool regis_successed = scan_regis_base->ScanMatch(scan_msg,v3_predict_motion,v3_regis_motion);

        if (regis_successed)
        {
            Eigen::Matrix4d T_laserNKFG_in_laserKFG;

            VectorToTranform(v3_regis_motion,T_laserNKFG_in_laserKFG);

            m_T_baseNKF_in_odom = m_T_baseKF_in_odom*m_T_laserG_in_base*
                                    T_laserNKFG_in_laserKFG*
                                    m_T_laserG_in_base.inverse();

            //debug                 
            debug_tools.PublishTF(m_T_baseNKF_in_odom,scan_msg->header.stamp,"odom","base_footprint");
            debug_tools.PublishPath(m_T_baseNKF_in_odom,scan_msg->header.stamp,"odom");
            debug_tools.WritePath(m_T_baseNKF_in_odom,cur_time);
            ////

            if(IsNewKeyframe(T_laserNKFG_in_laserKFG))
            {
                pose_extrapolator.UpdateVelocity(cur_time,v3_regis_motion);//这部分与关键帧机制相关
                scan_regis_base->UpdateRefScan(scan_msg);
                m_T_baseKF_in_odom = m_T_baseNKF_in_odom;
            }
        }
        return ;
    }
}

bool ScanOdom::IsNewKeyframe(const Eigen::Matrix4d& T_base_in_kf)
{
    static int scan_count_ = 0;
    scan_count_++;

    if (T_base_in_kf.block<3,3>(0,0).eulerAngles(2,1,0)(0) > 0.0523)
        return true;

    if (scan_count_ == 10)
    {
        scan_count_ = 0;
        return true;
    }
        
    double x = T_base_in_kf(0,3);
    double y = T_base_in_kf(1,3);
    if (x * x + y * y > 1)
        return true;
    return false;
    //return true;
}

