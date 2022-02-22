
#ifndef _SCANODOM_H
#define _SCANODOM_H

#include <iostream>
#include <memory>
#include <Eigen/Eigen>


#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "SliceDeque/SliceDeque.h"
#include "ScanRegis/ScanRegisBase.h"
#include "ScanRegis/ScanRegisFactory.h"
#include "PoseExtrapolator/PoseExtrapolator.h"



class ScanOdom
{
public:
    //note 后续可以参考一下ORBSLAM2
    enum ScanOdomStatus
    {
        Initialzing,
        Initialzed
    };

public:
    ScanOdom();
    ~ScanOdom(){};

    void Run();//后续考虑隔离ROS
    void PublishTFAndOdometry();

private:
    bool IsNewKeyframe(const Eigen::Matrix4d& T_base_in_kf);

private:
    ros::Subscriber m_scan_sub;
    ros::Publisher  m_odom_pub;
    SliceDeque m_slice_queue;
    std::thread m_thread_run;

    std::shared_ptr<ScanRegisBase> scan_regis_base;
    PoseExtrapolator pose_extrapolator;
    ScanOdomStatus scan_odom_status;

    sensor_msgs::LaserScanConstPtr last_scan_msg;

    Eigen::Matrix4d m_T_baseKF_in_odom;
    Eigen::Matrix4d m_T_baseNKF_in_odom;
    Eigen::Matrix4d m_T_laserG_in_base;

};



#endif