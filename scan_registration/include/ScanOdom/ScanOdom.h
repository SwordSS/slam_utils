
#ifndef _SCANODOM_H
#define _SCANODOM_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <iostream>
#include <string>
#include <memory>
#include <Eigen/Eigen>

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

    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);//后续考虑隔离ROS
    void PublishTFAndOdometry();

private:
    ros::Subscriber m_scan_sub;
    ros::Publisher  m_odom_pub;

    std::shared_ptr<ScanRegisBase> scan_regis_base;
    PoseExtrapolator pose_extrapolator;
    ScanOdomStatus scan_odom_status;

    sensor_msgs::LaserScan::ConstPtr last_scan_msg;

    Eigen::Matrix4d m_T_base_in_odom;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    
};



#endif