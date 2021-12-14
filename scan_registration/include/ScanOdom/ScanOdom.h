
#ifndef _SCANODOM_H
#define _SCANODOM_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <string>
#include <memory>

#include "ScanRegis/ScanRegisBase.h"
#include "ScanRegis/ScanRegisFactory.h"

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
    sensor_msgs::LaserScan::ConstPtr last_scan_msg;
    ScanOdomStatus scan_odom_status;
    Eigen::Matrix4d m_base_in_odom;
};



#endif