#ifndef _SCANREGISBASE_H
#define _SCANREGISBASE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Eigen>

class ScanRegisBase
{
public:
    ScanRegisBase(){};
    virtual ~ScanRegisBase(){};
    virtual void Init()=0;
    virtual void Init(const sensor_msgs::LaserScan::ConstPtr& cur_scan_msg)=0;
    virtual void UpdateRefScan(const sensor_msgs::LaserScan::ConstPtr& ref_scan_msg)=0;
    virtual void ScanMatch(
        const sensor_msgs::LaserScan::ConstPtr& cur_scan_msg,
        Eigen::Matrix4d& transform )=0;
    
};

#endif