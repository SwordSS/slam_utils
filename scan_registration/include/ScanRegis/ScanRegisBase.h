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
    virtual bool ScanMatch(
        const sensor_msgs::LaserScan::ConstPtr& cur_scan_msg,
        Eigen::Vector3d& predict_motion,
        Eigen::Vector3d& real_motion)=0;
};

#endif