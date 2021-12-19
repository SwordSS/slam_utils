#ifndef _SCANREGISWITHPLICP_H
#define _SCANREGISWITHPLICP_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include "ScanRegis/ScanRegisBase.h"
#include <csm/csm_all.h>

class ScanRegisWithPLICP:public ScanRegisBase
{
private:
    enum ScanRegisStatus
    {
        Initialzing,
        Initialzed
    };

public:
    ScanRegisWithPLICP();//可以放置一些需要长时间的初始化操作，与系统有关
    ~ScanRegisWithPLICP(){};
    virtual void Init(){};//可以放置一些需要短时间的初始化操作，但需要等系统等一些前置确认后才启动
    void Init(const sensor_msgs::LaserScan::ConstPtr& cur_scan_msg);//可以放置一些需要短时间的初始化操作，但需要等待数据来才可以进行
    void UpdateRefScan(const sensor_msgs::LaserScan::ConstPtr& ref_scan_msg);
    bool ScanMatch(
        const sensor_msgs::LaserScan::ConstPtr& cur_scan_msg,
        Eigen::Vector3d& predict_motion,
        Eigen::Vector3d& real_motion );


private:
    void InitParams();
    void LaserScanToLDP(const sensor_msgs::LaserScan::ConstPtr &scan_msg, LDP &ldp);

private:
    sm_params m_input;
    sm_result m_output;

    LDP m_ldp_ref_scan;
    ScanRegisStatus scan_regis_status;

};

#endif