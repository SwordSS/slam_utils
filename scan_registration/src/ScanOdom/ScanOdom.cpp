
#include "ScanOdom/ScanOdom.h"


ScanOdom::ScanOdom()
{
    ros::NodeHandle node_handle;
    ros::NodeHandle pnode_handle("~");
    m_scan_sub = node_handle.subscribe(
        "/scan", 1, &ScanOdom::ScanCallback, this);
    // m_odom_pub = node_handle.advertise<nav_msgs::Odometry>(
    //     "/scan_odom", 50);
    scan_regis_base = ScanRegisFactory::CreateScanRegisMethod(ScanRegisFactory::PLICP);

    scan_odom_status = Initialzing;
}

void ScanOdom::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    if(scan_odom_status==Initialzing)
    {
        last_scan_msg = scan_msg;
        scan_regis_base-> Init(scan_msg);
        scan_odom_status = Initialzed;
        return ;
    }
    else if(scan_odom_status == Initialzed)
    {
        sensor_msgs::LaserScan::ConstPtr cur_scan_msg = scan_msg;
        scan_regis_base->ScanMatch(last_scan_msg,cur_scan_msg,m_base_in_odom);

    }
}