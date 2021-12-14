#include <iostream>
#include <ros/ros.h>
#include "ScanOdom/ScanOdom.h" 

int main(int argc,char** argv)
{
    ros::init(argc, argv, "scan_odom");
    ScanOdom scan_odom;
    ros::spin();
    return 0;
}