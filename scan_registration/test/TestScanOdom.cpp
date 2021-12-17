#include <iostream>
#include <ros/ros.h>
#include <typeinfo>
#include "ScanOdom/ScanOdom.h" 

int main(int argc,char** argv)
{
    ros::init(argc, argv, "scan_odom");
    // ScanOdom scan_odom;
    // ros::spin();
    int i=0;

    typedef float data;
    if(i==0)
    {
        typedef double data;
    }
    else if (i==1)
    {
        typedef int data;
    }

    data abc;
    abc = 0.0;
    std::cout << typeid(abc).name()<<std::endl; 
    
    return 0;
}