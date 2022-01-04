#ifndef _DEBUGTOOLS_H
#define _DEBUGTOOLS_H

#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <ctime>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class DebugTools
{
public:
    DebugTools()
    {

    };

    ~DebugTools()
    {
        if(file.is_open())
        {
            file.close();
        }
    }

    void Init()
    {
        ros::NodeHandle nh;
        path_pub = nh.advertise<nav_msgs::Path>("scan_odom_path",50);
        file.open("/home/yyj/path/plicp_path.txt",std::ios::out);
    };

    void PublishTF(const Eigen::Matrix4d& T,ros::Time time,std::string frame_id,std::string child_frame_id)
    {
        static tf2_ros::TransformBroadcaster tf_broadcaster;
        geometry_msgs::TransformStamped transform;
        Eigen::Quaterniond q_v = Eigen::Quaterniond(T.block<3,3>(0,0));
        transform.header.stamp = time;
        transform.header.frame_id = frame_id;
        transform.child_frame_id = child_frame_id;
        transform.transform.translation.x = T(0,3);
        transform.transform.translation.y = T(1,3);
        transform.transform.translation.z = T(2,3);
        transform.transform.rotation.x = q_v.x();
        transform.transform.rotation.y = q_v.y();
        transform.transform.rotation.z = q_v.z();
        transform.transform.rotation.w = q_v.w();
        tf_broadcaster.sendTransform(transform);
    }

    void PublishPath(const Eigen::Matrix4d& T,ros::Time time,std::string frame_id)
    {
        static nav_msgs::Path path;
        path.header.stamp = time;
        path.header.frame_id = frame_id;

        geometry_msgs::PoseStamped transform;
        Eigen::Quaterniond q_v = Eigen::Quaterniond(T.block<3,3>(0,0));
        transform.header.stamp = time;
        transform.header.frame_id = frame_id;
        transform.pose.position.x = T(0,3);
        transform.pose.position.y = T(1,3);
        transform.pose.position.z = T(2,3);
        transform.pose.orientation.x = q_v.x();
        transform.pose.orientation.y = q_v.y();
        transform.pose.orientation.z = q_v.z();
        transform.pose.orientation.w = q_v.w();
        path.poses.push_back(transform);
        path_pub.publish(path);
    }

    void WritePath(const Eigen::Matrix4d& T,double data_time)
    {
        if(file.is_open())
        {
            Eigen::Quaterniond q_v = Eigen::Quaterniond(T.block<3,3>(0,0));
            file << std::fixed << data_time << " "
                               << T(0,3)    << " "
                               << T(1,3)    << " "
                               << T(2,3)    << " "
                               << q_v.x()   << " "
                               << q_v.y()   << " "
                               << q_v.z()   << " "
                               << q_v.w()   << "\n";
        }
    }

    void CalTimer(int number_timer,std::string print_str)
    {
        static std::map<int,clock_t> map_timer;
        std::map<int,clock_t>::iterator itr = map_timer.find(number_timer);
        if(itr!=map_timer.end())
        {
            clock_t begin = itr->second;
            clock_t end = clock();
            map_timer.erase(number_timer);
            std::cout << print_str << "'s time is " << float(end-begin)/CLOCKS_PER_SEC <<std::endl;
        }
        else
        {
            clock_t begin = clock();
            map_timer.insert(std::pair<int,clock_t>(number_timer,begin));
        }
    }   

private:
    ros::Publisher path_pub;
    std::ofstream file;
};
#endif