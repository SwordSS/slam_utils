#ifndef _SCANREGISWITHPLICP_H
#define _SCANREGISWITHPLICP_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include "ScanRegis/ScanRegisBase.h"
#include <csm/csm_all.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"
#include <nav_msgs/Odometry.h>

class ScanRegisWithPLICP:public ScanRegisBase
{
public:
    ScanRegisWithPLICP();
    ~ScanRegisWithPLICP(){};
    virtual void Init(){};
    void Init(const sensor_msgs::LaserScan::ConstPtr& cur_scan_msg);
    void ScanMatch(
        const sensor_msgs::LaserScan::ConstPtr& last_scan_msg,
        const sensor_msgs::LaserScan::ConstPtr& cur_scan_msg,
        Eigen::Matrix4d& transform );
private:
    void InitParams();
    void CreateCache(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    void LaserScanToLDP(const sensor_msgs::LaserScan::ConstPtr &scan_msg, LDP &ldp);
    void GetPrediction(double &prediction_change_x,
                                   double &prediction_change_y,
                                   double &prediction_change_angle,
                                   double dt);
    void CreateTfFromXYTheta(double x, double y, double theta, tf2::Transform &t);
    void PublishTFAndOdometry(tf2::Transform& base_in_odom_);

private:
    sm_params input_;
    sm_result output_;

    std::vector<double> a_cos_;
    std::vector<double> a_sin_;

    //不一定用
    ros::Publisher odom_publisher_;   

    ros::Time last_icp_time_;
    ros::Time current_time_;

    std::string odom_frame_;
    std::string base_frame_;

    double kf_dist_linear_;
    double kf_dist_linear_sq_;
    double kf_dist_angular_;
    int kf_scan_count_;
    int scan_count_;

    geometry_msgs::Twist latest_velocity_;

};

#endif