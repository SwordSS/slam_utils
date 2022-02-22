#ifndef _SLICEDEQUE_H_
#define _SLICEDEQUE_H_

//c++
#include <iostream>
#include <thread>
#include <mutex>
#include <deque>
#include <vector>
#include <assert.h>

//ros
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#define USE_ASSERT  1

struct ScanSliceBag
{
    std::vector<sensor_msgs::LaserScanConstPtr> vector_ptr_scan;
    std::vector<nav_msgs::OdometryConstPtr> vector_ptr_odom;
    std::vector<sensor_msgs::ImuConstPtr> vector_ptr_imu;
};

class SliceDeque
{
    //主要功能接口
    public:
        SliceDeque():
        pn("~")
        {
            scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, &SliceDeque::ScanCallback,this);
            odom_sub = n.subscribe<nav_msgs::Odometry>("/odom", 10, &SliceDeque::OdomCallback,this);
            imu_sub = n.subscribe<sensor_msgs::Imu>("/imu", 10, &SliceDeque::ImuCallback,this);
            m_thread_slice_process = std::thread(&SliceDeque::SliceProcess,this);
        }

        bool GetScanSliceBag(ScanSliceBag& scan_slice_bag)
        {
            //等待，直到拿出数据
            while(ros::ok())
            {
                if(!m_ScanSliceBag_queue.empty())
                {
                    std::unique_lock<std::mutex> ScanSliceBag_lock(m_ScanSliceBag_mutex);
                    scan_slice_bag = m_ScanSliceBag_queue.front();
                    m_ScanSliceBag_queue.pop_front();
                    return true;
                }
            }
        }

    //非主要功能接口
    public:
        int GetScanSliceQueueSzie()
        {
            std::unique_lock<std::mutex> ScanSliceBag_lock(m_ScanSliceBag_mutex);
            int size = m_ScanSliceBag_queue.size();  
            return size;
        }

        int GetScanBufSzie()
        {
            std::unique_lock<std::mutex> buf_lock(m_buf_mutex);
            int size = m_scan_buf.size();  
            return size;
        }

        int GetOdomBufSzie()
        {
            std::unique_lock<std::mutex> buf_lock(m_buf_mutex);
            int size = m_odom_buf.size();  
            return size;   
        }

        int GetImuBufSzie()
        {
            std::unique_lock<std::mutex> buf_lock(m_buf_mutex);
            int size = m_imu_buf.size();    
            return size; 
        }

    //内部调用接口
    private:
        void ScanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg_ptr)
        {
            std::unique_lock<std::mutex> buf_lock(m_buf_mutex);
            m_scan_buf.push_back(scan_msg_ptr);
            m_buf_mutex.unlock();
            
            //question 打印机制(有问题，1.0改成2.0差了一半)
            {
                // double time_increment_min_value = 1;
                // static Timer scan_print_timer(time_increment_min_value);
                // static int scan_msg_nums = 0;
                // scan_msg_nums++;
                // if(scan_print_timer.IsReachTime())
                // {
                //     double frequence = 2.0/(scan_print_timer.GetThroughTime()/double(scan_msg_nums));
                //     std::cout << "scan_hz : " << frequence <<std::endl<<std::endl;
                //     scan_print_timer.UapdateLastTime();
                //     scan_msg_nums = 0;
                // }
            }
        }

        void OdomCallback(const nav_msgs::OdometryConstPtr& odom_msg_ptr)
        {
            std::unique_lock<std::mutex> buf_lock(m_buf_mutex);
            m_odom_buf.push_back(odom_msg_ptr);
            m_buf_mutex.unlock();
        }

        void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr)
        {
            std::unique_lock<std::mutex> buf_lock(m_buf_mutex);
            m_imu_buf.push_back(imu_msg_ptr);
            m_buf_mutex.unlock();
        }

        void SliceProcess();

    //内部成员变量
    private:
        //句柄
        ros::NodeHandle n;
        ros::NodeHandle pn;

        //ros订阅器
        ros::Subscriber scan_sub;
        ros::Subscriber odom_sub;
        ros::Subscriber imu_sub;

        //多线程
        std::mutex m_buf_mutex;
        std::mutex m_ScanSliceBag_mutex;
        std::thread m_thread_slice_process;

        //缓冲队列
        std::deque<sensor_msgs::LaserScanConstPtr> m_scan_buf;
        std::deque<nav_msgs::OdometryConstPtr> m_odom_buf;
        std::deque<sensor_msgs::ImuConstPtr> m_imu_buf;
        std::deque<ScanSliceBag> m_ScanSliceBag_queue;
    
};



#endif