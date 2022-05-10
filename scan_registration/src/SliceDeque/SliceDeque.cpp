#include "SliceDeque/SliceDeque.h"

//note:切片队列处理核心，处理上有些凌乱，后续需要梳理
void SliceDeque::SliceProcess()
{
    int sums_wait = 0;
    bool is_init_flag = false;
    while(ros::ok())
    {
        //输入前置判断，可以根据USE_ASSERT开启关闭对应部分
        {
            //观察输入输出（生产与消费的速度）是否跟得上
            #if USE_ASSERT
                std::lock_guard<std::mutex> buf_lock(m_buf_mutex);
                int scan_buf_size = m_scan_buf.size();
                int odom_buf_size = m_odom_buf.size();
                int imu_buf_size  = m_imu_buf.size();
                //buf_lock.unlock();
                int max_size = 50000;
                
                if(scan_buf_size >= max_size)
                {
                    std::cout << "now scan_buf_size>=" << max_size
                              << ",scan_buf is out of bounds!" 
                              << std::endl;
                    assert(scan_buf_size<max_size);
                }
                else if(odom_buf_size >= max_size)
                {
                    std::cout << "now odom_buf_size>=" << max_size 
                              << ",odom_buf is out of bounds!"
                              << std::endl;
                    assert(odom_buf_size<max_size);
                }
                else if(imu_buf_size >= max_size)
                {
                    std::cout << "now max_size is " << max_size
                              << ",imu_buf_size is " << imu_buf_size
                              << std::endl;
                    assert(imu_buf_size<max_size);
                }
            #endif    
        }
        
        ScanSliceBag cur_bag;//用于外发的激光切片包，每个激光数据左右必定都有imu与odom存在
        std::lock_guard<std::mutex> buf_lock(m_buf_mutex);
        //step1：若所有队列不是全部都有数据，则等待
        if(m_odom_buf.empty()||m_imu_buf.empty()||m_scan_buf.empty())
        {
            continue;
        }

        //step2：确保odom与imu的最新数据时间上大于激光最新的数据
        //note :（这里的激光就是只有一帧，可能写的不太好，后续改进）
        if(m_scan_buf.back()->header.stamp.toSec() < m_odom_buf.back()->header.stamp.toSec() &&
           m_scan_buf.back()->header.stamp.toSec() < m_imu_buf.back()->header.stamp.toSec() )
        {
            //step2.1:确保每帧的odom时间上小于等于最新激光的，以及刚刚大于的那一帧odom都被放入容器中，等待输出
            //（小于激光，且最近的那一帧之前的odom都在队列pop出）
            while(true)
            {
                typedef std::deque<nav_msgs::OdometryConstPtr>::iterator deque_itr;
                deque_itr itr = m_odom_buf.begin();
                if((*itr)->header.stamp.toSec() <= m_scan_buf.back()->header.stamp.toSec())
                {
                    cur_bag.vector_ptr_odom.push_back(*itr);
                    ++itr;
                    if((*itr)->header.stamp.toSec()>m_scan_buf.back()->header.stamp.toSec())
                    {
                        cur_bag.vector_ptr_odom.push_back(*itr);
                        break;
                    }
                    else
                    {
                       m_odom_buf.pop_front();
                    }
                }
                else break;
            }

            //step2.2:确保每帧的imu时间上小于等于最新激光的，以及刚刚大于的那一帧imu都被放入容器中，等待输出
            //（小于激光，且最近的那一帧之前的imu都在队列pop出）
            while(true)
            {
                typedef std::deque<sensor_msgs::ImuConstPtr>::iterator deque_itr;
                deque_itr itr = m_imu_buf.begin();
                if((*itr)->header.stamp.toSec() <= m_scan_buf.back()->header.stamp.toSec())
                {
                    cur_bag.vector_ptr_imu.push_back(*itr);
                    ++itr;
                    if((*itr)->header.stamp.toSec()>m_scan_buf.back()->header.stamp.toSec())
                    {
                        cur_bag.vector_ptr_imu.push_back(*itr);
                        break;
                    }
                    else
                    {
                        m_imu_buf.pop_front();
                    }
                }
                else break;
            }

            //step2.3:激光都装入容器中
            while(!m_scan_buf.empty())
            {
                cur_bag.vector_ptr_scan.push_back(m_scan_buf.back());
                m_scan_buf.pop_front();
            }

            //note：第一个包丢弃，且确保后续至少有两个数据（这里需要后续考虑一下更优的实现）
            if(!is_init_flag)
            {
                if(cur_bag.vector_ptr_odom.size()>=2 && cur_bag.vector_ptr_imu.size()>=2)
                {
                    is_init_flag=true;
                }
                else continue;
            }
            std::lock_guard<std::mutex> ScanSliceBag_lock(m_ScanSliceBag_mutex);
            m_ScanSliceBag_queue.push_back(cur_bag);
        }
        //buf_lock.unlock();

        //输出后置判断，可以根据USE_ASSERT开启关闭对应部分
        {
            #if USE_ASSERT
                int ScanSliceBag_queue_size = m_ScanSliceBag_queue.size();
                int max_size = 5000;
                if(ScanSliceBag_queue_size >= max_size)
                {
                    std::cout << "now max_size is" << max_size
                                <<",ScanSliceBag_queue_size is " << ScanSliceBag_queue_size 
                                << std::endl;
                    assert(ScanSliceBag_queue_size<max_size);
                }

            #endif    
        }
    }

}