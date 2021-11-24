#ifndef _TEST_MACHINE_H_
#define _TEST_MACHINE_H_

//c++
#include <iostream>
#include <thread>
#include <string>

//ros
#include <ros/ros.h>
#include <rosbag/bag.h>

//own
#include "SliceDeque.h"
#include "common/Timer.h"

class TestMachine
{
public:
    TestMachine(SliceDeque& slice_queue)
    {
        double time_increment_value = 0.15;//时间最小间隔
        bool is_write_bag_flag = 1;//是否写包进行查看
        bool is_print_flag = 1;//是否打印队列与获取包状况
        int ScanSliceBag_nums_of_rosbag = 10;//多少个切片包组成一个rosbag
        std::string bag_doc_path = "/home/yyj/bag";
        m_thread_get = std::thread( boost::bind(&TestMachine::TestGetData,this,
                                                boost::ref(slice_queue),
                                                time_increment_value,
                                                ScanSliceBag_nums_of_rosbag,
                                                bag_doc_path,
                                                is_write_bag_flag,
                                                is_print_flag
                                                ) );
    };
    void TestGetData(SliceDeque& slice_queue ,
                 double time_increment_value ,
                 int ScanSliceBag_nums_of_rosbag,
                 std::string bag_doc_path ,
                 bool is_write_bag_flag ,
                 bool is_print_flag );

private:
    std::thread m_thread_get;
};

void TestMachine::TestGetData(SliceDeque& slice_queue ,
                 double time_increment_value ,
                 int ScanSliceBag_nums_of_rosbag,
                 std::string bag_doc_path ,
                 bool is_write_bag_flag ,
                 bool is_print_flag )
{
    if(bag_doc_path=="") 
    {
        std::cout << "bag_doc_path is empty" <<std::endl;
        return;
    }
    std::vector<ScanSliceBag> vector_output_ScanSliceBag;
    while(ros::ok())
    {
        static Timer GetData_timer(time_increment_value);

        if(GetData_timer.IsReachTime())
        {
            ScanSliceBag one_ScanSlice_bag;
            slice_queue.GetScanSliceBag(one_ScanSlice_bag);

            if(is_print_flag)
            {
                std::cout << "ScanBufSzie() : " << slice_queue.GetScanBufSzie()                  << " "
                          << "OdomBufSzie() : " << slice_queue.GetOdomBufSzie()                  << " "
                          << "ImuBufSzie() : "  << slice_queue.GetImuBufSzie()                   << " "
                          << "SliceSzie() : "   << slice_queue.GetScanSliceQueueSzie()           << " "
                          << "BagScanSzie() : " << one_ScanSlice_bag.vector_ptr_scan.size() << " "
                          << "BagOdomSzie() : " << one_ScanSlice_bag.vector_ptr_odom.size() << " "
                          << "BagIMUSzie() : "  << one_ScanSlice_bag.vector_ptr_imu.size()  << " "
                          << std::endl;
            }
            GetData_timer.UapdateLastTime();

            if(is_write_bag_flag)
            {
                //输出包看切片效果，每个切片包会对应编号话题错开便于查看
                //indoor.bag scan:15hz,odom:10hz,imu:100hz
                static int times = 0;
                vector_output_ScanSliceBag.push_back(one_ScanSlice_bag);
                times++;

                if(times%ScanSliceBag_nums_of_rosbag==0)
                {
                    static int i = 0 ;
                    rosbag::Bag bag(bag_doc_path+"/"+std::to_string(i)+".bag",rosbag::bagmode::Write);
                    for(int index_bag=0;index_bag<vector_output_ScanSliceBag.size();index_bag++)
                    {
                        auto &vector_ptr_scan = vector_output_ScanSliceBag[index_bag].vector_ptr_scan;
                        for(int index_scan=0;index_scan<vector_ptr_scan.size();index_scan++)
                        {
                            bag.write("/"+std::to_string(index_bag)+"_scan",vector_ptr_scan[index_scan]->header.stamp,vector_ptr_scan[index_scan]);
                        }
                        auto &vector_ptr_odom = vector_output_ScanSliceBag[index_bag].vector_ptr_odom;
                        for(int index_odom=0;index_odom<vector_ptr_odom.size();index_odom++)
                        {
                            bag.write("/"+std::to_string(index_bag)+"_odom",vector_ptr_odom[index_odom]->header.stamp,vector_ptr_odom[index_odom]);
                        }
                        auto &vector_ptr_imu = vector_output_ScanSliceBag[index_bag].vector_ptr_imu;
                        for(int index_imu=0;index_imu<vector_ptr_imu.size();index_imu++)
                        {
                            bag.write("/"+std::to_string(index_bag)+"_imu",vector_ptr_imu[index_imu]->header.stamp,vector_ptr_imu[index_imu]);
                        }
                    }
                    bag.close();
                    vector_output_ScanSliceBag.clear();
                    i++;
                }

            }
        } 
    }
}

#endif 