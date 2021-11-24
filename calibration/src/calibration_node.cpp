//c++
#include <iostream>

//ros
#include <ros/ros.h>

//own
#include "SliceDeque.h"
#include "common/Timer.h"
#include "TestMachine/TestMachine.h"

int main(int argc,char** argv)
{
    /*
        伪代码：
        SliceQueueConfig = CreateSliceQueueConfig()//生成切片队列配置信息
        SliceDeque(SliceQueueConfig)//根据配置信息构建切片队列
        CalibrationConfig = CreateCalibrationConfig()//生成切片队列配置信息
        Calibration(CalibrationConfig)//构建外参标定器
            while(true)
                ScanSliceBag = SliceDeque.GetScanSliceBag()//获取消息队列中的数据包
                Calibrating(ScanSliceBag)//把数据包输入到外参标定器中
                IsCalibrateSuccessed = GetCalibrateData(&CalibrateData)//生成标定结果
                if(IsCalibrateSuccessed)//是否生成了标定结果
                    PrintCalibrateResult(CalibrateData)//打印标定结果
    */
    ros::init(argc, argv, "calibrantion_node");
    ros::NodeHandle n;
    
    SliceDeque slice_queue;
    TestMachine test_machine(slice_queue);

    ros::spin();
    return 0;
}
