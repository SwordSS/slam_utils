/*
struct CalibrationConfig;

函数：
    void Calibrating(ScanSliceBag)//把数据包输入到外参标定器中
        if(Initing)
            InitOdomPart();
            InitScanPart();
            Initing -> Inited;
        else if(Inited)
            CalOdomPart();//生成边与点
            GetOdomData();//获取最新的边与点

            CalScanPart();//生成边与点
            GetScanData();//获取最新的边与点
            
            GnerateResAndPara();
            
            Optimization();
成员变量：
    OdomPart;//生成最新的边与点
    ScanPart;//生成最新的边与点
    ResAndParaPart;//这里更多负责如何根据新生成的边与点，利用自己的视角构建出总计算图
    OptimizationPart;//这里更多负责Ceres相关实现

    MachineState{
        Initing
        Inited
    }

*/