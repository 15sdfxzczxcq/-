# -
完全模型组 国二开源代码 非常适合小白使用，基于官方代码的修改
开源代码，用来供大家学习交流和修改🦁山

大佬说的队
1.简介
本项目为大连民族大学第二十届全国智能汽车大赛完全模型组上位机开源(下位机用得tc264)。使用了多线程架构，拆分任务，提升舵机反应速度，所有元素在1.6-1.8m/s状态下非常稳定(因为车子坏了，没有时间调高速代码了，此代码上限也有均速2.3m/s)
2.软件框架
大佬说的队
|  build(编译环境 可以查看官方教程)
|--.res
|    calibration
|    model(存放AI模型)
|    samples(查看图像用来分析)
|--.src
|    config
|          config.json(调参文件)
|          config_ppncnms.json
|          config_ppncnna.json
|    include
|          common.hpp
|          detection.hpp(模型检测)
|          json.hpp
|          motion.hpp(调参文件)
|          uart.hpp(串口)
|    src  
|         detection(模型视觉)
│            bridge.cpp // 桥识别及处理
│            bridge.h
│            catering.cpp // 汉堡识别及处理
│            catering.h
│            parking.cpp // 充电区识别及处理
│            parkiing.h
│            crosswalk.cpp // 斑马线识别及处理
│            crosswalk.h
│            layby.cpp // 临时停车区识别及处理
│            layby.h
│            obstacle.cpp // 障碍区识别及处理
│            obstacle.h
|         recognition
|            crossroad.cpp //十字
|            ring.cpp  //环岛
|            tracking  //循迹
|         controlcenter.cpp  // 中线处理
|         detection.cpp
|         icar.cpp  // 主函数
|         mapping.cpp
|         motion.cpp
|         preprocess.cpp
