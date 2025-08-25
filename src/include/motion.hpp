#pragma once
#include "../include/common.hpp"
#include "../include/json.hpp"
#include "controlcenter.cpp"

/**
 * @brief 运动控制器
 */
class Motion {
public:
    struct Params {
    float speedturning=0; //转弯速度
    float speedLow = 0.8;       // 智能车最低速
    float speedHigh = 0.8;      // 智能车最高速
    float speedBridge = 0.6;    // 坡道速度
    float speedCatering = 0.6;  // 快餐店速度
    float speedLayby = 0.6;     // 临时停车区速度
    float speedLayby1=0.6;
    float speedObstacle = 0.6;  // 障碍区速度
    float speedblock = 0;
    float speedParking = 0.6;   // 停车场速度
    float speedRing1 = 0.6;      // 环岛速度
    float speedRing2 = 0.6;      // 环岛速度
    float speedDown = 0.5;      // 特殊区域降速速度
    float speedtrackout=0;
    float turnP2 = 3.5;          // 一阶比例系数：转弯控制量
    float turnD2 = 3.5;          // 一阶微分系数：转弯控制量
    bool debug = false;         // 调试模式使能
    bool saveImg = false;       // 存图使能
    uint16_t rowCutUp = 10;     // 图像顶部切行
    uint16_t rowCutBottom = 10; // 图像顶部切行
    bool bridge = true;         // 坡道区使能
    bool catering = true;       // 快餐店使能
    bool layby = true;          // 临时停车区使能
    bool obstacle = true;       // 障碍区使能
    bool parking = true;        // 停车场使能
    bool ring = true;           // 环岛使能
    bool cross = true;          // 十字道路使能
    bool stop = true;           // 停车区使能
    int trackout_count1=0;      // 倒车出库时间
    float obstaclecone=0;
    float obstacleperson=0;
    float Kp = 2.5;         // 合并后的比例系数（原runP1+runP2功能）
    float Kd = 1.2;         // 微分系数（带自动增益调整）
    


    int cateringTruningTime = 50;
    int cateringTravelTime = 10;
    int cateringStopTime = 400;
    int cateringExtendtime = 80;

 
    float prospect21 = 0;   //环岛
    float prospect22 = 0; 
    float prospect23 = 0;
    float prospect24 = 0;
    float prospect25 = 0;  

    float prospect7 = 0;   //充电区
    float prospect6 = 0;   //临时停车
    float prospect5 = 0;   //快餐
    float prospect4 = 0;   //障碍区
    float prospect51=0;
    float prospect01 = 0;   //基础赛道 直到
    float prospect02 = 0;   //弯道
    float stoptime   = 0 ;  //停车时间

    float prospect1=0;//十字

    int   bridgetime = 0;
    float odisperson=0;
    float odiscone=0;
    //临时停车
    int   STOP_LINE_Y_THRESHOLD=0;
    int   STOP_LINE_Y_THRESHOLD1=0;
    int   stopTime=0;
    float score = 0.5; 

    //充电区
    int   parkingstopTime=0;
    float   parkingtruningTime=0;
    float parkingswerveTime=0;
    int   speedtrackout_plus=0;
    float   exitCountdown=0;
    float speedParking_stop;
    float   parkingtruningTime2=0;
    int stop_line=200;  //停车高度 

     std::string model = "../res/model/yolov3_mobilenet_v1"; // 确保这行存在
    std::string video = "../res/samples/demo.mp4";
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Params, speedLow, speedHigh, speedBridge,
                                   speedCatering, speedLayby, speedLayby1,speedObstacle,speedblock,
                                   speedParking,speedRing1,speedRing2,speedDown,
                                   debug, saveImg, rowCutUp,
                                   rowCutBottom, bridge, catering, layby, obstacle,
                                   parking, ring, cross,stop, score, model,
                                   video,cateringTruningTime, cateringTravelTime, cateringStopTime, cateringExtendtime,
                                   prospect21,prospect22,prospect23,prospect24,prospect25,prospect7,prospect6,prospect5,prospect51,prospect4,prospect01,prospect02,prospect1,stoptime,bridgetime
                                   ,STOP_LINE_Y_THRESHOLD,stopTime,exitCountdown,STOP_LINE_Y_THRESHOLD1,speedturning,stop_line
                                   ,parkingstopTime,parkingtruningTime,parkingswerveTime,speedtrackout,speedtrackout_plus,speedParking_stop,trackout_count1,parkingtruningTime2);
    };

    Motion();
    void poseCtrl(int controlCenter);
    void speedCtrl(bool enable, bool slowDown, ControlCenter control);

    Params params;
    uint16_t servoPwm = PWMSERVOMID;
    float speed = 0.3;
    float currentSpeed = 0.3;

private:
    int countShift = 0;
    float lowPassFilter(float input, float* prevOutput, float alpha);
};