#pragma once
/**
 * @file ring.cpp
 * @author Leo
 * @brief 环岛识别（基于track赛道识别后）
 * @version 0.1
 * @date 2022-02-28
 *
 * @copyright Copyright (c) 2022
 *
 * @note  环岛识别步骤（ringStep）：
 *          1：环岛识别（初始化）
 *          2：入环处理
 *          3：环中处理
 *          4：出环处理
 *          5：出环结束
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "tracking.cpp"

using namespace cv;
using namespace std;

class Ring
{
public:
    bool enabled = true;
    uint16_t counterShield = 0; // 环岛检测屏蔽计数器：屏蔽车库误检测
    int countWide = 0; // 环岛入口变宽区域行数
    int countWide2 = 0;
    int withi1=0;
    int withi2=0;
    int withi5=0;
    int withi6=0;
    int withi7=230;
    int middlepoint=0;
    bool ringcircle=false; //圆环标志
    bool outcircle=false;
    int count=0;
    int gocount=0;
    int gocount2=0;
    int findcount=0;
    int findcount2=0;
    int outcount=0;
    int k=0;
    int finish=0;
    POINT loseleft;
    POINT loseright;
    bool exit=false;
    bool lookforB1=false;
    bool lookforB2=false;
    bool Aswitch=false;
    bool Cswitch=true;
    bool linepoint=false;
    int count_yuzhi=12;
    int gocount_yuzhi=25;
    /**
     * @brief 环岛识别初始化|复位
     *
     */
    void reset(void)
    {
        enabled = true;
        ringType = RingType::RingNone; // 环岛类型
        ringStep = RingStep::None;     // 环岛处理阶段
        int rowRepairLine = 0;                  // 用于环补线的点（行号）
        int colRepairLine = 0;                  // 用于环补线的点（列号
        ringcircle=false;
        outcircle=false;
        countWide = 0;
        countWide2 = 0;
        withi1=0;
        withi2 =0;
        withi5=0;
        withi6=0;
        withi7=230;
        middlepoint=0;
        count=0;
        gocount=0;
        gocount2=0;
        findcount=0;
        findcount2=0;
        outcount=0;
        k=0;
        exit=false;
        finish=0;
        counterShield=0;
        lookforB1=false;
        lookforB2=false;
        linepoint=false;
    }
    /**
     * @brief 环岛识别与行径规划
     *
     * @param track 基础赛道识别结果
     * @param imagePath 赛道路径图像
     */
    bool process(Tracking &track, Mat &imagePath)
    {
        // // 如果环岛识别被禁用，直接返回 false
        // if (!enabled) {
        //     return false;
        // }


        // 边缘斜率重计算（边缘修正之后）
        track.stdevLeft = track.stdevEdgeCal(track.pointsEdgeLeft, ROWSIMAGE);
        track.stdevRight = track.stdevEdgeCal(track.pointsEdgeRight, ROWSIMAGE);

        if (counterShield < 40)// 确保在环岛检测被触发后的一段时间内不会再次触发
        {
            counterShield++;
            return false;
        }
        bool ringEnable = false;                                 // 判环标志
        RingType ringTypeTemp = RingType::RingNone;              // 环岛类型：临时变量
        _index = 0;
        _ringPoint = POINT(0, 0);
        // if(ringStep > 0){
        //     std::cout<<"RingStep: "<<ringStep<<std::endl;
        //     //std::cout<<"findcount: "<<findcount<<std::endl;
        // }
       
        

        // 判环
        int with=0;
        int redcircle=0;
        bool rightpoint=false;
        bool rightpoint2=false;
        bool exitpoint = false;
        bool outpoint = false;
        bool twopointB = false;
        bool twopointC = false;
        bool leftring = false;
        bool rightring = false;
        //bool lookforB1=false;                 //如果只用第一种方法找b点的话，把此行代码打开
        
        if (imagePath.channels() == 3) {
            //std::cout<<"为rgb图像"<<std::endl;
            for (int row = 0; row < ROWSIMAGE; row++) {  // 遍历所有行
                bool foundRed = false;  // 标记当前行是否发现红色
                for (int col = 0; col < COLSIMAGE; col++) {  // 遍历行内所有列
                    // 获取当前像素的BGR值
                    Vec3b pixel = imagePath.at<Vec3b>(row, col);
                    uchar b = pixel[0];
                    uchar g = pixel[1];
                    uchar r = pixel[2];  // 红色通道在索引2

                // 红色判定：R通道值高，B/G通道值低
                if (r > 100 && r < 200 && r > g * 1.1 && r > b * 1.1) {
                    foundRed = true;
                    break;  // 发现红色即跳出列循环
                }
            }
                if (foundRed) redcircle++;  // 该行有红色则计数
        }
    }

        //从下往上搜索
        //gai
        for (int i = track.widthBlock.size()-6; i > 3; --i)
        {   
            //1环岛1
            //std::cout<<"\tle:"<<track.stdevLeft<<"\tri:"<<track.stdevRight<<std::endl;
            //上行宽度大于下行，并且宽度大于图像列宽0.6并且行大于30，左点集标准差大于120，右点集标准差小于50，判断赛道平直程度
            if (track.widthBlock[i].y - track.widthBlock[i-2].y>20&&((track.stdevLeft>200 &&track.stdevRight<20)||(track.stdevRight>200&&track.stdevLeft<20))
            &&(ringStep == RingStep::None||ringStep == RingStep::Entering)&&track.widthBlock[i].x>100&&track.widthBlock[i].x<230&&Aswitch==true) // 搜索突然变宽的路径行数//stdevLeft>100能左弯极限进环岛但尽量不要用
            {
                    std::cout<<"找到A点"<<std::endl;
                    count=0;
                    rightpoint=true;
                    //ringStep = RingStep::Entering;
                    if(track.widthBlock[i].x > track.widthBlock[withi1].x){
                    countWide++;
                    }
                    if(countWide > 5)
                    {
                        lookforB1=true;
                    }
                    withi1=i;
                    for (int i = 0; i < COLSIMAGE; i++)
                    {
                        circle(imagePath, Point(i,track.widthBlock[withi1].x), 2,
                            Scalar(255, 255, 0), -1); // 天空蓝
                    }
                    for(int ii = withi1+20; ii < track.widthBlock.size()-10; ii++){        //从找到角点下往上搜索最小点//像此赛道宽度的判定条件就是因为循迹问题(斑马线接弯道导致的)
                        if((track.pointsEdgeRight[ii].y<track.pointsEdgeRight[ii-10].y&&track.pointsEdgeRight[ii].y<track.pointsEdgeRight[ii+10].y&&track.stdevLeft<10 &&track.stdevRight>200&&abs(track.pointsEdgeRight[ii+10].y-track.pointsEdgeRight[ii-10].y)<5)||
                        (track.pointsEdgeLeft[ii].y>track.pointsEdgeLeft[ii-5].y&&track.pointsEdgeLeft[ii].y>track.pointsEdgeLeft[ii+5].y && track.stdevLeft>110 && track.stdevRight<10&&abs(track.pointsEdgeLeft[ii+5].y-track.pointsEdgeLeft[ii-5].y)<5)&&(abs(track.widthBlock[withi1].x-track.widthBlock[i].x)>10&&abs(track.widthBlock[withi1].x-track.widthBlock[i].x)<160)){
                            lookforB2=true;
                            std::cout<<"1环岛第一种方法找到B点"<<std::endl;
                            if(track.pointsEdgeLeft[withi1-20].y-track.pointsEdgeLeft[ii].y>30||track.pointsEdgeRight[ii].y-track.pointsEdgeRight[withi1].y>30)
                                break;
                            ringStep = RingStep::Entering;
                            withi2=ii;
                            rightpoint2=true;
                            for (int i = 0; i < COLSIMAGE; i++)
                            {
                                circle(imagePath, Point(i,track.widthBlock[withi2].x), 2,
                                    Scalar(250, 250, 250), -1); // 粉色
                            }
                            break;
                        }
                    }//zhi
                    break;
            }
            
            //2环岛1//gai方差
            else{
                 rightpoint=false;
                 if(track.widthBlock[i].y-track.widthBlock[i+1].y>15&&Cswitch==true&&ringcircle ==false
                    &&((track.stdevLeft<40&&track.stdevRight>90&&track.stdevLeft>1&&track.pointsEdgeLeft[track.pointsEdgeLeft.size()-1].y > 3)||(track.stdevLeft>90 &&track.stdevRight<40&&track.stdevRight>1&&track.pointsEdgeRight[track.pointsEdgeRight.size()-1].y<COLSIMAGE-3)))//2环岛先找c点
                    {
                        if(redcircle){
                            std::cout<<"redcircle:"<<redcircle<<std::endl;
                            break;
                        }
                        std::cout<<"2环岛找到C点"<<std::endl;
                        withi1 = i;
                        twopointC = true;
                        withi6 = track.widthBlock[i].y-track.widthBlock[i+1].y;
                        std::cout<<"withi6:"<<withi6<<std::endl;
                        if(track.stdevLeft<40&&track.stdevRight>90){
                            rightring = true;
                            std::cout<<"暂定右环岛"<<std::endl;
                        }
                        else if(track.stdevLeft>90 &&track.stdevRight<40){
                            leftring = true;
                            std::cout<<"暂定左环岛"<<std::endl;
                        }
                        for (int i = 0; i < COLSIMAGE; i++)
                            {
                                circle(imagePath, Point(i,track.widthBlock[withi1].x), 2,
                                    Scalar(0, 0, 0), -1); // 粉色
                            }
                            circle(imagePath, Point(track.pointsEdgeLeft[withi1+1].y,track.pointsEdgeLeft[withi1+1].x), 2,
                            Scalar(0, 102, 255), -1); // 橙黄点
                            circle(imagePath, Point(track.pointsEdgeRight[withi1+1].y,track.pointsEdgeRight[withi1+1].x), 2,
                            Scalar(0, 102, 255), -1); // 橙黄点
                        for(int ii = withi1-1; ii > 0; ii--){        //2环岛在找到c点的同时开始找b点//gai bc点之间的距离  右边ii+6 左右边界距离  ii和withi1的距离(原来是ii和withi3)//
                            //找B
                            if(((track.pointsEdgeRight[ii].y<track.pointsEdgeRight[ii-20].y&&track.pointsEdgeRight[ii].y<track.pointsEdgeRight[ii+20].y&&abs(track.pointsEdgeRight[ii].y-track.pointsEdgeRight[withi1+3].y)<150&&track.pointsEdgeRight[ii+20].y < (COLSIMAGE - 5) &&track.pointsEdgeRight[ii-20].y < (COLSIMAGE - 5)&& rightring == true &&track.pointsEdgeLeft[ii].y >1&&track.pointsEdgeRight[ii].y > track.pointsEdgeRight[withi1+1].y)||
                                (track.pointsEdgeLeft[ii].y>track.pointsEdgeLeft[ii-20].y  &&track.pointsEdgeLeft[ii].y>track.pointsEdgeLeft[ii+20].y  &&abs(track.pointsEdgeLeft[ii].y-track.pointsEdgeLeft[withi1+3].y)<150  &&track.pointsEdgeLeft[ii+20].y > 5         &&track.pointsEdgeLeft[ii-20].y > 5         && leftring == true && track.pointsEdgeRight[ii].y < COLSIMAGE-1 &&track.pointsEdgeLeft[ii].y < track.pointsEdgeLeft[withi1+1].y))
                                &&abs(track.widthBlock[ii].x-track.widthBlock[withi1+1].x)>20&&track.widthBlock[ii].x<190)
                               {
                                std::cout<<"2环岛找到B点"<<std::endl;
                                std::cout<<"两点之间纵距离为："<<abs(track.widthBlock[ii].x-track.widthBlock[withi1+1].x)<<std::endl;
                                 twopointB = true;
                                 withi2 = ii;
                                 ringStep = RingStep::Entering;
                                  for (int i = 0; i < COLSIMAGE; i++)
                                {
                                circle(imagePath, Point(i,track.widthBlock[withi2].x), 2,
                                    Scalar(255, 255, 255), -1); // 粉色
                                }
                                circle(imagePath, Point(track.pointsEdgeLeft[withi2+1].y+3,track.pointsEdgeLeft[withi2+1].x), 2,
                                Scalar(0, 102, 255), -1); // 橙黄点
                                circle(imagePath, Point(track.pointsEdgeRight[withi2+1].y+3,track.pointsEdgeRight[withi2+1].x), 2,
                                Scalar(0, 102, 255), -1); // 橙黄点
                                 break;
                               }
                            //找A
                            // else if(track.widthBlock[ii+2].y - track.widthBlock[ii].y>10 && ringcircle == false && withi6 < COLSIMAGE/3){
                            //     withi5 = ii;
                            //     if(rightring == true){
                            //         std::cout<<"2环岛暂且补右直线"<<std::endl;
                            //         float k=(float)(track.pointsEdgeRight[withi5].y-track.pointsEdgeRight[0].y)/(float)(track.pointsEdgeRight[withi5].x-track.pointsEdgeRight[0].x);     
                            //         for(int n=0;n<track.pointsEdgeRight.size();n++){
                            //         track.pointsEdgeRight[n].y=(int)(track.pointsEdgeRight[withi5].y-k*(n-withi5));           //进环前补线
                            //         }
                            //     }
                            //     if(leftring == true){
                            //         std::cout<<"2环岛暂且补左直线"<<std::endl;
                            //         float k=(float)(track.pointsEdgeLeft[withi5].y-track.pointsEdgeLeft[0].y)/(float)(track.pointsEdgeLeft[withi5].x-track.pointsEdgeLeft[0].x);     
                            //         for(int n=0;n<track.pointsEdgeLeft.size();n++){
                            //         track.pointsEdgeLeft[n].y=(int)(track.pointsEdgeLeft[withi5].y-k*(n-withi5));           //进环前补线
                            //         }
                            //     }
                            // }
                        }
                        if(twopointB == false){
                            for(int ii = withi1-1; ii > 0; ii--){
                                if(track.widthBlock[ii+2].y - track.widthBlock[ii].y>10 && ringcircle == false && withi6 < COLSIMAGE/3){
                                    withi5 = ii;
                                    gocount2++;
                                    if(track.stdevLeft<40&&track.stdevRight>90){
                                        std::cout<<"2环岛暂且补右直线"<<std::endl;
                                        float k=(float)(track.pointsEdgeRight[withi5].y-track.pointsEdgeRight[0].y)/(float)(track.pointsEdgeRight[withi5].x-track.pointsEdgeRight[0].x);     
                                        for(int n=0;n<track.pointsEdgeRight.size();n++){
                                        track.pointsEdgeRight[n].y=(int)(track.pointsEdgeRight[withi5].y-k*(n-withi5));           //进环前补线
                                        }
                                    }
                                    if(track.stdevLeft>90 &&track.stdevRight<40){
                                        std::cout<<"2环岛暂且补左直线"<<std::endl;
                                        float k=(float)(track.pointsEdgeLeft[withi5].y-track.pointsEdgeLeft[0].y)/(float)(track.pointsEdgeLeft[withi5].x-track.pointsEdgeLeft[0].x);     
                                        for(int n=0;n<track.pointsEdgeLeft.size();n++){
                                        track.pointsEdgeLeft[n].y=(int)(track.pointsEdgeLeft[withi5].y-k*(n-withi5));           //进环前补线
                                        }
                                    }
                                }
                            }
                        }
                       break;
                    }
                    // if(twopointB == false && gocount2 > 3&&ringcircle == false&&withi6 < COLSIMAGE/3){
                    //     for(int i = 0;i<track.widthBlock.size()-10;i++){
                    //         if(track.widthBlock[i+2].y - track.widthBlock[i].y>10){
                    //             withi7 = i;
                    //             if(rightring == true){
                    //                     std::cout<<"2环岛暂且补右直线"<<std::endl;
                    //                     float k=(float)(track.pointsEdgeRight[withi7].y-track.pointsEdgeRight[0].y)/(float)(track.pointsEdgeRight[withi7].x-track.pointsEdgeRight[0].x);     
                    //                     for(int n=0;n<track.pointsEdgeRight.size();n++){
                    //                     track.pointsEdgeRight[n].y=(int)(track.pointsEdgeRight[withi7].y-k*(n-withi7));           //进环前补线
                    //                     }
                    //                 }
                    //                 if(leftring == true){
                    //                     std::cout<<"2环岛暂且补左直线"<<std::endl;
                    //                     float k=(float)(track.pointsEdgeLeft[withi7].y-track.pointsEdgeLeft[0].y)/(float)(track.pointsEdgeLeft[withi7].x-track.pointsEdgeLeft[0].x);     
                    //                     for(int n=0;n<track.pointsEdgeLeft.size();n++){
                    //                     track.pointsEdgeLeft[n].y=(int)(track.pointsEdgeLeft[withi7].y-k*(n-withi7));           //进环前补线
                    //                     }
                    //                 }

                    //             for (int i = 0; i < COLSIMAGE; i++)
                    //            {
                    //             circle(imagePath, Point(i,track.widthBlock[withi7].x), 2,
                    //                 Scalar(0, 80, 0), -1); // 粉色
                    //            } 
                    //         }
                    //     }
                    // }
             }
            
            //1环岛2
            if(lookforB1 == true&&lookforB2 == false){
                std::cout<<"开始找B点"<<std::endl;
                if((track.pointsEdgeRight[i].y<track.pointsEdgeRight[i-10].y&&track.pointsEdgeRight[i].y<track.pointsEdgeRight[i+10].y&&track.stdevLeft<20 &&track.stdevRight>120)||
                    (track.pointsEdgeLeft[i].y>track.pointsEdgeLeft[i-10].y&&track.pointsEdgeLeft[i].y>track.pointsEdgeLeft[i+10].y && track.stdevLeft>120 && track.stdevRight<20)){
                        std::cout<<"1环岛第二种方法找到B点:"<<track.pointsEdgeRight[i].x<<std::endl;
                        if(track.pointsEdgeLeft[withi1-20].y-track.pointsEdgeLeft[i].y>30||track.pointsEdgeRight[i].y-track.pointsEdgeRight[withi1].y>30)
                            break;
                        //withi2=ii;
                        middlepoint = i;
                        rightpoint2=true;
                        countWide2++;
                        if(countWide2 > 5){
                        ringStep = RingStep::Entering;
                        lookforB1 = false;
                        }
                        for (int i = 0; i < COLSIMAGE; i++)
                        {
                            circle(imagePath, Point(i,track.widthBlock[middlepoint].x), 2,
                            Scalar(250, 250, 250), -1); // 白色
                        }
                            break;
                }
            }
        }

        if(twopointC == false && twopointB == false && gocount2 > 3&&ringcircle == false&&withi6 < COLSIMAGE/3){
                        for(int i = 0;i<withi7;i++){
                            if(track.widthBlock[i+2].y - track.widthBlock[i].y>10 && i < withi1){
                                //linepoint = true;
                                withi7 = i;
                                if(track.stdevLeft<40&&track.stdevRight>90){
                                        std::cout<<"无环岛暂且补右直线"<<std::endl;
                                        float k=(float)(track.pointsEdgeRight[withi7].y-track.pointsEdgeRight[0].y)/(float)(track.pointsEdgeRight[withi7].x-track.pointsEdgeRight[0].x);     
                                        for(int n=0;n<track.pointsEdgeRight.size();n++){
                                        track.pointsEdgeRight[n].y=(int)(track.pointsEdgeRight[withi7].y-k*(n-withi7));           //进环前补线
                                        }
                                    }
                                    if(track.stdevLeft>90 &&track.stdevRight<40){
                                        std::cout<<"无环岛暂且补左直线"<<std::endl;
                                        float k=(float)(track.pointsEdgeLeft[withi7].y-track.pointsEdgeLeft[0].y)/(float)(track.pointsEdgeLeft[withi7].x-track.pointsEdgeLeft[0].x);     
                                        for(int n=0;n<track.pointsEdgeLeft.size();n++){
                                        track.pointsEdgeLeft[n].y=(int)(track.pointsEdgeLeft[withi7].y-k*(n-withi7));           //进环前补线
                                        }
                                    }

                                for (int i = 0; i < COLSIMAGE; i++)
                               {
                                circle(imagePath, Point(i,track.widthBlock[withi7].x), 2,
                                    Scalar(0, 80, 0), -1); // 粉色
                               } 
                               break;
                            }
                        }
                    }

        //std::cout<<rowYendStraightside<<std::endl;
                    // [1] 入环判断 countwide计数器大于五，并且存在岔路点，则认为入环
        if ((ringStep == RingStep::None&&countWide>3)||rightpoint==true||ringStep  == RingStep::Entering)
        {
            // if(ringStep  == RingStep::Entering&&rightpoint==false){
            // for(int ii=10;ii < track.widthBlock.size()-30; ii++){
            //     if((track.pointsEdgeRight[ii].y<track.pointsEdgeRight[ii-10].y&&track.pointsEdgeRight[ii].y<track.pointsEdgeRight[ii+10].y)||
            //     (track.pointsEdgeLeft[ii].y>track.pointsEdgeLeft[ii-5].y&&track.pointsEdgeLeft[ii].y-track.pointsEdgeLeft[ii-5].y<40
            //     &&track.pointsEdgeLeft[ii].y>track.pointsEdgeLeft[ii+5].y&&track.pointsEdgeLeft[ii].y-track.pointsEdgeLeft[ii+5].y<40)){
            //         ringStep = RingStep::Entering;
            //         withi2=ii;
            //         rightpoint2=true;
            //                                     for (int i = 0; i < COLSIMAGE; i++)
            //                 {
            //                     circle(imagePath, Point(i,track.widthBlock[withi2].x), 2,
            //                         Scalar(255, 0, 0), -1); // 红色点
            //                 }
            //         break;
            //         }
            //     }
            // }
            //std::cout<<"环岛";
            if (ringTypeTemp == RingType::RingNone) // 环岛方向判定
            {
                if (track.stdevRight<35&&track.stdevLeft>75)
                {
                    std::cout<<"左环岛";
                    ringTypeTemp = RingType::RingLeft;            // 环岛类型：左入环
                    if(ringStep  == RingStep::Entering&&ringType==RingType::RingNone)
                        ringType=RingType::RingLeft;  
                }
                //如果存在前第5个点右集col小于当前col，说明有向右拐的趋向
                else if (track.stdevLeft<35&&track.stdevRight>75&&ringType==RingType::RingNone)
                {
                    std::cout<<"右环岛";
                    ringTypeTemp = RingType::RingRight;            // 环岛类型：右入环
                    if(ringStep  == RingStep::Entering)
                        ringType= RingType::RingRight;  
                }
            }

             if(twopointB == false)
            {
            // if(ringStep == RingStep::Entering&&track.widthBlock[withi1].x>ROWSIMAGE/2)
            //     rightpoint=false;
            if((ringTypeTemp==RingType::RingRight||ringType==RingType::RingRight)&&rightpoint==true&&rightpoint2==false){
                std::cout<<"进去补1直线"<<std::endl;
                float k=(float)(track.pointsEdgeRight[0].y-track.pointsEdgeRight[withi1 - 10].y)/(float)(track.pointsEdgeRight[0].x-track.pointsEdgeRight[withi1 - 10].x);
                if(ringStep == RingStep::Entering&&track.pointsEdgeRight[withi1].x<ROWSIMAGE/2){
                    int row=0;
                    for(int n=0;n<track.pointsEdgeLeft.size();n++){
                        if(track.pointsEdgeLeft[n].y>1){
                            track.pointsEdgeRight[n].y=COLSIMAGE-track.pointsEdgeLeft[n].y;
                            row=n;
                            break;
                        }
                    }
                    for(int n=ROWSIMAGE/4;n<track.pointsEdgeRight.size();n++){
                        if(track.pointsEdgeRight[n].y<track.pointsEdgeRight[n+1].y){
                            k=(float)(track.pointsEdgeRight[0].y-track.pointsEdgeRight[n-1].y)/(float)(n-1-row);
                            break;
                        }
                    }
                    for(int n=0;n<track.pointsEdgeRight.size();n++){
                        int c=(int)(k*(float)(n-row));
                        track.pointsEdgeRight[n].y=track.pointsEdgeRight[row].y-c;
                    }  
                }
                else{
                    for(int n=withi1-10;n<track.pointsEdgeRight.size();n++){
                        track.pointsEdgeRight[n].y=(int)(track.pointsEdgeRight[withi1-11].y-k*(n-withi1+10));           //进环前补线
                    }
                }
            }
            else if(ringType==RingType::RingRight&&ringStep == RingStep::Entering&&rightpoint2==true){
                 std::cout<<"进去补2直线"<<std::endl;
                 if(rightpoint==false){
                    std::cout<<"没有";
                    for(int n=withi2;n>withi1;n--){
                        track.pointsEdgeRight[n].y=(int)track.pointsEdgeRight[withi2].y;
                    }
                 }
                else{
                    std::cout<<"进去补3直线"<<std::endl;
                    float k=(float)(track.pointsEdgeRight[withi2].y-track.pointsEdgeRight[0].y)/(float)withi2;
                    for(int n=withi1-5;n<track.pointsEdgeLeft.size();n++){
                        int c=(int)(k*(float)(n-withi1+5));
                        track.pointsEdgeRight[n].y=track.pointsEdgeRight[withi1-5].y+c;
                    }
                }
            }
            if((ringTypeTemp==RingType::RingLeft||ringType==RingType::RingLeft)&&rightpoint==true&&rightpoint2==false){
                std::cout<<"进去补1直线"<<std::endl;
                float k=(float)(track.pointsEdgeLeft[0].y-track.pointsEdgeLeft[withi1 - 10].y)/(float)(track.pointsEdgeLeft[0].x-track.pointsEdgeLeft[withi1 - 10].x);
                // if(ringStep == RingStep::Entering&&track.pointsEdgeLeft[withi1].x<ROWSIMAGE/2){
                //     int row=0;
                //     for(int n=0;n<track.pointsEdgeRight.size();n++){
                //         if(track.pointsEdgeRight[n].y<COLSIMAGE-1){
                //             track.pointsEdgeLeft[n].y=COLSIMAGE-track.pointsEdgeRight[n].y;
                //             row=n;
                //             break;
                //         }
                //     }
                //     for(int n=ROWSIMAGE/4;n<track.pointsEdgeLeft.size();n++){
                //         if(track.pointsEdgeLeft[n+1].y<track.pointsEdgeLeft[n].y){
                //             k=(float)(track.pointsEdgeLeft[n-1].y-track.pointsEdgeLeft[0].y)/(float)(n-1-row);
                //             break;
                //         }
                //     }
                //     for(int n=0;n<track.pointsEdgeLeft.size();n++){
                //         int c=(int)(k*(float)(n-row));
                //         track.pointsEdgeLeft[n].y=track.pointsEdgeLeft[row].y+c;
                //     }  
                // }
                // else{
                    for(int n=withi1-10;n<track.pointsEdgeLeft.size();n++){
                        track.pointsEdgeLeft[n].y=(int)(track.pointsEdgeLeft[withi1-11].y-k*(n-withi1+10));           //进环前补线
                    }
                //}
            }
            else if(ringType==RingType::RingLeft&&ringStep == RingStep::Entering&&rightpoint2==true){
                 std::cout<<"进去补2直线"<<std::endl;
                if(rightpoint==false){
                    std::cout<<"没有";
                    for(int n=withi2;n>withi1;n--){
                        track.pointsEdgeLeft[n].y=(int)track.pointsEdgeLeft[withi2].y;
                    }
                 }
                else{
                    std::cout<<"进去补3直线"<<std::endl;
                    float k=(float)(track.pointsEdgeLeft[withi2].y-track.pointsEdgeLeft[0].y)/(float)withi2;
                    for(int n=withi1-5;n<track.pointsEdgeLeft.size();n++){
                        int c=(int)(k*(float)(n-withi1+5));
                        track.pointsEdgeLeft[n].y=track.pointsEdgeLeft[withi1-5].y+c;
                    }
                }
            }
            //std::cout<<ringTypeTemp<<std::endl;;)
         }
         else 
         {
            if(ringType == RingType::RingRight){
                std::cout<<"2环岛补右直线"<<std::endl;
                float k=(float)(track.pointsEdgeRight[withi1+3].y-track.pointsEdgeRight[withi2].y)/(float)(track.pointsEdgeRight[withi1+3].x-track.pointsEdgeRight[withi2].x);     
                for(int n=0;n<track.pointsEdgeRight.size();n++){
                    track.pointsEdgeRight[n].y=(int)(track.pointsEdgeRight[withi2].y-k*(n-withi2));           //进环前补线
                }
            }
            else if(ringType == RingType::RingLeft){
                std::cout<<"2环岛补左直线"<<std::endl;
                float k=(float)(track.pointsEdgeLeft[withi1+3].y-track.pointsEdgeLeft[withi2].y)/(float)(track.pointsEdgeLeft[withi1+3].x-track.pointsEdgeLeft[withi2].x);     
                for(int n=0;n<track.pointsEdgeLeft.size();n++){
                    track.pointsEdgeLeft[n].y=(int)(track.pointsEdgeLeft[withi2].y-k*(n-withi2));           //进环前补线
                }
            }
         }
        }//ting


        //c点阶段
        if(ringStep == RingStep::Entering||ringStep == RingStep::Inside){
            std::cout<<"开始找C点"<<std::endl;
           bool phase=false;
            if(ringType==RingType::RingLeft&&ringStep == RingStep::Entering){
                for(int i=0;i<track.pointsEdgeLeft.size();i++){
                    if(track.pointsEdgeLeft[i].y<2&&track.pointsEdgeLeft[i-1].y-track.pointsEdgeLeft[i].y>20){
                        if(track.pointsEdgeLeft[i-5].y-track.pointsEdgeLeft[i-10].y>0)
                           phase=true;
                    }
                }
            }//gai
            for (int i = track.widthBlock.size(); i >1; i--){
                if(track.widthBlock[i].y-track.widthBlock[i+1].y>10&&
                track.widthBlock[i].y>COLSIMAGE*0.5&&track.widthBlock[i+1].y<COLSIMAGE*0.7&&track.widthBlock[i].x<150&&track.widthBlock[i].x>35&&
                ((ringType== RingType::RingLeft&&abs(track.pointsEdgeLeft[i+1].y-track.pointsEdgeLeft[i+5].y)<3)||(ringType== RingType::RingRight&&abs(track.pointsEdgeRight[i+1].y-track.pointsEdgeRight[i+5].y)<3))&&
                phase==false&&rightpoint==false&&rightpoint2==false&&twopointB == false){
                    if(Cswitch == false){
                    if(findcount == 0){
                        if((ringType== RingType::RingLeft)||(ringType== RingType::RingRight&&track.pointsEdgeRight[i+1].y<COLSIMAGE*0.98&&track.pointsEdgeRight[i].y<COLSIMAGE*0.98))
                        {
                            std::cout<<"第一个C点找到"<<std::endl;
                            withi2=i;
                            circle(imagePath, Point(track.pointsEdgeLeft[withi2].y,track.pointsEdgeLeft[withi2].x), 2,
                            Scalar(0, 102, 255), -1); // 橙黄点
                            circle(imagePath, Point(track.pointsEdgeRight[withi2].y,track.pointsEdgeRight[withi2].x), 2,
                            Scalar(0, 102, 255), -1); // 橙黄点
                            ringStep = RingStep::Inside;
                            ringcircle=true;
                            findcount++;
                            break;
                        }
                        else{
                            break;
                        }
                    }
                    else{
                    std::cout<<"其他C点找到"<<std::endl;
                    withi2=i;
                    circle(imagePath, Point(track.pointsEdgeLeft[withi2].y,track.pointsEdgeLeft[withi2].x), 2,
                        Scalar(0, 102, 255), -1); // 橙黄点
                    circle(imagePath, Point(track.pointsEdgeRight[withi2].y,track.pointsEdgeRight[withi2].x), 2,
                        Scalar(0, 102, 255), -1); // 橙黄点
                            
                    ringStep = RingStep::Inside;
                    ringcircle=true;
                    break;}
                    }
                    else if(Cswitch == true){
                         if(findcount == 0&&abs(track.widthBlock[withi1].x - track.widthBlock[i].x) < 30&&track.widthBlock[i].x < track.widthBlock[withi2].x){
                            std::cout<<"C点找到"<<std::endl;
                            withi2=i;
                            circle(imagePath, Point(track.pointsEdgeLeft[withi2].y,track.pointsEdgeLeft[withi2].x), 2,
                            Scalar(0, 102, 255), -1); // 橙黄点
                            circle(imagePath, Point(track.pointsEdgeRight[withi2].y,track.pointsEdgeRight[withi2].x), 2,
                            Scalar(0, 102, 255), -1); // 橙黄点
                            
                            ringStep = RingStep::Inside;
                            ringcircle=true;
                            findcount++;
                            break;
                         }
                         else if(findcount>0){
                            std::cout<<"其他C点找到"<<std::endl;
                            withi2=i;
                            circle(imagePath, Point(track.pointsEdgeLeft[withi2].y,track.pointsEdgeLeft[withi2].x), 2,
                            Scalar(0, 102, 255), -1); // 橙黄点
                            circle(imagePath, Point(track.pointsEdgeRight[withi2].y,track.pointsEdgeRight[withi2].x), 2,
                            Scalar(0, 102, 255), -1); // 橙黄点
                            
                            ringStep = RingStep::Inside;
                            ringcircle=true;
                            break;
                         }
                    }
                }
                if( i < 3 && ringcircle==true){
                    count++;
                    if(count>12){     //小圆环为5 原为12
                        ringStep = RingStep::Circle;
                    }
                }
            }
        }
        if(ringStep == RingStep::Inside||ringStep == RingStep::Circle){
            bool line=false;
            if(ringStep == RingStep::Circle){
                if(track.pointsEdgeLeft.size()<10||track.pointsEdgeRight.size()<10){
                    line=true;
                    std::cout<<"缺";
                }
            }
            if(ringType== RingType::RingRight&&(ringStep == RingStep::Inside)||(line==true&&ringStep == RingStep::Circle)){
                std::cout<<"C点补线"<<std::endl;
                for (int i = 0; i < COLSIMAGE; i++)
                {
                    circle(imagePath, Point(i,track.pointsEdgeRight[withi2].x), 2,
                        Scalar(255, 255, 255), -1); // 红色点
                }
                int x= track.pointsEdgeRight[withi2].x-30;
                int y= COLSIMAGE/2;
                POINT startPoint=track.pointsEdgeLeft[0];
                for(int i=COLSIMAGE;i>COLSIMAGE*0.8;i--){
                    startPoint = track.pointsEdgeLeft[COLSIMAGE-i];
                    if( track.pointsEdgeLeft[COLSIMAGE-i].y>2)
                        break;
                }
                POINT midPoint(x, y);                                            // 补线：中点
                POINT endPoint(track.pointsEdgeLeft[withi2-10].x,COLSIMAGE);                          // 补线：终点

                vector<POINT> input = {startPoint, midPoint, endPoint};
                vector<POINT> b_modify = Bezier(0.01, input);
                track.pointsEdgeLeft.resize( b_modify.size());
                track.pointsEdgeRight.resize( b_modify.size());
                for (int kk = 1; kk < b_modify.size(); ++kk)
                {
                    track.pointsEdgeLeft[kk]=b_modify[kk];
                }
            }
            if(ringType== RingType::RingLeft&&(ringStep == RingStep::Inside)||(line==true&&ringStep == RingStep::Circle)){
                std::cout<<"C点补线"<<std::endl;
                for (int i = 0; i < COLSIMAGE; i++)
                {
                    circle(imagePath, Point(i,track.pointsEdgeLeft[withi2].x), 2,
                        Scalar(255, 255, 255), -1); // 红色点
                }
                int x= track.pointsEdgeLeft[withi2].x-30;
                int y= COLSIMAGE/2;
                POINT startPoint=track.pointsEdgeRight[0];
                for(int i=COLSIMAGE;i>COLSIMAGE*0.8;i--){
                    startPoint = track.pointsEdgeRight[COLSIMAGE-i];
                    if( track.pointsEdgeRight[COLSIMAGE-i].y<COLSIMAGE)
                        break;
                }
                POINT midPoint(x, y);                                            // 补线：中点
                POINT endPoint( track.pointsEdgeLeft[withi2-10].x,0);                          // 补线：终点

                vector<POINT> input = {startPoint, midPoint, endPoint};
                vector<POINT> b_modify = Bezier(0.01, input);
                track.pointsEdgeRight.resize( b_modify.size());
                track.pointsEdgeLeft.resize( b_modify.size());
                for (int kk = 1; kk < b_modify.size(); ++kk)
                {
                    track.pointsEdgeRight[kk]=b_modify[kk];
                }
            }
        }
        if(ringStep == RingStep::Circle || ringStep == RingStep::Exiting){
            int withi3=0;
            for(int i=10;i<track.widthBlock.size();i++){
                    if(i<track.widthBlock.size()-10){
                        if((ringType== RingType::RingRight&&(track.pointsEdgeLeft[i].y>track.pointsEdgeLeft[i+5].y&&track.pointsEdgeLeft[i].y>track.pointsEdgeLeft[i-5].y))||
                        (ringType== RingType::RingLeft&&(track.pointsEdgeRight[i].y<track.pointsEdgeRight[i+5].y&&track.pointsEdgeRight[i].y<track.pointsEdgeRight[i-5].y))){
                        //防止鬼打墙
                        if(findcount2 == 0 && track.widthBlock[i].x > track.widthBlock[track.widthBlock.size()-1].x && ((ringType== RingType::RingRight&&track.pointsEdgeLeft[i+5].y>5&&track.pointsEdgeLeft[i-5].y>5&&track.pointsEdgeLeft[i+5].y<COLSIMAGE -5&&track.pointsEdgeLeft[i-5].y<COLSIMAGE -5)||(ringType== RingType::RingLeft&&track.pointsEdgeRight[i+5].y<COLSIMAGE -5&&track.pointsEdgeRight[i-5].y<COLSIMAGE-5&&track.pointsEdgeRight[i+5].y>5&&track.pointsEdgeRight[i-5].y>5))){
                            exit=true;
                            exitpoint = true;
                            withi3=i;
                            findcount2++;
                            break;
                        }
                        else if(findcount2 > 0){
                            exit=true;
                            exitpoint = true;
                            withi3=i;
                            break;
                        }
                      }
                    }
            }
            if(withi3<track.widthBlock.size()-20&&withi3>0){
                withi2=withi3;
                ringStep = RingStep::Exiting;
            }
            if(ringType == RingType::RingRight&&exit==true){
                    std::cout<<"出环岛"<<withi2<<std::endl;
                    for (int i = 0; i < COLSIMAGE; i++)
                        {
                            circle(imagePath, Point(i,withi2), 2,
                                Scalar(0, 0, 0), -1); // 红色点
                        }
                    int x= track.pointsEdgeLeft[withi2+5].x;
                    int y= track.pointsEdgeLeft[withi2+5].y;
                    POINT startPoint=track.pointsEdgeLeft[0];
                    POINT midPoint(x, y);                                            // 补线：中点
                    POINT endPoint = track.pointsEdgeRight.back();
                    if(track.pointsEdgeRight[0].y==320){
                        for(int ii=0;ii<track.pointsEdgeRight.size();ii++){
                            if(track.pointsEdgeRight[ii].y<320){
                                endPoint.x=track.pointsEdgeRight[ii].x;
                                endPoint.y=320;
                                break;
                            }
                        }
                    }
                    else{
                        endPoint.x=track.pointsEdgeRight.back().x;
                        endPoint.y=320;
                    }
                    vector<POINT> input = {startPoint, midPoint, endPoint};
                    vector<POINT> b_modify = Bezier(0.01, input);
                    track.pointsEdgeLeft.resize( b_modify.size());
                    track.pointsEdgeRight.resize( b_modify.size());
                    for (int kk = 1; kk < b_modify.size(); ++kk)
                    {
                        track.pointsEdgeLeft[kk]=b_modify[kk];
                    }
                }

            if(ringType == RingType::RingLeft&&exit==true){
                    std::cout<<"出环岛"<<withi2<<std::endl;
                    for (int i = 0; i < COLSIMAGE; i++)
                        {
                            circle(imagePath, Point(i,withi2), 2,
                                Scalar(0, 0, 255), -1); // 红色点
                        }
                    int n= track.pointsEdgeRight.size();
                    int x= track.pointsEdgeRight[n-withi2].x;
                    int y= track.pointsEdgeRight[n-withi2].y;
                    POINT startPoint=track.pointsEdgeRight[0];
                    POINT midPoint(x, y);                                            // 补线：中点
                    POINT endPoint = track.pointsEdgeLeft.back();
                    if(track.pointsEdgeLeft[0].y==0){
                        for(int ii=0;ii<track.pointsEdgeLeft.size();ii++){
                            if(track.pointsEdgeLeft[ii].y>10){
                                endPoint.x=track.pointsEdgeLeft[ii].x;
                                endPoint.y=0;
                                break;
                            }
                        }
                    }
                    else{
                        endPoint.x=track.pointsEdgeLeft.back().x;
                        endPoint.y=0;
                    }
                    vector<POINT> input = {startPoint, midPoint, endPoint};
                    vector<POINT> b_modify = Bezier(0.01, input);
                    track.pointsEdgeLeft.resize( b_modify.size());
                    track.pointsEdgeRight.resize( b_modify.size());
                    for (int kk = 1; kk < b_modify.size(); ++kk)
                    {
                        track.pointsEdgeRight[kk]=b_modify[kk];
                    }
            }
        }
        if(ringStep == RingStep::Exiting){
            if((ringType == RingType::RingRight&&track.stdevLeft<150&&track.pointsEdgeLeft.back().x<ROWSIMAGE/2)||
            (ringType == RingType::RingLeft&&track.stdevRight<150&&track.pointsEdgeRight.back().x<ROWSIMAGE/2))//出环条件变松
            {
                std::cout<<"出环在加"<<std::endl;
                gocount++;
            }
            if(gocount > 25){//中小圆环为15 大圆环为40
                ringStep = RingStep::Finish;
            }//ci
        }
        
        if(ringStep == RingStep::Finish ){//jiade
            std::cout<<"开始找D点"<<std::endl;
            for (int i = track.widthBlock.size(); i >1; i--){
                if(outcount == 0){
                if(track.widthBlock[i].y-track.widthBlock[i+1].y>15&&track.widthBlock[i].y>COLSIMAGE*0.50&&
                track.widthBlock[i+1].y<COLSIMAGE*0.8&&((track.stdevLeft<50 &&track.stdevRight>100)||(track.stdevLeft>100 &&track.stdevRight<50))&&exitpoint==false){
                    std::cout<<"第一个D点找到"<<std::endl;
                    //ringStep = RingStep::Finish;//jiade
                    withi2=i;
                    for (int i = 0; i < COLSIMAGE; i++)
                   {
                    circle(imagePath, Point(i,track.widthBlock[withi2].x), 2,
                        Scalar(255, 255, 255), -1); // 红色点
                   }
                    circle(imagePath, Point(track.pointsEdgeLeft[withi2].y,track.pointsEdgeLeft[withi2].x), 2,
                        Scalar(0, 102, 255), -1); // 橙黄点
                    circle(imagePath, Point(track.pointsEdgeRight[withi2].y,track.pointsEdgeRight[withi2].x), 2,
                        Scalar(0, 102, 255), -1); // 橙黄点
                   outpoint = true;  
                   outcircle = true;
                   outcount++;
                    break;
                }
            }
            else
            {
                    if((track.widthBlock[i].y-track.widthBlock[i+1].y>15)&&track.widthBlock[i].x > ROWSIMAGE/5){
                    std::cout<<"其它D点找到"<<std::endl;  
                    withi2=i;
                    for (int i = 0; i < COLSIMAGE; i++)
                   {
                    circle(imagePath, Point(i,track.widthBlock[withi2].x), 2,
                        Scalar(255, 255, 255), -1); // 红色点
                   }
                    circle(imagePath, Point(track.pointsEdgeLeft[withi2].y,track.pointsEdgeLeft[withi2].x), 2,
                        Scalar(0, 102, 255), -1); // 橙黄点
                    circle(imagePath, Point(track.pointsEdgeRight[withi2].y,track.pointsEdgeRight[withi2].x), 2,
                        Scalar(0, 102, 255), -1); // 橙黄点
                   outpoint = true;  
                   outcircle = true;
                    break;
                }
            }
                if( i < 3 && outcircle==true){
                        ringStep = RingStep::None;
                        reset();
                }
            }

            if(outpoint == true && withi2 > 0){
                    /*if(ringType == RingType::RingRight){
                    std::cout<<"出去补右直线"<<std::endl;
                    float k=(float)(track.pointsEdgeRight[withi2+3].x-track.pointsEdgeRight[withi2].x)/(float)(track.pointsEdgeRight[withi2+3].y-track.pointsEdgeRight[withi2].y);     
                    for(int n=0;n<withi2;n++){
                        track.pointsEdgeRight[n].y=(int)(track.pointsEdgeRight[withi2+1].y+k*(withi2+1-n));           //进环前补线
                    }
                  }
                    if(ringType == RingType::RingLeft){
                    std::cout<<"出去补左直线"<<std::endl;
                    float k=(float)(track.pointsEdgeLeft[withi2+3].x-track.pointsEdgeLeft[withi2].x)/(float)(track.pointsEdgeLeft[withi2+3].y-track.pointsEdgeLeft[withi2].y);
                    for(int n=0;n<withi2;n++){
                        track.pointsEdgeLeft[n].y=(int)(track.pointsEdgeLeft[withi2+1].y+k*(withi2+1-n));           //进环前补线
                    }
                  }*/

                if(ringType == RingType::RingRight){
                    std::cout<<"出去补右直线"<<std::endl;
                    track.pointsEdgeRight.clear();
                    RepairLineBySlope(track.pointsEdgeRight[withi2+3], track.pointsEdgeRight[withi2+30], 239, track.pointsEdgeRight, false,5000);
                }
                if(ringType == RingType::RingLeft){
                    track.pointsEdgeLeft.clear();
                    std::cout<<"出去补左直线"<<std::endl;
                    RepairLineBySlope(track.pointsEdgeLeft[withi2+3], track.pointsEdgeLeft[withi2+30], 239, track.pointsEdgeLeft, false,5000);
                }
            }
        }


        // 返回识别结果
        if (ringStep == RingStep::None)
            return false;
        else
            return true;
    };


    //补线函数开始
    // 基于斜率补线的函数
    void RepairLineBySlope(
        const POINT& startPoint, 
        const POINT& endPoint, 
        int targetY,        // 设置为239
        std::vector<POINT>& outputLine,
        bool isLeftLine = true,  // 这个为true就是限制x轴范围
        int maxLength = 300        // 最大补线长度
    ) 
    {
        // 计算斜率
        float slope = 0.0f;
        if (endPoint.y != startPoint.y) {
            slope = static_cast<float>(endPoint.x - startPoint.x) / (endPoint.y - startPoint.y);
        }
        else if(endPoint.y == startPoint.y)
        {
            endPoint.y+2;
            slope = static_cast<float>(endPoint.x - startPoint.x) / (endPoint.y - startPoint.y);
        }

        cout<<"slope "<<slope<<endl;
        
        // 生成补线点
        outputLine.clear();
        // if(abs(slope>5))
        // {
            for (int x = targetY; x >=0 ; x--) {
                // int x = startPoint.x + static_cast<int>(slope * (y - startPoint.y));
                int y= (x-startPoint.x)/slope+startPoint.y;

                POINT pt = {x, y};
                outputLine.push_back(pt);
        
                // 计算当前点到起点的距离
                float distance = std::sqrt(std::pow(pt.x - startPoint.x, 2) + std::pow(pt.y - startPoint.y, 2));
                
                // cout<<"distance "<<distance<<endl;
                // 如果距离超过最大长度，停止补线
                if (distance >= maxLength) {
                    break;
                }
            }
        // }
    }
    //补线函数结束
    /**
     * @brief 绘制环岛识别图像
     *
     * @param ringImage 需要叠加显示的图像
     */
    void drawImage(Tracking track, Mat &ringImage)
    {
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(ringImage, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 2,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(ringImage, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 2,
                   Scalar(0, 255, 255), -1); // 黄色点
        }

        for (int i = 0; i < track.spurroad.size(); i++)
        {
            circle(ringImage, Point(track.spurroad[i].y, track.spurroad[i].x), 5,
                   Scalar(0, 0, 255), -1); // 红色点
        }

        putText(ringImage, to_string(_ringStep) + " " + to_string(_ringEnable) + " " + to_string(_tmp_ttttt),
                Point(COLSIMAGE - 80, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

        putText(ringImage, to_string(_index), Point(80, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

        putText(ringImage, to_string(track.validRowsRight) + " | " + to_string(track.stdevRight),
                Point(COLSIMAGE - 100, ROWSIMAGE - 50),
                FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 1, cv::LINE_AA);
        putText(ringImage, to_string(track.validRowsLeft) + " | " + to_string(track.stdevLeft),
                Point(30, ROWSIMAGE - 50), FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 1, cv::LINE_AA);

        putText(ringImage, "[7] RING - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        circle(ringImage, Point(_ringPoint.y, _ringPoint.x), 4, Scalar(255, 0, 0), -1); // 蓝色点，入环点
    }
    uint16_t counterSpurroad = 0; // 岔路计数器
    // 临时测试用参数
    int _ringStep;
    int _ringEnable;
    int _tmp_ttttt;
    int _index = 0;
    POINT _ringPoint = POINT(0, 0);

    /**
     * @brief 环岛类型
     *
     */
    enum RingType
    {
        RingNone = 0, // 未知类型
        RingLeft,     // 左入环岛
        RingRight     // 右入环岛
    };

    /**
     * @brief 环岛运行步骤/阶段
     *
     */
    enum RingStep
    {
        None = 0, // 未知类型
        Entering, // 入环
        Inside,   // 环中
        Circle,
        Exiting,  // 出环
        Finish    // 环任务结束
    };
    RingStep ringStep = RingStep::None;     // 环岛处理阶段
    RingType ringType = RingType::RingNone; // 环岛类型
    int rowRepairLine = 0;                  // 用于环补线的点（行号）
    int colRepairLine = 0;                  // 用于环补线的点（列号）
};