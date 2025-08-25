#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2025; SaiShu.Lcc.; HC; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习,请勿商用).
 *            The code ADAPTS the corresponding hardware circuit board(代码适配百度Edgeboard-智能汽车赛事版),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file parking.cpp
 * @author HC (sasu@saishukeji.com)
 * @brief 充电停车场
 * @version 0.1
 * @date 2025/03/04 20:29:04
 * @copyright  :Copyright (c) 2024
 * @note 具体功能模块:
 */
 
#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../../include/detection.hpp"
#include "../recognition/tracking.cpp"
 
using namespace cv;
using namespace std;
 
class Parking
{
public:
    vector<Vec4i> hengxian;
    int carY = ROWSIMAGE;//改为成员变量
    POINT hengxian1=POINT(0,0);
    int exitCountdown = 0;        // 退出倒计时(毫秒)
    int trackout_count=0;         //倒车延时
    int trackout_count1=20;        //每次延时

    float chao_slope=2.6;
    float slope6=0;//斜率检测


    POINT LA_right=POINT(0,0);//拉线点，不拉太不稳定了
    POINT LA_right1=POINT(0,0);//拉线点二

    bool turning_ing=false;//拐弯时机

    bool garageFirst = true;      // 进入一号车库,一号车库和二号车库参数不一样,一号车库是远端,二号车库是近端

    bool turningspeed=false;      //拐弯速度再降速一些

    bool stopEnable=false;//停车使能

    bool into_turn=false;//转弯使能

    vector<Vec4i> stophenxian;//停车横线


    /**
     * @brief 停车步骤
     *
     */
    enum ParkStep
    {
        none = 0, // 未知状态
        enable,   // 停车场使能
        turning,  // 入库转向
        stop,     // 停车
        trackout, // 出库
        exit      // 倒计时退出状态
    };
    
    ParkStep step = ParkStep::none; // 停车步骤
    uint16_t counterSession = 0;  // 图像场次计数器
    bool found_carY=false;//计算是否找到carY
    
    void setParams7(const Motion::Params &params) {
       stopTime=params.parkingstopTime;
       truningTime=params.parkingtruningTime;
       truningTime2=params.parkingtruningTime2;
       swerveTime=params.parkingswerveTime;
       exitCountdown=params.exitCountdown;
       trackout_count1=params.trackout_count1;
       stop_line=params.stop_line;
    }
    
    bool process(Tracking &track, Mat &image, vector<PredictResult> predict)
    {
        // stophenxian.clear();
        counterSession++;
        if (step != ParkStep::none && counterSession > 500) // 超时退出
        {
            resetState();
            std::cout << "超时退出停车场" << std::endl;
        }
        
        switch (step)
        {
            case ParkStep::none: // AI未识别
            {
                for (size_t i = 0; i < predict.size(); i++)
                {
                    if ((predict[i].type == LABEL_BATTERY) && predict[i].score > 0.5&& (predict[i].y + predict[i].height) > ROWSIMAGE * 0.15)
                    {
                        cout<<"高度 "<<(predict[i].y + predict[i].height)<<endl;
                        counterRec++;
                        if(predict[i].x<COLSIMAGE/2)
                            rightEnable=false;      //左停
                        else
                            rightEnable=true;       //右停
                        break;
                    }
                }
                if (counterRec) // 检测到一帧后开始连续监测AI标志是否满足条件
                {
                    if (counterRec >= 1 && counterSession < 2)
                    {
                        resetCounters();
                        step = ParkStep::enable; // 检测到停车场标志
                        std::cout << "进入停车场" << std::endl;
                        return true;
                    }
                    else if (counterSession >= 2)
                    {
                        resetCounters();
                    }
                }
                return false;
                break;
            }
           case ParkStep::enable: // 停车场使能
            {

                
                int batteryY = ROWSIMAGE;     // 充电站标识高度
                int batteryX =0;
                found_carY = false; // 初始化


                if(rightEnable)    //右停
                {
                    // // 固定位置，让其稳定
                    // if (track.pointsEdgeRight.size() < track.pointsEdgeLeft.size())
                    // track.pointsEdgeLeft.resize(track.pointsEdgeRight.size());

                    // for (size_t i = 0; i < track.pointsEdgeLeft.size(); i++)
                    // {
                    //     track.pointsEdgeLeft[i].y = track.pointsEdgeLeft[i].y+40;
                    // }

                    // if (track.pointsEdgeRight.size() > track.pointsEdgeLeft.size())
                    // track.pointsEdgeRight.resize(track.pointsEdgeLeft.size());

                    // for (size_t i = 0; i < track.pointsEdgeRight.size(); i++)
                    // {
                    //     track.pointsEdgeRight[i].y =track.pointsEdgeLeft[i].y+220 ;
                    // }

                    for (size_t i = 0; i < predict.size(); i++)
                    {
                        if (predict[i].type == LABEL_CAR && predict[i].score > 0.92)
                        {
                            carY = predict[i].y + predict[i].height;   // 计算智能车的中心高度
                            cout << "识别到carY " << carY << endl;
                            found_carY = true;
                        }
                        else if ((predict[i].type == LABEL_BATTERY) && predict[i].score > 0.4)
                        {
                            batteryY = predict[i].y;   // 计算标识牌最高高度
                            batteryX=  predict[i].x;
                        }
                    }

                    //  拉线处理
                    
                    //第一步------找点
                    for(int i=track.pointsEdgeRight.size()-20;i>5;i--)
                    {
                        if(track.pointsEdgeRight[i].y-track.pointsEdgeRight[i-5].y<-10&&track.pointsEdgeRight[i].y<batteryX&&track.pointsEdgeRight[i].x<batteryY)
                        {
                            LA_right=track.pointsEdgeRight[track.pointsEdgeRight.size()-20];
                            LA_right1=track.pointsEdgeRight[track.pointsEdgeRight.size()-23];
                            cout<<"找到拉线角点了"<<endl;
                            cout<<"LA_right.y "<<LA_right.y<<"LA_right.x "<<LA_right.x<<endl;
                            cout<<"LA_right1.y "<<LA_right1.y<<"LA_right1.x "<<LA_right1.x<<endl;
                            break;
                        }
                    }

                    //补线
                    if(LA_right.y>100)
                    {   
                        slope6=Lineslope(LA_right,LA_right1);
                        if(abs(slope6)>chao_slope&&LA_right.y<310)
                        {
                            track.pointsEdgeRight.clear();
                            RepairLineBySlope(LA_right,LA_right1,239,track.pointsEdgeRight,false,5000);
                        }
                        else
                        {

                            LA_right=track.pointsEdgeRight[0];
                            LA_right1=track.pointsEdgeRight[10];
                            slope6=Lineslope(LA_right,LA_right1);
                            if(abs(slope6)>chao_slope)
                            {
                                track.pointsEdgeRight.clear();
                                RepairLineBySlope(LA_right,LA_right1,239,track.pointsEdgeRight,false,5000);
                            }
                        }
                    }
                    //-------------------完事-----------------------------------------
                    
                    if (!found_carY) {
                        carY = ROWSIMAGE; // 未检测到车辆时设为默认值
                        cout << "未检测到车辆，carY 设为默认值 " << carY << endl;
                    }
                    
                    // 图像预处理
                    Mat edges;
                    Canny(image, edges, 50, 150);

                    // 霍夫变换检测直线
                    vector<Vec4i> lines;
                    HoughLinesP(edges, lines, 1, CV_PI/180, 35, 3, 20);

                    vector<Vec4i> horizontalLines;
                    Mat imgRes = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
                    
                    // 先筛选出所有接近水平的线
                    for(const Vec4i& line : lines) 
                    {
                        Point pt1(line[0], line[1]);
                        Point pt2(line[2], line[3]);
                        
                        double angle = atan2(pt2.y - pt1.y, pt2.x - pt1.x) * 180.0 / CV_PI;
                        int midX = (line[0] + line[2]) / 2;
                        int midY = (line[1] + line[3]) / 2;

                        cout<<"batteryY "<<batteryY<<endl;

                        // 筛选直线，直线只出现在右侧并且在充电标识牌的上方
                        if(abs(angle) < 30 && midX > COLSIMAGE/2 && midY>35)
                        {
                            horizontalLines.push_back(line);
                            cv::line(imgRes, Point(line[0], line[1]), Point(line[2], line[3]), Scalar(0, 0, 255), 2);
                        }
                    }
                    
                    // 合并距离小于10像素的横线
                    vector<Vec4i> mergedLines;
                    vector<bool> merged(horizontalLines.size(), false);
                    
                    for (size_t i = 0; i < horizontalLines.size(); ++i) {
                        if (merged[i]) continue;
                        
                        Vec4i currentLine = horizontalLines[i];
                        int currentStartY = min(currentLine[1], currentLine[3]);
                        int currentEndY = max(currentLine[1], currentLine[3]);
                        
                        for (size_t j = i + 1; j < horizontalLines.size(); ++j) {
                            if (merged[j]) continue;
                            
                            Vec4i otherLine = horizontalLines[j];
                            int otherStartY = min(otherLine[1], otherLine[3]);
                            int otherEndY = max(otherLine[1], otherLine[3]);
                            
                            // 计算两条线在y轴上的重叠或距离
                            int yDistance = abs((currentStartY + currentEndY)/2 - (otherStartY + otherEndY)/2);
                            
                            if (yDistance < 35) { // 如果距离小于10像素，合并这两条线
                                // 合并为一条更长的线
                                int newX1 = min(currentLine[0], otherLine[0]);
                                int newY1 = min(currentLine[1], otherLine[1]);
                                int newX2 = max(currentLine[2], otherLine[2]);
                                int newY2 = max(currentLine[3], otherLine[3]);
                                
                                currentLine = Vec4i(newX1, newY1, newX2, newY2);
                                currentStartY = min(newY1, newY2);
                                currentEndY = max(newY1, newY2);
                                
                                merged[j] = true; // 标记为已合并
                            }
                        }
                        
                        mergedLines.push_back(currentLine);
                        merged[i] = true;
                    }
                    
                    hengxian.clear();
                    hengxian = mergedLines; // 使用合并后的线
                    
                    // 绘制合并后的线
                    Mat mergedImg = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3);
                    for (const auto& line : mergedLines) {
                        cv::line(mergedImg, Point(line[0], line[1]), Point(line[2], line[3]), Scalar(0, 255, 0), 2);
                    }
                    // imshow("Merged Lines", mergedImg);
                    // waitKey(0);
                    
                    cout << "合并后的横线数量: " << mergedLines.size() << endl;
                    
                    if (mergedLines.size() >= 2 )
                    {
                        cout << "进来充电第二步了" << endl;
                        // 查找平行且距离大于20的线对
                        for(size_t i = 0; i < mergedLines.size(); i++) {
                            for(size_t j = i + 1; j < mergedLines.size(); j++) {
                                // 计算两条线的中点y坐标
                                int midY1 = (mergedLines[i][1] + mergedLines[i][3]) / 2;
                                int midY2 = (mergedLines[j][1] + mergedLines[j][3]) / 2;
                                
                                Vec4i line1 = mergedLines[i];
                                Vec4i line2 = mergedLines[j];

                                // 计算两条线的角度
                                double angle1 = atan2(line1[3] - line1[1], line1[2] - line1[0]) * 180.0 / CV_PI;
                                double angle2 = atan2(line2[3] - line2[1], line2[2] - line2[0]) * 180.0 / CV_PI;

                                // 检测条件
                                cout << "max(midY1,midY2) " << max(midY1, midY2) << endl;
                                cout << "min(midY1,midY2) " << min(midY1, midY2) << endl;
                                cout << "abs(angle1 - angle2) " << abs(angle1 - angle2) << endl;
                                
                                // 如果两条线距离大于20像素并且角度差小于25度
                                if(abs(midY1 - midY2) > 20  && step == ParkStep::enable )
                                {
                                    cout << "进来充电第三步了" << " 此时的carY" << carY << endl << endl;
                                    counterSession = 0;
                                    if ((carY > max(midY1, midY2) || carY == max(midY1, midY2))) // 这个写得很好，防止误判线
                                    {
                                        cout << "1号车库的max()lineY " << max(midY1, midY2) << endl;
                                        garageFirst = true;         // 进入一号车库，最远端的车库
                                        lineY = min(midY1, midY2);  // 获取距离最远的线控制车入库
                                        step = ParkStep::turning; // 开始入库
                                        counterSession = 0;
                                        std::cout << "1号车库" << std::endl;
                                    }
                                    else if ((carY < min(midY1, midY2)))
                                    {
                                        cout << "2号车库的min()lineY " << min(midY1, midY2) << endl;
                                        garageFirst = false;        // 进入二号车库，最近端的车库
                                        lineY = min(midY1, midY2);  // 获取距离最远的线控制车入库
                                        step = ParkStep::turning; // 开始入库
                                        counterSession = 0;
                                        std::cout << "2号车库" << std::endl;
                                    }
                                    else
                                    {
                                        cout << "1号车库的max()lineY " << max(midY1, midY2) << endl;
                                        garageFirst = true;         // 进入一号车库，最远端的车库
                                        lineY = min(midY1, midY2);  // 获取距离最远的线控制车入库
                                        step = ParkStep::turning; // 开始入库
                                        counterSession = 0;
                                        std::cout << "1号车库" << std::endl;    
                                    }
                                    break;
                                }
                            }
                        }
                    }
                    break;
                }
                if(!rightEnable)    //左停
                {
                    // // 固定位置，让其稳定
                    // if (track.pointsEdgeRight.size() < track.pointsEdgeLeft.size())
                    // track.pointsEdgeLeft.resize(track.pointsEdgeRight.size());

                    // for (size_t i = 0; i < track.pointsEdgeRight.size(); i++)
                    // {
                    //     track.pointsEdgeRight[i].y = track.pointsEdgeRight[i].y-40;
                    // }

                    // if (track.pointsEdgeRight.size() > track.pointsEdgeLeft.size())
                    // track.pointsEdgeRight.resize(track.pointsEdgeLeft.size());

                    // for (size_t i = 0; i < track.pointsEdgeRight.size(); i++)
                    // {
                    //     track.pointsEdgeRight[i].y =track.pointsEdgeLeft[i].y+220 ;
                    // }

                    //  拉线处理
                    
                    //第一步------找点
                    for(int i=track.pointsEdgeLeft.size()-20;i>5;i--)
                    {
                        if(track.pointsEdgeLeft[i].y-track.pointsEdgeLeft[i-5].y>10&&track.pointsEdgeLeft[i].y>batteryX&&track.pointsEdgeLeft[i].x<batteryY)
                        {
                            LA_right=track.pointsEdgeLeft[track.pointsEdgeLeft.size()-1];
                            LA_right1=track.pointsEdgeLeft[track.pointsEdgeLeft.size()-4];
                            cout<<"找到拉线角点了"<<endl;
                            cout<<"LA_right.y "<<LA_right.y<<"LA_right.x "<<LA_right.x<<endl;
                            cout<<"LA_right1.y "<<LA_right1.y<<"LA_right1.x "<<LA_right1.x<<endl;
                            break;
                        }
                    }

                    //补线
                    if(LA_right.y>100)
                    {   
                        slope6=Lineslope(LA_right,LA_right1);
                        if(abs(slope6)>chao_slope&&LA_right.y>10)
                        {
                            track.pointsEdgeLeft.clear();
                            RepairLineBySlope(LA_right,LA_right1,239,track.pointsEdgeLeft,false,5000);
                        }
                        else
                        {
                            LA_right=track.pointsEdgeLeft[0];
                            LA_right1=track.pointsEdgeLeft[10];
                            slope6=Lineslope(LA_right,LA_right1);
                            if(abs(slope6)>chao_slope)
                            {
                                track.pointsEdgeLeft.clear();
                                RepairLineBySlope(LA_right,LA_right1,239,track.pointsEdgeLeft,false,5000);
                            }
                        }
                    }
                    //-------------------完事-----------------------------------------
                    


                    for (size_t i = 0; i < predict.size(); i++)
                    {
                        if (predict[i].type == LABEL_CAR && predict[i].score > 0.95)
                        {
                            carY = predict[i].y + predict[i].height;   // 计算智能车的中心高度
                            cout << "识别到carY " << carY << endl;
                            found_carY = true;
                        }
                        else if ((predict[i].type == LABEL_BATTERY) && predict[i].score > 0.4)
                        {
                            batteryY = predict[i].y;   // 计算标识牌最高高度
                        }
                    }
                    if (!found_carY) {
                        carY = ROWSIMAGE; // 未检测到车辆时设为默认值
                        cout << "未检测到车辆，carY 设为默认值 " << carY << endl;
                    }
                    
                    // 图像预处理
                    Mat edges;
                    Canny(image, edges, 50, 150);

                    // 霍夫变换检测直线
                    vector<Vec4i> lines;
                    HoughLinesP(edges, lines, 1, CV_PI/180, 40, 5, 10);

                    vector<Vec4i> horizontalLines;
                    Mat imgRes = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
                    
                    // 先筛选出所有接近水平的线
                    for(const Vec4i& line : lines) 
                    {
                        Point pt1(line[0], line[1]);
                        Point pt2(line[2], line[3]);
                        
                        double angle = atan2(pt2.y - pt1.y, pt2.x - pt1.x) * 180.0 / CV_PI;
                        int midX = (line[0] + line[2]) / 2;
                        int midY = (line[1] + line[3]) / 2;

                        cout<<"midY "<<midY<<endl;

                        // 筛选直线，直线只出现在右侧并且在充电标识牌的上方
                        if(abs(angle) < 25 && midX < COLSIMAGE/2 && midY>25)
                        {
                            horizontalLines.push_back(line);
                            cv::line(imgRes, Point(line[0], line[1]), Point(line[2], line[3]), Scalar(0, 0, 255), 2);
                        }
                    }
                    
                    // 合并距离小于10像素的横线
                    vector<Vec4i> mergedLines;
                    vector<bool> merged(horizontalLines.size(), false);
                    
                    for (size_t i = 0; i < horizontalLines.size(); ++i) {
                        if (merged[i]) continue;
                        
                        Vec4i currentLine = horizontalLines[i];
                        int currentStartY = min(currentLine[1], currentLine[3]);
                        int currentEndY = max(currentLine[1], currentLine[3]);
                        
                        for (size_t j = i + 1; j < horizontalLines.size(); ++j) {
                            if (merged[j]) continue;
                            
                            Vec4i otherLine = horizontalLines[j];
                            int otherStartY = min(otherLine[1], otherLine[3]);
                            int otherEndY = max(otherLine[1], otherLine[3]);
                            
                            // 计算两条线在y轴上的重叠或距离
                            int yDistance = abs((currentStartY + currentEndY)/2 - (otherStartY + otherEndY)/2);
                            
                            if (yDistance < 35) { // 如果距离小于10像素，合并这两条线
                                // 合并为一条更长的线
                                int newX1 = min(currentLine[0], otherLine[0]);
                                int newY1 = min(currentLine[1], otherLine[1]);
                                int newX2 = max(currentLine[2], otherLine[2]);
                                int newY2 = max(currentLine[3], otherLine[3]);
                                
                                currentLine = Vec4i(newX1, newY1, newX2, newY2);
                                currentStartY = min(newY1, newY2);
                                currentEndY = max(newY1, newY2);
                                
                                merged[j] = true; // 标记为已合并
                            }
                        }
                        
                        mergedLines.push_back(currentLine);
                        merged[i] = true;
                    }
                    
                    hengxian.clear();
                    hengxian = mergedLines; // 使用合并后的线
                    
                    // 绘制合并后的线
                    Mat mergedImg = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3);
                    for (const auto& line : mergedLines) {
                        cv::line(mergedImg, Point(line[0], line[1]), Point(line[2], line[3]), Scalar(0, 255, 0), 2);
                    }
                    // imshow("Merged Lines", mergedImg);
                    // waitKey(0);
                    
                    cout << "合并后的横线数量: " << mergedLines.size() << endl;
                    
                    if (mergedLines.size() >= 2 )
                    {
                        cout << "进来充电第二步了" << endl;
                        // 查找平行且距离大于20的线对
                        for(size_t i = 0; i < mergedLines.size(); i++) {
                            for(size_t j = i + 1; j < mergedLines.size(); j++) {
                                // 计算两条线的中点y坐标
                                int midY1 = (mergedLines[i][1] + mergedLines[i][3]) / 2;
                                int midY2 = (mergedLines[j][1] + mergedLines[j][3]) / 2;
                                
                                Vec4i line1 = mergedLines[i];
                                Vec4i line2 = mergedLines[j];

                                // 计算两条线的角度
                                double angle1 = atan2(line1[3] - line1[1], line1[2] - line1[0]) * 180.0 / CV_PI;
                                double angle2 = atan2(line2[3] - line2[1], line2[2] - line2[0]) * 180.0 / CV_PI;

                                // 检测条件
                                cout << "max(midY1,midY2) " << max(midY1, midY2) << endl;
                                cout << "min(midY1,midY2) " << min(midY1, midY2) << endl;
                                cout << "abs(angle1 - angle2) " << abs(angle1 - angle2) << endl;
                                
                                // 如果两条线距离大于20像素并且角度差小于25度
                                if(abs(midY1 - midY2) > 20  && step == ParkStep::enable )
                                {
                                    cout << "进来充电第三步了" << " 此时的carY" << carY << endl << endl;
                                    counterSession = 0;
                                    if ((carY > max(midY1, midY2) || carY == max(midY1, midY2))) // 这个写得很好，防止误判线
                                    {
                                        cout << "1号车库的max()lineY " << max(midY1, midY2) << endl;
                                        garageFirst = true;         // 进入一号车库，最远端的车库
                                        lineY = min(midY1, midY2);  // 获取距离最远的线控制车入库
                                        step = ParkStep::turning; // 开始入库
                                        counterSession = 0;
                                        std::cout << "1号车库" << std::endl;
                                    }
                                    else if ((carY < min(midY1, midY2)))
                                    {
                                        cout << "2号车库的min()lineY " << min(midY1, midY2) << endl;
                                        garageFirst = false;        // 进入二号车库，最近端的车库
                                        lineY = min(midY1, midY2);  // 获取距离最远的线控制车入库
                                        step = ParkStep::turning; // 开始入库
                                        counterSession = 0;
                                        std::cout << "2号车库" << std::endl;
                                    }
                                    else
                                    {
                                        cout << "1号车库的max()lineY " << max(midY1, midY2) << endl;
                                        garageFirst = true;         // 进入一号车库，最远端的车库
                                        lineY = min(midY1, midY2);  // 获取距离最远的线控制车入库
                                        step = ParkStep::turning; // 开始入库
                                        counterSession = 0;
                                        std::cout << "1号车库" << std::endl;    
                                    }
                                    break;
                                }
                            }
                        }
                    }
                    break;
                }
            }
            case ParkStep::turning: // 入库转向
            {
                if(rightEnable)     //右停
                {   
                    LA_right=POINT(0,0);
                    LA_right1=POINT(0,0);
                    //第一步------找点
                    for(int i=track.pointsEdgeRight.size()-20;i>5;i--)
                    {
                        if(track.pointsEdgeRight[i].y-track.pointsEdgeRight[i-5].y<-10)
                        {
                            LA_right=track.pointsEdgeRight[track.pointsEdgeRight.size()-1];
                            LA_right1=track.pointsEdgeRight[track.pointsEdgeRight.size()-4];
                            cout<<"找到拉线角点了"<<endl;
                            cout<<"LA_right.y "<<LA_right.y<<"LA_right.x "<<LA_right.x<<endl;
                            cout<<"LA_right1.y "<<LA_right1.y<<"LA_right1.x "<<LA_right1.x<<endl;
                            break;
                        }
                    }

                    //补线
                    if(LA_right.y>100)
                    {   
                        slope6=Lineslope(LA_right,LA_right1);
                        if(abs(slope6)>chao_slope&&LA_right.y<310)
                        {
                            track.pointsEdgeRight.clear();
                            RepairLineBySlope(LA_right,LA_right1,239,track.pointsEdgeRight,false,5000);
                        }
                    }

                    // // 固定位置，让其稳定
                    // if (track.pointsEdgeRight.size() < track.pointsEdgeLeft.size())
                    // track.pointsEdgeLeft.resize(track.pointsEdgeRight.size());

                    // for (size_t i = 0; i < track.pointsEdgeLeft.size(); i++)
                    // {
                    //     track.pointsEdgeLeft[i].y = track.pointsEdgeLeft[i].y+40;
                    // }

                    Mat edges;
                    Canny(image, edges, 50, 150);

                    // 霍夫变换检测直线
                    vector<Vec4i> lines;
                    HoughLinesP(edges, lines, 1, CV_PI/180, 40, 30, 10);

                    // horizontalLines表示有效(即接近水平并且在右侧)的线
                    vector<Vec4i> horizontalLines;
                    Mat imgRes = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
                    for(const Vec4i& line : lines) {
                        Point pt1(line[0], line[1]);//起点
                        Point pt2(line[2], line[3]);//终点
                        
                        int midX = (line[0] + line[2]) / 2;

                        double angle = atan2(pt2.y - pt1.y, pt2.x - pt1.x) * 180.0 / CV_PI;
                        
                        // 接近水平并且在右侧
                        if(abs(angle) < 40  &&  midX > 100)     //也可以用这个角度>0来写临时停车 
                        { 
                            horizontalLines.push_back(line);
                            cv::line(imgRes, Point(line[0], line[1]), Point(line[2], line[3]), Scalar(0, 0, 255), 2);
                            int midY = (line[1] + line[3]) / 2;     // 计算直线中点y坐标
                            if (midY > lineY && (midY - lineY) <= 20) // 限制线段增加值,更新最新线段
                            {
                                lineY = midY; // 更新直线高度,    //最远的横线
                                ptA = pt1;    // 更新端点，起点
                                ptB = pt2;    //终点
                                cout<<"更新了lineY"<<endl;
                                hengxian1=POINT(250,lineY);
                            }
                        }
                        else
                        {
                            cout<<" 未找到线 不能拐弯"<<endl; 
                        }
                    }
                    // imshow("Detected Lines", imgRes);
                    // waitKey(0);
                    
                    if (lineY >= ROWSIMAGE * swerveTime) // 控制转弯时机  240*0.3      这里开始停车，改改这个逻辑，一号库用左上点(起点)，二号库用右下点(终点)
                    {
                        if (!startTurning)
                        {
                            counterSession = 0;
                            startTurning = true; // 已经开始转弯
                            turning_ing=true;
                        }

                        turningspeed=true;

                        std::cout << "控制转弯" <<" 拐弯直线(LineY)： "<<lineY <<std::endl;
                        // 计算直线的斜率
                        double slope = static_cast<double>(ptB.y - ptA.y) / (ptB.x - ptA.x + 1e-5); // 避免除零

                        int y3 = slope * (0 - ptA.x) + ptA.y;        // 延长起点的Y坐标
                        int y4 = slope * (COLSIMAGE - ptA.x) + ptA.y;// 延长终点的Y坐标

                        cout<<"y3 "<<y3<<endl;
                        cout<<"y4 "<<y4<<endl;

    //--------------------------------------------------------贝塞尔补线------------------------------------------------------- 
                        
                        //左边点
                        POINT start(315,0);     // 延长起点                
                        POINT end(315,240);           // 延长终点
                            
                    
                        vector<POINT> controlPoints = {end,start};
                        vector<POINT> bezierCurve = Bezier(0.001, controlPoints);
                        
                        //右边点
                        POINT start_right(320,0);  //起点
                        POINT end_right(320,240);     //终点

                        vector<POINT> controlPoints_right = {end_right,start_right};
                        vector<POINT> bezierCurve_right = Bezier(0.001, controlPoints_right);

                        track.pointsEdgeLeft.clear(); // 清空原始点集
                        track.pointsEdgeRight.clear(); // 清空原始点集

                        // for (int x = start.x; x <= end.x; x++) {
                            // int y = static_cast<int>(start.y + slope * (x - start.x)); // 根据斜率计算每一个点的 y 值
                            // POINT pt;
                            // pt.x = y; // 将 cv::Point 的 x 赋值给 POINT 的 y
                            // pt.y = x; // 将 cv::Point 的 y 赋值给 POINT 的 x
                        //     track.pointsEdgeLeft.push_back(pt); // 将 POINT 存入点集
                        // }

                        for (const auto& pt : bezierCurve) {
                            POINT convertedPt;
                            convertedPt.x = pt.y;  // 示例转换,pt是没有问题的
                            convertedPt.y = pt.x;
                            // cout<<"pt.x "<<pt.x<<" pt.y "<<pt.y<<endl;
                            // cout<<"convertedPt.x "<<convertedPt.x<<" convertedPt.y "<<convertedPt.y<<endl;
                            track.pointsEdgeLeft.push_back(convertedPt);  // 直接添加贝塞尔曲线点
                        }

                        for (const auto& pt1 : bezierCurve_right) {
                            POINT convertedPt_right;
                            convertedPt_right.x = pt1.y;  // 示例转换,pt是没有问题的
                            convertedPt_right.y = pt1.x;
                            // cout<<"pt.x "<<pt.x<<" pt.y "<<pt.y<<endl;
                            // cout<<"convertedPt.x "<<convertedPt.x<<" convertedPt.y "<<convertedPt.y<<endl;
                            track.pointsEdgeRight.push_back(convertedPt_right);  // 直接添加贝塞尔曲线点
                        }
                        
                        pathsEdgeLeft.push_back(track.pointsEdgeLeft); // 记录进厂轨迹,补线
                        pathsEdgeRight.push_back(track.pointsEdgeRight);
                        cout<<"counterSession "<<counterSession<<endl;
                        into_turn=true;
                        // for(int i=0;i<)
                    }

                    //加霍夫图像处理检测直线来停车
                    if(counterSession >= truningTime2&&into_turn)//加个倒计时限制
                    {
                        Mat edges1;
                        Canny(image, edges1, 50, 150);

                        // 霍夫变换检测直线
                        vector<Vec4i> lines1;
                        HoughLinesP(edges1, lines1, 1, CV_PI/180, 40, 30, 10);
                        
                        vector<Vec4i> stopLine;//停车线

                        stopLine.clear();//先清空

                        // horizontalLines表示有效(即接近水平并且在右侧)的线
                        // vector<Vec4i> horizontalLines;
                        // Mat imgRes = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
                        for(const Vec4i& line : lines1) {
                            Point pt1(line[0], line[1]);//起点
                            Point pt2(line[2], line[3]);//终点
                            
                            int midY3 = (line[1] + line[3]) / 2;

                            double angle3 = atan2(pt2.y - pt1.y, pt2.x - pt1.x) * 180.0 / CV_PI;
                            
                            // 接近水平并且在右侧
                            if(abs(angle3) < 30  &&  midY3 > stop_line)     //筛选有效停车线段 
                            { 
                                cout<<"停车角度 "<<angle3<<endl;
                                stopLine.push_back(line);
                            }
                        }

                        if(stopLine.size()==1)
                        {   
                            stophenxian=stopLine;       //用来绘画的
                            if (stopLine.size() > 0) {
                                Vec4i line3 = stopLine[0];
                                int midY4 = (line3[1] + line3[3]) / 2;
                                cout<<"停车横线 "<<midY4<<endl;
                                if (midY4 > stop_line) {
                                    stopEnable = true;
                                }
                            }
                            else
                            {   
                                cout<<" 未到时机，不能停车"<<endl; 
                                stopEnable=false;
                            }
                        }
                    }

                    if(garageFirst)//一号车库
                    {
                        if (stopEnable&& startTurning) // 开始停车状态     //转弯时间也是倒车的时间
                        {
                            turningspeed=false;
                            stopEnable=false;       //
                            into_turn=false;
                            std::cout << "开始停车 一号车库" << std::endl;
                            step =  ParkStep::stop; // 开始停车
                            counterSession=0;
                        }
                    }
                    if(!garageFirst)//二号车库
                    {
                        if (stopEnable && startTurning) // 开始停车状态     //转弯时间也是倒车的时间
                        {
                            turningspeed=false;
                            stopEnable=false;
                            into_turn=false;
                            std::cout << "开始停车 二号车库" << std::endl;
                            step =  ParkStep::stop; // 开始停车
                            counterSession=0;
                        }
                    }
                    break;
                }
                if(!rightEnable)     //左停
                {
                    // // 固定位置，让其稳定
                    // if (track.pointsEdgeRight.size() < track.pointsEdgeLeft.size())
                    // track.pointsEdgeLeft.resize(track.pointsEdgeRight.size());

                    // for (size_t i = 0; i < track.pointsEdgeRight.size(); i++)
                    // {
                    //     track.pointsEdgeRight[i].y = track.pointsEdgeRight[i].y-40;
                    // }


                    //第一步------找点
                    for(int i=track.pointsEdgeLeft.size()-20;i>5;i--)
                    {
                        if(track.pointsEdgeLeft[i].y-track.pointsEdgeLeft[i-5].y>10)
                        {
                            LA_right=track.pointsEdgeLeft[track.pointsEdgeLeft.size()-1];
                            LA_right1=track.pointsEdgeLeft[track.pointsEdgeLeft.size()-4];
                            cout<<"找到拉线角点了"<<endl;
                            cout<<"LA_right.y "<<LA_right.y<<"LA_right.x "<<LA_right.x<<endl;
                            cout<<"LA_right1.y "<<LA_right1.y<<"LA_right1.x "<<LA_right1.x<<endl;
                            break;
                        }
                    }

                    //补线
                    if(LA_right.y>100)
                    {   
                        slope6=Lineslope(LA_right,LA_right1);
                        if(abs(slope6)>chao_slope&&LA_right.y>10)
                        {
                            track.pointsEdgeLeft.clear();
                            RepairLineBySlope(LA_right,LA_right1,239,track.pointsEdgeLeft,false,5000);
                        }
                    }
                    //-------------------完事-----------------------------------------
                    

                    Mat edges;
                    Canny(image, edges, 50, 150);
                    
                    // 霍夫变换检测直线
                    vector<Vec4i> lines;
                    HoughLinesP(edges, lines, 1, CV_PI/180, 40, 20, 10);

                    // horizontalLines表示有效(即接近水平并且在右侧)的线
                    vector<Vec4i> horizontalLines;
                    Mat imgRes = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
                    for(const Vec4i& line : lines) {
                        Point pt1(line[0], line[1]);//起点
                        Point pt2(line[2], line[3]);//终点
                        
                        int midX = (line[0] + line[2]) / 2;

                        double angle = atan2(pt2.y - pt1.y, pt2.x - pt1.x) * 180.0 / CV_PI;
                        
                        // 接近水平并且在左侧
                        if(abs(angle) < 40  &&  midX < 160)     //也可以用这个角度>0来写临时停车 
                        { 
                            horizontalLines.push_back(line);
                            cv::line(imgRes, Point(line[0], line[1]), Point(line[2], line[3]), Scalar(0, 0, 255), 2);
                            int midY = (line[1] + line[3]) / 2;     // 计算直线中点y坐标
                            if (midY > lineY && (midY - lineY) <= 20) // 限制线段增加值,更新最新线段
                            {
                                lineY = midY; // 更新直线高度,    //最远的横线
                                ptA = pt1;    // 更新端点，起点
                                ptB = pt2;    //终点
                                cout<<"更新了lineY"<<endl;
                                hengxian1=POINT(50,lineY);
                            }
                        }
                        else
                        {
                            cout<<" 未找到线 不能拐弯"<<endl; 
                        }
                    }
                    // imshow("Detected Lines", imgRes);
                    // waitKey(0);
                    
                    if (lineY >= ROWSIMAGE * swerveTime) // 控制转弯时机  240*0.3
                    {
                        if (!startTurning)
                        {
                            counterSession = 0;
                            startTurning = true; // 已经开始转弯
                            turning_ing=true;
                        }

                        turningspeed=true;

                        std::cout << "控制转弯" <<" 拐弯直线(LineY)： "<<lineY <<std::endl;
                        // 计算直线的斜率
                        double slope = static_cast<double>(ptB.y - ptA.y) / (ptB.x - ptA.x + 1e-5); // 避免除零

                        int y3 = slope * (0 - ptA.x) + ptA.y;        // 延长起点的Y坐标
                        int y4 = slope * (COLSIMAGE - ptA.x) + ptA.y;// 延长终点的Y坐标

                        cout<<"y3 "<<y3<<endl;
                        cout<<"y4 "<<y4<<endl;

    //--------------------------------------------------------贝塞尔补线------------------------------------------------------- 
                        
                        //左边点
                        POINT start1(0,0);     // 延长起点                
                        POINT end1(0,240);           // 延长终点
                            
                    
                        vector<POINT> controlPoints1 = {end1,start1};
                        vector<POINT> bezierCurve1 = Bezier(0.001, controlPoints1);
                        
                        //右边点
                        POINT start_right1(20,0);  //起点
                        POINT end_right1(20,240);     //终点

                        vector<POINT> controlPoints_right1 = {end_right1,start_right1};
                        vector<POINT> bezierCurve_right1 = Bezier(0.001, controlPoints_right1);

                        track.pointsEdgeLeft.clear(); // 清空原始点集
                        track.pointsEdgeRight.clear(); // 清空原始点集

                        // for (int x = start.x; x <= end.x; x++) {
                            // int y = static_cast<int>(start.y + slope * (x - start.x)); // 根据斜率计算每一个点的 y 值
                            // POINT pt;
                            // pt.x = y; // 将 cv::Point 的 x 赋值给 POINT 的 y
                            // pt.y = x; // 将 cv::Point 的 y 赋值给 POINT 的 x
                        //     track.pointsEdgeLeft.push_back(pt); // 将 POINT 存入点集
                        // }

                        for (const auto& pt : bezierCurve1) {
                            POINT convertedPt1;
                            convertedPt1.x = pt.y;  // 示例转换,pt是没有问题的
                            convertedPt1.y = pt.x;
                            // cout<<"pt.x "<<pt.x<<" pt.y "<<pt.y<<endl;
                            // cout<<"convertedPt.x "<<convertedPt.x<<" convertedPt.y "<<convertedPt.y<<endl;
                            track.pointsEdgeLeft.push_back(convertedPt1);  // 直接添加贝塞尔曲线点
                        }

                        for (const auto& pt1 : bezierCurve_right1) {
                            POINT convertedPt_right1;
                            convertedPt_right1.x = pt1.y;  // 示例转换,pt是没有问题的
                            convertedPt_right1.y = pt1.x;
                            // cout<<"pt.x "<<pt.x<<" pt.y "<<pt.y<<endl;
                            // cout<<"convertedPt.x "<<convertedPt.x<<" convertedPt.y "<<convertedPt.y<<endl;
                            track.pointsEdgeRight.push_back(convertedPt_right1);  // 直接添加贝塞尔曲线点
                        }
                        
                        pathsEdgeLeft.push_back(track.pointsEdgeLeft); // 记录进厂轨迹,补线
                        pathsEdgeRight.push_back(track.pointsEdgeRight);
                        cout<<"counterSession "<<counterSession<<endl;
                        into_turn=true;
                        // for(int i=0;i<)
                    }

                    //加霍夫图像处理检测直线来停车
                    if(counterSession >= truningTime&&into_turn)
                    {
                        Mat edges1;
                        Canny(image, edges1, 50, 150);

                        // 霍夫变换检测直线
                        vector<Vec4i> lines1;
                        HoughLinesP(edges1, lines1, 1, CV_PI/180, 40, 30, 10);
                        
                        vector<Vec4i> stopLine;//停车线

                        stopLine.clear();//先清空

                        // horizontalLines表示有效(即接近水平并且在右侧)的线
                        // vector<Vec4i> horizontalLines;
                        // Mat imgRes = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
                        for(const Vec4i& line : lines1) {
                            Point pt1(line[0], line[1]);//起点
                            Point pt2(line[2], line[3]);//终点
                            
                            int midY3 = (line[1] + line[3]) / 2;

                            double angle3 = atan2(pt2.y - pt1.y, pt2.x - pt1.x) * 180.0 / CV_PI;
                            
                            // 接近水平并且在右侧
                            if(abs(angle3) < 30  &&  midY3 > stop_line)     //筛选有效停车线段 
                            { 
                                cout<<"停车角度 "<<angle3<<endl;
                                stopLine.push_back(line);
                            }
                        }

                        if(stopLine.size()==1)
                        {   
                            stophenxian=stopLine;
                            if (stopLine.size() > 0) {
                                Vec4i line3 = stopLine[0];
                                int midY4 = (line3[1] + line3[3]) / 2;
                                cout<<"停车横线 "<<midY4<<endl;
                                if (midY4 > stop_line) {
                                    stopEnable = true;
                                }
                            }
                            else
                            {   
                                cout<<" 未到时机，不能停车"<<endl; 
                                stopEnable=false;
                            }
                        }
                    }

                    if(garageFirst)//一号车库
                    {
                        if (stopEnable && startTurning ) // 开始停车状态     //转弯时间也是倒车的时间
                        {
                            turningspeed=false;
                            into_turn=false;
                            stopEnable=false;      //防止下次直接停车
                            std::cout << "开始停车 一号车库" << std::endl;
                            step =  ParkStep::stop; // 开始停车
                            counterSession=0;
                        }
                    }
                    if(!garageFirst)//二号车库
                    {
                        if (stopEnable&&startTurning) // 开始停车状态     //转弯时间也是倒车的时间
                        {
                            turningspeed=false;
                            into_turn=false;
                            stopEnable=false;
                            std::cout << "开始停车 二号车库" << std::endl;
                            step =  ParkStep::stop; // 开始停车
                            counterSession=0;
                        }
                    }
                    break;
                }
            }
            case ParkStep::stop: // 停车
            {
                if (counterSession > stopTime) // 倒车状态
                {
                    step =  ParkStep::trackout; // 开始倒车
                    std::cout << "开始倒车1" << std::endl;
                }
                break;
            }
            case ParkStep::trackout: // 出库
            {
                std::cout << "开始倒车2" << std::endl;
                if(counterSession > trackout_count1 +100) 
                {
                    resetState();
                    std::cout << "路径为空，退出停车场1" << std::endl;
                    return true;
                }
                
                track.pointsEdgeLeft = pathsEdgeLeft[pathsEdgeLeft.size() - 1];
                track.pointsEdgeRight = pathsEdgeRight[pathsEdgeRight.size() - 1];
                
                
                if (counterSession > trackout_count1 )
                {
                    // 路径播放完毕，开始倒计时退出
                    step = ParkStep::exit;
                    counterSession = 0;
                    std::cout << "路径播放完毕，开始3秒倒计时退出" << std::endl;   
                }
                break;
            }
            case ParkStep::exit: // 倒计时退出状态
            { 
                LA_right=POINT(0,0);
                LA_right1=POINT(0,0);

                if(rightEnable)     //右停
                {
                    //第一步------找点
                    for(int i=track.pointsEdgeRight.size()-20;i>5;i--)
                    {
                        if(track.pointsEdgeRight[i].y-track.pointsEdgeRight[i-5].y<-10)
                        {
                            LA_right=track.pointsEdgeRight[track.pointsEdgeRight.size()-1];
                            LA_right1=track.pointsEdgeRight[track.pointsEdgeRight.size()-4];
                            cout<<"找到拉线角点了"<<endl;
                            cout<<"LA_right.y "<<LA_right.y<<"LA_right.x "<<LA_right.x<<endl;
                            cout<<"LA_right1.y "<<LA_right1.y<<"LA_right1.x "<<LA_right1.x<<endl;
                            break;
                        }
                    }

                    //补线
                    if(LA_right.y>10)
                    {   
                        slope6=Lineslope(LA_right,LA_right1);
                        if(abs(slope6)>chao_slope&&LA_right.y<310)
                        {
                            track.pointsEdgeRight.clear();
                            RepairLineBySlope(LA_right,LA_right1,239,track.pointsEdgeRight,false,5000);
                        }
                    }
                }

                if(!rightEnable)    //左停
                {
                     //第一步------找点
                    for(int i=track.pointsEdgeLeft.size()-20;i>5;i--)
                    {
                        if(track.pointsEdgeLeft[i].y-track.pointsEdgeLeft[i-5].y>10)
                        {
                            LA_right=track.pointsEdgeLeft[track.pointsEdgeLeft.size()-1];
                            LA_right1=track.pointsEdgeLeft[track.pointsEdgeLeft.size()-4];
                            cout<<"找到拉线角点了"<<endl;
                            cout<<"LA_right.y "<<LA_right.y<<"LA_right.x "<<LA_right.x<<endl;
                            cout<<"LA_right1.y "<<LA_right1.y<<"LA_right1.x "<<LA_right1.x<<endl;
                            break;
                        }
                    }

                    //补线
                    if(LA_right.y>100)
                    {   
                        slope6=Lineslope(LA_right,LA_right1);
                        if(abs(slope6)>chao_slope&&LA_right.y>10)
                        {
                            track.pointsEdgeLeft.clear();
                            RepairLineBySlope(LA_right,LA_right1,239,track.pointsEdgeLeft,false,5000);
                        }
                    }
                    //-------------------完事-----------------------------------------
                }

                if (counterSession % 30 == 0) // 每30帧(约1秒)输出一次倒计时
                {
                    int remainingSeconds = exitCountdown / 1000;
                    std::cout << "倒计时退出: " << remainingSeconds << "秒" << std::endl;
                }
                
                if (exitCountdown <= 0)
                {
                    resetState();
                    std::cout << "倒计时结束，完全退出停车场" << std::endl;
                }
                else
                {
                    // 假设帧率是30fps，每帧减少约33ms
                    exitCountdown -= 33;
                }
                break;
            }
        }
        return true;
    }
 
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

     //计算斜率函数
    float Lineslope(const POINT& startPoint, 
        const POINT& endPoint)
    {
        float slope3 = 0.0f;
        if (endPoint.y != startPoint.y) {
            slope3 = abs(static_cast<float>(endPoint.x - startPoint.x) / (endPoint.y - startPoint.y+0.00001));
        }
        else if(endPoint.y == startPoint.y)
        {
            endPoint.y+2;
            slope3 = abs(static_cast<float>(endPoint.x - startPoint.x) / (endPoint.y - startPoint.y+0.00001));
        }   
        cout<<"slope3 "<<slope3<<endl;
        return slope3;
    }

    /**
     * @brief 识别结果图像绘制
     *
     */
    void drawImage(Tracking track, Mat &image)
    {
        // 赛道边缘
        for (size_t i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 1,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (size_t i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(image, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1,
                   Scalar(0, 255, 255), -1); // 黄色点
        }
        
        if (step != ParkStep::none)
        {
            if (step == ParkStep::exit)
            {
                int remainingSeconds = exitCountdown / 1000 + 1; // 向上取整显示
                putText(image, "[EXIT] " + to_string(remainingSeconds) + "s", 
                       Point(COLSIMAGE / 2 - 50, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, 
                       cv::Scalar(0, 0, 255), 1, CV_AA);
            }
            else
            {
                putText(image, "[1] BATTERY - ENABLE", Point(COLSIMAGE / 2 - 30, 10), 
                       cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
            }
        }
    }
 
private:
    // 重置所有计数器和状态
    void resetState()
    {
        counterRec = 0;
        counterSession = 0;
        step = ParkStep::none;
        startTurning = false;
        garageFirst = true;
        lineY = 0;
        ptA = Point(0, 0);
        ptB = Point(0, 0);
        pathsEdgeRight.clear();
        pathsEdgeLeft.clear();
        exitCountdown = 0;
        turning_ing=false;
    }
    
    // 重置计数器
    void resetCounters()
    {
        counterRec = 0;
        counterSession = 0;
    }
    
    uint16_t counterRec = 0;      // 加油站标志检测计数器
    int lineY = 0;                // 直线高度
    bool startTurning = false;    // 开始转弯
    vector<vector<POINT>> pathsEdgeLeft; // 记录入库路径
    vector<vector<POINT>> pathsEdgeRight;
    Point ptA = Point(0, 0);      // 记录线段的两个端点
    Point ptB = Point(0, 0);
    float truningTime = 38;         // 转弯时间 21帧
    float truningTime2= 30;          // 二号车库的参数
    float stopTime = 50;            // 停车时间 40帧
    float swerveTime = 0.45;      // 转向时机 0.2 （转弯线出现在屏幕上方0.2处）
    bool rightEnable=true;        // 默认右入库
    int stop_line=200;            //停车线高度
};