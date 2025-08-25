#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE CODE
 *
 *                      (c) Copyright 2025; SaiShu.Lcc.; HC; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习,请勿商用).
 *            The code ADAPTS the corresponding hardware circuit board(代码适配百度Edgeboard-智能汽车赛事版),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file catering.cpp
 * @author HC (sasu@saishukeji.com)
 * @brief 餐饮区
 * @version 0.1
 * @date 2025/03/03 09:53:17
 * @copyright :Copyright (c) 2024
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../../include/detection.hpp"
#include "../recognition/tracking.cpp"
#include "../include/motion.hpp"      

using namespace cv;
using namespace std;

class Catering
{
public:
    bool stopEnable = false;        // 停车使能标志
    bool noRing = false;            // 用来区分环岛路段
    bool bugerflag=false;
    bool burgerCompleted = false;   // 新增：汉堡任务完成标志
    int leftMostY = COLSIMAGE;
    int RightMostY = 0;

    void setParams(const Motion::Params &params) {
        truningTime = params.cateringTruningTime;
        travelTime = params.cateringTravelTime;
        stopTime = params.cateringStopTime;
        extendtime = params.cateringExtendtime;
    }
    bool process(Tracking &track, Mat &image, vector<PredictResult> predict)
    {
        if (burgerCompleted) {  // 如果汉堡任务已完成，直接返回false
            return false;
        }

        if (cateringEnable) // 进入岔路
        {   
            if (!stopEnable && turning)
            {
                for (size_t i = 0; i < predict.size(); i++)
                {
                    if (predict[i].type == LABEL_BURGER)
                    {
                        burgerY = predict[i].y;   // 计算汉堡最高高度
                    }
                }

                // 边缘检测
                Mat edges;
                Mat blurred;
                GaussianBlur(image, blurred, Size(5, 5), 0);
                Canny(blurred, edges, 20, 50, 5);

                // 霍夫变换检测直线
                vector<Vec4i> lines;
                HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 50, 10);
            
            for(size_t i = 20; i < track.pointsEdgeLeft.size()-120; i++) 
            {
                if(track.pointsEdgeLeft[i].y < leftMostY) 
                {
                    leftMostY = track.pointsEdgeLeft[i].y;
                }
            }

            // 找到右边界最右侧的点
            for(size_t i = 20; i < track.pointsEdgeRight.size()-120; i++) 
            {
                if(track.pointsEdgeRight[i].y > RightMostY) 
                {
                    RightMostY = track.pointsEdgeRight[i].y;
                }
            }   
            
                // 遍历检测到的直线
                for (size_t i = 0; i < lines.size(); i++) {
                    Vec4i line = lines[i];
                    Point pt1(line[0], line[1]);
                    Point pt2(line[2], line[3]);

                    cout << "leftMostY : " << leftMostY  << " pt1.x: " << pt1.x <<endl;
                    cout << "RightMostY : " << RightMostY  <<" pt2.x: " << pt2.x <<endl;

                    double slope = static_cast<double>(pt2.y - pt1.y) / (pt2.x - pt1.x + 1e-5);
         
          if(burgerLeft)
         {  
            if(pt1.x < leftMostY || pt2.x < leftMostY)
            continue;
         }
         else
         { 
            if( pt2.x > RightMostY || pt1.x > RightMostY )
            continue;
         }
                    int maxY = max(line[1], line[3]);
                    if (maxY > burgerY - 5 )
                        continue;
                    
                    // 找到左边界最左侧的点
            

                    if ((slope > -0.3 || slope < -1.5) && burgerLeft) {
                        continue;
                    }
                    else if ((slope < 0.25 || slope > 1.5) && !burgerLeft) {
                        continue;
                    }
                    
                           

                    int y3 = static_cast<int>(slope * (0 - pt1.x) + pt1.y);
                    int y4 = static_cast<int>(slope * (COLSIMAGE - pt1.x) + pt1.y);
                    
                    Point start(0, y3);
                    Point end(COLSIMAGE, y4);
                    
                    if (burgerLeft)
                        track.pointsEdgeLeft.clear();
                    else
                        track.pointsEdgeRight.clear();

              if(burgerLeft){
                    for (int x = start.x; x <= end.x; x++) {
                        int y = static_cast<int>(start.y + slope * (x - start.x));
                       
                        POINT pt;
                        pt.x = y;
                        pt.y = x;
                       
                            track.pointsEdgeLeft.push_back(pt);
                            cout << "y4 value: " << y4 << ", slope: " << slope << endl;
                        }
              }
              else{
                    for (int x = end.x; x >= start.x; x--) {
                        int y = static_cast<int>(start.y + slope * (x - start.x));
                       
                        POINT pt;
                        pt.x = y;
                        pt.y = x;
                       
                            track.pointsEdgeRight.push_back(pt);
                            cout << "y4 value: " << y4 << ", slope: " << slope << endl;
                        }
              }



                    }

                }
            
    
            else if (extend) {
                bugerflag=true;
            }

            counterSession++;
            if (counterSession > (truningTime + travelTime + stopTime + extendtime))
            {
                counterRec = 0;
                counterSession = 0;
                cateringEnable = false;
                turning = true;
                noRing = false;
                extend = false;
                burgerCompleted = true;  // 设置汉堡任务完成标志
            }
            else if (counterSession > (truningTime + travelTime + stopTime)) {
                extend = true;
                stopEnable = false;
            }
            else if (counterSession > (truningTime + travelTime)) {
                stopEnable = true;
            }
            else if (counterSession > truningTime) {
                turning = false;
            }

            return true;
        }

        else // 检测汉堡标志
        {
            if (burgerCompleted) {  // 如果汉堡任务已完成，直接返回false
                return false;
            }
            for (size_t i = 0; i < predict.size(); i++)
            {    if(predict[i].type == LABEL_BURGER)
                {area = predict[i].height * predict[i].width;
                 
                 }
                 if(predict[i].type == LABEL_BURGER)
                {cout << "area:" << area <<  endl;
                }
                if (predict[i].type == LABEL_BURGER && predict[i].score > 0.75 && 
                    (predict[i].y + predict[i].height) > ROWSIMAGE * 0.35 && area<3000)
                {
                    counterRec++;
                    noRing = true;
                    burgerLeft = (predict[i].x < COLSIMAGE / 2);
                    break;
                }
            }

            if (counterRec)
            {
                counterSession++;
                if (counterRec >= 1 && counterSession < 2)
                {
                    counterRec = 0;
                    counterSession = 0;
                    cateringEnable = true;
                    return true;
                }
                else if (counterSession >= 2)
                {
                    counterRec = 0;
                    counterSession = 0;
                }
            }

            return false;
        }
    }

    void drawImage(Tracking track, Mat &image)
    {
        // 绘制原始边缘（绿色-左边缘，黄色-右边缘）
        for (auto &p : track.pointsEdgeLeft) {
            circle(image, Point(p.y, p.x), 1, Scalar(0, 255, 0), -1);
        }
        for (auto &p : track.pointsEdgeRight) {
            circle(image, Point(p.y, p.x), 1, Scalar(0, 255, 255), -1);
        }


    }

private:
    uint16_t counterSession = 0;
    uint16_t counterRec = 0;
    bool cateringEnable = false;
    bool burgerLeft = false;
    bool turning = true;
    bool extend = false;
    int burgerY = 0;
    int truningTime = 50;
    int travelTime = 10;
    int stopTime = 400;
    int extendtime = 80;
    int area;
};