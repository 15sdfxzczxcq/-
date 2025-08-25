#pragma once
/*
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE

 *                                         版权所属       唐文博
 *
 *********************************************************************************************************
 * @file crossroad.cpp
 * @author Leo
 * @brief 十字道路识别与图像处理
 * @version 0.1
 * @date 2022-03-14
 *
 * @copyright Copyright (c) 2022
 *
 * @note 十字道路处理步骤：
 *                      [01] 入十字类型识别：tracking.cpp
 *                      [02] 补线起止点搜索
 *                      [03] 边缘重计算
 *
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

class Crossroad
{
public:
    Tracking tracking;
    POINT pointBreakLU= POINT(0, 0);
    POINT pointBreakLD= POINT(0, 0);
    POINT pointBreakRU= POINT(0, 0);
    POINT pointBreakRD= POINT(0, 0);

    POINT pointBreakLU_2=POINT(0,0);
    POINT pointBreakLD_2=POINT(0,0);
    POINT pointBreakRU_2=POINT(0,0);
    POINT pointBreakRD_2=POINT(0,0);

    //左斜入的角点
    POINT pointBreakLD_3=POINT(0,0);
    POINT pointBreakLD_4=POINT(0,0);
    POINT pointBreakRD_3=POINT(0,0);
    POINT pointBreakRD_4=POINT(0,0);

    //左斜入的角点
    POINT pointBreakLD_5=POINT(0,0);
    POINT pointBreakLD_6=POINT(0,0);
    POINT pointBreakRD_5=POINT(0,0);
    POINT pointBreakRD_6=POINT(0,0);

    float parameterB=0; //斜率
    float parameterA=0; //截距    

    float slope1=0; //斜率不合格就不要
    float slope2=0;

    int right_loss_line=0;  //右丢线,清0
    int left_loss_line=0;   //左丢线，清0

    bool xieru_PID=false;   //斜入PID,直道为true,斜入则为false
    
    /**
     * @brief 初始化
     *
     */
    void reset(void)
    {
        crossroadType = CrossroadType::None; // 十字道路类型
    }

    /**
     * @brief 十字道路识别与图像处理
     *
     * @param track 赛道识别结果
     * @param imagePath 输入图像
     */
    bool crossRecognition(Tracking &track,Mat &imgcross)
    {
        bool repaired = false;               // 十字识别与补线结果
        bool detect_cross=false;//检测十字
        bool found_RD=false;    //右下点
        bool found_RU=false;    //右上点
        bool found_LD=false;    //左下点
        bool found_LU=false;    //左上点
        bool found_LU_three=false;  //三点入十字左上点
        bool found_RU_three=false;  //三点入十字右上点

        //斜入十字的
        bool found_LD_1=false;  //斜入左下
        bool found_RD_1=false;

        bool found_LD_2=false;  //斜入右下
        bool found_RD_2=false;


        crossroadType = CrossroadType::None; // 十字道路类型
        pointBreakLU = POINT(0, 0);          //左上
        pointBreakLU_2=POINT(0,0);           //左上一
        pointBreakLD = POINT(0, 0);          //左下
        pointBreakLD_2=POINT(0,0);           //左下一
        pointBreakRU = POINT(0, 0);          //右上
        pointBreakRU_2=POINT(0,0);           //右上一
        pointBreakRD = POINT(0, 0);          //右下
        pointBreakRD_2=POINT(0,0);           //右下一

        //斜入的角点
        //左斜
        pointBreakLD_3=POINT(0,0);
        pointBreakLD_4=POINT(0,0);
        pointBreakRD_3=POINT(0,0);
        pointBreakRD_4=POINT(0,0);

        //右斜
        pointBreakLD_5=POINT(0,0);
        pointBreakLD_6=POINT(0,0);
        pointBreakRD_5=POINT(0,0);
        pointBreakRD_6=POINT(0,0);


        int found_point=0;                   //找到的点数
        POINT pointBreakRU_left_three=POINT(0,240); //左三点的右上点
        // POINT pointBreakRD_left_three=POINT(0,0);
        POINT pointBreakLU_right_three=POINT(0,0);//右三点的左上点
        int LU,LD,RU,RD=0;
        int temp1=0,temp2=0;    //临时变量
        int loss_line=0;        //丢线,清0

        vector<POINT>  cross_right;//补右边线
        vector<POINT>  cross_left;//补左边线

        
        // 尝试写个bool类型的函数来写写
        // for(int i=0;i<track.widthBlock.size()-1;i++)
        // {
        //     if(track.widthBlock[i].y>300)
        //     {
        //         loss_line++;
        //         if(loss_line>80)
        //         {
        //             detect_cross=true;
        //             cout<<loss_line<<endl;
        //             break;
        //         }
        //     }
        //     else
        //     {   
        //         detect_cross=false;
        //     }
        // }

        // for(int i=track.pointsEdgeLeft.size();i>0;i--)
        // {
        //     if(track.pointsEdgeLeft[i].y<2)
        //     {
        //         left_loss_line++;
        //     }
        // }
        // for(int i=track.pointsEdgeRight.size();i>0;i--)
        // {
        //     if(track.pointsEdgeRight[i].y>318)
        //     {
        //         right_loss_line++;
        //     }
        // }

        // cout<<"left_loss_line "<<left_loss_line<<endl;
        // cout<<"right_loss_line "<<right_loss_line<<endl;

        // if(left_loss_line>50&&right_loss_line>35)
        // {
        //     detect_cross=true;
        // }
        // else
        // {
        //     detect_cross=false;
        // }

        for(int i=0;i<track.widthBlock.size()-1;i++)
        {
            if(track.widthBlock[i].y>318)
            {
                loss_line++;
            }
            if(loss_line>20)
            {
                detect_cross=true;
                break;
            }
            else
            {
                detect_cross=false;
            }
        }


        // cout<<"loss_line  "<<loss_line<<endl;

        //直入十字

        // bool found_cross1=found_cross(tracking);
        if(detect_cross)//因为赛道是230，丢线情况下
        {
            //先初始化
            left_loss_line=0;
            right_loss_line=0;
            loss_line=0;


            //左下角点  
            if(track.pointsEdgeLeft.size()>50)
            {
                for(int col=15;col<track.pointsEdgeLeft.size()/5*3;col++)//纵向找
                {
                    if((track.pointsEdgeLeft[col].y-track.pointsEdgeLeft[col-10].y)<=-10&&track.pointsEdgeLeft[col-8].y>5)
                    {
                        if(track.pointsEdgeLeft[col-5].y>0&&track.pointsEdgeLeft[col-5].y<320&&track.pointsEdgeLeft[col-5].x>0&&track.pointsEdgeLeft[col-5].x<240)//防止数字长度超出范围
                        {
                            pointBreakLD=track.pointsEdgeLeft[col-12];
                            pointBreakLD_2=track.pointsEdgeLeft[col-15];//计算斜率的
                            // temp1=col+10;
                            // cout<<"左下角点 "<<"pointBreakLD.y "<<pointBreakLD.y<<"pointBreakLD.x "<<pointBreakLD.x<<endl;
                            // cout<<"左下角点二 "<<"pointBreakLD_2.y "<<pointBreakLD_2.y<<"pointBreakLD_2.x "<<pointBreakLD_2.x<<endl;
                            found_LD=true;
                            found_point+=1;
                            break;
                        }
                    }
                    else
                    {
                        found_LD=false;
                        // pointBreakLD=track.pointsEdgeLeft[5];
                    }
                }
                //左上角点
                // if(pointBreakLD.x>120&&pointBreakLD.x>0)
                // {
                for(int col=track.pointsEdgeLeft.size()-15;col>40;col--)//纵向找
                {
                    if(abs(track.pointsEdgeLeft[col].y-track.pointsEdgeLeft[col-5].y)>10&&track.pointsEdgeLeft[col].y>20&&track.pointsEdgeLeft[col-10].y<10)
                    {   
                        if(track.pointsEdgeLeft[col+3].y>0&&track.pointsEdgeLeft[col+3].y<320&&track.pointsEdgeLeft[col+3].x>0&&track.pointsEdgeLeft[col+3].x<240)//防止数字长度超出范围
                        {
                            pointBreakLU=track.pointsEdgeLeft[col+4];
                            pointBreakLU_2=track.pointsEdgeLeft[col+14];//第二个斜率点
                            // cout<<"左上角点 "<<"pointBreakLU.y "<<pointBreakLU.y<<"pointBreakLU.x "<<pointBreakLU.x<<endl;
                            // cout<<"左上角点二 "<<"pointBreakLU_2.y "<<pointBreakLU_2.y<<"pointBreakLU_2.x "<<pointBreakLU_2.x<<endl;
                            found_LU=true;
                            found_point+=1;
                            break;
                        }
                    }
                    else
                    {
                        found_LU=false;
                        // pointBreakLU=track.pointsEdgeLeft[track.pointsEdgeLeft.size()-5];
                    }
                }
            }    
                //后续点 --------这个先暂时不用--------------
                //右三点的左上点
                // for(int col=20;col<track.pointsEdgeLeft.size()-10;col++)//纵向找
                // {
                //     if(abs(track.pointsEdgeLeft[col].y-track.pointsEdgeLeft[col-3].y)>10&&track.pointsEdgeLeft[col-3].y<10&&track.pointsEdgeLeft[col].y>10)
                //     {
                //         if(track.pointsEdgeLeft[col+3].y>0&&track.pointsEdgeLeft[col+3].y<320&&track.pointsEdgeLeft[col+3].x>0&&track.pointsEdgeLeft[col+3].x<240)//防止数字长度超出范围
                //         {
                //             pointBreakLU_right_three=track.pointsEdgeLeft[col+3];

                //             // temp2=col+10;
                //             // cout<<"右下角点 "<<"pointBreakRD.y "<<pointBreakRD.y<<"pointBreakRD.x "<<pointBreakRD.x<<endl;
                //             break;
                //         }
                //     }
                // }
            // else
            // {
            //     pointBreakLD=POINT(0,0);
            //     pointBreakLU=POINT(0,0);
            //     pointBreakLD_right_three=POINT(0,0);
            // }
            // }
            //右下角点
            if(track.pointsEdgeRight.size()>50)
            {
                for(int col=10;col<track.pointsEdgeRight.size()/5*3;col++)//纵向找
                {
                    if(abs(track.pointsEdgeRight[col].y-track.pointsEdgeRight[col-3].y)>10&&track.pointsEdgeRight[col-3].y<315&&track.pointsEdgeRight[col+3].y>310)
                    {
                        if(track.pointsEdgeRight[col-10].y>0&&track.pointsEdgeRight[col-10].y<320&&track.pointsEdgeRight[col-10].x>0&&track.pointsEdgeRight[col-10].x<240)//防止数字长度超出范围
                        {
                            pointBreakRD=track.pointsEdgeRight[col-6];
                            pointBreakRD_2=track.pointsEdgeRight[col-10];//斜率点
                            // temp2=col+10;
                            // cout<<"右下角点 "<<"pointBreakRD.y "<<pointBreakRD.y<<"pointBreakRD.x "<<pointBreakRD.x<<endl;
                            // cout<<"右下角点二 "<<"pointBreakRD_2.y "<<pointBreakRD_2.y<<"pointBreakRD_2.x "<<pointBreakRD_2.x<<endl;
                            found_RD=true;
                            found_point+=1;
                            break;
                        }
                    }
                    else
                    {
                        found_RD=false;
                        // pointBreakRD=track.pointsEdgeRight[5];
                    }
                }
                //右上角点
                // if(pointBreakRD.x>120&&pointBreakRD.x>0)
                // {
                for(int col=track.pointsEdgeRight.size()-15;col>40;col--)//纵向找
                {
                    if(abs(track.pointsEdgeRight[col].y-track.pointsEdgeRight[col-3].y)>10&&track.pointsEdgeRight[col].y<300&&track.pointsEdgeRight[col-3].y>300)
                    {
                        if(track.pointsEdgeRight[col+3].y>0&&track.pointsEdgeRight[col+3].y<320&&track.pointsEdgeRight[col+3].x<240&&track.pointsEdgeRight[col+3].x>0)    
                        {
                            pointBreakRU=track.pointsEdgeRight[col+10];
                            pointBreakRU_2=track.pointsEdgeRight[col+14];//第二个斜率点
                            // cout<<"右上角点 "<<"pointBreakRU.y "<<pointBreakRU.y<<"pointBreakRU.x "<<pointBreakRU.x<<endl;
                            // cout<<"右上角点二 "<<"pointBreakRU_2.y "<<pointBreakRU_2.y<<"pointBreakRU_2.x "<<pointBreakRU_2.x<<endl;                         
                            found_RU=true;
                            found_point+=1;
                            break;
                        }
                    }
                    else
                    {
                        found_RU=false;
                        // pointBreakRU=track.pointsEdgeRight[track.pointsEdgeLeft.size()-5];
                    }

                }
            }
                //后续点
              //  左三点的右上点
                // for(int col=20;col<track.pointsEdgeRight.size()-10;col++)//纵向找
                // {
                //     if(abs(track.pointsEdgeRight[col].y-track.pointsEdgeRight[col-3].y)>10&&track.pointsEdgeRight[col-3].y>310&&track.pointsEdgeRight[col].y<300)
                //     {
                //         if(track.pointsEdgeRight[col+3].y>0&&track.pointsEdgeRight[col+3].y<320&&track.pointsEdgeRight[col+3].x>0&&track.pointsEdgeRight[col+3].x<240)
                //         {
                //             pointBreakRU_left_three=track.pointsEdgeRight[col+3];
                //             // temp2=col+10;
                //             // cout<<"右下角点 "<<"pointBreakRD.y "<<pointBreakRD.y<<"pointBreakRD.x "<<pointBreakRD.x<<endl;
                //             break;
                //         }
                //     }
                //     // else
                //     // {
                //     //     pointBreakRD_left_three=POINT(0,300);
                //     // }
                // }
            
            // else
            // {
            //     pointBreakRD=POINT(0,0);
            //     pointBreakRU=POINT(0,0);
            //     pointBreakRD_left_three=POINT(0,0);
            // }
            // }
            // //后续点
            // //左三点的右上点
            // for(int col=20;col<track.pointsEdgeRight.size()-10;col++)//纵向找
            // {
            //     if(abs(track.pointsEdgeRight[col].y-track.pointsEdgeRight[col-3].y)>10&&track.pointsEdgeRight[col-3].y>310&&track.pointsEdgeRight[col].y<300)
            //     {
            //         if(track.pointsEdgeRight[col+3].y>0&&track.pointsEdgeRight[col+3].y<320&&track.pointsEdgeRight[col+3].x>0&&track.pointsEdgeRight[col+3].x<240)
            //         {
            //             pointBreakRD_left_three=track.pointsEdgeRight[col+3];
            //             // temp2=col+10;
            //             // cout<<"右下角点 "<<"pointBreakRD.y "<<pointBreakRD.y<<"pointBreakRD.x "<<pointBreakRD.x<<endl;
            //             break;
            //         }
            //     }
            //     // else
            //     // {
            //     //     pointBreakRD_left_three=POINT(0,300);
            //     // }
            // }
            // //右三点的左上点
            // for(int col=20;col<track.pointsEdgeLeft.size()-10;col++)//纵向找
            // {
            //     if(abs(track.pointsEdgeLeft[col].y-track.pointsEdgeLeft[col-3].y)>10&&track.pointsEdgeLeft[col-3].y<10&&track.pointsEdgeLeft[col].y>10)
            //     {
            //         if(track.pointsEdgeLeft[col+3].y>0&&track.pointsEdgeLeft[col+3].y<320&&track.pointsEdgeLeft[col+3].x>0&&track.pointsEdgeLeft[col+3].x<240)
            //         {
            //             pointBreakLD_right_three=track.pointsEdgeLeft[col+3];
            //             // temp2=col+10;
            //             // cout<<"右下角点 "<<"pointBreakRD.y "<<pointBreakRD.y<<"pointBreakRD.x "<<pointBreakRD.x<<endl;
            //             break;
            //         }
            //     }
            // }
        } 

        


        // //找到角点之后写入十字的情况
        // //情况一：直入十字(四点)
        // // if((pointBreakLD.x+pointBreakLU.x)/2>100&&(pointBreakRD.x+pointBreakRU.x)/2>100)
        // // {   
        // //     //先清空
        // //     track.pointsEdgeLeft.clear();
        // //     track.pointsEdgeRight.clear();
        // //     //先补左边线
        // //     double k = (double)(pointBreakLU.y - pointBreakLD.y) /
        // //                (double)(pointBreakLU.x - pointBreakLD.x);
        // //     double b = pointBreakLU.y - k * pointBreakLU.x;
        // //     for (int i = pointBreakLD.x; i <=pointBreakLU.x; i++)
        // //     {
        // //         track.pointsEdgeLeft[i].y = (int)(k * track.pointsEdgeLeft[i].x + b);
        // //     }
        // //     //再补右边线
        // //     double k1 = (double)(pointBreakRU.y - pointBreakRD.y) /
        // //                (double)(pointBreakRU.x - pointBreakRD.x);
        // //     double b1 = pointBreakRU.y - k1 * pointBreakRU.x;
        // //     for (int i = pointBreakRD.x; i <=pointBreakRU.x; i++)
        // //     {
        // //         track.pointsEdgeRight[i].y = (int)(k1 * track.pointsEdgeLeft[i].x + b1);
        // //     }            
        // //     cout<<"补线成功"<<endl;
        // //     repaired = true; // 补线成功 
        // // }    

        // cout<<"found_point "<<found_point<<endl;
        // cout<<"found_LD "<<found_LD<<endl;
        // cout<<"found_LU "<<found_LU<<endl;
        // cout<<"found_RD "<<found_RD<<endl;
        // cout<<"found_RU "<<found_RU<<endl;

        // if (abs(pointBreakLD.x - pointBreakLU.x)  > 70 && abs(pointBreakRD.x - pointBreakRU.x)  > 70&&found_point==4)//思路二：也可以写上下角点的差值大于80
        // {
        //     cout << "进入直入十字(四点）" << endl;
        //     // 先清空
        //     track.pointsEdgeLeft.clear();
        //     track.pointsEdgeRight.clear();

        //     //这个是从这开始补线
        //     // track.pointsEdgeLeft.resize(200-pointBreakLD.x);
        //     // track.pointsEdgeRight.resize(200-pointBreakRD.x);

        //     // 定义临时存储点的 vector
        //     // std::vector<cv::Point> tempLeftLine;
        //     // std::vector<cv::Point> tempRightLine;

        //     // 先补左边线
        //     /*-------------------------------------补直线太狗屎了，换一个-------------------------------
        //     double k = (double)(pointBreakLU.y - pointBreakLD.y) /
        //             (double)(pointBreakLU.x - pointBreakLD.x);
        //     double b = pointBreakLU.y - k * pointBreakLU.x;
        //     for (int i = pointBreakLD.x; i <= pointBreakLU.x; i++)
        //     {
        //         int y = (int)(k * i + b);
        //         tempLeftLine.push_back(cv::Point(i, y));
        //     }

        //     // 将左边线的点逐个推入 pointsEdgeLeft
        //     for (const auto& point : tempLeftLine)
        //     {
        //         POINT pt;
        //         pt.x=point.y;
        //         pt.y=point.x;
        //         track.pointsEdgeLeft.push_back(pt);
        //     }
        //     */
        //     // 贝塞尔补线
        //     // POINT mid1((pointBreakLD.y+pointBreakLU.y)/2,(pointBreakLD.x+pointBreakLU.x)/2);
        //     vector<POINT> controlPoints1 = {pointBreakLD,pointBreakLU};
        //     vector<POINT> tempLeftLine = Bezier(0.01, controlPoints1);

        //     for (const auto& point : tempLeftLine)
        //     {
        //         POINT pt;
        //         pt.y=point.y;
        //         pt.x=point.x;
        //         track.pointsEdgeLeft.push_back(pt);
        //     }

        //     // 再补右边线
        //     // double k1 = (double)(pointBreakRU.y - pointBreakRD.y) /
        //     //             (double)(pointBreakRU.x - pointBreakRD.x);
        //     // double b1 = pointBreakRU.y - k1 * pointBreakRU.x;
        //     // for (int i = pointBreakRD.x; i <= pointBreakRU.x; i++)
        //     // {
        //     //     int y = (int)(k1 * i + b1);
        //     //     tempRightLine.push_back(cv::Point(i, y));
        //     // }

        //     // // 将右边线的点逐个推入 pointsEdgeRight
        //     // for (const auto& point : tempRightLine)
        //     // {    
        //     //     POINT pt1;
        //     //     pt1.x=point.y;
        //     //     pt1.y=point.x;
        //     //     track.pointsEdgeRight.push_back(pt1);
        //     // }
        //     // POINT mid2((pointBreakRD.y+pointBreakRU.y)/2,(pointBreakRD.x+pointBreakRU.x)/2);
        //     vector<POINT> controlPoints2 = {pointBreakRD,pointBreakRU};
        //     vector<POINT> tempRightLine = Bezier(0.01, controlPoints2);

        //     for (const auto& point : tempRightLine)
        //     {
        //         POINT pt1;
        //         pt1.y=point.y;
        //         pt1.x=point.x;
        //         track.pointsEdgeRight.push_back(pt1);
        //     }

        //     cout << "直入十字(四点）补线成功" << endl;
        //     found_point=0;
        //     repaired = true; // 补线成功
        // }

    //     //左入十字(三个点)------找不到右下点的情况
    //     if((pointBreakLD.x-pointBreakLU.x)>70&&(pointBreakRU.x<100&&!found_RD&&found_RU)&&found_point==3)
    //     {
    //         //先清空
    //         // track.pointsEdgeLeft.resize(200-pointBreakLD.x);
    //         // track.pointsEdgeRight.resize(200-pointBreakRD.x);
    //         track.pointsEdgeLeft.clear();
    //         track.pointsEdgeRight.clear();
    //         //补线
    //         //补左边线
    //         vector<POINT> controlPoints4 = {pointBreakLD,pointBreakLU};
    //         vector<POINT> tempLeftLine2 = Bezier(0.01, controlPoints4);

    //         for (const auto& point : tempLeftLine2)
    //         {
    //             POINT pt3;
    //             pt3.y=point.y;
    //             pt3.x=point.x;
    //             track.pointsEdgeLeft.push_back(pt3);
    //         }

    //         // //补右边线
    //         // vector<POINT> controlPoints3 = {pointBreakRU_left_three,pointBreakRD};
    //         // vector<POINT> tempLeftLine1 = Bezier(0.01, controlPoints3);

    //         // for (const auto& point : tempLeftLine1)
    //         // {
    //         //     POINT pt2;
    //         //     pt2.y=point.y;
    //         //     pt2.x=point.x;
    //         //     track.pointsEdgeRight.push_back(pt2);
    //         // }

    //         //斜率补右边线
    //         RepairLineBySlope(pointBreakRU, pointBreakRU_2, 239, track.pointsEdgeRight, true,120);

    //         cout << "左入三点补线成功(找不到右下点)" << endl;
    //         repaired = true; // 补线成功
    //     }


    //     //左入十字(三个点)------找不到右上点的情况
    //     if((pointBreakLD.x-pointBreakLU.x)>70&&(pointBreakRU.x<100&&found_RD&&!found_RU)&&found_point==3)
    //     {
    //         //先清空
    //         // track.pointsEdgeLeft.resize(200-pointBreakLD.x);
    //         // track.pointsEdgeRight.resize(200-pointBreakRD.x);
    //         track.pointsEdgeLeft.clear();
    //         track.pointsEdgeRight.clear();
    //         //补线
    //         //补左边线
    //         vector<POINT> controlPoints4 = {pointBreakLD,pointBreakLU};
    //         vector<POINT> tempLeftLine2 = Bezier(0.01, controlPoints4);

    //         for (const auto& point : tempLeftLine2)
    //         {
    //             POINT pt3;
    //             pt3.y=point.y;
    //             pt3.x=point.x;
    //             track.pointsEdgeLeft.push_back(pt3);
    //         }

    //         // //补右边线
    //         // vector<POINT> controlPoints3 = {pointBreakRU_left_three,pointBreakRD};
    //         // vector<POINT> tempLeftLine1 = Bezier(0.01, controlPoints3);

    //         // for (const auto& point : tempLeftLine1)
    //         // {
    //         //     POINT pt2;
    //         //     pt2.y=point.y;
    //         //     pt2.x=point.x;
    //         //     track.pointsEdgeRight.push_back(pt2);
    //         // }

    //         RepairLineBySlope( pointBreakRU, pointBreakRU_2, 239, track.pointsEdgeRight, true,120);

    //         cout << "左入三点补线成功(找不到右上点)" << endl;
    //         repaired = true; // 补线成功
    //     }
        
    //     // //左入十字(两个点)  先不用这个两点，太容易误判了
    //     // if((pointBreakLD.x-pointBreakLU.x)>70&&(track.pointsEdgeRight[20].y>300&&track.pointsEdgeRight[100].y>300)&&imgcross.at<uchar>(20,310)>127&&!found_RD&&!found_RU)
    //     // {
    //     //     //先清空
    //     //     // track.pointsEdgeLeft.resize(200-pointBreakLD.x);
    //     //     // track.pointsEdgeRight.resize(200-pointBreakRD.x);
    //     //     track.pointsEdgeLeft.clear();
    //     //     track.pointsEdgeRight.clear();

    //     //     //补左边线
    //     //     vector<POINT> controlPoints5 = {pointBreakLD,pointBreakLU};
    //     //     vector<POINT> tempLeftLine3 = Bezier(0.01, controlPoints5);

    //     //     for (const auto& point : tempLeftLine3)
    //     //     {
    //     //         POINT pt4;
    //     //         pt4.y=point.y;
    //     //         pt4.x=point.x;
    //     //         track.pointsEdgeLeft.push_back(pt4);
    //     //     }

    //     //     //补右边线
    //     //     POINT pointBreakRD_1(pointBreakLD.x,pointBreakLD.y+230);
    //     //     POINT pointBreakRU_1(pointBreakLU.x,pointBreakRD.y+230);
    //     //     vector<POINT> controlPoints6 = {pointBreakRD_1,pointBreakRU_1};
    //     //     vector<POINT> tempLeftLine4 = Bezier(0.01, controlPoints6);

    //     //     for (const auto& point : tempLeftLine4)
    //     //     {
    //     //         POINT pt5;
    //     //         pt5.y=point.y;
    //     //         pt5.x=point.x;
    //     //         track.pointsEdgeRight.push_back(pt5);
    //     //     }

    //     //     cout << "左入两点补线成功" <<endl;
    //     //     repaired = true; // 补线成功
    //     // }


    //     //右入十字(三个点) -------------找不到左下点
    //     if((pointBreakRD.x-pointBreakRU.x)>70&&(pointBreakLU.x<100&&track.pointsEdgeLeft[20].y<20)&&!found_LD&&found_LU&&found_point==3)
    //     {

    //         //先清空
    //         // track.pointsEdgeLeft.resize(200-pointBreakLD.x);
    //         // track.pointsEdgeRight.resize(200-pointBreakRD.x);
    //         track.pointsEdgeLeft.clear();
    //         track.pointsEdgeRight.clear();
            
    //         //补线
    //         //补右边线
    //         vector<POINT> controlPoints7 = {pointBreakRD,pointBreakRU};
    //         vector<POINT> tempLeftLine5 = Bezier(0.01, controlPoints7);

    //         for (const auto& point : tempLeftLine5)
    //         {
    //             POINT pt6;
    //             pt6.y=point.y;
    //             pt6.x=point.x;
    //             track.pointsEdgeRight.push_back(pt6);
    //         }

    //         // //补左边线--------先不好使
    //         // vector<POINT> controlPoints8 = {pointBreakLU_right_three,pointBreakLD};
    //         // vector<POINT> tempLeftLine6 = Bezier(0.01, controlPoints8);

    //         // for (const auto& point : tempLeftLine6)
    //         // {
    //         //     POINT pt7;
    //         //     pt7.y=point.y;
    //         //     pt7.x=point.x;
    //         //     track.pointsEdgeLeft.push_back(pt7);
    //         // }

    //         RepairLineBySlope(pointBreakLU, pointBreakLU_2, 239, track.pointsEdgeLeft, true,120);

    //         cout << "右入三点补线成功(找不到左下点)" << endl;
    //         repaired = true; // 补线成功
    //     }
        
    //     //右入十字(三个点) -------------找不到左上点------------
    //     if((pointBreakRD.x-pointBreakRU.x)>70&&found_LD&&!found_LU&&found_point==3)
    //     {

    //         //先清空
    //         // track.pointsEdgeLeft.resize(200-pointBreakLD.x);
    //         // track.pointsEdgeRight.resize(200-pointBreakRD.x);
    //         track.pointsEdgeLeft.clear();
    //         track.pointsEdgeRight.clear();
            
    //         //补线
    //         //补右边线
    //         vector<POINT> controlPoints7 = {pointBreakRD,pointBreakRU};
    //         vector<POINT> tempLeftLine5 = Bezier(0.01, controlPoints7);

    //         for (const auto& point : tempLeftLine5)
    //         {
    //             POINT pt6;
    //             pt6.y=point.y;
    //             pt6.x=point.x;
    //             track.pointsEdgeRight.push_back(pt6);
    //         }

    //         // //补左边线--------先不好使
    //         // vector<POINT> controlPoints8 = {pointBreakLU_right_three,pointBreakLD};
    //         // vector<POINT> tempLeftLine6 = Bezier(0.01, controlPoints8);

    //         // for (const auto& point : tempLeftLine6)
    //         // {
    //         //     POINT pt7;
    //         //     pt7.y=point.y;
    //         //     pt7.x=point.x;
    //         //     track.pointsEdgeLeft.push_back(pt7);
    //         // }

    //         RepairLineBySlope(pointBreakLD, pointBreakLD_2, 239, track.pointsEdgeLeft, true,120);

    //         cout << "右入三点补线成功(找不到左上点)" << endl;
    //         repaired = true; // 补线成功
    //     //右入十字(两个点)  
    // //     if((pointBreakRD.x-pointBreakRU.x)>70&&(track.pointsEdgeLeft[20].y<20&&track.pointsEdgeLeft[100].y<20)&&imgcross.at<uchar>(20,20)>127&&!found_LD&&!found_LU)
    // //     {
    // //         //先清空
    // //         // track.pointsEdgeLeft.resize(200-pointBreakLD.x);
    // //         // track.pointsEdgeRight.resize(200-pointBreakRD.x);
    // //         track.pointsEdgeLeft.clear();
    // //         track.pointsEdgeRight.clear();

    // //         //补右边线
    // //         vector<POINT> controlPoints9 = {pointBreakRD,pointBreakRU};
    // //         vector<POINT> tempLeftLine7 = Bezier(0.01, controlPoints9);

    // //         for (const auto& point : tempLeftLine7)
    // //         {
    // //             POINT pt8;
    // //             pt8.y=point.y;
    // //             pt8.x=point.x;
    // //             track.pointsEdgeRight.push_back(pt8);
    // //         }

    // //         //补左边线
    // //         POINT pointBreakLD_1(pointBreakRD.x,pointBreakRD.y-230);
    // //         POINT pointBreakLU_1(pointBreakRU.x,pointBreakRU.y-230);
    // //         vector<POINT> controlPoints10 = {pointBreakLD_1,pointBreakLU_1};
    // //         vector<POINT> tempLeftLine8 = Bezier(0.01, controlPoints10);

    // //         for (const auto& point : tempLeftLine8)
    // //         {
    // //             POINT pt9;
    // //             pt9.y=point.y;
    // //             pt9.x=point.x;
    // //             track.pointsEdgeLeft.push_back(pt9);
    // //         }

    // //         cout << "右入两点补线成功" <<endl;
    // //         repaired = true; // 补线成功
    // //     }

    // //     // 补线之后 repaired 为 true


        //从头开始写十字
        //补上两点
        if(found_point>=2&&found_LU&&found_RU&&abs(pointBreakRU.x-pointBreakLU.x)<40)
        {
            slope1=Lineslope(pointBreakRU,pointBreakRU_2);
            slope2=Lineslope(pointBreakLU,pointBreakLU_2);
            if(slope1>1.3&&slope2>1.3)
            {
                //先清空
                // track.pointsEdgeLeft.resize(200-pointBreakLD.x);
                // track.pointsEdgeRight.resize(200-pointBreakRD.x);
                track.pointsEdgeLeft.clear();
                track.pointsEdgeRight.clear();
                
                //补线
                //补右边线
                RepairLineBySlope(pointBreakRU, pointBreakRU_2, 239, track.pointsEdgeRight, false,300);
                for(int i=0;i<track.pointsEdgeRight.size();i++)
                {
                    //cout<<"track.pointsEdgeRight["<<i<<"] y "<<track.pointsEdgeRight[i].y<<"track.pointsEdgeRight["<<i<<"] x "<<track.pointsEdgeRight[i].x<<endl;            
                }
                //最小二乘法现在用不出来
                // regression(2,pointBreakRU.x,pointBreakRU_2.x);
                // cout<<" 右斜率 +  "<<parameterB <<" 右截距 " << parameterA;
                // run(2,240,pointBreakRU_2.x,parameterB,parameterA);

                // float k=(float)(pointBreakRU.x-pointBreakRU_2.x)/(float)(pointBreakRU.y-pointBreakRU_2.y);  
                // float b=pointBreakRU.y-k*pointBreakRU.x;
                // cout<<"k "<<k<<"b "<<b<<endl;   
                //         for(int n=240;n>pointBreakRU_2.x;n--){
                //             track.pointsEdgeRight[240-n].y=(int)(b+k*(n));
                //             cout<<"track.pointsEdgeRight["<<240-n<<"] y "<<track.pointsEdgeRight[240-n].y<<"track.pointsEdgeRight["<<240-n<<"] x "<<track.pointsEdgeRight[240-n].x<<endl;            
                //         }

                //补左边线
                RepairLineBySlope(pointBreakLU, pointBreakLU_2, 239, track.pointsEdgeLeft, false,300);
                for(int i=0;i<track.pointsEdgeLeft.size();i++)
                {
                    //cout<<"track.pointsEdgeLeft["<<i<<"] y "<<track.pointsEdgeLeft[i].y<<"track.pointsEdgeLeft["<<i<<"] x "<<track.pointsEdgeLeft[i].x<<endl;            
                }
                // regression(2,pointBreakLU.x,pointBreakLU_2.x);
                // cout<<" 左斜率 +  "<<parameterB <<" 左截距 " << parameterA;
                // run(1,240,pointBreakLU_2.x,parameterB,parameterA);

                // 这个也不好用
                // float k1=(float)(pointBreakLU.x-pointBreakLU_2.x)/(float)(pointBreakLU.y-pointBreakLU_2.y);
                // float b1=pointBreakLU.y-k*pointBreakLU.x;   
                //         for(int n=240;n>pointBreakLU_2.x;n--){
                //             track.pointsEdgeLeft[240-n].y=(int)(b1+k1*(n));      
                //             cout<<"track.pointsEdgeLeft["<<240-n<<"] y "<<track.pointsEdgeLeft[240-n].y<<"track.pointsEdgeLeft["<<240-n<<"] x "<<track.pointsEdgeLeft[240-n].x<<endl;                 
                //         }

                cout << "补上两点成功" << endl;
                found_point=0;
                slope1=0;
                slope2=0;
                xieru_PID=false;
                repaired = true; // 补线成功
            }   
        }

        //补下两点       
        if(found_point>=2&&found_LD&&found_RD&&abs(pointBreakRD.x-pointBreakLD.x)<40)
        {
            slope1=Lineslope(pointBreakRD,pointBreakRD_2);
            slope2=Lineslope(pointBreakLD,pointBreakLD_2);
            if(slope1>1.3&&slope2>1.3)
            {
                track.pointsEdgeLeft.clear();
                track.pointsEdgeRight.clear();
                
                //补线
                //补右边线
                RepairLineBySlope(pointBreakRD, pointBreakRD_2, 239, track.pointsEdgeRight, false,300);
                for(int i=0;i<track.pointsEdgeRight.size();i++)
                {
                    //cout<<"track.pointsEdgeRight["<<i<<"] y "<<track.pointsEdgeRight[i].y<<"track.pointsEdgeRight["<<i<<"] x "<<track.pointsEdgeRight[i].x<<endl;            
                }
                //补左边线
                RepairLineBySlope(pointBreakLD, pointBreakLD_2, 239, track.pointsEdgeLeft, false,300);
                for(int i=0;i<track.pointsEdgeLeft.size();i++)
                {
                    //cout<<"track.pointsEdgeLeft["<<i<<"] y "<<track.pointsEdgeLeft[i].y<<"track.pointsEdgeLeft["<<i<<"] x "<<track.pointsEdgeLeft[i].x<<endl;            
                }
                cout << "补下两点成功" << endl;
                found_point=0;                
                slope1=0;
                slope2=0;
                xieru_PID=false; // 直道PID
                repaired = true; // 补线成功

            }
        }       
        
        //补斜着两点(左下和右上)
        if(found_point>=2&&found_LD&&found_RU)
        {
            slope1=Lineslope(pointBreakRU,pointBreakRU_2);
            slope2=Lineslope(pointBreakLD,pointBreakLD_2);
            if(slope1>0.9&&slope2>0.9)
            {
                track.pointsEdgeLeft.clear();
                track.pointsEdgeRight.clear();
                
                //补线
                //补右边线
                RepairLineBySlope(pointBreakRU, pointBreakRU_2, 239, track.pointsEdgeRight, false,300);
                for(int i=0;i<track.pointsEdgeRight.size();i++)
                {
                    //cout<<"track.pointsEdgeRight["<<i<<"] y "<<track.pointsEdgeRight[i].y<<"track.pointsEdgeRight["<<i<<"] x "<<track.pointsEdgeRight[i].x<<endl;            
                }
                //补左边线
                RepairLineBySlope(pointBreakLD, pointBreakLD_2, 239, track.pointsEdgeLeft, false,300);
                for(int i=0;i<track.pointsEdgeLeft.size();i++)
                {
                    //cout<<"track.pointsEdgeLeft["<<i<<"] y "<<track.pointsEdgeLeft[i].y<<"track.pointsEdgeLeft["<<i<<"] x "<<track.pointsEdgeLeft[i].x<<endl;            
                }
                cout << "补斜着两点(左下和右上)成功" << endl;
                found_point=0;
                slope1=0;
                slope2=0;
                xieru_PID=false;
                repaired = true; // 补线成功
            }
        }

        //补斜着两点(左上和右下)
        if(found_point>=2&&found_LD&&found_RU)
        {
            slope1=Lineslope(pointBreakLU,pointBreakLU_2);
            slope2=Lineslope(pointBreakRD,pointBreakRD_2);
            if(slope1>0.9&&slope2>0.9)
            {
                track.pointsEdgeLeft.clear();
                track.pointsEdgeRight.clear();
                
                //补线
                //补右边线
                RepairLineBySlope(pointBreakRD, pointBreakRD_2, 239, track.pointsEdgeRight, false,300);
                for(int i=0;i<track.pointsEdgeRight.size();i++)
                {
                    //cout<<"track.pointsEdgeRight["<<i<<"] y "<<track.pointsEdgeRight[i].y<<"track.pointsEdgeRight["<<i<<"] x "<<track.pointsEdgeRight[i].x<<endl;            
                }
                //补左边线
                RepairLineBySlope(pointBreakLU, pointBreakLU_2, 239, track.pointsEdgeLeft, false,300);
                for(int i=0;i<track.pointsEdgeLeft.size();i++)
                {
                    //cout<<"track.pointsEdgeLeft["<<i<<"] y "<<track.pointsEdgeLeft[i].y<<"track.pointsEdgeLeft["<<i<<"] x "<<track.pointsEdgeLeft[i].x<<endl;            
                }
                cout << "补斜着两点(左上和右下)成功" << endl;
                found_point=0;
                slope1=0;
                slope2=0;
                xieru_PID=false;
                repaired = true; // 补线成功
            }

        }
        
        //斜入十字


        //-------------------左斜入十字--------------------//
        //第一步---找角点
        //左下角点(左斜)
        for(int col=15;col<track.pointsEdgeLeft.size();col++)//纵向找
        {
            if((track.pointsEdgeLeft[col].y-track.pointsEdgeLeft[col-10].y)<=-10&&(track.pointsEdgeLeft[col].y-track.pointsEdgeLeft[col-1].y)<0&&track.pointsEdgeLeft[col-8].y>5)
            {
                if(track.pointsEdgeLeft[col-5].y>0&&track.pointsEdgeLeft[col-5].y<320&&track.pointsEdgeLeft[col-5].x>0&&track.pointsEdgeLeft[col-5].x<240)//防止数字长度超出范围
                {
                    // cout<<"col-4 "<<col-4<<endl;
                    pointBreakLD_3=track.pointsEdgeLeft[col-11];
                    pointBreakLD_4=track.pointsEdgeLeft[col-15];//计算斜率的
                    // temp1=col+10;
                    // cout<<"左下角点(左斜) "<<"pointBreakLD_3.y "<<pointBreakLD_3.y<<"pointBreakLD_3.x "<<pointBreakLD_3.x<<endl;
                    // cout<<"左下角点二(左斜) "<<"pointBreakLD_4.y "<<pointBreakLD_4.y<<"pointBreakLD_4.x "<<pointBreakLD_4.x<<endl;
                    found_LD_1=true;
                    break;
                }
            }
            else
            {
                found_LD_1=false;
                // pointBreakLD=track.pointsEdgeLeft[5];
            }
        }

        //右下角点(左斜)
        for(int col=10;col<track.pointsEdgeRight.size();col++)//纵向找
                {
                    if(abs(track.pointsEdgeRight[col].y-track.pointsEdgeRight[col-5].y)>10&&track.pointsEdgeRight[col-8].y>315&&track.pointsEdgeRight[col+3].y<310&&track.pointsEdgeRight[col-30].y>315&&track.pointsEdgeRight[20].y>315)
                    {
                        if(track.pointsEdgeRight[col-10].y>0&&track.pointsEdgeRight[col-10].y<320&&track.pointsEdgeRight[col-10].x>0&&track.pointsEdgeRight[col-10].x<240)//防止数字长度超出范围
                        {
                            pointBreakRD_3=track.pointsEdgeRight[col-6];
                            pointBreakRD_4=track.pointsEdgeRight[col-10];//斜率点
                            // temp2=col+10;
                            // cout<<"右下角点(左斜) "<<"pointBreakRD_3.y "<<pointBreakRD_3.y<<"pointBreakRD_3.x "<<pointBreakRD_3.x<<endl;
                            // cout<<"右下角点二(左斜) "<<"pointBreakRD_4.y "<<pointBreakRD_4.y<<"pointBreakRD_4.x "<<pointBreakRD_4.x<<endl;
                            found_RD_1=true;
                            break;
                        }
                    }
                    else
                    {
                        found_RD_1=false;
                        // pointBreakRD=track.pointsEdgeRight[5];
                    }
                }

        //找到斜入特征两个点------------这两个点和其它点不一样
        //左斜
        if(found_LD_1&&found_RD_1&&pointBreakLD_3.y>120)
        {
            if(pointBreakLD_3.y<pointBreakRD_3.y&&pointBreakLD_3.x>pointBreakRD_3.x&&abs(pointBreakLD_3.y-pointBreakRD_3.y)>40&&abs(pointBreakLD_3.y-pointBreakLD_4.y)<30&&abs(pointBreakLD_3.x-pointBreakRD_3.x)<165&&abs(pointBreakLD_3.x-pointBreakRD_3.x)>35&&pointBreakRD_3.x>30&&abs(pointBreakLD_3.y-pointBreakRD_3.y)>60)
            {
                track.pointsEdgeLeft.clear();
                track.pointsEdgeRight.clear();
                //先处理右边线
                RepairLineBySlope(pointBreakRD_3, pointBreakRD_4, 239, track.pointsEdgeRight, false,5000);
                
                //处理左边线
                // // track.pointsEdgeLeft.resize(220-pointBreakLD_3.x);
                RepairLineBySlope(pointBreakLD_3, pointBreakLD_4, 239, track.pointsEdgeLeft, false,5000);
                cout<<"左斜入十字补线成功 "<<endl;
                xieru_PID=true;
                repaired=true;      //补线成功
            }
        }

        ////-------------------右斜入十字--------------------//
        //第一步---找角点
        //右下角点(右斜)        ---在下面的
        for(int col=15;col<track.pointsEdgeRight.size();col++)//纵向找
        {
            if((track.pointsEdgeRight[col].y-track.pointsEdgeRight[col-10].y)>10&&(track.pointsEdgeRight[col].y-track.pointsEdgeRight[col-1].y)>0&&track.pointsEdgeRight[col-8].y<315)
            {
                if(track.pointsEdgeRight[col-5].y>0&&track.pointsEdgeRight[col-5].y<320&&track.pointsEdgeRight[col-5].x>0&&track.pointsEdgeRight[col-5].x<240)//防止数字长度超出范围
                {
                    // cout<<"col-4 "<<col-4<<endl;
                    pointBreakRD_5=track.pointsEdgeRight[col-11];
                    pointBreakRD_6=track.pointsEdgeRight[col-15];//计算斜率的
                    // temp1=col+10;
                    // cout<<"右下角点(右斜) "<<"pointBreakRD_5.y "<<pointBreakRD_5.y<<"pointBreakRD_5.x "<<pointBreakRD_5.x<<endl;
                    // cout<<"右下角点二(右斜) "<<"pointBreakRD_6.y "<<pointBreakRD_6.y<<"pointBreakRD_6.x "<<pointBreakRD_6.x<<endl;
                    found_RD_2=true;
                    break;
                }
            }
            else
            {
                found_RD_2=false;
                // pointBreakLD=track.pointsEdgeLeft[5];
            }
        }

        //左下角点(右斜)
        for(int col=15;col<track.pointsEdgeLeft.size();col++)//纵向找
                {
                    if(abs(track.pointsEdgeLeft[col].y-track.pointsEdgeLeft[col-8].y)>10&&track.pointsEdgeLeft[col-8].y<5&&track.pointsEdgeLeft[col+8].y>10&&track.pointsEdgeLeft[col-20].y<5&&track.pointsEdgeLeft[20].y<5)
                    {
                        // cout<<"进来第一步了"<<endl;
                        // if(track.pointsEdgeLeft[col-10].y>0&&track.pointsEdgeLeft[col-10].y<320&&track.pointsEdgeLeft[col-10].x>0&&track.pointsEdgeLeft[col-10].x<240)//防止数字长度超出范围
                        // {
                            pointBreakLD_5=track.pointsEdgeLeft[col-11];
                            pointBreakLD_6=track.pointsEdgeLeft[col-15];//斜率点
                            // temp2=col+10;
                            // cout<<"左下角点(右斜) "<<"pointBreakLD_5.y "<<pointBreakLD_5.y<<"pointBreakLD_5.x "<<pointBreakLD_5.x<<endl;
                            // cout<<"左下角点二(右斜) "<<"pointBreakLD_6.y "<<pointBreakLD_6.y<<"pointBreakLD_6.x "<<pointBreakLD_6.x<<endl;
                            found_LD_2=true;
                            break;
                        // }
                    }
                    else
                    {
                        found_LD_2=false;
                        // pointBreakRD=track.pointsEdgeRight[5];
                    }
                }

        //找到斜入特征两个点------------这两个点和其它点不一样
        //右斜
        if(found_LD_2&&found_RD_2&&pointBreakRD_5.y<220)
        {
            if(pointBreakLD_5.y<pointBreakRD_5.y&&pointBreakLD_5.x<pointBreakRD_5.x&&abs(pointBreakLD_5.y-pointBreakRD_5.y)>40&&abs(pointBreakRD_5.y-pointBreakRD_6.y)<30&&abs(pointBreakLD_5.x-pointBreakRD_5.x)<165&&abs(pointBreakLD_5.x-pointBreakRD_5.x)>35&&pointBreakLD_5.x>30&&abs(pointBreakLD_5.y-pointBreakRD_5.y)>60)
            {
                track.pointsEdgeLeft.clear();
                track.pointsEdgeRight.clear();
                //先处理右边线
                RepairLineBySlope(pointBreakRD_5, pointBreakRD_6, 239, track.pointsEdgeRight, false,5000);
                
                //处理左边线
                // // track.pointsEdgeLeft.resize(220-pointBreakLD_3.x);
                RepairLineBySlope(pointBreakLD_5, pointBreakLD_6, 239, track.pointsEdgeLeft, false,5000);
                cout<<"右入十字补线成功 "<<endl;
                xieru_PID=true;
                repaired=true;      //补线成功
            }
        }

        cout<<endl;
        cout<<endl;
        cout<<endl;
        cout<<endl;
        cout<<endl;
        cout<<endl;
        return repaired;
    }
    

    // //判断十字的函数(不好用注释掉了)
    // bool found_cross(Tracking &track)
    // {
    //     int lossline=0;
    //     for(int i=0;i<240;i++)
    //     {
    //         if(track.widthBlock[i].y>280)
    //         {
    //             lossline++;
    //         }
    //     }
    //     if(lossline>50)
    //     {
    //         lossline=0;        
    //         cout<<"十字YES"<<endl;            
    //         return true;
    //     }
    //     else
    //     {
    //         return false;
    //         lossline=0;        
    //     }
    // }

    // ---------------------------这个函数太垃圾了 我要用最小二乘法来写-----------------------------
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

    // //最小二乘法求斜率和截距
    // void regression(int type, int startline, int endline)
    // {
    //     int i = 0;
    //     int sumlines = endline - startline;
    //     int sumX = 0;
    //     int sumY = 0;
    //     float averageX = 0;
    //     float averageY = 0;
    //     float sumUp = 0;
    //     float sumDown = 0;
       
    //     if (type == 1)//拟合左线
    //     {
    //         for (i = startline; i < endline; i++)
    //         {
    //             sumX += i;
    //             sumY += tracking.pointsEdgeLeft[i].y;
    //         }
    //         if (sumlines == 0) sumlines = 1;
    //         averageX = sumX / sumlines;     //x的平均值
    //         averageY = sumY / sumlines;     //y的平均值
    //         for (i = startline; i < endline; i++)
    //         {
    //             //SetText("lefetline"+i+" " +lefetline[i] + " averageY" +" "+ averageY);
    //             sumUp += (tracking.pointsEdgeLeft[i].y - averageY) * (i - averageX);
    //             sumDown += (i - averageX) * (i - averageX);
    //         }
    //         if (sumDown == 0) parameterB = 0;
    //         else parameterB = sumUp / sumDown;
    //         parameterA = averageY - parameterB * averageX;
    //     }
    //     else if (type == 2)//拟合右线
    //     {
    //         for (i = startline; i < endline; i++)
    //         {
    //             sumX += i;
    //             sumY += tracking.pointsEdgeRight[i].y;
    //         }
    //         if (sumlines == 0) sumlines = 1;
    //         averageX = sumX / sumlines;     //x的平均值
    //         averageY = sumY / sumlines;     //y的平均值
    //         for (i = startline; i < endline; i++)
    //         {
    //             sumUp += (tracking.pointsEdgeRight[i].y - averageY) * (i - averageX);
    //             sumDown += (i - averageX) * (i - averageX);
    //         }
    //         if (sumDown == 0) parameterB = 0;
    //         else parameterB = sumUp / sumDown;
    //         parameterA = averageY - parameterB * averageX;

    //     }
    // }

    // //计算线段
    // //拟合直线函数
    // void run(int type, int startline, int endline, float parameterB, float parameterA)
    //     {
    //         if (type == 1) //左
    //         {
    //             for (int i = startline; i < endline; i++)
    //             {
    //                 tracking.pointsEdgeLeft[i].y = (parameterB * i + parameterA);
    //                 if (tracking.pointsEdgeLeft[i].y < 0)
    //                 {
    //                     tracking.pointsEdgeLeft[i].y = 320;

    //                 }
    //                 if (tracking.pointsEdgeLeft[i].y > 320)
    //                 {
    //                     tracking.pointsEdgeLeft[i].y = 320;

    //                 }
    //             }
    //         }
    //         else if (type == 2)            //右
    //         {
    //             for (int i = startline; i < endline; i++)
    //             {
    //                 tracking.pointsEdgeRight[i].y = (parameterB * i + parameterA);
    //                 if (tracking.pointsEdgeRight[i].y < 0)
    //                 {
    //                     tracking.pointsEdgeRight[i].y = 320;

    //                 }
    //                 if (tracking.pointsEdgeRight[i].y> 320)
    //                 {
    //                     tracking.pointsEdgeRight[i].y = 320;

    //                 }
    //             }
    //         }
    //     }


    // /**
    //  * @brief 绘制十字道路识别结果
    //  *
    //  * @param Image 需要叠加显示的图像/RGB
    //  */
    void drawImage(Tracking track, Mat &Image)
    {
        // 绘制边缘点
        for (size_t i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(Image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 2,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (size_t i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(Image, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 2,
                   Scalar(0, 255, 255), -1); // 黄色点
        }
            
        // //画角点
        // circle(Image, Point(pointBreakLU.y, pointBreakLU.x), 5,
        //         Scalar(0, 69, 255), -1);
        // circle(Image, Point(pointBreakLD.y, pointBreakLD.x), 5,
        //         Scalar(0, 0, 255), -1);
        // circle(Image, Point(pointBreakRD.y, pointBreakRD.x), 5,
        //         Scalar(255, 0, 255), -1);
        // circle(Image, Point(pointBreakRU.y, pointBreakRU.x), 5,
        //         Scalar(255, 255, 255), -1);                           


            //绘制左下角点，增加报错认识(程序崩溃，大概率是因为识别到非赛道，数组为空)
    if (pointBreakLD.y>0) 
    {
      circle(Image, Point(pointBreakLD.y, pointBreakLD.x), 10, Scalar(255, 0, 0), -1); //左下蓝色
    } 
    else
    {
      // 处理越界情况，例如记录日志或进行其他操作
      // std::cerr << "Index track.rightup is out of range!" << std::endl;
    }

    //绘制左上角点，增加报错认识(程序崩溃，大概率是因为识别到非赛道，数组为空)
    if (pointBreakLU.y>0) 
    {
      circle(Image, Point(pointBreakLU.y, pointBreakLU.x), 10, Scalar(0, 0, 255), -1); //左上红色
    } 
    else
    {
      // 处理越界情况，例如记录日志或进行其他操作
      // std::cerr << "Index track.rightup is out of range!" << std::endl;
    }

    //绘制右上角点，增加报错认识(程序崩溃，大概率是因为识别到非赛道，数组为空)
    if (pointBreakRU.y>0) 
    {
      circle(Image, Point(pointBreakRU.y, pointBreakRU.x), 10, Scalar(0, 255, 0), -1);//右上绿色
    } 
    else
    {
      // 处理越界情况，例如记录日志或进行其他操作
      // std::cerr << "Index track.rightup is out of range!" << std::endl;
    }

    //绘制右下角点，增加报错认识(程序崩溃，大概率是因为识别到非赛道，数组为空)
    if (pointBreakRD.y>0) 
    {
      circle(Image, Point(pointBreakRD.y, pointBreakRD.x), 10, Scalar(255, 0, 255), -1);//右下橙色
    } 
    else
    {
      // 处理越界情况，例如记录日志或进行其他操作
      // std::cerr << "Index track.rightup is out of range!" << std::endl;
    }

    //绘制右下角点，增加报错认识(程序崩溃，大概率是因为识别到非赛道，数组为空)
    if (pointBreakRD_3.y>0) 
    {
      circle(Image, Point(pointBreakRD_3.y, pointBreakRD_3.x), 10, Scalar(0, 255, 255), -1);//右下橙色
    } 
    else
    {
      // 处理越界情况，例如记录日志或进行其他操作
      // std::cerr << "Index track.rightup is out of range!" << std::endl;
    }

    //绘制右下角点，增加报错认识(程序崩溃，大概率是因为识别到非赛道，数组为空)
    if (pointBreakLD_3.y>0) 
    {
      circle(Image, Point(pointBreakLD_3.y, pointBreakLD_3.x), 10, Scalar(255, 255, 255), -1);//右下橙色
    } 
    else
    {
      // 处理越界情况，例如记录日志或进行其他操作
      // std::cerr << "Index track.rightup is out of range!" << std::endl;
    }

    //绘制右下角点，增加报错认识(程序崩溃，大概率是因为识别到非赛道，数组为空)
    if (pointBreakRD_5.y>0) 
    {
      circle(Image, Point(pointBreakRD_5.y, pointBreakRD_5.x), 10, Scalar(0, 255, 255), -1);//右下橙色
    } 
    else
    {
      // 处理越界情况，例如记录日志或进行其他操作
      // std::cerr << "Index track.rightup is out of range!" << std::endl;
    }

    //绘制右下角点，增加报错认识(程序崩溃，大概率是因为识别到非赛道，数组为空)
    if (pointBreakLD_5.y>0) 
    {
      circle(Image, Point(pointBreakLD_5.y, pointBreakLD_5.x), 10, Scalar(123, 132, 255), -1);//右下橙色
    } 
    else
    {
      // 处理越界情况，例如记录日志或进行其他操作
      // std::cerr << "Index track.rightup is out of range!" << std::endl;
    }



        // // // 绘制岔路点
        // for (size_t i = 0; i < track.spurroad.size(); i++)
        // {
        //     circle(Image, Point(track.spurroad[i].y, track.spurroad[i].x), 6,
        //            Scalar(0, 0, 255), -1); // 红色点
        // }

        // // 斜入十字绘制补线起止点
        // if (crossroadType == CrossroadType::CrossroadRight) // 右入十字
        // {
        //     circle(Image, Point(pointBreakLU.y, pointBreakLU.x), 5, Scalar(226, 43, 138), -1); // 上补线点：紫色
        //     circle(Image, Point(pointBreakLD.y, pointBreakLD.x), 5, Scalar(255, 0, 255), -1);  // 下补线点：粉色
        //     if (pointBreakRU.x > 0)
        //         circle(Image, Point(pointBreakRU.y, pointBreakRU.x), 5, Scalar(226, 43, 138), -1); // 上补线点：紫色
        //     if (pointBreakRD.x > 0)
        //         circle(Image, Point(pointBreakRD.y, pointBreakRD.x), 5, Scalar(255, 0, 255), -1); // 下补线点：粉色

        //     putText(Image, "Right", Point(COLSIMAGE / 2 - 15, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
        // }
        // else if (crossroadType == CrossroadType::CrossroadLeft) // 左入十字
        // {
        //     circle(Image, Point(pointBreakRU.y, pointBreakRU.x), 5, Scalar(226, 43, 138), -1); // 上补线点：紫色
        //     circle(Image, Point(pointBreakRD.y, pointBreakRD.x), 5, Scalar(255, 0, 255), -1);  // 下补线点：粉色
        //     if (pointBreakLU.x > 0)
        //         circle(Image, Point(pointBreakLU.y, pointBreakLU.x), 5, Scalar(226, 43, 138), -1); // 上补线点：紫色
        //     if (pointBreakLD.x > 0)
        //         circle(Image, Point(pointBreakLD.y, pointBreakLD.x), 5, Scalar(255, 0, 255), -1); // 下补线点：粉色

        //     putText(Image, "Left", Point(COLSIMAGE / 2 - 15, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
        // }
        // else if (crossroadType == CrossroadType::CrossroadStraight) // 直入十字
        // {
        //     circle(Image, Point(pointBreakLU.y, pointBreakLU.x), 5, Scalar(226, 43, 138), -1); // 上补线点：紫色
        //     circle(Image, Point(pointBreakLD.y, pointBreakLD.x), 5, Scalar(255, 0, 255), -1);  // 下补线点：粉色
        //     circle(Image, Point(pointBreakRU.y, pointBreakRU.x), 5, Scalar(226, 43, 138), -1); // 上补线点：紫色
        //     circle(Image, Point(pointBreakRD.y, pointBreakRD.x), 5, Scalar(255, 0, 255), -1);  // 下补线点：粉色
        //     putText(Image, "Straight", Point(COLSIMAGE / 2 - 20, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
        // }

        putText(Image, "[6] CROSS - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
        putText(Image, to_string(_index), Point(COLSIMAGE / 2 - 5, ROWSIMAGE - 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 155), 1, CV_AA);
        
    }

private:
    int _index = 0; // 测试

    uint16_t counterFild = 0;
    /**
     * @brief 十字道路类型
     *
     */
    enum CrossroadType
    {
        None = 0,
        CrossroadLeft,     // 左斜入十字
        CrossroadRight,    // 右斜入十字
        CrossroadStraight, // 直入十字
    };

    CrossroadType crossroadType = CrossroadType::None; // 十字道路类型

    /**
     * @brief 左上角点
     *
     * @param pointsEdgeLeft
     * @return uint16_t
     */
    // uint16_t searchBreakLeftUp()
    // {
    //     uint16_t rowBreakLeftUp = pointsEdgeLeft.size() - 5;
    //     return rowBreakLeftUp;
    // }
    // /**
    //  * @brief 左下角点
    //  *
    //  * @param pointsEdgeLeft
    //  * @return uint16_t
    //  */
    // uint16_t searchBreakLeftDown(vector<POINT> pointsEdgeLeft)
    // {
    //     uint16_t rowBreakLeft = 0;
    //     return rowBreakLeft;
    // }
    // /**
    //  * @brief 右上角点
    //  *
    //  * @param pointsEdgeRight
    //  * @return uint16_t
    //  */
    // uint16_t searchBreakRightUp(vector<POINT> pointsEdgeRight)
    // {
    //     uint16_t rowBreakRightUp = pointsEdgeRight.size() - 5;
    //     return rowBreakRightUp;
    // }
    // /**
    //  * @brief 右下角点
    //  *
    //  * @param pointsEdgeRight
    //  * @return uint16_t
    //  */
    // uint16_t searchBreakRightDown(vector<POINT> pointsEdgeRight)
    // {
    //     uint16_t rowBreakRightDown = 0;
       
    //     return rowBreakRightDown;
    // }

    /**
     * @brief 直入十字搜索
     *
     * @param pointsEdgeLeft 赛道左边缘点集
     * @param pointsEdgeRight 赛道右边缘点集
     * @return true
     * @return false
     */
    
};