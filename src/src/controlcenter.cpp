#pragma once

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../include/common.hpp"
#include "recognition/tracking.cpp"
#include "recognition/ring.cpp"      //环岛道路识别与路径规划类

using namespace cv;
using namespace std;

class ControlCenter
{
public:
    Ring ring;                // 环岛识别类
    int controlCenter;           // 智能车控制中心（0~320）
    vector<POINT> centerEdge;    // 赛道中心点集
    uint16_t validRowsLeft = 0;  // 边缘有效行数（左）
    uint16_t validRowsRight = 0; // 边缘有效行数（右）
    double sigmaCenter = 0;      // 中心点集的方差
    std::vector<int> findFirstIndices(std::vector<POINT>& arr,int rowsFromBottom) {
        vector<int> thresholds;
        if(rowsFromBottom-ROWSIMAGE/6-arr.back().x>ROWSIMAGE/6){
            thresholds={rowsFromBottom+(arr[0].x-rowsFromBottom)/2,rowsFromBottom,rowsFromBottom-ROWSIMAGE/6,
            rowsFromBottom-ROWSIMAGE/3};  
        }
        else{
            thresholds={rowsFromBottom+(arr[0].x-rowsFromBottom)*2/3,rowsFromBottom+(arr[0].x-rowsFromBottom)/3,rowsFromBottom,arr.back().x};  
        }
        std::vector<int> indices;
        int i=0;
        if (!arr.empty()) {  
            for(int n=1;n<arr.size();n++){
                if(arr[n].x<thresholds[i]&&i<thresholds.size()){
                    indices.push_back(n-1);
                    i++;
                    //std::cout<<"第"<<i<<"\t"<<arr[n].x<<std::endl;
                }
            }
        }
        return indices;  
    }  
    /**
     * @brief 控制中心计算
     *
     * @param pointsEdgeLeft 赛道左边缘点集
     * @param pointsEdgeRight 赛道右边缘点集
     */

    void fitting(Tracking &track,float prospect, Mat &imagePath,int scene)
    {   
        //std::cout<<"比例"<<prospect;
        int rowsFromBottom=ROWSIMAGE*prospect;
        if(rowsFromBottom<track.pointsEdgeLeft.back().x)
            rowsFromBottom=track.pointsEdgeLeft[track.pointsEdgeLeft.size()-5].x;
        if(rowsFromBottom<track.pointsEdgeRight.back().x)
            rowsFromBottom=track.pointsEdgeRight[track.pointsEdgeRight.size()-5].x;
        sigmaCenter = 0;
        controlCenter = COLSIMAGE / 2;
        centerEdge.clear();
        vector<POINT> v_center(4); // 三阶贝塞尔曲线
        style = "STRIGHT";
        vector<int> left_indices;
        vector<int> right_indices;
        int left_down=0;
        int left_up=0;
        int right_down=0;
        int right_up=0;
        // 边缘斜率重计算（边缘修正之后）
        track.stdevLeft = track.stdevEdgeCal(track.pointsEdgeLeft, ROWSIMAGE);
        track.stdevRight = track.stdevEdgeCal(track.pointsEdgeRight, ROWSIMAGE);
        left_indices=findFirstIndices(track.pointsEdgeLeft,rowsFromBottom);
        // for(int ii=0;ii<left_indices.size();ii++){
        //     for (int i = 0; i < COLSIMAGE; i++)
        //     {
        //         circle(imagePath, Point(i,track.pointsEdgeLeft[left_indices[ii]].x), 2,
        //             Scalar(255, 255, 255), -1); // 红色点
        //     }
        // }
         right_indices=findFirstIndices(track.pointsEdgeRight,rowsFromBottom);
        // for(int ii=0;ii<4;ii++){
        //     for (int i = 0; i < COLSIMAGE; i++)
        //     {
        //         circle(imagePath, Point(i,track.pointsEdgeRight[right_indices[ii]].x), 2,
        //             Scalar(0, 0, 0), -1); // 红色点
        //     }
        // }
        if (track.pointsEdgeLeft.size() > 4 && track.pointsEdgeRight.size() > 4) // 通过双边缘有效点的差来判断赛道类型
        {
            //std::cout<<left_indices[3]<<"左"<<left_indices[4] <<"右"<<left_indices[5];
            //中心点
            if(scene==2){
                if(ring.ringStep == 1){
                v_center[0] = {(track.pointsEdgeLeft[0].x + track.pointsEdgeRight[0].x) / 2, (track.pointsEdgeLeft[0].y + track.pointsEdgeRight[0].y) / 2};

                v_center[1] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].x + track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].x) / 2,
                           (track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].y + track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].y) / 2};

                v_center[2] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].x + track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].x) / 2,
                           (track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].y + track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].y) / 2};

                v_center[3] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size()*0.8].x + track.pointsEdgeRight[track.pointsEdgeRight.size()*0.8].x) / 2,
                           (track.pointsEdgeLeft[track.pointsEdgeLeft.size()*0.8].y + track.pointsEdgeRight[track.pointsEdgeRight.size()*0.8].y) / 2};
                           
                
                }

                else{
                    v_center[0] = {(track.pointsEdgeLeft[0].x + track.pointsEdgeRight[0].x) / 2, (track.pointsEdgeLeft[0].y + track.pointsEdgeRight[0].y) / 2};

                v_center[1] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].x + track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].x) / 2,
                           (track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].y + track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].y) / 2};

                v_center[2] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].x + track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].x) / 2,
                           (track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].y + track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].y) / 2};

                v_center[3] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size()-1].x + track.pointsEdgeRight[track.pointsEdgeRight.size()-1].x) / 2,
                           (track.pointsEdgeLeft[track.pointsEdgeLeft.size()-1].y + track.pointsEdgeRight[track.pointsEdgeRight.size()-1].y) / 2};
                           
                
                }
                centerEdge = Bezier(0.03, v_center);
            }
            else if(scene == 4)
            {
                
                vector<POINT> v_center1(3);
                vector<POINT> v_center2(3); 
                vector<POINT> certer2;
                v_center1[0] = {(track.pointsEdgeLeft[0].x + track.pointsEdgeRight[0].x) / 2, (track.pointsEdgeLeft[0].y + track.pointsEdgeRight[0].y) / 2};
                v_center1[1] = {(track.pointsEdgeLeft[left_indices[0]].x + track.pointsEdgeRight[right_indices[0]].x) / 2,
                            (track.pointsEdgeLeft[left_indices[0]].y + track.pointsEdgeRight[right_indices[0]].y) / 2};
                v_center1[2] = {(track.pointsEdgeLeft[left_indices[1]].x + track.pointsEdgeRight[right_indices[1]].x) / 2,
                            (track.pointsEdgeLeft[left_indices[1]].y + track.pointsEdgeRight[right_indices[1]].y) / 2};
                v_center2[0] = {(track.pointsEdgeLeft[left_indices[1]].x + track.pointsEdgeRight[right_indices[1]].x) / 2,
                            (track.pointsEdgeLeft[left_indices[1]].y + track.pointsEdgeRight[right_indices[1]].y) / 2};
                v_center2[1] = {(track.pointsEdgeLeft[left_indices[2]].x + track.pointsEdgeRight[right_indices[2]].x) / 2,
                            (track.pointsEdgeLeft[left_indices[2]].y + track.pointsEdgeRight[right_indices[2]].y) / 2};
                v_center2[2] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size()*0.8].x + track.pointsEdgeRight[track.pointsEdgeRight.size()*0.8].x) / 2,
                            (track.pointsEdgeLeft[track.pointsEdgeLeft.size()*0.8].y + track.pointsEdgeRight[track.pointsEdgeRight.size()*0.8].y) / 2};
                centerEdge = Bezier(0.03, v_center1);
                certer2 = Bezier(0.03, v_center2);
                centerEdge.insert(centerEdge.end(), certer2.begin(), certer2.end());
            }
            // else if(scene==7){
            //     v_center[0] = {
            //         (track.pointsEdgeLeft[0].x + track.pointsEdgeRight[0].x) / 2,
            //         (track.pointsEdgeLeft[0].y + track.pointsEdgeRight[0].y) / 2};

            //     v_center[1] = {
            //         (track.pointsEdgeLeft[track.pointsEdgeLeft.size() *0.1].x +
            //         track.pointsEdgeRight[track.pointsEdgeRight.size() *0.1].x) /
            //             2,
            //         (track.pointsEdgeLeft[track.pointsEdgeLeft.size() *0.2].y +
            //         track.pointsEdgeRight[track.pointsEdgeRight.size() *0.2].y) /
            //             2};

            //     v_center[2] = {
            //         (track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 0.3].x +
            //         track.pointsEdgeRight[track.pointsEdgeRight.size() * 0.3].x) /
            //             2,
            //         (track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 0.3].y +
            //         track.pointsEdgeRight[track.pointsEdgeRight.size() * 0.3].y) /
            //             2};

            //     v_center[3] = {
            //         (track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 0.5].x +
            //         track.pointsEdgeRight[track.pointsEdgeRight.size() * 0.5].x) /
            //             2,
            //         (track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 0.5].y +
            //         track.pointsEdgeRight[track.pointsEdgeRight.size() * 0.5].y) /
            //             2};

            //     centerEdge = Bezier(0.03, v_center);
            // }
            else if(scene == 6)
            {
                //11111
                //半中线
                // vector<POINT> v_center1(3);
                // v_center1[0] = {(track.pointsEdgeLeft[0].x + track.pointsEdgeRight[0].x) / 2, (track.pointsEdgeLeft[0].y + track.pointsEdgeRight[0].y) / 2};
                // v_center1[1] = {(track.pointsEdgeLeft[left_indices[1]].x + track.pointsEdgeRight[right_indices[1]].x) / 2,
                //             (track.pointsEdgeLeft[left_indices[1]].y + track.pointsEdgeRight[right_indices[1]].y) / 2};
                // v_center1[2] = {(track.pointsEdgeLeft[left_indices[2]].x + track.pointsEdgeRight[right_indices[2]].x) / 2,
                //             (track.pointsEdgeLeft[left_indices[2]].y + track.pointsEdgeRight[right_indices[2]].y) / 2};
                // centerEdge = Bezier(0.03, v_center1);
                //半中线结束
                //11结束
                
                //2222
                //固定中线   
                v_center[0] = {
                    (track.pointsEdgeLeft[0].x + track.pointsEdgeRight[0].x) / 2,
                    (track.pointsEdgeLeft[0].y + track.pointsEdgeRight[0].y) / 2};

                v_center[1] = {
                    (track.pointsEdgeLeft[track.pointsEdgeLeft.size() *0.1].x +
                    track.pointsEdgeRight[track.pointsEdgeRight.size() *0.1].x) /
                        2,
                    (track.pointsEdgeLeft[track.pointsEdgeLeft.size() *0.2].y +
                    track.pointsEdgeRight[track.pointsEdgeRight.size() *0.2].y) /
                        2};

                v_center[2] = {
                    (track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 0.3].x +
                    track.pointsEdgeRight[track.pointsEdgeRight.size() * 0.3].x) /
                        2,
                    (track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 0.3].y +
                    track.pointsEdgeRight[track.pointsEdgeRight.size() * 0.3].y) /
                        2};

                v_center[3] = {
                    (track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 0.5].x +
                    track.pointsEdgeRight[track.pointsEdgeRight.size() * 0.5].x) /
                        2,
                    (track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 0.5].y +
                    track.pointsEdgeRight[track.pointsEdgeRight.size() * 0.5].y) /
                        2};

                centerEdge = Bezier(0.03, v_center);
                //固定中线结束    
                //22结束
            }

            /*else if(scene==4&&danger_cone==true){
                vector<POINT> v_center5(3);
                bool left_enable=false;
                bool right_enable=false;
                if(track.pointsEdgeLeft.size()>track.pointsEdgeRight.size()){
                    std::cout<<"左"<<track.pointsEdgeLeft.size()-track.pointsEdgeRight.size()<<std::endl;
                    right_enable=true;
                }
                if(track.pointsEdgeRight.size()>track.pointsEdgeLeft.size())
                    left_enable=true;
                if(right_enable){
                    for(int i=0;i<track.pointsEdgeRight.size();i++){
                        if(track.pointsEdgeRight[i].x<rowsFromBottom){
                            right_down=i-1;
                            for(int ii=i+1;ii<track.pointsEdgeRight.size();ii++){
                                if(track.pointsEdgeRight[ii].x<rowsFromBottom-ROWSIMAGE/6){
                                    right_up=ii;
                                    break;
                                }
                            }
                            break;
                        }
                    }
                    for(int i=0;i<track.pointsEdgeLeft.size();i++){
                        if(track.pointsEdgeLeft[i].x==track.pointsEdgeRight[right_down].x){
                            left_down=i;
                            for(int ii=i+1;ii<track.pointsEdgeLeft.size();ii++){
                                if(track.pointsEdgeLeft[ii].x==track.pointsEdgeRight[right_up].x){
                                    left_up=ii;
                                    break;
                                }
                            }
                            break;
                        }
                    }
                }
                if(left_enable){
                    for(int i=0;i<track.pointsEdgeLeft.size();i++){
                        if(track.pointsEdgeLeft[i].x<rowsFromBottom){
                            left_down=i-1;
                            for(int ii=i+1;ii<track.pointsEdgeLeft.size();ii++){
                                if(track.pointsEdgeLeft[ii].x<rowsFromBottom-ROWSIMAGE/5){
                                    left_up=ii;
                                    break;
                                }
                            }
                            break;
                        }
                    }
                    for(int i=0;i<track.pointsEdgeRight.size();i++){
                        if(track.pointsEdgeRight[i].x<track.pointsEdgeRight[left_down].x){
                            right_down=i-1;
                            for(int ii=i+1;ii<track.pointsEdgeRight.size();ii++){
                                if(track.pointsEdgeRight[ii].x<track.pointsEdgeLeft[left_up].x){
                                    right_up=ii-1;
                                    break;
                                }
                            }
                            break;
                        }
                    }
                }
                v_center5[0] = {(track.pointsEdgeLeft[left_down].x + track.pointsEdgeRight[right_down].x) / 2, 
                (track.pointsEdgeLeft[left_down].y + track.pointsEdgeRight[right_down].y) / 2};

                v_center5[1] = {(track.pointsEdgeLeft[(left_up+left_down)/2].x + track.pointsEdgeRight[(right_up+right_down)/2].x) / 2,
                           (track.pointsEdgeLeft[(left_up+left_down)/2].y + track.pointsEdgeRight[(right_up+right_down)/2].y) / 2};
                v_center5[2] = {(track.pointsEdgeLeft[left_up].x + track.pointsEdgeRight[right_up].x) / 2,
                           (track.pointsEdgeLeft[left_up].y + track.pointsEdgeRight[right_up].y) / 2};
                centerEdge = Bezier(0.03, v_center5);
            }*/
            else{
                //全中线
                vector<POINT> v_center1(3);
                vector<POINT> v_center2(3); 
                vector<POINT> certer2;
                v_center1[0] = {(track.pointsEdgeLeft[0].x + track.pointsEdgeRight[0].x) / 2, (track.pointsEdgeLeft[0].y + track.pointsEdgeRight[0].y) / 2};
                v_center1[1] = {(track.pointsEdgeLeft[left_indices[0]].x + track.pointsEdgeRight[right_indices[0]].x) / 2,
                            (track.pointsEdgeLeft[left_indices[0]].y + track.pointsEdgeRight[right_indices[0]].y) / 2};
                v_center1[2] = {(track.pointsEdgeLeft[left_indices[1]].x + track.pointsEdgeRight[right_indices[1]].x) / 2,
                            (track.pointsEdgeLeft[left_indices[1]].y + track.pointsEdgeRight[right_indices[1]].y) / 2};
                v_center2[0] = {(track.pointsEdgeLeft[left_indices[1]].x + track.pointsEdgeRight[right_indices[1]].x) / 2,
                            (track.pointsEdgeLeft[left_indices[1]].y + track.pointsEdgeRight[right_indices[1]].y) / 2};
                v_center2[1] = {(track.pointsEdgeLeft[left_indices[2]].x + track.pointsEdgeRight[right_indices[2]].x) / 2,
                            (track.pointsEdgeLeft[left_indices[2]].y + track.pointsEdgeRight[right_indices[2]].y) / 2};
                v_center2[2] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size()*0.8].x + track.pointsEdgeRight[track.pointsEdgeRight.size()*0.8].x) / 2,
                            (track.pointsEdgeLeft[track.pointsEdgeLeft.size()*0.8].y + track.pointsEdgeRight[track.pointsEdgeRight.size()*0.8].y) / 2};
                centerEdge = Bezier(0.03, v_center1);
                certer2 = Bezier(0.03, v_center2);
                centerEdge.insert(centerEdge.end(), certer2.begin(), certer2.end());
                //全中线结束
                //半中线
                // vector<POINT> v_center1(3);
                // v_center1[0] = {(track.pointsEdgeLeft[0].x + track.pointsEdgeRight[0].x) / 2, (track.pointsEdgeLeft[0].y + track.pointsEdgeRight[0].y) / 2};
                // v_center1[1] = {(track.pointsEdgeLeft[left_indices[1]].x + track.pointsEdgeRight[right_indices[1]].x) / 2,
                //             (track.pointsEdgeLeft[left_indices[1]].y + track.pointsEdgeRight[right_indices[1]].y) / 2};
                // v_center1[2] = {(track.pointsEdgeLeft[left_indices[2]].x + track.pointsEdgeRight[right_indices[2]].x) / 2,
                //             (track.pointsEdgeLeft[left_indices[2]].y + track.pointsEdgeRight[right_indices[2]].y) / 2};
                // centerEdge = Bezier(0.03, v_center1);
                //半中线结束
                //静态中线
                // v_center[0] = {(track.pointsEdgeLeft[0].x + track.pointsEdgeRight[0].x) / 2, (track.pointsEdgeLeft[0].y + track.pointsEdgeRight[0].y) / 2};

                // v_center[1] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].x + track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].x) / 2,
                //            (track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].y + track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].y) / 2};

                // v_center[2] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].x + track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].x) / 2,
                //            (track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].y + track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].y) / 2};

                // v_center[3] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size()-1].x + track.pointsEdgeRight[track.pointsEdgeRight.size()-1].x) / 2,
                //            (track.pointsEdgeLeft[track.pointsEdgeLeft.size()-1].y + track.pointsEdgeRight[track.pointsEdgeRight.size()-1].y) / 2};
                           
                // centerEdge = Bezier(0.03, v_center);
                //静态中线结束


            // if(rowsFromBottom>track.pointsEdgeLeft.back().x){
            //     vector<POINT> v_center3(3); 
            //     vector<POINT> certer3;     
            //     v_center3[0] = {(track.pointsEdgeLeft[left_indices[3]].x + track.pointsEdgeRight[right_indices[3]].x) / 2,
            //                 (track.pointsEdgeLeft[left_indices[3]].y + track.pointsEdgeRight[right_indices[3]].y) / 2};
            //     v_center3[1] = {(track.pointsEdgeLeft[left_indices[4]].x + track.pointsEdgeRight[right_indices[4]].x) / 2, 
            //                 (track.pointsEdgeLeft[left_indices[4]].y + track.pointsEdgeRight[right_indices[4]].y) / 2};
            //     // v_center3[2] = {(track.pointsEdgeLeft[left_indices[0]].x + track.pointsEdgeRight[right_indices[0]].x) / 2,
            //     //                (track.pointsEdgeLeft[left_indices[0]].y + track.pointsEdgeRight[right_indices[0]].y) / 2};
            //     v_center3[2] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 15].x + track.pointsEdgeRight[track.pointsEdgeRight.size() - 15].x) / 2,
            //                 (track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 15].y + track.pointsEdgeRight[track.pointsEdgeRight.size() - 15].y) / 2};
            //     certer3 = Bezier(0.03, v_center3);
            //     centerEdge.insert(centerEdge.end(), certer3.begin(), certer3.end());  
            // }
            }
            style = "STRIGHT";
        }
            // 左单边
        else if ((track.pointsEdgeLeft.size() > 0 && track.pointsEdgeRight.size() <= 4) ||
                 (track.pointsEdgeLeft.size() > 0 && track.pointsEdgeRight.size() > 0 && track.pointsEdgeLeft[0].x - track.pointsEdgeRight[0].x > ROWSIMAGE / 2))
        {
            style = "RIGHT";
            centerEdge = centerCompute(track.pointsEdgeLeft, 0);
        }
            // 右单边
        else if ((track.pointsEdgeRight.size() > 0 && track.pointsEdgeLeft.size() <= 4) ||
                 (track.pointsEdgeRight.size() > 0 && track.pointsEdgeLeft.size() > 0 && track.pointsEdgeRight[0].x - track.pointsEdgeLeft[0].x > ROWSIMAGE / 2))
        {
            style = "LEFT";
            centerEdge = centerCompute(track.pointsEdgeRight, 1);
        }
        else if (track.pointsEdgeLeft.size() > 4 && track.pointsEdgeRight.size() == 0) // 左单边
        {
            v_center[0] = {track.pointsEdgeLeft[0].x, (track.pointsEdgeLeft[0].y + COLSIMAGE - 1) / 2};

            v_center[1] = {track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].x,
                           (track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].y + COLSIMAGE - 1) / 2};

            v_center[2] = {track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].x,
                           (track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].y + COLSIMAGE - 1) / 2};

            v_center[3] = {track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].x,
                           (track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].y + COLSIMAGE - 1) / 2};

            centerEdge = Bezier(0.02, v_center);

            style = "RIGHT";
        }
        else if (track.pointsEdgeLeft.size() == 0 && track.pointsEdgeRight.size() > 4) // 右单边
        {
            v_center[0] = {track.pointsEdgeRight[0].x, track.pointsEdgeRight[0].y / 2};

            v_center[1] = {track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].x,
                           track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].y / 2};

            v_center[2] = {track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].x,
                           track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].y / 2};

            v_center[3] = {track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x,
                           track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].y / 2};

            centerEdge = Bezier(0.02, v_center);

            style = "LEFT";
        }
        // 加权控制中心计算
        int controlNum=1;
                    for (int i = 0; i < COLSIMAGE; i++)
    {
      circle(imagePath, Point(i,rowsFromBottom), 2,
      Scalar(0, 0, 255), -1); // 下
      circle(imagePath, Point(i,rowsFromBottom-ROWSIMAGE/6), 2,
      Scalar(0, 255, 0), -1); // 上
    }
        for (int i = 0; i < centerEdge.size(); i++)
    {
      circle(imagePath, Point(centerEdge[i].y,centerEdge[i].x), 2,
      Scalar(255, 0, 255), -1); // 红色点
    }
        if(scene==5){
            for (auto p : centerEdge)
            {
                if (p.x < ROWSIMAGE / 2)
                {
                    controlNum += ROWSIMAGE / 2;
                    controlCenter += p.y * ROWSIMAGE / 2;
                }
                else
                {
                    controlNum += (ROWSIMAGE - p.x);
                    controlCenter += p.y * (ROWSIMAGE - p.x);
                }
            }
        }
        else{
            for(auto p : centerEdge){
                if(p.x<rowsFromBottom && p.x>rowsFromBottom-ROWSIMAGE/6){
                    controlNum++; // 增加计数  
                    controlCenter += p.y;
                }
            }
        }
        if (controlNum > 1)
        {
            controlCenter =controlCenter / controlNum;
        }
        if (controlCenter > COLSIMAGE)
            controlCenter = COLSIMAGE;
        else if (controlCenter < 0)
            controlCenter = 0;
                // 控制率计算
         //std::cout<<"中线点数 "<<centerEdge.size()<<std::endl;
        if (centerEdge.size() > 20)
        {
            vector<POINT> centerV;
            float filt = centerEdge.size() / 68;
            int filta = filt*12;//红虚线  //6.20号为6(原20)   //改完响应之后3(原20)
            int filtb = filt*52;//绿虚线  //6.20号为16(原20)  //17(原20)           //yuan50
            for (int i = filta; i < filtb; i++) 
            {
                centerV.push_back(centerEdge[i]);
            }
            sigmaCenter = sigma(centerV);
            //画出方差线
             if(sigmaCenter)
            {
                int siga = centerEdge[filta].x;
                int sigb = centerEdge[filtb].x;
                for (int i = 0; i < COLSIMAGE; i+=3)
              {
                circle(imagePath, Point(i,siga), 1,
                Scalar(0, 0, 255), -1); // 下
                circle(imagePath, Point(i,sigb), 1,
                Scalar(0, 255, 0), -1); // 上
              }
            }
        }
        else
            sigmaCenter = 1000;
    }

    /**
     * @brief 车辆冲出赛道检测（保护车辆）
     *
     * @param track
     * @return true
     * @return false
     */
    bool derailmentCheck(Tracking track)
    {
        //计数器
        if (track.pointsEdgeLeft.size() < 30 && track.pointsEdgeRight.size() < 30) // 防止车辆冲出赛道
        {
            countOutlineA++;
            countOutlineB = 0;
            if (countOutlineA > 20)
                return true;
        }
        else
        {
            countOutlineB++;
            if (countOutlineB > 50)
            {
                countOutlineA = 0;
                countOutlineB = 50;
            }
        }
        return false;
    }

    /**
     * @brief 显示赛道线识别结果
     *
     * @param centerImage 需要叠加显示的图像
     */
    void drawImage(Tracking track, Mat &centerImage)
    {
        // 赛道边缘绘制
        for (uint16_t i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(centerImage, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 1, Scalar(0, 255, 0), -1); // 绿色点
        }
        for (uint16_t i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(centerImage, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1, Scalar(0, 255, 255), -1); // 黄色点
        }
        // 绘制中心点集
        for (uint16_t i = 0; i < centerEdge.size(); i++)
        {
            circle(centerImage, Point(centerEdge[i].y, centerEdge[i].x), 1, Scalar(0, 0, 255), -1);//红色点
        }
        // 绘制加权控制中心：方向
        Rect rect(controlCenter, ROWSIMAGE - 20, 10, 20);
        rectangle(centerImage, rect, Scalar(0, 0, 255), FILLED);

        // 详细控制参数显示
        int dis = 20;
        string str;
        putText(centerImage, style, Point(COLSIMAGE - 60, dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 赛道类型

        str = "Edge: " + formatDoble2String(track.stdevLeft, 1) + " | " + formatDoble2String(track.stdevRight, 1);
        putText(centerImage, str, Point(COLSIMAGE - 150, 2 * dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 斜率：左|右

        str = "Center: " + formatDoble2String(sigmaCenter, 2);
        putText(centerImage, str, Point(COLSIMAGE - 120, 3 * dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 中心点方差

        putText(centerImage, to_string(controlCenter), Point(COLSIMAGE / 2 - 10, ROWSIMAGE - 40), FONT_HERSHEY_PLAIN, 1.2, Scalar(0, 0, 255), 1); // 中心
    }

private:
    int countOutlineA = 0; // 车辆脱轨检测计数器
    int countOutlineB = 0; // 车辆脱轨检测计数器
    string style = "";     // 赛道类型
        vector<POINT> centerCompute(vector<POINT> pointsEdge, int side)
    {
        int step = 4;                    // 间隔尺度
        int offsetWidth = COLSIMAGE / 2; // 首行偏移量
        int offsetHeight = 0;            // 纵向偏移量

        vector<POINT> center; // 控制中心集合

        if (side == 0) // 左边缘
        {
            uint16_t counter = 0, rowStart = 0;
            for (int i = 0; i < pointsEdge.size(); i++) // 删除底部无效行
            {
                if (pointsEdge[i].y > 1)
                {
                    counter++;
                    if (counter > 2)
                    {
                        rowStart = i - 2;
                        break;
                    }
                }
                else
                    counter = 0;
            }

            offsetHeight = pointsEdge[rowStart].x - pointsEdge[0].x;
            counter = 0;
            for (int i = rowStart; i < pointsEdge.size(); i += step)
            {
                int py = pointsEdge[i].y + offsetWidth;
                if (py > COLSIMAGE - 1)
                {
                    counter++;
                    if (counter > 2)
                        break;
                }
                else
                {
                    counter = 0;
                    center.emplace_back(pointsEdge[i].x - offsetHeight, py);
                }
            }
        }
        else if (side == 1) // 右边沿
        {
            uint16_t counter = 0, rowStart = 0;
            for (uint16_t i = 0; i < pointsEdge.size(); i++) // 删除底部无效行
            {
                if (pointsEdge[i].y < COLSIMAGE - 1)
                {
                    counter++;
                    if (counter > 2)
                    {
                        rowStart = i - 2;
                        break;
                    }
                }
                else
                    counter = 0;
            }

            offsetHeight = pointsEdge[rowStart].x - pointsEdge[0].x;
            counter = 0;
            for (uint16_t i = rowStart; i < pointsEdge.size(); i += step)
            {
                int py = pointsEdge[i].y - offsetWidth;
                if (py < 1)
                {
                    counter++;
                    if (counter > 2)
                        break;
                }
                else
                {
                    counter = 0;
                    center.emplace_back(pointsEdge[i].x - offsetHeight, py);
                }
            }
        }

        return center;
        // return Bezier(0.2,center);
    }
   };