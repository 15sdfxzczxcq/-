#include <chrono>
#include <thread>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "../include/motion.hpp" 
using namespace std;
using namespace cv;

class Layby
{
public:
    uint16_t counterSession = 0;    // 图像场次计数器
    bool stopEnable = false;        // 停车使能标志
    bool loss_line = false;         // 默认没有丢线
    int distance1 = 0;              // 判断左右丢线
    bool laybyCooldown = false;     // 停车冷却标志
    std::chrono::steady_clock::time_point laybyCooldownStart; // 停车冷却开始时间
    const int laybyCooldownDuration = 50; // 停车冷却时间（毫秒） 
    uint16_t counterExit = 0;       // 停车倒计时
    uint16_t counterExit1 = 0;       // 停车倒计时
    vector<Vec4i> hengxian2;        // 绘制检测的横线
    vector<Vec4i> hengxian3;        // 绘制检测的横线
    bool will_park=false;
    bool will_park1=false;
    int STOP_LINE_Y_THRESHOLD1=0;
    int midY =0;

    // enum LaybyStep
    // {
    //     none = 0, // 未知状态
    //     will_park1
    // };

    // LaybyStep step=LaybyStep::none;

   // const int STOP_LINE_Y_THRESHOLD = 100; // 停车线Y轴阈值（240*0.5）
    int firstLineY=0;
    int secondLineY=0;

    void setParams6(const Motion::Params &params) {
       
        STOP_LINE_Y_THRESHOLD = params.STOP_LINE_Y_THRESHOLD;
        stopTime=params.stopTime;
        STOP_LINE_Y_THRESHOLD1=params.STOP_LINE_Y_THRESHOLD1;
    }
    
    bool process(Tracking &track, Mat &image, vector<PredictResult> predict)
    {
        auto now = std::chrono::steady_clock::now();
        
        // 检查是否在冷却时间内
        if (laybyCooldown && 
            std::chrono::duration_cast<std::chrono::milliseconds>(now - laybyCooldownStart).count() < laybyCooldownDuration) {
            return false; // 在冷却时间内，不进入停车模型
        }
        else if (laybyCooldown) {
            laybyCooldown = false; // 冷却时间结束，重置标志
        }
 
        if (laybyEnable) // 进入临时停车状态
        {   
            cout << "进入临时停车模型" << endl;
            Mat edges;
            Mat blurred;
 
            curtailTracking(track, leftEnable); // 缩减优化车道线（双车道→单车道）
            // 直线检测
            GaussianBlur(image, blurred, Size(3, 3), 0);  // 添加高斯模糊预处理
            Canny(blurred, edges, 30, 150, 3);  // 调整Canny参数，使用3x3 Sobel算子
 
            // 霍夫变换检测直线
            vector<Vec4i> lines;
            HoughLinesP(edges, lines, 1,        // rho
                       CV_PI/180,              // theta
                       25,                     // threshold：降低阈值
                       40,                     // minLineLength：减小最小线段长度
                       20);                    // maxLineGap：增大间隙容忍度
            
            // 存储合并后的线段
            mergedLines.clear();
            
            // 对检测到的线段进行排序（按y坐标）
            sort(lines.begin(), lines.end(), 
                [](const Vec4i &a, const Vec4i &b) {
                    return (a[1] + a[3])/2 < (b[1] + b[3])/2;
                });
            
            // 找到左边界最左侧的点
            int leftMostY = COLSIMAGE;
            for(size_t i = 20; i < track.pointsEdgeLeft.size()-120; i++) 
            {
                if(track.pointsEdgeLeft[i].y < leftMostY) 
                {
                    leftMostY = track.pointsEdgeLeft[i].y;
                }
            }

            // 找到右边界最右侧的点
            int RightMostY = 0;
            for(size_t i = 20; i < track.pointsEdgeRight.size()-120; i++) 
            {
                if(track.pointsEdgeRight[i].y > RightMostY) 
                {
                    RightMostY = track.pointsEdgeRight[i].y;
                }
            }
 
            // 合并距离相近的线段
            for(const Vec4i &line : lines) 
            {
                Point pt1(line[0], line[1]);
                Point pt2(line[2], line[3]);
 
                // 计算线段中点的y坐标
                midY= (pt1.y + pt2.y) / 2;
 
                // 替换原来的斜率计算部分
                double angle = atan2(static_cast<double>(pt2.y - pt1.y), 
                                    static_cast<double>(pt2.x - pt1.x)) * 180.0 / CV_PI;
                cout << "angle " << angle << endl;

                cout<<"midY "<<midY<<endl;
                // 然后修改判断条件，使用角度范围代替斜率       赛道最左边和赛道最右边
                if(abs(angle) > 15 || midY <= 60 || pt1.x < leftMostY || pt2.x > RightMostY)
                    continue; // 跳过非水平线（角度大于30度视为非水平线）
                    
                // if(will_park1&&midY<75)
                //     continue;

                bool shouldMerge = false;
                // 检查是否有距离相近的已存在线段
                for(Vec4i &merged : mergedLines) 
                {
                    int y1 = (merged[1] + merged[3])/2;  // 已有线段的平均y值
                    int y2 = (pt1.y + pt2.y)/2;         // 当前线段的平均y值
                    
                    if(abs(y1 - y2) < 50)  // 如果y方向距离小于20像素
                    {
                        // 合并并延长线段
                        int minX = min(min(merged[0], merged[2]), min(line[0], line[2]));
                        int maxX = max(max(merged[0], merged[2]), max(line[0], line[2]));
                        int avgY = (y1 + y2) / 2;  // 使用平均y值
                        
                        // 更新为合并后的线段
                        merged[0] = minX;
                        merged[1] = avgY;
                        merged[2] = maxX;
                        merged[3] = avgY;
                        
                        shouldMerge = true;
                        break;
                    }
                }
                
                if(!shouldMerge) 
                {
                    mergedLines.push_back(line);
                }
            }
            
            counterExit++;
            counterSession++;
            cout<<"counterSession "<<counterSession<<"counterExit "<<counterExit<<endl;
            
            if (counterSession > stopTime)  // 结束临时停车状态
            {
                counterRec = 0;
                counterSession = 0;
                laybyEnable = false;
                stopEnable = false;     // 停车使能
                searchingLine = false;  // 搜索直线标志
 
                // 开始冷却时间
                laybyCooldown = true;
                laybyCooldownStart = std::chrono::steady_clock::now();
            }
            else if (mergedLines.size() == 2) // 检测到两条直线
            {
                // 按Y坐标排序线段
                sort(mergedLines.begin(), mergedLines.end(), 
                    [](const Vec4i &a, const Vec4i &b) {
                        return (a[1] + a[3])/2 < (b[1] + b[3])/2;
                    });
                
                // 获取第一条线（Y坐标最小的线）
                Vec4i firstLine = mergedLines[0];
                firstLineY = (firstLine[1] + firstLine[3]) / 2;
                
                hengxian2 = mergedLines;
                searchingLine = true; // 搜索直线标志
                std::cout << "检测到两条线，第一条线Y位置: " << firstLineY << std::endl;
                
                // 如果第一条线位于阈值位置附近（±10像素范围内）
                if (firstLineY >= STOP_LINE_Y_THRESHOLD1 )
                {
                    std::cout << "到达停车位置，准备临时停车" << std::endl;
                    counterExit1++;
                    cout<<"counterExit1 "<<counterExit1<<endl;
                    if(counterExit1)
                    {
                        secondLineY=0;
                        will_park=true;//控制进入下一阶段
                        will_park1=true;//控制低速的
                    }
                }

                else
                {
                    counterExit = 0;
                    counterExit++;
                }
            }
            else if(will_park&&mergedLines.size() == 1)
            {
                // 获取第一条线（Y坐标最小的线）
                Vec4i firstLine1 = mergedLines[0];
                hengxian3=mergedLines;
                secondLineY = (firstLine1[1] + firstLine1[3]) / 2;
                std::cout << "检测到一条线，线Y位置: " << secondLineY << std::endl;
                if(secondLineY>STOP_LINE_Y_THRESHOLD)  
                {
                    firstLineY=0;
                    will_park=false;
                    will_park1=false;
                    counterExit1=0;
                    stopEnable = true;    // 停车使能
                    return true;
                }              
            }
            cout<<endl;
            cout<<endl;
            cout<<endl;
            cout<<endl;
            cout<<endl;
            cout<<endl;
            return true;
        }
            
        else // 检测标志
        {
            for (size_t i = 0; i < predict.size(); i++)
            {
                if (((predict[i].type == LABEL_SCHOOL || predict[i].type == LABEL_COMPANY)  && predict[i].score > 0.7)  && (predict[i].y + predict[i].height) > ROWSIMAGE * 0.20)
                {
                    counterRec++;
                    if (predict[i].x < COLSIMAGE / 2)   // 标识牌在左侧
                        leftEnable = true;
                    else
                        leftEnable = false;
                    break;
                }
            }
            
            if (counterRec)
            {
                counterSession++;
                cout<<"counterRec "<<counterRec<<"counterSession "<<counterSession<<endl;
                if (counterRec >= 1 && counterSession < 80)
                {
                    counterRec = 0;
                    counterSession = 0;
                    laybyEnable = true; // 检测到标识牌子
                    return true;
                }
                else if (counterSession >=80 )
                {
                    counterRec = 0;
                    counterSession = 0;
                }
            }
 
            return false;
        }
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
        
        // 绘制合并后的结果
        for(const Vec4i &line : mergedLines) 
        {
            cv::line(image, Point(line[0], line[1]),Point(line[2], line[3]), Scalar(0, 0, 255), 2);
        }
        
        if (laybyEnable)
            putText(image, "[1] Layby - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    }

    /**
     * @brief 缩减优化车道线（双车道→单车道）
     *
     * @param track
     * @param left
     */
    void curtailTracking(Tracking &track, bool left)
    {
        if (left) // 向左侧缩进
        {
            cout<<"左停"<<endl;
            if (track.pointsEdgeRight.size() > track.pointsEdgeLeft.size())
                track.pointsEdgeRight.resize(track.pointsEdgeLeft.size());

            for (size_t i = 0; i < track.pointsEdgeRight.size(); i++)
            {
                track.pointsEdgeRight[i].y = (track.pointsEdgeRight[i].y + track.pointsEdgeLeft[i].y+55) / 2;
            }
        }
        else // 向右侧缩进
        {
            cout<<"右停"<<endl;
            if (track.pointsEdgeRight.size() < track.pointsEdgeLeft.size())
                track.pointsEdgeLeft.resize(track.pointsEdgeRight.size());

            for (size_t i = 0; i < track.pointsEdgeLeft.size(); i++)
            {
                track.pointsEdgeLeft[i].y = (track.pointsEdgeRight[i].y + track.pointsEdgeLeft[i].y-55) / 2;
            }
        }
    }
private:
    uint16_t counterRec = 0;        // 标识牌检测计数器
    bool laybyEnable = false;       // 临时停车区域使能标志
    bool leftEnable = true;         // 标识牌在左侧
    bool searchingLine = false;     // 搜索直线标志
    vector<Vec4i> mergedLines;      // 合并后的线段用于绘制
    int moment = 20;               // 停车时机，屏幕上方的像素值，值越大越晚停车
    int stopTime = 400;            // 停车时间 40帧
    int STOP_LINE_Y_THRESHOLD = 100; // 停车线Y轴阈值（240*0.5）

};