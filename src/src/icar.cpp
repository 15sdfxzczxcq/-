/**
 ********************************************************************************************************
大佬说的队
 *********************************************************************************************************
 * @file icar.cpp
 * @author Leo
 * @brief 智能汽车-顶层框架（TOP）
 * @version 0.1
 * @date 2025-05-11
 * @copyright Copyright (c) 2025
 *
 */

#include "../include/common.hpp"     //公共类方法文件
#include "../include/detection.hpp"  //百度Paddle框架移动端部署
#include "../include/uart.hpp"       //串口通信驱动
#include "controlcenter.cpp"         //控制中心计算类
#include "detection/bridge.cpp"      //AI检测：坡道区
#include "detection/obstacle.cpp"    //AI检测：障碍区
#include "detection/catering.cpp"    //AI检测：餐饮区
#include "detection/layby.cpp"       //AI检测：临时停车区
#include "detection/parking.cpp"     //AI检测：充电停车场
#include "detection/crosswalk.cpp"   //AI检测：停车区
#include "motion.cpp"                //智能车运动控制类
#include "preprocess.cpp"            //图像预处理类
#include "recognition/crossroad.cpp" //十字道路识别与路径规划类
#include "recognition/ring.cpp"      //环岛道路识别与路径规划类
#include "recognition/tracking.cpp"  //赛道识别基础类
#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <signal.h>
#include <unistd.h>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <fcntl.h> //用于文件控制操作
#include <termios.h> //用于终端控制  
#include <unistd.h>     //用于标准输入文件描述符//为了停车这两条
#include "../include/motion.hpp" 

using namespace std;
using namespace cv;
cv::Scalar getCvcolor(int index);
void drawBox(Mat &img);

Mat ai_image;//创建AI图像

//创建空线程
std::thread ai_thread;//ai
std::timed_mutex  mut;

std::thread send_thread;//串口
std::mutex control_mutex;

std::vector<PredictResult> result;

std::atomic<float> atomic_speed(0);    // 原子变量存储速度
std::atomic<int> atomic_servo(PWMSERVOMID); // 原子变量存储舵机


struct ControlData {
    float speed = 0;
    int servoPwm = PWMSERVOMID;
};


ControlData current_control;         // 当前控制参数
std::condition_variable control_cv;        // 条件变量用于线程通知

bool start_ai=false;
bool new_data_available = false;    // 新数据标志

bool buzzer = false;
int buzzernum = 50; //蜂鸣器时长
int imgnum=0;
bool buzzerTriggered=false;
float  stoptime=0;
float speedstate = 0;

float prospect = 0;       //数字越低越高
    float prospect21 = 0;     //环岛
    float prospect22 = 0; 
    float prospect23 = 0;
    float prospect24 = 0;
    float prospect25 = 0;

    float prospect7 = 0;   //充电区
    float prospect6 = 0;   //临时停车
    float prospect5 = 0;   //快餐
    float prospect51 = 0;
    float prospect4 = 0;   //障碍区

    float prospect01 = 0;   //基础赛道 直到
    float prospect02 = 0;   //弯道

    float speedParking_stop=0; //停车速度

    float prospect1=0;

void thread_ai(std::shared_ptr<Detection> detection1){
  while(1){
    //std::cout<<"ai_working_now"<<std::endl;
    if(start_ai){
      mut.lock();
      Mat ii=ai_image.clone();
      start_ai=false;
      mut.unlock();
      detection1->inference(ii);//ai推理
      mut.lock();
      result=detection1->results;
      mut.unlock();
      //std::cout<<"ai_result_appear"<<std::endl;
    }
    else{
       std::this_thread::sleep_for(1ms);
    }
  }
}

void Send(std::shared_ptr<Uart> uart1) {
    int ret = uart1->open();
    if (ret != 0) {
        printf("[Error] Uart Open failed!\n");
        return;
    }

    // 设置非阻塞输入
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    const auto CYCLE_TIME = std::chrono::microseconds(12500); // 精确12.5ms发送一次串口
    auto nextCycle = std::chrono::steady_clock::now();

    while (true) {
        // 键盘检测 - 紧急停车
        char key = getchar();
        if (key == 'q' || key == 'Q') {
            printf("\n-----> Emergency Stop Triggered by Q key! <-----\n");
            
        // 发送5次停车指令确保可靠
        for (int i = 0; i < 5; i++) {
            uart1->carControl(0, PWMSERVOMID, false, 0);
            std::this_thread::sleep_for(10ms);
        }
            
            uart1->close();
            exit(0);
        }

        // 移除未使用的 hasNewData 变量
        std::unique_lock<std::mutex> lock(control_mutex);
        control_cv.wait_until(lock, nextCycle, [] {
            return new_data_available;
        });

        float sendSpeed = current_control.speed;
        int sendServo = current_control.servoPwm;
        new_data_available = false;
        lock.unlock();

        // 修改后的重试逻辑（不依赖返回值）
        int retryCount = 0;
        while (retryCount < 3) {
            uart1->carControl(sendSpeed, sendServo , buzzer, speedstate);
            //std::cout << speedstate << std::endl;
            std::this_thread::sleep_for(1ms);
            retryCount++;
            // 可添加硬件状态检测（如 CTS 信号）
            // if (uart1->isClearToSend()) break;
        }

        nextCycle += CYCLE_TIME;
        std::this_thread::sleep_until(nextCycle);

        // 调试输出保持原样
        /*static int frameCount = 0;
        if (++frameCount % 100 == 0) {
            std::cout << "[UART] 实际频率: " 
                      << 100000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - nextCycle + CYCLE_TIME).count()
                      << " Hz" << std::endl;
        }*/
        //std::cout << "speed:" << sendSpeed << "servo" << sendServo << std::endl;
    }
}


void mouseCallback(int event, int x, int y, int flags, void *userdata);
Display display; // 初始化UI显示窗口
void setParams1(const Motion::Params &params) {
        prospect21 = params.prospect21;
        prospect22 = params.prospect22;
        prospect23 = params.prospect23;
        prospect24 = params.prospect24;
        prospect25 = params.prospect25;
        prospect7 = params.prospect7;
        prospect6 = params.prospect6;
        prospect5 = params.prospect5;
        prospect51 = params.prospect51;
        prospect4 = params.prospect4;
        prospect01 = params.prospect01;
        prospect02 = params.prospect02;
        stoptime   = params.stoptime;
        speedParking_stop=params.speedParking_stop;
        prospect1=params.prospect1;
        
        // speedtrackout_plus=params.speedtrackout_plus;
    }
int main(int argc, char const *argv[]) {
  Preprocess preprocess;    // 图像预处理类
  Motion motion;            // 运动控制类
  Tracking tracking;        // 赛道识别类
  Crossroad crossroad;      // 十字道路识别类
  Ring ring;                // 环岛识别类
  Bridge bridge;            // 坡道区检测类
  Catering catering;        // 快餐店检测类
  Obstacle obstacle;        // 障碍区检测类
  Layby layby;              // 临时停车区检测类
  Parking parking;          // 充电停车场检测类
  StopArea stopArea;        // 停车区识别与路径规划类
  ControlCenter ctrlCenter; // 控制中心计算类
  VideoCapture capture;     // Opencv相机类
  int countInit = 0;        // 初始化计数器
  catering.setParams(motion.params);
  setParams1(motion.params);
  bridge.setParams2(motion.params);
  layby.setParams6(motion.params);
  obstacle.setParams4(motion.params);
  parking.setParams7(motion.params);
  
  // 目标检测类(AI模型文件)
  shared_ptr<Detection> detection = make_shared<Detection>(motion.params.model);
  detection->score = motion.params.score; // AI检测置信度

  // USB转串口初始化： /dev/ttyUSB0
  shared_ptr<Uart> uart = make_shared<Uart>("/dev/ttyUSB0"); // 初始化串口驱动
  //uart->startReceive(); // 启动数据接收子线程

  ai_thread = std::thread(thread_ai, detection);//启动ai推理
  send_thread = std::thread(Send, std::ref(uart));//启动串口发送
  //std::cout << "result" << result.size() << std::endl;

  // USB摄像头初始化
    capture = VideoCapture("/dev/video0"); // 打开摄像头
  if (!capture.isOpened()) {
    printf("can not open video device!!!\n");
    return 0;
  }

  VideoWriter vw;
  int fourcc = vw.fourcc('M','J','P','G');//设置摄像头编码方式
  capture.set(CAP_PROP_FOURCC,fourcc);
  capture.set(CAP_PROP_FRAME_WIDTH, COLSIMAGE);  // 设置图像分辨率
  capture.set(CAP_PROP_FRAME_HEIGHT, ROWSIMAGE); // 设置图像分辨率
  capture.set(CAP_PROP_FPS, 100);                 // 设置帧率


  // 初始化参数
  Scene scene = Scene::NormalScene;     // 初始化场景：常规道路
  long preTime;
  Mat img;
  if(motion.params.debug && !motion.params.saveImg){
  namedWindow("a", WINDOW_AUTOSIZE);
  }

  while (1) {

    // 循环开始计时
    auto loop_start = std::chrono::high_resolution_clock::now(); 

    //a计数器
    auto a_start = std::chrono::high_resolution_clock::now(); 
    //摄像头读取
    if (!capture.read(img))
      continue;

    //a结束计数器
    //计时器结束位置
    auto a_end = std::chrono::high_resolution_clock::now();
    double a_time = std::chrono::duration<double>(a_end - a_start).count() * 1000; // 毫秒
    //std::cout << "摄像头耗时: " << a_time << " ms" << std::endl;
    
    // [在需要测量的代码块前后插入计时器]
    auto t_start = std::chrono::high_resolution_clock::now();
    //[02] 图像克隆
    Mat imgCorrect = img.clone();
    //ai线程获取ai_image图像
    if (mut.try_lock()) {  
        ai_image=img.clone();
        start_ai=true;
        mut.unlock();  
    }
    Mat imgBinary = preprocess.binaryzation(imgCorrect); // 图像二值化




    //[04] 赛道识别
    tracking.rowCutUp = motion.params.rowCutUp; // 图像顶部切行（前瞻距离）
    tracking.rowCutBottom = motion.params.rowCutBottom; // 图像底部切行（盲区距离）
    tracking.trackRecognition(imgBinary);


  //ai线程获取ai_image图像
    if (mut.try_lock()) {  
        ai_image=img.clone();
        start_ai=true;
        mut.unlock();  
    }


    //[05] 停车区检测
    if (motion.params.stop) {
      if (stopArea.process(result)) 
      {
        scene = Scene::StopScene;
        if (stopArea.countExit > stoptime) {
            
      //串口1
          atomic_speed.store(0);               // 直接设置目标速度 // [!code focus]
          atomic_servo.store(PWMSERVOMID);      // 直接设置舵机中立位 // [!code focus]
      {
          std::lock_guard<std::mutex> lock(control_mutex);
          current_control.speed = atomic_speed.load(); // [!code focus]
          current_control.servoPwm = atomic_servo.load(); // [!code focus]
          new_data_available = true;
      }
          control_cv.notify_one();  // 必须添加通知！ // [!code focus]
      //串口1结束
          
          sleep(1);
          printf("-----> System Exit!!! <-----\n");
          exit(0); // 程序退出
        }
      }
    }
    
    //[06] 快餐店检测
    if ((scene == Scene::NormalScene || scene == Scene::CateringScene) && motion.params.catering) {
    if (catering.process(tracking, imgBinary, result)) {
        scene = Scene::CateringScene;
        
        if (!buzzerTriggered) { 

            buzzer = true;
            buzzerTriggered = true;
            
        }
    } else {
        scene = Scene::NormalScene;
        buzzerTriggered = false; // 检测结束重置状态
    }
}

    //[07] 临时停车区检测
    if ((scene == Scene::NormalScene || scene == Scene::LaybyScene) &&
        motion.params.layby) {
      if (layby.process(tracking,imgBinary,result))  // 传入二值化图像进行再处理
        {
        scene = Scene::LaybyScene;
        cout<<motion.speed<<endl;
        buzzer=true;
        }
      else
        scene = Scene::NormalScene;
    }

    //[08] 充电停车场检测
    if ((scene == Scene::NormalScene || scene == Scene::ParkingScene) &&
        motion.params.parking) {
      if (parking.process(tracking,imgBinary,result))  // 传入二值化图像进行再处理
        {
          scene = Scene::ParkingScene;
          buzzer=true;
        }
      else
        scene = Scene::NormalScene;
    }
    
    //[09] 坡道区检测
    if ((scene == Scene::NormalScene || scene == Scene::BridgeScene) &&
        motion.params.bridge) {
      if (bridge.process(tracking, result))
        scene = Scene::BridgeScene;
      else
        scene = Scene::NormalScene;
    }

    // [10] 障碍区检测
    if ((scene == Scene::NormalScene || scene == Scene::ObstacleScene) &&
        motion.params.obstacle) {
      if (obstacle.process(tracking, result)) {
        buzzer=true;
        scene = Scene::ObstacleScene;
      } else
        scene = Scene::NormalScene;
    }

    //[11] 十字道路识别与路径规划
    if ((scene == Scene::NormalScene || scene == Scene::CrossScene) &&
        motion.params.cross) {
      if (crossroad.crossRecognition(tracking, imgBinary))
      {
        scene = Scene::CrossScene;
        buzzer=true;
      }
      else
        scene = Scene::NormalScene;
    }

    //[12] 环岛识别与路径规划
    if ((scene == Scene::NormalScene || scene == Scene::RingScene) &&
        motion.params.ring) {
      if (ring.process(tracking, imgCorrect)){
        scene = Scene::RingScene;
        buzzer=true;
      }
      else
        scene = Scene::NormalScene;
    }

    //计时器结束位置
    auto t_end = std::chrono::high_resolution_clock::now();
    double time_cost = std::chrono::duration<double>(t_end - t_start).count() * 1000; // 毫秒
    //std::cout << "赛道处理耗时: " << time_cost << " ms" << std::endl;

    //中线计数器
    auto t_cap_start = std::chrono::high_resolution_clock::now();

    //[13] 车辆控制中心拟合
   


        if (scene == 2) {
          if(ring.ringStep == 1){  
          prospect=prospect21;
          speedstate = 0;
          }
          else if(ring.ringStep == 2){
          prospect=prospect22;
          speedstate = 4;
          }
          else if(ring.ringStep == 3){
          prospect=prospect23;
          speedstate = 1;
          }
          else if(ring.ringStep == 4){
          prospect=prospect24;
          speedstate = 1;
          }
          else if(ring.ringStep == 5){
          prospect=prospect25;
          speedstate = 1;
          }

        }

        else if(scene==1)//十字
        {
          if(!crossroad.xieru_PID)
          {
            speedstate=0;
            prospect=0.85;
          }
          else
          {
            speedstate=4;
            prospect=prospect1;
          }
        }

        else if(scene == 7)//充电区   //在这个里面写取中线是最好的
        {
        //  prospect7 = 0.95;
          // 添加对 exit 步态的特殊处理
          if (parking.step == parking.ParkStep::exit) {
              speedstate = 0;         // 直线状态
          } 
          else if(parking.step==parking.Parking::enable)
          {
            prospect=prospect7;
            speedstate=0;
          }
          else if (parking.step == parking.ParkStep::turning)//转弯
          {
            if(!parking.turning_ing)
            {
              speedstate = 0;
            }
            else
            {
              speedstate=1;
            }
          }
          else if (parking.step==parking.ParkStep::trackout)//出库中
          {
            speedstate=1;
          }
          else  
          {
            if (motion.speed == motion.params.speedHigh){
          //      prospect01 = 0.67;//直道实线 原0.60            yuan0.62  数字越低越高！！！
                prospect=prospect01;
                speedstate = 0;                           //0是直线
            }
            else{
          //      prospect02 = 0.75; //弯道实线 原0.67           yuan0.75  有压弯情况出现调大弯道prospect
                prospect=prospect02;
                speedstate = 1;                           //1是弯道
            }
          }
        }

        else if(scene==6)//临时停车
        {
        //  prospect6=0.85;
          prospect=prospect6;
          speedstate = 2;                           //0是直线

        }
        else if(scene == 5) //快餐
        {
          if(catering.bugerflag==false)
            {prospect=prospect5;
            speedstate = 2;}
            else{
            prospect=prospect51;
            speedstate = 2;

            }
        }
        else if(scene == 4)  //障碍
        {
        //    prospect4 = 0.75;
            prospect=prospect4;

            if(obstacle.block==false)
           { speedstate = 1;}
          else 
         { speedstate = 2;}

        } else 
        
        {
            if (motion.speed == motion.params.speedHigh){
          //      prospect01 = 0.67;//直道实线 原0.60            yuan0.62  数字越低越高！！！
                prospect=prospect01;
                speedstate = 0;                           //0是直线
            }
            else{
          //      prospect02 = 0.75; //弯道实线 原0.67           yuan0.75  有压弯情况出现调大弯道prospect
                prospect=prospect02;
                speedstate = 1;                           //1是弯道
            }
        }

        //std::cout << "speed:" << motion.speed << std::endl;
        ctrlCenter.fitting(tracking, prospect, imgCorrect, scene);



    //[14] 运动控制(速度+方向)
    if (countInit > 0) // 非调试模式下
    {
      // 触发停车
      if ((catering.stopEnable && scene == Scene::CateringScene) || (layby.stopEnable && scene == Scene::LaybyScene))
      {
        motion.speed = 0;
      }
      else if(parking.step == parking.ParkStep::stop)//入库停车
      {
        motion.speed=motion.params.speedParking_stop;
      }
      else if (scene == Scene::CateringScene)
        motion.speed = motion.params.speedCatering;
      else if (scene == Scene::LaybyScene&&!layby.will_park)
        motion.speed = motion.params.speedLayby;
      else if(scene==Scene::LaybyScene&&layby.will_park1)
        motion.speed=motion.params.speedLayby1;
      else if (scene == Scene::ParkingScene && parking.step == parking.ParkStep::trackout) // 倒车出库
        motion.speed = motion.params.speedtrackout;
      else if (scene==Scene::ParkingScene&&parking.step==parking.ParkStep::exit)//出库延时
      {
        motion.speed=motion.params.speedtrackout_plus;
        // prospect=prospect01;
        // speedstate = 0;                           //0是直线
      }
      else if (scene == Scene::ParkingScene&&!parking.turningspeed) // 充电区速度
        motion.speed = motion.params.speedParking;
      else if(parking.turningspeed)   //转弯速度
      {
        // cout<<"停车速度"<<motion.params.speedturning<<endl;
        motion.speed = motion.params.speedturning;
      }
      else if (scene == Scene::ParkingScene) // 减速
        motion.speed = motion.params.speedParking;
      else if (scene == Scene::BridgeScene) // 坡道速度
        motion.speed = motion.params.speedBridge;
      else if (scene == Scene::ObstacleScene) // 危险区速度
      
       { if(obstacle.block==false)
       motion.speed = motion.params.speedObstacle;
       else
       motion.speed = motion.params.speedblock;
       }
     else if (scene == Scene::RingScene) {
        if(ring.ringStep == 1||ring.ringStep == 2){
          motion.speed =  motion.params.speedRing1;
        }
        else{
          motion.speed = motion.params.speedRing2;
        }
      }
      else if (scene == Scene::StopScene)
        motion.speed = motion.params.speedLow;
      else
        motion.speedCtrl(true, false, ctrlCenter); // 车速控制

      motion.poseCtrl(ctrlCenter.controlCenter); // 姿态控制（舵机）

        //串口发送3
      atomic_speed.store(motion.speed);     // 更新当前速度 // [!code focus]
      atomic_servo.store(motion.servoPwm);   // 更新当前舵机 // [!code focus]
    {
      std::lock_guard<std::mutex> lock(control_mutex);
      current_control.speed = atomic_speed.load(); // [!code focus]
      current_control.servoPwm = atomic_servo.load(); // [!code focus]
      new_data_available = true;
    }
      control_cv.notify_one();  // 必须添加通知！ // [!code focus]
    //串口3结束 

    } else
      countInit++;

    //中线计数器结束
     auto t_cap_end = std::chrono::high_resolution_clock::now();
     double time_cap = std::chrono::duration<double>(t_cap_end - t_cap_start).count() * 1000;
     //std::cout << "中线耗时: " << time_cap << " ms" << std::endl;

    //存图计数器
    auto t_track_start = std::chrono::high_resolution_clock::now();
    //显示图像
    if(motion.params.debug && !motion.params.saveImg){
        //display.setNewWindow(2, "Binary", imgBinary);
        //Mat imgRes =Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
        detection->drawBox(imgCorrect); // 图像绘制AI结果
        ctrlCenter.drawImage(tracking,imgCorrect); // 图像绘制路径计算结果（控制中心）
        //ring.drawImage(tracking, imgCorrect);
        putText(imgCorrect, "scene" + formatDoble2String(scene, 1) , Point(70, 20),
        FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
        putText(imgCorrect, formatDoble2String(motion.speed, 1) + "m/s", Point(COLSIMAGE - 70, 80),
        FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 显示车速
        putText(imgCorrect, "pro" + formatDoble2String(prospect, 1) , Point(70, 80),
        FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 显示车速
        //display.setNewWindow(1, "Ctrl", imgCorrect);
        //display.show(); // 显示综合绘图
        imshow("a",imgCorrect);
        waitKey(1);    // 等待显示
     }

      if(buzzer)
   {
    imgnum++;
    if(imgnum>=buzzernum)
    {
      imgnum=0;
      buzzer=false;
    }
   }

   //[15]存图
    if (motion.params.saveImg && !motion.params.debug) {
        drawBox(imgCorrect);
        putText(imgCorrect, "scene" + formatDoble2String(scene, 1) , Point(70, 20),
        FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
        putText(imgCorrect, formatDoble2String(motion.speed, 1) + "m/s", Point(COLSIMAGE - 70, 80),
        FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 显示车速
        putText(imgCorrect, "pro:0." + formatDoble2String(prospect*100, 1) , Point(70, 80),
        FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 显示车速
        putText(imgCorrect, "ringstep" + formatDoble2String(ring.ringStep, 1) , Point(100, 100),
        FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 显示车速
        ctrlCenter.drawImage(tracking, imgCorrect);
        putText(imgCorrect,"counterSession "+formatDoble2String(parking.counterSession,1),Point(0,150),FONT_HERSHEY_PLAIN,1, Scalar(0, 0, 255), 1);//显示停车计时
        putText(imgCorrect,"firstLineY "+formatDoble2String(layby.firstLineY,1),Point(0,180),
        FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);//停车时机
        putText(imgCorrect,"secondLineY "+formatDoble2String(layby.secondLineY,1),Point(0,200),
        FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);//减速时机
        putText(imgCorrect,"counterSession "+formatDoble2String(layby.counterSession,1),Point(0,220),
        FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);//停车倒计时
        putText(imgCorrect,"exitCountdown "+formatDoble2String(parking.exitCountdown,1),Point(0,160),
        FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);//出库延时倒计时
        putText(imgCorrect,"slope6 "+formatDoble2String(parking.slope6,1),Point(0,240),
        FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);//布线斜率
        putText(imgCorrect,"speedstate "+formatDoble2String(speedstate,1),Point(0,230),
        FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);//状态
        putText(imgCorrect,"turningspeed "+formatDoble2String(parking.turningspeed,1),Point(0,130),
        FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);//状态
        putText(imgCorrect,"count "+formatDoble2String(ring.count,1),Point(100,130),
        FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);//状态
        
        // 绘制横线
        for (const auto& line : parking.hengxian) {
        Point startPoint(line[0], line[1]); // 起点
        Point endPoint(line[2], line[3]);   // 终点
        cv::line(imgCorrect, startPoint,endPoint , Scalar(255, 0, 0), 2);
        }
        // 绘制横线
        for (const auto& line : layby.hengxian2) {
        Point startPoint(line[0], line[1]); // 起点
        Point endPoint(line[2], line[3]);   // 终点
        cv::line(imgCorrect, startPoint,endPoint , Scalar(255, 0, 0), 2);
        }
        // 绘制横线
        for (const auto& line : layby.hengxian3) {
        Point startPoint(line[0], line[1]); // 起点
        Point endPoint(line[2], line[3]);   // 终点
        cv::line(imgCorrect, startPoint,endPoint , Scalar(255, 0, 0), 2);
        }
        // 绘制横线
        for (const auto& line : parking.stophenxian) {
        Point startPoint(line[0], line[1]); // 起点
        Point endPoint(line[2], line[3]);   // 终点
        cv::line(imgCorrect, startPoint,endPoint , Scalar(0, 0, 255), 2);
        }
        //绘制点
        if(parking.hengxian1.x>0)
        {
           circle(imgCorrect, Point(parking.hengxian1.x,parking.hengxian1.y), 8, Scalar(255, 0, 0), -1);
        }
        //画边缘线范围
        for (int i = 0; i < COLSIMAGE; i+=3)
        {
            circle(imgCorrect, Point(i,tracking.pointsEdgeRight[5 * tracking.pointsEdgeRight.size() / 6].x), 1,
            Scalar(192, 192, 192), -1); // 下
            circle(imgCorrect, Point(i,tracking.pointsEdgeLeft[5 * tracking.pointsEdgeLeft.size() / 6].x), 1,
            Scalar(192, 192, 192), -1); // 上
        }
        savePicture(imgCorrect);
    }

    //存图计数器结束
    auto t_track_end = std::chrono::high_resolution_clock::now();
    double time_track = std::chrono::duration<double>(t_track_end - t_track_start).count() * 1000;
   //std::cout << "存图耗时: " << time_track << " ms" << std::endl;  

    if (scene == Scene::ObstacleScene)
      scene = Scene::NormalScene;
    else if (scene == Scene::CrossScene)
      scene = Scene::NormalScene;
    else if (scene == Scene::RingScene)
      scene = Scene::NormalScene;
    else if (scene == Scene::CateringScene)
      scene = Scene::NormalScene;
    else if (scene == Scene::LaybyScene)
      scene = Scene::NormalScene;
    else if (scene == Scene::ParkingScene)
      scene = Scene::NormalScene;
    else if (scene == Scene::StopScene)
      scene = Scene::NormalScene;


    auto loop_end = std::chrono::high_resolution_clock::now(); // 循环结束计时
    double loop_time = std::chrono::duration<double>(loop_end - loop_start).count() * 1000; // 毫秒
    //std::cout << "单次循环耗时: " << loop_time << " ms" << std::endl;
  }

  start_ai = false;
  if (ai_thread.joinable()) { // 检查线程是否已被创建（即可连接）  
    ai_thread.join(); // 等待线程结束  
  }


if (uart) {
    uart->close(); // 关闭串口并停止接收线程
}

if (send_thread.joinable()) {
    send_thread.join(); // 等待发送线程结束
}

  capture.release();
  return 0;
}

void drawBox(Mat &img)
{
    for (uint16_t i = 0; i < result.size(); i++)
    {
        PredictResult result1 = result[i];

        auto score = std::to_string(result1.score);
        int pointY = result1.y - 20;
        if (pointY < 0)
            pointY = 0;
        cv::Rect rectText(result1.x, pointY, result1.width, 20);
        cv::rectangle(img, rectText, getCvcolor(result1.type), -1);
        std::string label_name = result1.label + " [" + score.substr(0, score.find(".") + 3) + "]";
        cv::Rect rect(result1.x, result1.y, result1.width, result1.height);
        cv::rectangle(img, rect, getCvcolor(result1.type), 1);
        cv::putText(img, label_name, Point(result1.x, result1.y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 254), 1);
    }
}

cv::Scalar getCvcolor(int index)
{
    switch (index)
    {
    case 0:
        return cv::Scalar(0, 255, 0); // 绿
        break;
    case 1:
        return cv::Scalar(255, 255, 0); // 天空蓝
        break;
    case 2:
        return cv::Scalar(0, 0, 255); // 大红
        break;
    case 3:
        return cv::Scalar(0, 250, 250); // 大黄
        break;
    case 4:
        return cv::Scalar(250, 0, 250); // 粉色
        break;
    case 5:
        return cv::Scalar(0, 102, 255); // 橙黄
        break;
    case 6:
        return cv::Scalar(255, 0, 0); // 深蓝
        break;
    case 7:
        return cv::Scalar(255, 255, 255); // 大白
        break;
    case 8:
        return cv::Scalar(247, 43, 113);
        break;
    case 9:
        return cv::Scalar(40, 241, 245);
        break;
    case 10:
        return cv::Scalar(237, 226, 19);
        break;
    case 11:
        return cv::Scalar(245, 117, 233);
        break;
    case 12:
        return cv::Scalar(55, 13, 19);
        break;
    case 13:
        return cv::Scalar(255, 255, 255);
        break;
    case 14:
        return cv::Scalar(237, 226, 19);
        break;
    case 15:
        return cv::Scalar(0, 255, 0);
        break;
    default:
        return cv::Scalar(255, 0, 0);
        break;
    }
}