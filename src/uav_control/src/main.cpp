#include <ros/ros.h>
#include <std_msgs/String.h>
#include <thread>
#include <cstdlib>
#include "pose_subscriber.h"
#include "usb32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"  // 多个浮点数的消息类型
#include "pid.h"
#include <locale.h>
#include "json.h"
#include <iostream>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <string.h>
#include "utils.h"
#include <cmath>

int32_t out1,out2;
int32_t out3,out4;
int32_t out5;

USB32_Structure USB32;

// 启动 Python 脚本的函数
void run_python_script()
{
    // 使用 std::system 启动 Python 脚本
    int ret_code = std::system("python /home/ubuntu/catkin_ws/src/uav_control/scripts/web.py");
    if (ret_code != 0)
    {
        ROS_ERROR("Failed to run Python script.");
    }
}

void restartHectorSlamService()
{
    int ret = system("sudo /bin/systemctl restart ros_hector_slam.service");
    if (ret == 0)
    {
        ROS_INFO("成功重启 ros_hector_slam.service");
    }
    else
    {
        ROS_ERROR("重启 ros_hector_slam.service 失败，返回码：%d", ret);
    }
}
// main 线程函数
void main_thread()
{
    const char* filepath1 = "/dev/shm/shared_memory1";
    const char* filepath2 = "/dev/shm/shared_memory2";
    const int max_size1 = 1024;  // 第一个共享内存文件的大小
    const int max_size2 = 1024;  // 第二个共享内存文件的大小

    // 创建第一个共享内存文件
    int fd1 = open(filepath1, O_CREAT | O_RDWR | O_TRUNC, 0666);
    if (fd1 == -1) {
        perror("open failed for shared_memory1");
        return;
    }
    // 设置第一个文件的大小
    if (ftruncate(fd1, max_size1) == -1) {
        perror("ftruncate failed for shared_memory1");
        close(fd1);
        return;
    }
    // 映射第一个文件到内存
    void* ptr1 = mmap(0, max_size1, PROT_WRITE, MAP_SHARED, fd1, 0);
    if (ptr1 == MAP_FAILED) {
        std::cerr << "mmap failed for shared_memory1\n";
        return;
    }

    // 创建第二个共享内存文件
    int fd2 = open(filepath2, O_CREAT | O_RDWR | O_TRUNC, 0666);
    if (fd2 == -1) {
        perror("open failed for shared_memory2");
        return;
    }
    // 设置第二个文件的大小
    if (ftruncate(fd2, max_size2) == -1) {
        perror("ftruncate failed for shared_memory2");
        close(fd2);
        return;
    }
    // 映射第二个文件到内存
    void* ptr2 = mmap(0, max_size2, PROT_WRITE, MAP_SHARED, fd2, 0);
    if (ptr2 == MAP_FAILED) {
        std::cerr << "mmap failed for shared_memory2\n";
        return;
    }

    // 将数据写入共享内存区域
    const char* message1 = "{\"niganma\":123}";
    const char* message2 = "{\"rrr\":456}";

    // 将第一个消息写入第一个共享内存区域
    strncpy((char*)ptr1, message1, max_size1);
    // 将第二个消息写入第二个共享内存区域
    strncpy((char*)ptr2, message2, max_size2);

    ros::NodeHandle nh;

    // 创建一个Publisher对象，发布到 "position_topic" 话题，消息类型为 std_msgs::Float32MultiArray
    ros::Publisher position_pub = nh.advertise<std_msgs::Float32MultiArray>("c_ros_python", 1000);

    ros::Rate rate(50);  // 设定频率为10Hz (即100ms)

    std_msgs::Float32MultiArray position_msg;  // 用来存储多个位置的数据

    while (ros::ok())
    {
        rate.sleep();

        PositionStructure Pos;
        Pos = getPosition();

        char str[max_size2];
        sprintf(str,"{\
\"x-Outer\": %d,\
\"x-Inner\": %d,\
\"y-Outer\": %d,\
\"y-Inner\": %d,\
\"z-out\": %d,\
\"x-Pose\": %f,\
\"y-Pose\": %f,\
\"z-Pose\": %f,\
\"kp_velocity\": %f,\
\"ki_velocity\": %f,\
\"kd_velocity\": %f,\
\"kp_position\": %f,\
\"ki_position\": %f,\
\"kd_position\": %f\
}",\
            out1,out2,out3,out4,out5,\
            Pos.x,Pos.y,USB32.Altitude,\
            xSpeedPID.Proportion,xSpeedPID.Integral,xSpeedPID.Derivative,\
            xDisplacePID.Proportion,xDisplacePID.Integral,xDisplacePID.Derivative
            );
        strncpy((char*)ptr2, str, max_size2);


        // // 在每次循环时读取共享内存
        char* shared_str1 = static_cast<char*>(ptr1);
        // std::cout << "Read from shared memory: " << shared_str1 << std::endl;
        char* shared_str2 = static_cast<char*>(ptr2);
        // std::cout << "Read from shared memory: " << shared_str2 << std::endl;

        try
        {
            // 清理数据
            std::string json_clean = shared_str1;
            json_clean.erase(0, json_clean.find_first_not_of(" \t\r\n"));
            json_clean.erase(json_clean.find_last_not_of(" \t\r\n") + 1);

            JSON::Value root = JSON::parse(json_clean.c_str());

            if (root.objectValue.find("position_p") != root.objectValue.end()) 
            {
                xDisplacePID.Proportion = root.objectValue["position_p"].numberValue;
                yDisplacePID.Proportion = root.objectValue["position_p"].numberValue;
            }
            if (root.objectValue.find("position_i") != root.objectValue.end()) 
            {
                xDisplacePID.Integral = root.objectValue["position_i"].numberValue;
                yDisplacePID.Integral = root.objectValue["position_i"].numberValue;
            }
            if (root.objectValue.find("position_d") != root.objectValue.end()) 
            {
                xDisplacePID.Derivative = root.objectValue["position_d"].numberValue;
                yDisplacePID.Derivative = root.objectValue["position_d"].numberValue;
            }

            if (root.objectValue.find("speed_p") != root.objectValue.end()) 
            {
                xSpeedPID.Proportion = root.objectValue["speed_p"].numberValue;
                ySpeedPID.Proportion = root.objectValue["speed_p"].numberValue;
            }
            if (root.objectValue.find("speed_i") != root.objectValue.end()) 
            {
                xSpeedPID.Integral = root.objectValue["speed_i"].numberValue;
                ySpeedPID.Integral = root.objectValue["speed_i"].numberValue;
            }
            if (root.objectValue.find("speed_d") != root.objectValue.end()) 
            {
                xSpeedPID.Derivative = root.objectValue["speed_d"].numberValue;
                ySpeedPID.Derivative = root.objectValue["speed_d"].numberValue;
            }
            if (root.objectValue.find("hector_slam") != root.objectValue.end()) 
            {
                // 重启建图
                restartHectorSlamService();
            }
        }
        catch (const std::exception& e) 
        {
            ROS_ERROR("Error parsing JSON: %s", e.what());
        }
    }
    // 清理
    munmap(ptr1, max_size1);
    munmap(ptr2, max_size2);
    close(fd1);
    close(fd2);
}

void timer10msCallback(const ros::TimerEvent&)
{
    PositionStructure Pos;
    Pos = getPosition();

    ScanPositionStructure ScanPos;
    ScanPos = getScanPosition();

    USB32 = get_USB32_Status();

    static bool SetPositionFlag = true;
    // 触发打点
    if(USB32.SetPosition == true && SetPositionFlag == true)
    {
        xDisplacePID.SetPoint = Pos.x*100.0;
        yDisplacePID.SetPoint = Pos.y*100.0;

        // 清除积分
        xDisplacePID.SumError = 0;
        yDisplacePID.SumError = 0;

        xSpeedPID.SumError = 0;
        ySpeedPID.SumError = 0;

        SetPositionFlag = false;
    }
    else if(USB32.SetPosition == false)
    {
        SetPositionFlag = true;
    }

    // // 位置环控制
    // out1 = pid_ctrl(&xDisplacePID, Pos.x * 100.0, 0);
    // // // 速度前馈
    // // float x_speed_feedforward = feedforward_controller(&xFeedforward,Pos.x*100.0); // 设定速度前馈增益
    // // out1 += static_cast<int>(round(x_speed_feedforward));  // 将前馈加入位置控制输出
    // // 限制位置控制输出
    // out1 = out1 > 25 ? 25 : out1;
    // out1 = out1 < -25 ? -25 : out1;
    // // 将位置控制输出作为速度环的目标
    // xSpeedPID.SetPoint = out1;
    // // 速度环控制
    // out2 = pid_ctrl(&xSpeedPID, Pos.x_s * 100.0, 0);
    // // // 加速度前馈
    // // float x_acceleration_feedforward = feedforward_controller(&x_sFeedforward,Pos.x_s*100.0); // 设定加速度前馈增益
    // // out2 += static_cast<int>(round(x_acceleration_feedforward));  // 将前馈加入速度控制输出
    // // 限制速度控制输出
    // out2 = out2 > 250 ? 250 : out2;
    // out2 = out2 < -250 ? -250 : out2;

    // 位置环控制
    out3 = pid_ctrl(&yDisplacePID, Pos.y * 100.0, 0);
    // 速度前馈
    float y_speed_feedforward = -ScanPos.y_s * 100.0; // 设定速度前馈增益
    out3 += static_cast<int>(round(y_speed_feedforward));  // 将前馈加入位置控制输出

    // 限制位置控制输出
    out3 = out3 > 25 ? 25 : out3;
    out3 = out3 < -25 ? -25 : out3;
    // 将位置控制输出作为速度环的目标
    ySpeedPID.SetPoint = out3;
    // 速度环控制
    out4 = pid_ctrl(&ySpeedPID, ScanPos.y_s * 100.0, 0);
    // // 加速度前馈
    // float y_acceleration_feedforward = feedforward_controller(&y_sFeedforward,Pos.y_s*100.0); // 设定加速度前馈增益
    // out4 += static_cast<int>(round(y_acceleration_feedforward));  // 将前馈加入速度控制输出
    // 限制速度控制输出
    out4 = out4 > 250 ? 250 : out4;
    out4 = out4 < -250 ? -250 : out4;

    // ROS_INFO("Position -> x: %d, y: %d", out2, out4);

    // ROS_INFO("%f,%d,%d",Pos.x * 100.0,static_cast<int>(round(x_speed_feedforward)),static_cast<int>(round(x_acceleration_feedforward)));
    // ROS_INFO("%f,%d,%d\r\n",Pos.y * 100.0,static_cast<int>(round(y_speed_feedforward)),static_cast<int>(round(y_acceleration_feedforward)));
    // ROS_INFO("%f,%f\r\n",Pos.x_s*100.0,Pos.y_s*100.0);

    // 定高
    HeightPID.SetPoint = 100.0;
    out5 = pid_ctrl(&HeightPID,USB32.Altitude,0);
    out5 = out5>500?500:out5;
    out5 = out5<-500?-500:out5;

    usb32_send_frame(0,out4,0);
}

int main(int argc, char** argv)
{
    // 设置本地化环境，启用 UTF-8 支持
    setlocale(LC_ALL, "en_US.UTF-8");

    ///////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////

    ros::init(argc, argv, "uav_control");
    ros::NodeHandle nh;
    ros::Time::init();

    feedforward_init(&xFeedforward,1.4,0.7);
    feedforward_init(&x_sFeedforward,1.4,0.7);
    feedforward_init(&yFeedforward,1.4,0.7);
    feedforward_init(&y_sFeedforward,1.4,0.7);

    pid_init(&xDisplacePID);
    pid_init(&xSpeedPID);
    pid_init(&yDisplacePID);
    pid_init(&ySpeedPID);
    pid_init(&HeightPID);

    initLowPassFilter(&x_sLowPassFilter,2.0,10.0);
    initLowPassFilter(&y_sLowPassFilter,2.0,10.0);
    KalmanFilter_Init(&xDisKalmanFilter,1,1.0,1,0);
    KalmanFilter_Init(&yDisKalmanFilter,1,1.0,1,0);
    
    initLowPassFilter(&x_sImuLowPassFilter,2.0,10.0);
    initLowPassFilter(&y_sImuLowPassFilter,2.0,10.0);
    KalmanFilter_Init(&xImuDisKalmanFilter,0.12,1.0,1,0);
    KalmanFilter_Init(&yImuDisKalmanFilter,0.12,1.0,1,0);

    initLowPassFilter(&x_sScanLowPassFilter,2.0,10.0);
    initLowPassFilter(&y_sScanLowPassFilter,2.0,10.0);
    KalmanFilter_Init(&xScanDisKalmanFilter,0.52,1.0,1,0);
    KalmanFilter_Init(&yScanDisKalmanFilter,0.52,1.0,1,0);
    // 创建一个 10ms 的定时器
    ros::Timer timer = nh.createTimer(ros::Duration(0.05), timer10msCallback);

    // 启动串口读取线程
    std::thread _Usb32_thread(usb32_thread);

    // 启动 main 线程
    std::thread _Main_thread(main_thread);

    // 启动 Python 脚本线程
    std::thread _Python_thread(run_python_script);

    // 订阅多个话题
    ros::Subscriber pose_sub = subscribeToPoseTopic(nh);  // SLAM位姿
    ros::Subscriber scan_pose = subscribeToScanPoseTopic(nh);
    // // ROS 运行
    // ros::spin();
    // 使用 AsyncSpinner 代替 ros::spin
    ros::AsyncSpinner spinner(4);  // 支持 4 个线程同时处理回调
    spinner.start();

    // 等待所有线程结束
    _Usb32_thread.join();
    _Main_thread.join();
    _Python_thread.join();

    return 0;
}
