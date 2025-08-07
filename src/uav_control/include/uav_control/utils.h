#ifndef _UTILS_H
#define _UTILS_H

#include <ros/ros.h>

// 定义一阶低通滤波器结构体
typedef struct {
    double alpha;   // 滤波系数
    double prev_y;  // 上一个输出
} LowPassFilter;
extern LowPassFilter x_sLowPassFilter;
extern LowPassFilter y_sLowPassFilter;
extern LowPassFilter x_sImuLowPassFilter;
extern LowPassFilter y_sImuLowPassFilter;
extern LowPassFilter x_sScanLowPassFilter;
extern LowPassFilter y_sScanLowPassFilter;

// 初始化滤波器
void initLowPassFilter(LowPassFilter* filter, float cutoff_frequency, float sample_rate);
// 应用低通滤波器
float applyLowPassFilter(LowPassFilter* filter, float input);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 定义卡尔曼滤波器结构体
typedef struct
{
    double Q; // 过程噪声协方差
    double R; // 观测噪声协方差
    double P; // 估计误差协方差
    double X; // 估计值
    double K; // 卡尔曼增益
} KalmanFilter;
extern KalmanFilter xDisKalmanFilter;
extern KalmanFilter yDisKalmanFilter;
extern KalmanFilter xImuDisKalmanFilter;
extern KalmanFilter yImuDisKalmanFilter;
extern KalmanFilter xScanDisKalmanFilter;
extern KalmanFilter yScanDisKalmanFilter;

// 初始化滤波器
void KalmanFilter_Init(KalmanFilter* kf, double process_noise, double sensor_noise, double estimated_error, double initial_value);
// 预测（可选）
void KalmanFilter_Predict(KalmanFilter* kf);
// 更新
double KalmanFilter_Update(KalmanFilter* kf, double measurement);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct {
    float prev_result;        // 存储上一次滤波后的前馈信号
    float Per_velocity;      // 上两次的速度
    float Last_velocity;     // 上一次的速度
    float a;
    float b;
    float result;
    float filtered_result;
} FeedforwardController;
extern FeedforwardController xFeedforward;
extern FeedforwardController x_sFeedforward;

extern FeedforwardController yFeedforward;
extern FeedforwardController y_sFeedforward;

void feedforward_init(FeedforwardController* controller,float a,float b);
float feedforward_controller(FeedforwardController* controller, float current_rin);

#endif
