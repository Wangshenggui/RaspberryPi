#include "utils.h"

// 定义结构体
LowPassFilter x_sLowPassFilter;
LowPassFilter y_sLowPassFilter;
LowPassFilter x_sImuLowPassFilter;
LowPassFilter y_sImuLowPassFilter;
LowPassFilter x_sScanLowPassFilter;
LowPassFilter y_sScanLowPassFilter;

// 初始化滤波器
void initLowPassFilter(LowPassFilter* filter, float cutoff_frequency, float sample_rate) 
{
    // 计算滤波器的系数 alpha
    float RC = 1.0f / (2 * 3.141592653589793 * cutoff_frequency);
    float dt = 1.0f / sample_rate;  // 采样周期
    filter->alpha = dt / (RC + dt);
    filter->prev_y = 0.0;  // 初始输出为0
}

// 应用低通滤波器
float applyLowPassFilter(LowPassFilter* filter, float input) 
{
    // 计算当前的输出
    float output = filter->alpha * input + (1 - filter->alpha) * filter->prev_y;
    
    // 更新前一个输出
    filter->prev_y = output;
    
    return output;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

KalmanFilter xDisKalmanFilter;
KalmanFilter yDisKalmanFilter;

KalmanFilter xImuDisKalmanFilter;
KalmanFilter yImuDisKalmanFilter;

KalmanFilter xScanDisKalmanFilter;
KalmanFilter yScanDisKalmanFilter;

// 初始化滤波器
void KalmanFilter_Init(KalmanFilter* kf, double process_noise, double sensor_noise, double estimated_error, double initial_value)
{
    kf->Q = process_noise;
    kf->R = sensor_noise;
    kf->P = estimated_error;
    kf->X = initial_value;
}

// 预测（可选）
void KalmanFilter_Predict(KalmanFilter* kf)
{
    kf->P = kf->P + kf->Q;  // 预测误差增加
}

// 更新
double KalmanFilter_Update(KalmanFilter* kf, double measurement)
{
    kf->K = kf->P / (kf->P + kf->R);  // 计算卡尔曼增益
    kf->X = kf->X + kf->K * (measurement - kf->X);  // 结合测量值更新
    kf->P = (1 - kf->K) * kf->P + kf->Q;  // 更新误差协方差（优化版）
    return kf->X;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////


FeedforwardController xFeedforward = {0};
FeedforwardController x_sFeedforward = {0};

FeedforwardController yFeedforward = {0};
FeedforwardController y_sFeedforward = {0};

void feedforward_init(FeedforwardController* controller,float a,float b)
{
    controller->prev_result = 0.0f;
    controller->Per_velocity = 0.0f;
    controller->Last_velocity = 0.0f;
    controller->a = a;
    controller->b = b;
    controller->result = 0.0f;
    controller->filtered_result = 0.0f;
}

// 前馈控制器函数
float feedforward_controller(FeedforwardController* controller, float current_rin) {
    float alpha = 0.8;  // 低通滤波系数（0~1，越接近1，响应越快）

    // 计算前馈控制信号
    controller->result = controller->a * (current_rin - controller->Last_velocity) + 
                   controller->b * (current_rin - 2.0f * controller->Last_velocity + controller->Per_velocity);

    // 低通滤波
    controller->filtered_result = alpha * controller->result + (1.0f - alpha) * controller->prev_result;  // 结合过去的结果
    controller->prev_result = controller->filtered_result;  // 存储滤波后的前馈信号

    // 更新历史数据
    controller->Per_velocity = controller->Last_velocity;
    controller->Last_velocity = current_rin;

    return controller->filtered_result;
}