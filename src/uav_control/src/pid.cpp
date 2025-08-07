#include "pid.h"

PID_TypeDef xDisplacePID={0};
PID_TypeDef xSpeedPID={0};

PID_TypeDef yDisplacePID={0};
PID_TypeDef ySpeedPID={0};

PID_TypeDef HeightPID = {0};


/**
 * @brief       pid初始化
 * @param       无
 * @retval      无
 */
void pid_init(PID_TypeDef *PID)
{
    PID->SetPoint = 0;       /* 设定目标值 */
    PID->ActualValue = 0.0;  /* 期望输出值 */
    PID->SumError = 0.0;     /* 积分值 */
    PID->Error = 0.0;        /* Error[1] */
    PID->LastError = 0.0;    /* Error[-1] */
    PID->PrevError = 0.0;    /* Error[-2] */
    if(PID==&xDisplacePID)
    {
      PID->Proportion = LocationKP;    /* 比例常数 Proportional Const */
      PID->Integral = LocationKI;      /* 积分常数 Integral Const */
      PID->Derivative = LocationKD;    /* 微分常数 Derivative Const */ 
    }
    else if(PID==&xSpeedPID)
    {
      PID->Proportion = SpeedKP;    /* 比例常数 Proportional Const */
      PID->Integral = SpeedKI;      /* 积分常数 Integral Const */
      PID->Derivative = SpeedKD;    /* 微分常数 Derivative Const */ 
    }
    if(PID==&yDisplacePID)
    {
      PID->Proportion = LocationKP;    /* 比例常数 Proportional Const */
      PID->Integral = LocationKI;      /* 积分常数 Integral Const */
      PID->Derivative = LocationKD;    /* 微分常数 Derivative Const */ 
    }
    else if(PID==&ySpeedPID)
    {
      PID->Proportion = SpeedKP;    /* 比例常数 Proportional Const */
      PID->Integral = SpeedKI;      /* 积分常数 Integral Const */
      PID->Derivative = SpeedKD;    /* 微分常数 Derivative Const */ 
    }
    else if(PID==&HeightPID)
    {
      PID->Proportion = 6.0;    /* 比例常数 Proportional Const */
      PID->Integral = 0.0;      /* 积分常数 Integral Const */
      PID->Derivative = 1.0;    /* 微分常数 Derivative Const */ 
    }
}

void pid_clear(PID_TypeDef *PID)
{
    PID->SetPoint = 0;       /* 设定目标值 */
    PID->ActualValue = 0.0;  /* 期望输出值 */
    PID->SumError = 0.0;     /* 积分值 */
    PID->Error = 0.0;        /* Error[1] */
    PID->LastError = 0.0;    /* Error[-1] */
    PID->PrevError = 0.0;    /* Error[-2] */
    PID->Proportion = 0;    /* 比例常数 Proportional Const */
    PID->Integral = 0;      /* 积分常数 Integral Const */
    PID->Derivative = 0;    /* 微分常数 Derivative Const */ 
}

/**
 * @brief       pid闭环控制
 * @param       *PID：PID结构体变量地址
 * @param       Feedback_value：当前实际值
 * @retval      期望输出值
 */
int32_t pid_ctrl(PID_TypeDef *PID, float Feedback_value, int PID_Mode)
{
    PID->Error = (float)(PID->SetPoint - Feedback_value);                   /* 计算偏差 */
    
    if (PID_Mode == 1)  /* 增量式PID */
    {
        PID->ActualValue += (PID->Proportion * (PID->Error - PID->LastError))                          /* 比例环节 */
                        + (PID->Integral * PID->Error)                                             /* 积分环节 */
                        + (PID->Derivative * (PID->Error - 2 * PID->LastError + PID->PrevError));  /* 微分环节 */
    
        PID->PrevError = PID->LastError;                                        /* 存储偏差，用于下次计算 */
        PID->LastError = PID->Error;
    }
    else  /* 位置式PID */
    {
        // 积分
        PID->SumError += PID->Error;
        // 高度积分限幅
        if(PID==&HeightPID)
        {
          int32_t INTEGRAL_LIMIT = 200;  // 定义积分限制
          // 积分限幅：防止积分项过大
          PID->SumError = (PID->SumError > INTEGRAL_LIMIT) ? INTEGRAL_LIMIT : PID->SumError;
          PID->SumError = (PID->SumError < -INTEGRAL_LIMIT) ? -INTEGRAL_LIMIT : PID->SumError;
        }
        else if(PID==&xDisplacePID)
        {
          int32_t INTEGRAL_LIMIT = 100;  // 定义积分限制
          // 积分限幅：防止积分项过大
          PID->SumError = (PID->SumError > INTEGRAL_LIMIT) ? INTEGRAL_LIMIT : PID->SumError;
          PID->SumError = (PID->SumError < -INTEGRAL_LIMIT) ? -INTEGRAL_LIMIT : PID->SumError;
        }
        else if(PID==&yDisplacePID)
        {
          int32_t INTEGRAL_LIMIT = 100;  // 定义积分限制
          // 积分限幅：防止积分项过大
          PID->SumError = (PID->SumError > INTEGRAL_LIMIT) ? INTEGRAL_LIMIT : PID->SumError;
          PID->SumError = (PID->SumError < -INTEGRAL_LIMIT) ? -INTEGRAL_LIMIT : PID->SumError;
        }

        PID->ActualValue = (PID->Proportion * PID->Error)                       /* 比例环节 */
                        + (PID->Integral * PID->SumError)                    /* 积分环节 */
                        + (PID->Derivative * (PID->Error - PID->LastError)); /* 微分环节 */
        PID->LastError = PID->Error;
    }
    
    return ((int32_t)(PID->ActualValue));                                   /* 返回计算后输出的数值 */
}

