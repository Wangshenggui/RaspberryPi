#ifndef __PID_H
#define __PID_H

#include <ros/ros.h>

/******************************************************************************************/
/* PID相关参数 */

#define  INCR_LOCT_SELECT  0         /* 0：位置式 ，1：增量式 */

#if INCR_LOCT_SELECT


/* 增量式PID参数相关宏 */
#define  KP      0.00f               /* P参数*/
#define  KI      0.00f               /* I参数*/
#define  KD      0.00f               /* D参数*/

#else

/* 位置式PID参数相关宏 */
#define  LocationKP      0.0f               /* P参数*/
#define  LocationKI      0.0f               /* I参数*/
#define  LocationKD      0.0f                /* D参数*/

#define  SpeedKP      0.0f               /* P参数*/
#define  SpeedKI      0.0f               /* I参数*/
#define  SpeedKD      0.0f                /* D参数*/

#endif

/* PID参数结构体 */
typedef struct
{
    volatile float  SetPoint;            /* 设定目标 */
    volatile float  ActualValue;         /* 期望输出值 */
    volatile float  SumError;            /* 误差累计 */
    volatile float  Proportion;          /* 比例常数 P */
    volatile float  Integral;            /* 积分常数 I */
    volatile float  Derivative;          /* 微分常数 D */
    volatile float  Error;               /* Error[1] */
    volatile float  LastError;           /* Error[-1] */
    volatile float  PrevError;           /* Error[-2] */
} PID_TypeDef;

extern PID_TypeDef xDisplacePID;
extern PID_TypeDef xSpeedPID;

extern PID_TypeDef yDisplacePID;
extern PID_TypeDef ySpeedPID;

extern PID_TypeDef HeightPID;
/******************************************************************************************/

void pid_init(PID_TypeDef *PID);                 /* pid初始化 */
void pid_clear(PID_TypeDef *PID);
int32_t pid_ctrl(PID_TypeDef *PID, float Feedback_value, int PID_Mode);      /* pid闭环控制 */

#endif
