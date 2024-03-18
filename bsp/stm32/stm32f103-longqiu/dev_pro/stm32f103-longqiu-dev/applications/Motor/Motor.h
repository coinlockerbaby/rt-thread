/**
  ******************************************************************************
  * @file           : motor.h
  * @author         : flose
  * @brief          : None
  * @attention      : None
  * @date           : 2024/3/14
  ******************************************************************************
  */

#ifndef RTTHREAD_MOTOR_H
#define RTTHREAD_MOTOR_H
#include <rtthread.h>
#include <rtdevice.h>

typedef __packed struct
{
    rt_uint8_t MotorID;
    /* 电机控制相关*/
    rt_uint8_t dir;     /* 方向*/
    rt_uint8_t running;		  	//转动完成标志
    rt_uint8_t divnum;		  	//分频数
    rt_uint8_t speedenbale;		//是否使能速度控制
    rt_uint8_t clockwise;		//顺时针方向对应的值

    uint32_t step_move;				//总共需要移动的步数
    uint32_t step_spmax;			//移动过程中的最大速度
    uint32_t step_accel;			//加速度/减速度
    uint32_t step_acced;			//加速阶段的加速步数

    rt_uint32_t step_middle;				//移动的中点, = (step_move - 1) >> 1
    rt_uint32_t step_count;				//已经运行的步数
    rt_uint32_t step_frac;				    //步数计数器分数,
    rt_uint32_t step_speed;				//当前速度, 16.8 bit format (HI byte always 0)
    rt_uint32_t speed_frac;				//速度片段累加器，每次都累加step_accel，该数值大于某个值后step_speed增加
    rt_uint8_t step_state;					//运动状态

    uint32_t CurrentPosition;			//当前位置,按照细分前的步数统计
    uint32_t MaxPosition;				//最大位置,超过该位置置0,按照细分前的步数统计
    uint32_t CurrentPosition_Pulse;		//当前位置
    uint32_t MaxPosition_Pulse;			//最大位置

    /* 硬件使用资源*/
    rt_base_t MOTOR_DIR_PIN;
    rt_base_t MOTOR_EN_PIN;
    rt_base_t MOTOR_PUL_PIN;

    const char* TIMx; /* 定时器*/
    rt_hwtimerval_t  timeout_s; /* 定时器超时值*/
    rt_device_t timer_dev;      /* 定时器设备句柄*/

}MotorStr;

extern MotorStr motor1;
extern MotorStr motor2;
extern MotorStr motor3;
extern MotorStr motor4;

//电机方向
#define MOTOR_CCW          0         //右
#define MOTOR_CW           1         //左

//运动状态
#define MOTOR_IDLE				0
#define MOTOR_ACCELERATING		1
#define MOTOR_AT_MAX			2
#define MOTOR_DECELERATING		3
#define MOTOR_STOP              0
#define MOTOR_RUNNING           1

static void Motor_GPIO_Init(rt_uint8_t MotorID);
static void Motor_TIMx_Init(MotorStr *motor);

/* 中断处理函数*/
static rt_err_t MOTOR1_IRQHandler(rt_device_t dev, rt_size_t size);
static rt_err_t MOTOR2_IRQHandler(rt_device_t dev, rt_size_t size);
static rt_err_t MOTOR3_IRQHandler(rt_device_t dev, rt_size_t size);
static rt_err_t MOTOR4_IRQHandler(rt_device_t dev, rt_size_t size);

/* 初始化电机*/
void __Motor_Init(MotorStr* motor);
void Initial_Motor_param(MotorStr *motor);

/* 相对移动位置*/
void box_moverel(MotorStr *motor, rt_int32_t step, rt_uint32_t accel, rt_uint32_t speed);
#endif //RTTHREAD_MOTOR_H
