/**
  ******************************************************************************
  * @file           : Motor_APP.c
  * @author         : flose
  * @brief          : 电机控制应用层
  * @attention      : None
  * @date           : 2024/3/15
  ******************************************************************************
  */

#include <rtthread.h>
#include <rtdevice.h>
#include "Motor_APP.h"
#include "Motor.h"

#define USER_ACCEL  400000
#define USER_MAXSPEED   30000

#define  ANGLE_PULSE    370

int Motor_Init()
{
    rt_memset(&motor1, 0, sizeof(motor1));
    rt_memset(&motor2, 0, sizeof(motor2));
    rt_memset(&motor3, 0, sizeof(motor3));
    rt_memset(&motor4, 0, sizeof(motor4));
    __Motor_Init(&motor1);
    __Motor_Init(&motor2);
    __Motor_Init(&motor3);
    __Motor_Init(&motor4);
}

void Motor_Run(MotorState motorstate, rt_uint16_t cir)
{
    switch(motorstate)
    {
        case Forward:
            box_moverel(&motor1, cir*200, USER_ACCEL, USER_MAXSPEED);
            box_moverel(&motor2, cir*200, USER_ACCEL, USER_MAXSPEED);
            box_moverel(&motor3, cir*200, USER_ACCEL, USER_MAXSPEED);
            box_moverel(&motor4, cir*200, USER_ACCEL, USER_MAXSPEED);
    }
}

#ifdef FINSH_USING_MSH
INIT_APP_EXPORT(Motor_Init);
#endif
