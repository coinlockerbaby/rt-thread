/**
  ******************************************************************************
  * @file           : motor_test.c
  * @author         : flose
  * @brief          : None
  * @attention      : None
  * @date           : 2024/3/13
  ******************************************************************************
  */

#include "rtthread.h"
#include "rtdevice.h"
#include "Motor.h"

static int motor_test(int argc, char *argv[])
{
    rt_memset(&motor1, 0, sizeof(motor1));
    rt_memset(&motor2, 0, sizeof(motor2));
    rt_memset(&motor3, 0, sizeof(motor3));
    rt_memset(&motor4, 0, sizeof(motor4));

    __Motor_Init(&motor1);
    __Motor_Init(&motor2);
    box_moverel(&motor1 ,20000, 4500, 30000);
    box_moverel(&motor2, 20000, 4500, 30000);

}

/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(motor_test, motor_test);