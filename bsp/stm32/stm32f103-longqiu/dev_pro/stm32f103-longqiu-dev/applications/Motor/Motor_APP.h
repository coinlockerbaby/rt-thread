/**
  ******************************************************************************
  * @file           : Motor_APP.h
  * @author         : flose
  * @brief          : None
  * @attention      : None
  * @date           : 2024/3/15
  ******************************************************************************
  */

#ifndef RTTHREAD_MOTOR_APP_H
#define RTTHREAD_MOTOR_APP_H

typedef enum
{
    Forward,
    Left,
    Right,
    Back,
}MotorState;

int Motor_Init();
void Motor_Run(MotorState motorstate, rt_uint16_t cir);

#endif //RTTHREAD_MOTOR_APP_H
