/**
  ******************************************************************************
  * @file           : Motor_thread.c
  * @author         : flose
  * @brief          : None
  * @attention      : 电机测试程序 IO延时翻转实现
  * @date           : 2024/3/10
  ******************************************************************************
  */
#include "rt-thread/include/rtthread.h"
#include "rtdevice.h"
#include "board.h"
#include "stdlib.h"

#define PULSE_PER_R     1600

#define MOTOR1_EN_PIN     GET_PIN(G, 10)
#define MOTOR1_DIR_PIN    GET_PIN(G, 9)
#define MOTOR1_PULSE_PIN  GET_PIN(C, 6)

#define THREAD_PRIORITY      25
#define THREAD_STACK_SIZE    512
#define THREAD_TIMESLICE     5

enum MotorState
{
    Forward,
    Back,
};

void motor_dir(enum MotorState state, int i)
{
    switch(state)
    {
        case Forward: {
            rt_pin_write(MOTOR1_EN_PIN, PIN_HIGH);
            rt_pin_write(MOTOR1_DIR_PIN, PIN_HIGH);
                for (int x = 0; x <= (i/(360.0/PULSE_PER_R)) ; x++) { //一圈200个脉冲    两个pul为一个脉冲    16个脉冲就是28.8°距离30°是1.2°
                    rt_pin_write(MOTOR1_PULSE_PIN, PIN_HIGH);
                    rt_thread_delay(1);
                    rt_pin_write(MOTOR1_PULSE_PIN, PIN_LOW);
                    rt_thread_delay(1);
                }
        } break;
        case Back:{

        } break;
    }

}

int motor_Init(void)
{
    rt_pin_mode(MOTOR1_EN_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(MOTOR1_DIR_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(MOTOR1_PULSE_PIN, PIN_MODE_OUTPUT);
    /* 初始化完关掉使能锁定位置*/
    rt_pin_write(MOTOR1_EN_PIN, PIN_HIGH);
    rt_pin_write(MOTOR1_DIR_PIN, PIN_HIGH);
    rt_pin_write(MOTOR1_PULSE_PIN, PIN_HIGH);

}

void thread1_entry(void *parameter)
{
    motor_Init();
    int i = atoi(parameter);
    rt_kprintf("%d\n", i);
    motor_dir(Forward, i);
    rt_free(parameter);
}

int motor_thread(int argc, char **argv)
{
    rt_thread_t tid;
    unsigned char *input_str = rt_malloc(sizeof(char)*8);
    memset(input_str, 0, sizeof(input_str));
    strcpy(input_str, argv[1]);
    /* 创建线程1 */
    tid = rt_thread_create("thread1",
                           thread1_entry, input_str,
                           THREAD_STACK_SIZE,
                           THREAD_PRIORITY,
                           THREAD_TIMESLICE);
    if (tid != RT_NULL)
        rt_thread_startup(tid);
    return 0;
}
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(motor_thread, motor_thread);