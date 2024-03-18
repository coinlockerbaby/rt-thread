/**
  ******************************************************************************
  * @file           : Motor.c
  * @author         : flose
  * @brief          : None
  * @attention      : None
  * @date           : 2024/3/14
  ******************************************************************************
  */

#include "board.h"
#include "Motor.h"

#define LOG_TAG     "MOTOR"
#include "ulog.h"

/* 定义每个电机使用的定时器*/
#define MOTOR1_TIM_DEV_NAME   "timer2"     /* 定时器名称 */
#define MOTOR2_TIM_DEV_NAME   "timer4"     /* 定时器名称 */
#define MOTOR3_TIM_DEV_NAME   "timer5"     /* 定时器名称 */
#define MOTOR4_TIM_DEV_NAME   "timer6"     /* 定时器名称 */

/* 实例化电机*/
MotorStr motor1;
MotorStr motor2;
MotorStr motor3;
MotorStr motor4;

void __Motor_Init(MotorStr* motor)
{
    /* 清除结构体内存区域的数据*/
    motor1.MotorID = 1;
    motor2.MotorID = 2;
    motor3.MotorID = 3;
    motor4.MotorID = 4;

    Motor_GPIO_Init(motor->MotorID);
    Motor_TIMx_Init(motor);
    Initial_Motor_param(motor);
}

static void Motor_GPIO_Init(rt_uint8_t MotorID)
{
    switch(MotorID)
    {
        case 1:
            // motor1 IO初始化
            motor1.MOTOR_PUL_PIN = GET_PIN(C, 6);
            motor1.MOTOR_DIR_PIN = GET_PIN(G, 9);
            motor1.MOTOR_EN_PIN  = GET_PIN(G, 10);
            /* IO初始化*/
            rt_pin_mode(motor1.MOTOR_PUL_PIN, PIN_MODE_OUTPUT);
            rt_pin_mode(motor1.MOTOR_DIR_PIN, PIN_MODE_OUTPUT);
            rt_pin_mode(motor1.MOTOR_EN_PIN,  PIN_MODE_OUTPUT);
            rt_pin_write(motor1.MOTOR_EN_PIN, PIN_LOW); /* 锁定电机*/
            rt_pin_write(motor1.MOTOR_PUL_PIN,PIN_HIGH); /* 脉冲常态为高电平*/
            break;
        case 2:
            // motor2 IO初始化
            motor2.MOTOR_PUL_PIN = GET_PIN(C, 7);
            motor2.MOTOR_DIR_PIN = GET_PIN(D, 7);
            motor2.MOTOR_EN_PIN =  GET_PIN(D, 6);
            /* IO初始化*/
            rt_pin_mode(motor2.MOTOR_PUL_PIN, PIN_MODE_OUTPUT);
            rt_pin_mode(motor2.MOTOR_DIR_PIN, PIN_MODE_OUTPUT);
            rt_pin_mode(motor2.MOTOR_EN_PIN,  PIN_MODE_OUTPUT);
            rt_pin_write(motor2.MOTOR_EN_PIN, PIN_LOW); /* 锁定电机*/
            rt_pin_write(motor2.MOTOR_PUL_PIN,PIN_HIGH); /* 脉冲常态为高电平*/
            break;
        case 3:
            // motor3 IO初始化
            motor3.MOTOR_PUL_PIN = GET_PIN(B, 3);
            motor3.MOTOR_DIR_PIN = GET_PIN(B, 3);
            motor3.MOTOR_EN_PIN =  GET_PIN(B, 3);
            /* IO初始化*/
            rt_pin_mode(motor3.MOTOR_PUL_PIN, PIN_MODE_OUTPUT);
            rt_pin_mode(motor3.MOTOR_DIR_PIN, PIN_MODE_OUTPUT);
            rt_pin_mode(motor3.MOTOR_EN_PIN,  PIN_MODE_OUTPUT);
            rt_pin_write(motor3.MOTOR_EN_PIN, PIN_LOW); /* 锁定电机*/
            rt_pin_write(motor3.MOTOR_PUL_PIN,PIN_HIGH); /* 脉冲常态为高电平*/
            break;
        case 4:
            // motor4 IO初始化
            motor4.MOTOR_PUL_PIN = GET_PIN(B, 3);
            motor4.MOTOR_DIR_PIN = GET_PIN(B, 3);
            motor4.MOTOR_EN_PIN =  GET_PIN(B, 3);
            /* IO初始化*/
            rt_pin_mode(motor4.MOTOR_PUL_PIN, PIN_MODE_OUTPUT);
            rt_pin_mode(motor4.MOTOR_DIR_PIN, PIN_MODE_OUTPUT);
            rt_pin_mode(motor4.MOTOR_EN_PIN,  PIN_MODE_OUTPUT);
            rt_pin_write(motor4.MOTOR_EN_PIN, PIN_LOW); /* 锁定电机*/
            rt_pin_write(motor4.MOTOR_PUL_PIN,PIN_HIGH); /* 脉冲常态为高电平*/
            break;
        default:
            LOG_E("error MotorID");
            break;
    }
}

static void Motor_TIMx_Init(MotorStr *motor)
{
    rt_err_t ret = RT_EOK;
    switch(motor->MotorID)
    {
        case 1:
            motor->TIMx = MOTOR1_TIM_DEV_NAME;
            break;
        case 2:
            motor->TIMx = MOTOR2_TIM_DEV_NAME;
            break;
        case 3:
            motor->TIMx = MOTOR3_TIM_DEV_NAME;
            break;
        case 4:
            motor->TIMx = MOTOR4_TIM_DEV_NAME;
            break;
        default:
            LOG_E("Motor_TIMx_Init:ERROR MotorID not found");
    }
    motor->timer_dev = rt_device_find(motor->TIMx);
    rt_hwtimer_mode_t mode;
    rt_uint32_t freq = 1000000;               /* 计数频率 */
    if (motor->timer_dev == RT_NULL)
    {
        rt_kprintf("hwtimer sample run failed! can't find %s device!\n", motor->TIMx);
    }
    /* 以读写方式打开设备 */
    ret = rt_device_open(motor->timer_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", motor->TIMx);
    }
    /* 设置超时回调函数 */
    switch(motor->MotorID)
    {
        case 1:
            rt_device_set_rx_indicate(motor->timer_dev, MOTOR1_IRQHandler);
            break;
        case 2:
            rt_device_set_rx_indicate(motor->timer_dev, MOTOR2_IRQHandler);
            break;
        case 3:
            rt_device_set_rx_indicate(motor->timer_dev, MOTOR3_IRQHandler);
            break;
        case 4:
            rt_device_set_rx_indicate(motor->timer_dev, MOTOR4_IRQHandler);
            break;
        default:
            LOG_E("Motor_TIMx_Init: ERROR MotorID not found");
            break;
    }
    /* 设置计数频率(若未设置该项，默认为1Mhz 或 支持的最小计数频率) */
    rt_device_control(motor->timer_dev, HWTIMER_CTRL_FREQ_SET, &freq);
    mode = HWTIMER_MODE_PERIOD;
    ret = rt_device_control(motor->timer_dev, HWTIMER_CTRL_MODE_SET, &mode);
    if (ret != RT_EOK)
    {
        rt_kprintf("set mode failed! ret is :%d\n", ret);
    }

    /* 填充定时器并开启定时器计数*/
//    if (rt_device_write(motor->timer_dev, 0, &motor->timeout_s, sizeof(motor->timeout_s)) != sizeof(motor->timeout_s))
//    {
//        rt_kprintf("set timeout value failed\n");
//    }
    /* 读取定时器当前值 如果这个打印出来代表定时器开启成功 */
    rt_device_read(motor->timer_dev, 0, &motor->timeout_s, sizeof(motor->timeout_s));
    LOG_W("Motor%d Read: Sec = %d, Usec = %d", motor->MotorID, motor->timeout_s.sec, motor->timeout_s.usec);
    motor->timeout_s.sec = 0;
    motor->timeout_s.usec = 80; /* 推荐范围为5-15*/
}

void Initial_Motor_param(MotorStr *motor)
{
    rt_pin_write(motor->MOTOR_EN_PIN, PIN_HIGH); // 使能电机
    motor->clockwise = MOTOR_CW;    // 顺时针对应值
    motor->divnum = 8;              // 细分 200pulse/R对应1细分
    //motor1.MaxPosition=10000;   //最大位置,按照细分前的步数统计
    //motor1.MaxPosition_Pulse=(10000*1);
}

void box_moverel(MotorStr *motor, rt_int32_t step, rt_uint32_t accel, rt_uint32_t speed)
{
    if(step < 0) // 步数为负数
    {
        motor->dir=MOTOR_CCW;      // 逆时针方向旋转
        rt_pin_write(motor->MOTOR_DIR_PIN, MOTOR_CCW);
        step =-step;   // 获取步数绝对值
    }
    else if(step  == 0)
    {
        return;
    }
    else
    {
        motor->dir=MOTOR_CW; // 顺时针方向旋转
        rt_pin_write(motor->MOTOR_DIR_PIN, MOTOR_CW);
    }

    motor->running=MOTOR_RUNNING;    //转动完成标志
    motor->speedenbale=0;  //关闭速度控制
    motor->step_move=step*(motor->divnum);   //移动的步数
    motor->step_middle=motor->step_move>>1;
    motor->step_spmax=speed;   //最大速度
    motor->step_accel=accel;   //加速度
    motor->step_state=MOTOR_ACCELERATING;
    motor->step_frac=0;
    motor->speed_frac=0;
    motor->step_acced=0;
    motor->step_speed=0;
    motor->step_count=0;
    /* 打开定时器*/
    if (rt_device_write(motor->timer_dev, 0, &motor->timeout_s, sizeof(motor->timeout_s)) != sizeof(motor->timeout_s))
    {
        rt_kprintf("set timeout value failed\n");
    }
    LOG_W("Motor%d start", motor->MotorID);
}

/* 电机SPTA算法公共处理函数*/
void TIM_IRQHandler_SPTA(MotorStr *motor)
{
    int carry=0;
    rt_err_t ret = RT_EOK;
    /*拉低脉冲信号*/
    rt_pin_write(motor->MOTOR_PUL_PIN, PIN_LOW);

    /*根据速度累加器是否溢出，决定是否产生一个步进脉冲*/
    motor->step_frac += motor->step_speed;
    carry = motor->step_frac >> 16;
    motor->step_frac -= carry << 16;
    //carry=1说明溢出
    if(carry!=0)
    {
        //溢出则产生一个脉冲
        motor->step_count+=1;
        /*拉高脉冲信号产生一个步进脉冲*/
        rt_pin_write(motor->MOTOR_PUL_PIN, PIN_HIGH);

        //产生脉冲，则需要位置计算，根据方向调整位置
        if(motor->clockwise==motor->dir)
        {
            motor->CurrentPosition_Pulse++;
            if(motor->CurrentPosition_Pulse>=motor->MaxPosition_Pulse)
            {
                motor->CurrentPosition_Pulse=0;
            }
        }
        else
        {
            motor->CurrentPosition_Pulse--;
            if(motor->CurrentPosition_Pulse==0xffffffff)
            {
                motor->CurrentPosition_Pulse=motor->MaxPosition_Pulse-1;
            }
        }
        motor->CurrentPosition=motor->CurrentPosition_Pulse/motor->divnum;
    }

    //速度控制-已关闭
    if(motor->speedenbale)
    {
        if( (motor->step_speed>=motor->step_spmax&&motor->step_speed-motor->step_spmax<=3)||
            (motor->step_speed<=motor->step_spmax&&motor->step_spmax-motor->step_speed<=3))
        {
            return;
        }
    }

    /*根据电机的状态进行状态转换以及参数变换*/
    switch(motor->step_state)
    {
        /* 加速 */
        case MOTOR_ACCELERATING:
            //计算加速阶段的步数
            if(carry)
            {
                motor->step_acced++;
            }
            //加速阶段，速度累加器要根据设定的加速度值进行累加
            motor->speed_frac+=motor->step_accel;
            //溢出判断
            carry=motor->speed_frac>>17;
            motor->speed_frac-=carry<<17;
            if(carry)
            {
                //如果速度累加器溢出，则步数速度器需要加上该值，以便推动
                //步数速度器的增长，为步数累加器溢出准备
                motor->step_speed+=carry;
            }
            if(!motor->speedenbale)
            {
                //到达了中值就要反转为减速
                if(motor->step_middle!=0)
                {
                    if(motor->step_count==motor->step_middle)
                    {
                        motor->step_state=MOTOR_DECELERATING;
                    }
                }
                else if(motor->step_count>0)
                {
                    motor->step_state=MOTOR_DECELERATING;
                }
            }

            if(motor->step_speed>=motor->step_spmax)
            {
                //加速阶段到达一定的时刻，就达到了设定的最大速度
                motor->step_speed=motor->step_spmax;
                //转为最大速度状态
                motor->step_state=MOTOR_AT_MAX;
            }
            break;

            /* 达到最大速度 */
        case MOTOR_AT_MAX:
            if((motor->step_move - motor->step_count) <= motor->step_acced)
            {
                motor->step_state=MOTOR_DECELERATING;
            }
            break;

            /* 减速 */
        case MOTOR_DECELERATING:
            //减速阶段与加速阶段是相反的过程，原理也相同
            if(carry&&motor->step_acced>0)
            {
                motor->step_acced--;
            }
            motor->speed_frac+=motor->step_accel;
            carry=motor->speed_frac>>17;
            motor->speed_frac-=carry<<17;

            if(carry&&motor->step_speed>carry)
            {
                motor->step_speed-=carry;
            }
            if(!motor->speedenbale)
            {
                //在运行设定步数以后停止运行
                if(motor->step_count>=motor->step_move)
                {
                    motor->step_state=MOTOR_IDLE;
                }
            }
            break;

            /* 停止 */
        case MOTOR_IDLE:
            ret = rt_device_control(motor->timer_dev, HWTIMER_CTRL_STOP, RT_NULL);
            if (ret != RT_EOK)
            {
                rt_kprintf("set mode failed! ret is :%d\n", ret);
            }
            motor->running=MOTOR_STOP;
            motor->step_spmax=0;
    }
}
static rt_err_t MOTOR1_IRQHandler(rt_device_t dev, rt_size_t size) {
    /*
    请不要在此处加入多余的代码,那怕是if判断语句!!!
    此处只打开了一种中断类型,所以无需判断!!!
    加入任何多余的代码,都会影响电机运动的实时性
    */
        TIM_IRQHandler_SPTA(&motor1);
}

static rt_err_t MOTOR2_IRQHandler(rt_device_t dev, rt_size_t size) {
    /*
    请不要在此处加入多余的代码,那怕是if判断语句!!!
    此处只打开了一种中断类型,所以无需判断!!!
    加入任何多余的代码,都会影响电机运动的实时性
    */
    TIM_IRQHandler_SPTA(&motor2);
}
static rt_err_t MOTOR3_IRQHandler(rt_device_t dev, rt_size_t size) {
    /*
    请不要在此处加入多余的代码,那怕是if判断语句!!!
    此处只打开了一种中断类型,所以无需判断!!!
    加入任何多余的代码,都会影响电机运动的实时性
    */
    TIM_IRQHandler_SPTA(&motor3);
}
static rt_err_t MOTOR4_IRQHandler(rt_device_t dev, rt_size_t size) {
    /*
    请不要在此处加入多余的代码,那怕是if判断语句!!!
    此处只打开了一种中断类型,所以无需判断!!!
    加入任何多余的代码,都会影响电机运动的实时性
    */
    TIM_IRQHandler_SPTA(&motor4);
}