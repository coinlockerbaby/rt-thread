/**
  ******************************************************************************
  * @file           : thread_sample.c
  * @author         : flose
  * @brief          : None
  * @attention      : None
  * @date           : 2024/3/9
  ******************************************************************************
  */

/*
 * 程序清单：创建/删除、初始化线程
 *
 * 这个例子会创建两个线程，一个动态线程，一个静态线程。
 * 一个线程在运行完毕后自动被系统删除，另一个线程一直打印计数。
 */

#define LOG_TAG "thread_sample"
#include <rtthread.h>
#include <ulog.h>

#define THREAD_PRIORITY 25
#define THREAD_STACK_SIZE 512
#define THREAD_TIMESLICE 5

static rt_thread_t tid1 = RT_NULL;

static void thread1_entry(void *parameter)
{
    rt_uint16_t count = 1;
    while(count++){
        LOG_I("thread1 count: %d", count);
        rt_thread_delay(500);
    }
}

rt_align(RT_ALIGN_SIZE)

static rt_uint8_t thread2_stack[1024];
static struct rt_thread thread2;

static void thread2_entry(void *parameter)
{
    rt_uint16_t count = 1;
    while(count++){
        LOG_I("thread2 count: %d", count);
        rt_thread_delay(500);
    }
}

int thread_sample(void)
{
    /* 动态创建线程*/
    tid1 = rt_thread_create("thread1",
                     thread1_entry, RT_NULL,
                     THREAD_STACK_SIZE,
                     THREAD_PRIORITY-1, THREAD_TIMESLICE);

    if (tid1 != RT_NULL){
        rt_thread_startup(tid1);
    }
    /* 静态创建线程*/
    rt_thread_init(&thread2,
                   "thread2",
                   thread2_entry,
                   RT_NULL,
                   &thread2_stack[0],
                   sizeof(thread2_stack),
                   THREAD_PRIORITY,
                   THREAD_TIMESLICE);
    rt_thread_startup(&thread2);
}

MSH_CMD_EXPORT(thread_sample, thread_sample);