#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define LOG_TAG "Flose"
#include <ulog.h>

#define LED0_PIN    GET_PIN(B, 12)

int main(void)
{
    rt_uint16_t count = 1;
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);

    /* output different level log by LOG_X API */
    LOG_D("Welcome To SuperMaket Robot 1st System!");
    LOG_I("Welcome To SuperMaket Robot 1st System!");
    LOG_W("Welcome To SuperMaket Robot 1st System!");
    LOG_E("Welcome To SuperMaket Robot 1st System!\n");

/* output Using bsp*/
#ifdef BSP_USING_GPIO
    LOG_W("BSP_USING_GPIO");
#endif
#ifdef BSP_USING_UART
    LOG_W("BSP_USING_UART");
#endif
#ifdef BSP_USING_KEY
    LOG_W("BSP_USING_KEY");
#endif

    while (count++)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }

    return RT_EOK;
}
