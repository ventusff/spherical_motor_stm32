#include <rtdevice.h>
#include <rtthread.h>

#include <finsh.h>

static rt_serial_t *serial;
static rt_uint8_t buffer[] = {0xAA, 0x55, 0x09, 0x02, 0x03, 0x00, 0x00, 0x00, 0xF7};
static rt_int32_t __delay_tick = DELAY_MS(100);

static rt_timer_t pTimer;
static struct rt_semaphore* pRes;

static void timeout_ind(void* parameter)
{
    rt_sem_release(pRes);
}

int example_usart_system_init(void)
{
    serial = (rt_serial_t *)rt_device_find("uart2");
    serial->config.baud_rate = 115200;
    serial->config.stop_bits = STOP_BITS_1;
    serial->config.data_bits = DATA_BITS_8;
    serial->config.parity = PARITY_NONE;
    serial->config.bufsz = 30;

    serial->ops->configure(serial, &(serial->config));
    rt_device_open(&serial->parent, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_DMA_RX);
    serial->ops->control(serial,RT_DEVICE_CTRL_SET_INT, (void*)RT_DEVICE_FLAG_DMA_RX);

    // no indicate function. we only send.

    pRes = rt_sem_create("timeout lock", 0, RT_IPC_FLAG_FIFO);

    pTimer = rt_timer_create("tSendVel",timeout_ind, RT_NULL, __delay_tick, RT_TIMER_FLAG_PERIODIC);
    rt_timer_control(pTimer, RT_TIMER_CTRL_SET_TIME, &__delay_tick);
    rt_timer_start(pTimer);
    return 0;
}
INIT_DEVICE_EXPORT(example_usart_system_init)

void example_usart_poll(void)
{
    if(rt_sem_take(pRes, RT_WAITING_FOREVER) == RT_EOK)
    {
        rt_device_write(&serial->parent, 0, buffer, sizeof(buffer));
    }
}

void set_send_freq(int argc, char **argv)
{
    if (argc != 2)
    {
        return;
    }
    int data;
    sscanf(argv[1],"%d",&data);
    if (data >= 0)
    {
        rt_enter_critical();
        if (data  == 0) {
            __delay_tick = -1;
            rt_kprintf("Stop sending. \n");
            rt_timer_stop(pTimer);
        }
        else
        {
            __delay_tick = DELAY_MS(1000.0/data);
            rt_kprintf("Setting frequency to %.1f .\n", 1000.0 / FROM_TICK_TO_MS(__delay_tick));
            rt_timer_control(pTimer, RT_TIMER_CTRL_SET_TIME, &__delay_tick);
            rt_timer_start(pTimer);
        }
        rt_exit_critical();
    }
}
MSH_CMD_EXPORT(set_send_freq, set send freq. 0 to stop.)