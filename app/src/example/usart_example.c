#include <rtdevice.h>
#include <rtthread.h>

#include <finsh.h>

static rt_serial_t *serial;
static rt_uint8_t buffer[] = {0xAA, 0x55, 0x09, 0x02, 0x03, 0x00, 0x00, 0x00, 0xF7};
static rt_tick_t __delay_tick = DELAY_MS(100);

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
    return 0;
}
INIT_DEVICE_EXPORT(example_usart_system_init)

void example_usart_poll(void)
{
    rt_device_write(&serial->parent, 0, buffer, sizeof(buffer));
    rt_thread_delay(__delay_tick);
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
        }
        else
        {
            __delay_tick = DELAY_MS(1000.0/data);
            rt_kprintf("Setting frequency to %.1f .\n", 1000.0 / FROM_TICK_TO_MS(__delay_tick));
        }
        rt_exit_critical();
    }
}
MSH_CMD_EXPORT(set_send_freq, set send freq. 0 to stop.)