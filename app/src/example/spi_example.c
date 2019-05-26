#include <rtthread.h>
#include <rtdevice.h>

#include <rt_spi.h>

#ifdef RT_USING_SPI

void spi_example(void)
{
    static struct rt_spi_device spi_device;
    static struct stm32_spi_cs  spi_cs;

    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

    /* spi1_0: PA4 */
    spi_cs.GPIOx = GPIOA;
    spi_cs.GPIO_Pin = GPIO_Pin_4;
    RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = spi_cs.GPIO_Pin;
    GPIO_SetBits(spi_cs.GPIOx, spi_cs.GPIO_Pin);
    GPIO_Init(spi_cs.GPIOx, &GPIO_InitStructure);

    rt_spi_bus_attach_device(&spi_device, "spi1_0", "spi1", (void*)&spi_cs);
}

void spi1_test(void)
{
    struct rt_spi_device * spi = (struct rt_spi_device *)rt_device_find("spi1_0");
    rt_uint8_t test[4] = {1,2,3,4};
    rt_device_write(&spi->parent, 0, test, 4);
}
MSH_CMD_EXPORT(spi1_test, spi1 send test data)

#endif /* end of RT_USING_SPI */