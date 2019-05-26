#include <rtthread.h>
#include <board.h>

#ifdef RT_USING_LWIP
#include "stm32_eth.h"
#endif /* RT_USING_LWIP */

#ifdef RT_USING_SPI
#include "rt_spi.h"

#if defined(RT_USING_DFS) && defined(RT_USING_SPI_MSD)
#include "spi_msd.h"
#endif /* RT_USING_DFS */

/*
 * SPI1_MOSI: PA7
 * SPI1_MISO: PA6
 * SPI1_SCK : PA5
 *
 * CS0: PA4  SD card.
*/

/* SPI bus GPIO define. */
#define SPI1_GPIO_SCK       GPIO_Pin_5
#define SPI1_SCK_PIN_SOURCE GPIO_PinSource5
#define SPI1_GPIO_MISO       GPIO_Pin_6
#define SPI1_MISO_PIN_SOURCE GPIO_PinSource6
#define SPI1_GPIO_MOSI       GPIO_Pin_7
#define SPI1_MOSI_PIN_SOURCE GPIO_PinSource7
#define SPI1_GPIO          GPIOA
#define SPI1_GPIO_RCC      RCC_AHB1Periph_GPIOA
#define RCC_APBPeriph_SPI1 RCC_APB2Periph_SPI1

#define SPI2_GPIO_SCK       GPIO_Pin_13
#define SPI2_SCK_PIN_SOURCE GPIO_PinSource13
#define SPI2_GPIO_MISO       GPIO_Pin_14
#define SPI2_MISO_PIN_SOURCE GPIO_PinSource14
#define SPI2_GPIO_MOSI       GPIO_Pin_15
#define SPI2_MOSI_PIN_SOURCE GPIO_PinSource15
#define SPI2_GPIO          GPIOB
#define SPI2_GPIO_RCC      RCC_AHB1Periph_GPIOB
#define RCC_APBPeriph_SPI2 RCC_APB1Periph_SPI2

/* SPI device GPIO define */
#define MOUSE1_GPIO_CS GPIO_Pin_15
#define MOUSE1_GPIO GPIOG
#define MOUSE1_GPIO_RCC RCC_AHB1Periph_GPIOG

#define MOUSE2_GPIO_CS GPIO_Pin_9
#define MOUSE2_GPIO GPIOF
#define MOUSE2_GPIO_RCC RCC_AHB1Periph_GPIOF


static void rt_hw_spi_init(void)
{
#ifdef RT_USING_SPI1
    /* register spi bus */
    {
        static struct stm32_spi_bus stm32_spi;
        GPIO_InitTypeDef GPIO_InitStructure;

        /* Enable GPIO clock */
        RCC_AHB1PeriphClockCmd(SPI1_GPIO_RCC,
        ENABLE);

        GPIO_InitStructure.GPIO_Pin   = SPI1_GPIO_SCK | SPI1_GPIO_MISO | SPI1_GPIO_MOSI;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

        GPIO_PinAFConfig(SPI1_GPIO,SPI1_SCK_PIN_SOURCE, GPIO_AF_SPI1);
        GPIO_PinAFConfig(SPI1_GPIO,SPI1_MISO_PIN_SOURCE, GPIO_AF_SPI1);
        GPIO_PinAFConfig(SPI1_GPIO,SPI1_MOSI_PIN_SOURCE, GPIO_AF_SPI1);

        GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);

        stm32_spi_register(SPI1, &stm32_spi, "spi1");
    }
    /* attach spi device for mouse 1*/
    {
        static struct rt_spi_device spi_device;
        static struct stm32_spi_cs  spi_cs;

        GPIO_InitTypeDef GPIO_InitStructure;

        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

        /* spi10: PA4 */
        spi_cs.GPIOx = MOUSE1_GPIO;
        spi_cs.GPIO_Pin = MOUSE1_GPIO_CS;
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

        GPIO_InitStructure.GPIO_Pin = spi_cs.GPIO_Pin;
        GPIO_SetBits(spi_cs.GPIOx, spi_cs.GPIO_Pin);
        GPIO_Init(spi_cs.GPIOx, &GPIO_InitStructure);

        rt_spi_bus_attach_device(&spi_device, "mouse1", "spi1", (void*)&spi_cs);
    }
    /* attach spi device for mouse 2*/
    {
        static struct rt_spi_device spi_device;
        static struct stm32_spi_cs  spi_cs;

        GPIO_InitTypeDef GPIO_InitStructure;

        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

        /* spi10: PA4 */
        spi_cs.GPIOx = MOUSE2_GPIO;
        spi_cs.GPIO_Pin = MOUSE2_GPIO_CS;
        RCC_AHB1PeriphClockCmd(MOUSE2_GPIO_RCC, ENABLE);

        GPIO_InitStructure.GPIO_Pin = spi_cs.GPIO_Pin;
        GPIO_SetBits(spi_cs.GPIOx, spi_cs.GPIO_Pin);
        GPIO_Init(spi_cs.GPIOx, &GPIO_InitStructure);

        rt_spi_bus_attach_device(&spi_device, "mouse2", "spi1", (void*)&spi_cs);
    }

#endif /* RT_USING_SPI1 */

#ifdef RT_USING_SPI2
    /* register spi bus */
    {
        static struct stm32_spi_bus stm32_spi;
        GPIO_InitTypeDef GPIO_InitStructure;

        /* Enable GPIO clock */
        RCC_AHB1PeriphClockCmd(SPI2_GPIO_RCC,
        ENABLE);

        GPIO_InitStructure.GPIO_Pin   = SPI2_GPIO_SCK | SPI2_GPIO_MISO | SPI2_GPIO_MOSI;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

        GPIO_PinAFConfig(SPI2_GPIO,SPI2_SCK_PIN_SOURCE, GPIO_AF_SPI2);
        GPIO_PinAFConfig(SPI2_GPIO,SPI2_MISO_PIN_SOURCE, GPIO_AF_SPI2);
        GPIO_PinAFConfig(SPI2_GPIO,SPI2_MOSI_PIN_SOURCE, GPIO_AF_SPI2);

        GPIO_Init(SPI2_GPIO, &GPIO_InitStructure);

        stm32_spi_register(SPI2, &stm32_spi, "spi2");
    }
#endif /* RT_USING_SPI2 */

}
#endif /* RT_USING_SPI */


int rt_platform_init(void)
{
#ifdef RT_USING_SPI
    rt_hw_spi_init();

#if defined(RT_USING_DFS) && defined(RT_USING_SPI_MSD)
    /* init sdcard driver */
    {
        extern void rt_hw_msd_init(void);
        GPIO_InitTypeDef  GPIO_InitStructure;

        /* PC4 : SD Power */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOC, &GPIO_InitStructure);

        /* SD card power on. */
        GPIO_ResetBits(GPIOC, GPIO_Pin_4);
        rt_thread_delay(2);

        msd_init("sd0", "spi10");
    }
#endif /* RT_USING_DFS && RT_USING_SPI_MSD */

#endif // RT_USING_SPI

#ifdef RT_USING_LWIP
    /* initialize eth interface */
    rt_hw_stm32_eth_init();
#endif /* RT_USING_LWIP */
    return 0;
}
INIT_BOARD_EXPORT(rt_platform_init)
