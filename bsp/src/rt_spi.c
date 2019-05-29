/**
 * @file rt_spi.c
 * @author Jianfei (Ventus) Guo (ffventus@gmail.com)
 * @brief modified from https://github.com/RT-Thread/rt-thread/blob/stable-v3.0.x/bsp/stm32f107/drivers/rt_stm32f10x_spi.c
 * @version 0.1
 * @date 2019-05-25
 * 
 * @copyright Copyright (c) 2019
 * 
 */

/**
 * spi no.  rx                  tx
 * spi1     dma2_stream0        dma2_stream3
 * spi2     dma1_stream3        dma1_stream4
 * spi3     conflict_uart4/5    dma1_stream7    solution: NO.
 * */

// added by Ventus
#ifdef SPI3_USING_DMA
#error "No available DMA stream for SPI3_RX!"
#endif

#include <rt_spi.h>
#include <rtdevice.h>

static rt_err_t configure(struct rt_spi_device* device, struct rt_spi_configuration* configuration);
static rt_uint32_t xfer(struct rt_spi_device* device, struct rt_spi_message* message);

static struct rt_spi_ops stm32_spi_ops =
{
    configure,
    xfer
};

//------------------ DMA ------------------
#ifdef RT_SPI_USE_DMA
static uint8_t dummy = 0xFF;
#endif /*RT_SPI_USE_DMA*/

#ifdef RT_SPI_USE_DMA
static void DMA_Configuration(struct stm32_spi_bus * stm32_spi_bus, const void * send_addr, void * recv_addr, rt_size_t size)
{
    DMA_InitTypeDef DMA_InitStructure;

    if(!stm32_spi_bus->dma)
    {
         return;
    }

    /* -----------------------  stm32f107 version  ----------------------- */
    // DMA_ClearFlag(stm32_spi_bus->dma->priv_data->DMA_Channel_RX_FLAG_TC
    //               | stm32_spi_bus->dma->priv_data->DMA_Channel_RX_FLAG_TE
    //               | stm32_spi_bus->dma->priv_data->DMA_Channel_TX_FLAG_TC
    //               | stm32_spi_bus->dma->priv_data->DMA_Channel_TX_FLAG_TE);

    // /* RX channel configuration */
    // DMA_Cmd(stm32_spi_bus->dma->priv_data->DMA_Channel_RX, DISABLE);
    // DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(stm32_spi_bus->SPI->DR));
    // DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    // DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    // DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    // DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    // DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    // DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    // DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    // DMA_InitStructure.DMA_BufferSize = size;

    // if(recv_addr != RT_NULL)
    // {
    //     DMA_InitStructure.DMA_MemoryBaseAddr = (u32) recv_addr;
    //     DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    // }
    // else
    // {
    //     DMA_InitStructure.DMA_MemoryBaseAddr = (u32) (&dummy);
    //     DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    // }

    // DMA_Init(stm32_spi_bus->dma->priv_data->DMA_Channel_RX, &DMA_InitStructure);

    // DMA_ITConfig(stm32_spi_bus->dma->priv_data->DMA_Channel_RX, DMA_IT_TC, ENABLE);
    // DMA_Cmd(stm32_spi_bus->dma->priv_data->DMA_Channel_RX, ENABLE);

    // /* TX channel configuration */
    // DMA_Cmd(stm32_spi_bus->dma->priv_data->DMA_Channel_TX, DISABLE);
    // DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(stm32_spi_bus->SPI->DR));
    // DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    // DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    // DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    // DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    // DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    // DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    // DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    // DMA_InitStructure.DMA_BufferSize = size;

    // if(send_addr != RT_NULL)
    // {
    //     DMA_InitStructure.DMA_MemoryBaseAddr = (u32)send_addr;
    //     DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    // }
    // else
    // {
    //     DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(&dummy);;
    //     DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    // }

    // DMA_Init(stm32_spi_bus->dma->priv_data->DMA_Channel_TX, &DMA_InitStructure);

    // DMA_ITConfig(stm32_spi_bus->dma->priv_data->DMA_Channel_TX, DMA_IT_TC, ENABLE);
    // DMA_Cmd(stm32_spi_bus->dma->priv_data->DMA_Channel_TX, ENABLE);

    /* -----------------------  stm32f4xx version  ----------------------- */
    /* RX stream configuration */
    DMA_Cmd(stm32_spi_bus->dma->priv_data->rx_stream, DISABLE);
    DMA_DeInit(stm32_spi_bus->dma->priv_data->rx_stream);
    DMA_InitStructure.DMA_Channel = stm32_spi_bus->dma->priv_data->rx_ch;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(stm32_spi_bus->SPI->DR));
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_InitStructure.DMA_BufferSize = size;

    if(recv_addr != RT_NULL)
    {
        DMA_InitStructure.DMA_Memory0BaseAddr = (u32) recv_addr;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    }
    else
    {
        DMA_InitStructure.DMA_Memory0BaseAddr = (u32) (&dummy);
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    }

    DMA_Init(stm32_spi_bus->dma->priv_data->rx_stream, &DMA_InitStructure);

    DMA_ITConfig(stm32_spi_bus->dma->priv_data->rx_stream, DMA_IT_TC, ENABLE);
    DMA_Cmd(stm32_spi_bus->dma->priv_data->rx_stream, ENABLE);

    /* TX channel configuration */
    DMA_Cmd(stm32_spi_bus->dma->priv_data->tx_stream, DISABLE);
    DMA_DeInit(stm32_spi_bus->dma->priv_data->tx_stream);
    DMA_InitStructure.DMA_Channel = stm32_spi_bus->dma->priv_data->tx_ch;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(stm32_spi_bus->SPI->DR));
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_InitStructure.DMA_BufferSize = size;

    if(send_addr != RT_NULL)
    {
        DMA_InitStructure.DMA_Memory0BaseAddr = (u32)send_addr;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    }
    else
    {
        DMA_InitStructure.DMA_Memory0BaseAddr = (u32)(&dummy);;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    }

    DMA_Init(stm32_spi_bus->dma->priv_data->tx_stream, &DMA_InitStructure);

    DMA_ITConfig(stm32_spi_bus->dma->priv_data->tx_stream, DMA_IT_TC, ENABLE);
    DMA_Cmd(stm32_spi_bus->dma->priv_data->tx_stream, ENABLE);
}

#ifdef SPI1_USING_DMA
static const struct stm32_spi_dma_private dma1_priv =
{
    DMA2_Stream0,       //rx_stream, spi1
    DMA_Channel_3,      //rx_ch, spi1
    DMA_IT_TCIF0,       //rx_flag, spi1
    DMA2_Stream0_IRQn,  //rx_irq_ch, spi1
    DMA2_Stream3,       //tx_stream, spi1
    DMA_Channel_3,      //tx_ch, spi1
    DMA_IT_TCIF3,       //tx_flag, spi1
    DMA2_Stream3_IRQn,  //tx_irq_ch
};
static struct stm32_spi_dma dma1 =
{
    &dma1_priv,
};

void DMA2_Stream0_IRQHandler(void) {
    /* enter interrupt */
    rt_interrupt_enter();
    
    // DMA_ClearFlag(dma1.priv_data->rx_gl_flag);
    if (DMA_GetFlagStatus(dma1.priv_data->rx_stream, dma1.priv_data->rx_flag) != RESET)
    {
        rt_event_send(&dma1.event, SPI_DMA_RX_DONE);
        DMA_ClearFlag(dma1.priv_data->rx_stream, dma1.priv_data->rx_flag);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA2_Stream3_IRQHandler(void) {
    /* enter interrupt */
    rt_interrupt_enter();
    
    // DMA_ClearFlag(dma1.priv_data->tx_gl_flag);
    if (DMA_GetFlagStatus(dma1.priv_data->tx_stream, dma1.priv_data->tx_flag) != RESET)
    {
        rt_event_send(&dma1.event, SPI_DMA_TX_DONE);
        DMA_ClearFlag(dma1.priv_data->tx_stream, dma1.priv_data->tx_flag);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

#endif /*SPI1_USING_DMA*/
#ifdef SPI2_USING_DMA
//  spi2     dma1_stream3        dma1_stream4
static const struct stm32_spi_dma_private dma2_priv =
{
    DMA1_Stream3,
    DMA_Channel_0,
    DMA_IT_TCIF3,
    DMA1_Stream3_IRQn,
    DMA1_Stream4,
    DMA_Channel_0,
    DMA_IT_TCIF4,
    DMA1_Stream4_IRQn,
};
static struct stm32_spi_dma dma2 =
{
    &dma2_priv,
};
void DMA1_Stream3_IRQHandler(void) {
    /* enter interrupt */
    rt_interrupt_enter();
    
    // DMA_ClearFlag(dma2.priv_data->rx_gl_flag);
    if (DMA_GetFlagStatus(dma2.priv_data->rx_stream, dma2.priv_data->rx_flag) != RESET)
    {
        rt_event_send(&dma2.event, SPI_DMA_RX_DONE);
        DMA_ClearFlag(dma2.priv_data->rx_stream, dma2.priv_data->rx_flag);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}
void DMA1_Stream4_IRQHandler(void) {
    /* enter interrupt */
    rt_interrupt_enter();
    
    // DMA_ClearFlag(dma1.priv_data->tx_gl_flag);
    if (DMA_GetFlagStatus(dma2.priv_data->tx_stream, dma2.priv_data->tx_flag) != RESET)
    {
        rt_event_send(&dma2.event, SPI_DMA_TX_DONE);
        DMA_ClearFlag(dma2.priv_data->tx_stream, dma2.priv_data->tx_flag);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /*SPI2_USING_DMA*/

#ifdef SPI3_USING_DMA
// static const struct stm32_spi_dma_private dma3_priv =
// {
//     DMA2_Channel2,
//     DMA2_Channel1,
//     DMA2_FLAG_TC2,
//     DMA2_FLAG_TE2,
//     DMA2_FLAG_TC1,
//     DMA2_FLAG_TE1,
//     DMA2_Channel2_IRQn,
//     DMA2_Channel1_IRQn,
//     DMA2_FLAG_GL2,
//     DMA2_FLAG_GL1,
// };
// static struct stm32_spi_dma dma3 =
// {
//     &dma3_priv,
// };
// void DMA2_Channel1_IRQHandler(void) {
//     /* enter interrupt */
//     rt_interrupt_enter();
//     rt_event_send(&dma3.event, SPI_DMA_TX_DONE);
//     DMA_ClearFlag(dma3.priv_data->tx_gl_flag);
//     /* leave interrupt */
//     rt_interrupt_leave();
// }
// void DMA2_Channel2_IRQHandler(void) {
//     /* enter interrupt */
//     rt_interrupt_enter();
//     rt_event_send(&dma3.event, SPI_DMA_RX_DONE);
//     DMA_ClearFlag(dma3.priv_data->rx_gl_flag);
//     /* leave interrupt */
//     rt_interrupt_leave();
// }
#endif /*SPI3_USING_DMA*/

#endif /*RT_SPI_USE_DMA*/

/* depends on different board */
rt_inline uint16_t get_spi_BaudRatePrescaler(SPI_TypeDef* spi, rt_uint32_t max_hz)
{
    uint16_t SPI_BaudRatePrescaler;
    uint32_t APB_Clock_Half = SystemCoreClock / 8;

    /* SPI1 MAX: 42Mbit/s, SPI2,SPI3 MAX: 21Mbit/s */
    if(spi == SPI1)
    {
        APB_Clock_Half = SystemCoreClock / 4;
    }
    else if(spi == SPI2 || spi == SPI3)
    {
        APB_Clock_Half = SystemCoreClock / 8;
    }

    if(max_hz >= APB_Clock_Half / 2)
    {
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    }
    else if(max_hz >= APB_Clock_Half/4)
    {
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    }
    else if(max_hz >= APB_Clock_Half/8)
    {
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    }
    else if(max_hz >= APB_Clock_Half/16)
    {
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    }
    else if(max_hz >= APB_Clock_Half/32)
    {
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    }
    else if(max_hz >= APB_Clock_Half/64)
    {
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
    }
    else if(max_hz >= APB_Clock_Half/128)
    {
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
    }
    else
    {
        /* min prescaler 256 */
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    }

    return SPI_BaudRatePrescaler;
}

static rt_err_t configure(struct rt_spi_device* device, struct rt_spi_configuration* configuration)
{
    struct stm32_spi_bus * stm32_spi_bus = (struct stm32_spi_bus *)device->bus;
    SPI_InitTypeDef SPI_InitStructure;

    SPI_StructInit(&SPI_InitStructure);

    /* data_width */
    if(configuration->data_width <= 8)
    {
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    }
    else if(configuration->data_width <= 16)
    {
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    }
    else
    {
        return RT_EIO;
    }
    /* baudrate */
    SPI_InitStructure.SPI_BaudRatePrescaler = get_spi_BaudRatePrescaler(stm32_spi_bus->SPI, configuration->max_hz);
    /* CPOL */
    if(configuration->mode & RT_SPI_CPOL)
    {
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    }
    else
    {
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    }
    /* CPHA */
    if(configuration->mode & RT_SPI_CPHA)
    {
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    }
    else
    {
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    }
    /* MSB or LSB */
    if(configuration->mode & RT_SPI_MSB)
    {
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    }
    else
    {
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
    }
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_NSS  = SPI_NSS_Soft;

    /* init SPI */
    SPI_I2S_DeInit(stm32_spi_bus->SPI);
    SPI_Init(stm32_spi_bus->SPI, &SPI_InitStructure);
    /* Enable SPI_MASTER */
    SPI_Cmd(stm32_spi_bus->SPI, ENABLE);
    SPI_CalculateCRC(stm32_spi_bus->SPI, DISABLE);

    return RT_EOK;
};

static rt_uint32_t xfer(struct rt_spi_device* device, struct rt_spi_message* message)
{
    struct stm32_spi_bus * stm32_spi_bus = (struct stm32_spi_bus *)device->bus;
    struct rt_spi_configuration * config = &device->config;
    SPI_TypeDef * SPI = stm32_spi_bus->SPI;
    struct stm32_spi_cs * stm32_spi_cs = device->parent.user_data;
    rt_uint32_t size = message->length;

    /* take CS */
    if(message->cs_take && stm32_spi_cs)
    {
        GPIO_ResetBits(stm32_spi_cs->GPIOx, stm32_spi_cs->GPIO_Pin);
    }
#ifdef RT_SPI_USE_DMA
    if(
       (stm32_spi_bus->parent.parent.flag & (RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX)) &&
        stm32_spi_bus->dma &&
        message->length > 32)
    {
        if(config->data_width <= 8)
        {
            rt_uint32_t ev = 0;
            DMA_Configuration(stm32_spi_bus, message->send_buf, message->recv_buf, message->length);
            SPI_I2S_DMACmd(SPI, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);
            rt_event_recv(&stm32_spi_bus->dma->event, SPI_DMA_COMPLETE,
                    RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &ev);
            SPI_I2S_DMACmd(SPI, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, DISABLE);
            DMA_ITConfig(stm32_spi_bus->dma->priv_data->tx_stream, DMA_IT_TC, DISABLE);
            DMA_ITConfig(stm32_spi_bus->dma->priv_data->rx_stream, DMA_IT_TC, DISABLE);
        }
    }
    else
#endif /*RT_SPI_USE_DMA*/
    {
        if(config->data_width <= 8)
        {
            const rt_uint8_t * send_ptr = message->send_buf;
            rt_uint8_t * recv_ptr = message->recv_buf;

            while(size--)
            {
                rt_uint8_t data = 0xFF;

                if(send_ptr != RT_NULL)
                {
                    data = *send_ptr++;
                }

                //Wait until the transmit buffer is empty
                while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET);
                // Send the byte
                SPI_I2S_SendData(SPI, data);

                //Wait until a data is received
                while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET);
                // Get the received data
                data = SPI_I2S_ReceiveData(SPI);

                if(recv_ptr != RT_NULL)
                {
                    *recv_ptr++ = data;
                }
            }
        }
        else if(config->data_width <= 16)
        {
            const rt_uint16_t * send_ptr = message->send_buf;
            rt_uint16_t * recv_ptr = message->recv_buf;

            while(size--)
            {
                rt_uint16_t data = 0xFF;

                if(send_ptr != RT_NULL)
                {
                    data = *send_ptr++;
                }

                //Wait until the transmit buffer is empty
                while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET);
                // Send the byte
                SPI_I2S_SendData(SPI, data);

                //Wait until a data is received
                while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET);
                // Get the received data
                data = SPI_I2S_ReceiveData(SPI);

                if(recv_ptr != RT_NULL)
                {
                    *recv_ptr++ = data;
                }
            }
        }
    }

    /* release CS */
    if(message->cs_release && stm32_spi_cs)
    {
        GPIO_SetBits(stm32_spi_cs->GPIOx, stm32_spi_cs->GPIO_Pin);
    }

    return message->length;
};

/** \brief init and register stm32 spi bus.
 *
 * \param SPI: STM32 SPI, e.g: SPI1,SPI2,SPI3.
 * \param stm32_spi: stm32 spi bus struct.
 * \param spi_bus_name: spi bus name, e.g: "spi1"
 * \return
 *
 */
rt_err_t stm32_spi_register(SPI_TypeDef * SPI,
                            struct stm32_spi_bus * stm32_spi,
                            const char * spi_bus_name)
{
    rt_err_t res = RT_EOK;
#ifdef RT_SPI_USE_DMA
    NVIC_InitTypeDef NVIC_InitStructure;
#endif
    rt_uint32_t flags = 0;
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    if(SPI == SPI1)
    {
    	stm32_spi->SPI = SPI1;
#ifdef RT_SPI_USE_DMA
#ifdef SPI1_USING_DMA
        {
            rt_event_init(&dma1.event, "spi1ev", RT_IPC_FLAG_FIFO);
            stm32_spi->dma = &dma1;
            /* rx dma interrupt config */
            NVIC_InitStructure.NVIC_IRQChannel = dma1.priv_data->tx_irq_ch;
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
            NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStructure);
            NVIC_InitStructure.NVIC_IRQChannel = dma1.priv_data->rx_irq_ch;
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStructure);
            /* Enable the DMA1 Clock */
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
            flags |= RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX;
        }
#else /*!SPI1_USING_DMA*/
        stm32_spi->dma = RT_NULL;
#endif /*SPI1_USING_DMA*/
#endif /*RT_SPI_USE_DMA*/
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    }
    else if(SPI == SPI2)
    {
        stm32_spi->SPI = SPI2;
#ifdef RT_SPI_USE_DMA
#ifdef SPI2_USING_DMA
        {
            rt_event_init(&dma2.event, "spi2ev", RT_IPC_FLAG_FIFO);
            stm32_spi->dma = &dma2;
            /* rx dma interrupt config */
            NVIC_InitStructure.NVIC_IRQChannel = dma2.priv_data->tx_irq_ch;
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
            NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStructure);
            NVIC_InitStructure.NVIC_IRQChannel = dma2.priv_data->rx_irq_ch;
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStructure);
            /* Enable the DMA1 Clock */
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
            flags |= RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX;
        }
#else /*!SPI2_USING_DMA*/
        stm32_spi->dma = RT_NULL;
#endif /*SPI2_USING_DMA*/
#endif /*RT_SPI_USE_DMA*/
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    }
    else if(SPI == SPI3)
    {
    	stm32_spi->SPI = SPI3;
#ifdef RT_SPI_USE_DMA
#ifdef SPI3_USING_DMA
        // {
        //     rt_event_init(&dma3.event, "spi3ev", RT_IPC_FLAG_FIFO);
        //     stm32_spi->dma = &dma3;
        //     /* rx dma interrupt config */
        //     NVIC_InitStructure.NVIC_IRQChannel = dma3.priv_data->tx_irq_ch;
        //     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        //     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        //     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        //     NVIC_Init(&NVIC_InitStructure);
        //     NVIC_InitStructure.NVIC_IRQChannel = dma3.priv_data->rx_irq_ch;
        //     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        //     NVIC_Init(&NVIC_InitStructure);
        //     /* Enable the DMA1 Clock */
        //     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
        //     flags |= RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX;
        // }
#else /*!SPI3_USING_DMA*/
        stm32_spi->dma = RT_NULL;
#endif /*SPI3_USING_DMA*/
#endif /*RT_SPI_USE_DMA*/
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
    }
    else
    {
        return RT_ENOSYS;
    }

    // formally register a SPI bus
    res = rt_spi_bus_register(&stm32_spi->parent, spi_bus_name, &stm32_spi_ops);
    stm32_spi->parent.parent.flag |= flags;

    return res;
}