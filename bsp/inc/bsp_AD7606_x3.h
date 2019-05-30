#ifndef __BSP_AD7606__H__INCLUDED
#define __BSP_AD7606__H__INCLUDED

#include <configuration.h>

typedef enum
{
	AD7606_PORT1 = 0,
	AD7606_PORT2 = 1,
	AD7606_PORT3 = 2
}AD7606_PORT;

#define AD7606_CHANNEL_MAX (8)

#ifdef AD7606_USE_BOARD_RANGE_SEL
void AD7606_SetInputRange(uint8_t _ucRange);
#else
#define _cst_ADC_1_BIT_TO_V        0.0001525879f    //10V/65536
#define _cst_ADC_1_V_TO_BIT        6553.6f          //65536/10V
#endif

void AD7606_x3_Reset(void);
uint8_t AD7606x_IsReady(uint8_t ADC_port);
void AD7606x_StartConvst(uint8_t ADC_port);
void AD7606x_ReadNowAdc_RawArray(uint8_t ADC_port, int16_t *rcvBuf, uint8_t num);
void AD7606x_ReadNowAdc_VoltageArray(uint8_t ADC_port, float *rcvBuf, uint8_t num);


#endif