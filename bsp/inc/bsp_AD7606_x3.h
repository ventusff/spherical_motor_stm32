#ifndef __BSP_AD7606__H__INCLUDED
#define __BSP_AD7606__H__INCLUDED

#include <configuration.h>

typedef enum
{
	AD7606_PORT1 = 0,
	AD7606_PORT2 = 1,
	AD7606_PORT3 = 2
}AD7606_PORT;

/* AD7606 FSMC总线地址，只能读，无需写 */
	//6			C			4			0			0			0			0			0
	//0110 	1100 	0100 	0000 	0000 	0000 	0000 	0000
	//31:28	27:24	23:20	19:16	15:12	11:8	7:4		3:0
	//A27,A26由FSMC_NE1,FSMC_NE2,FSMC_NE3,FSMC_NE4决定，选4个bank
	//A25:0是自己定的地址

//球形电机：使用138译码器对3个AD7606进行片选
//连接关系：
	//G1常高，G2A,G2B均连接FSMC_NE4(PG12)
	//A:A23,		B:A24,		C:A25;
	//Y0:AD7606_CS1,		Y1:AD7606_CS2,		Y2:AD7606_CS3
//使能表：
	//000:AD7606_1
	//001:AD7606_2
	//010:AD7606_3

//AD7606_1:   00 0000 x -> 00 0000 x
	//6			C			0			0			0			0			0			0
	//0110 	1100 	0000 	0000 	0000 	0000 	0000 	0000
	//31:28	27:24	23:20	19:16	15:12	11:8	7:4		3:0
#define AD7606_RESULT_1() *(__IO uint16_t *)0x6C000000

//AD7606_2:   01 0000 x -> 00 1000 x
	//6			D			0			0			0			0			0			0
	//0110 	1101 	0000 	0000 	0000 	0000 	0000 	0000
	//31:28	27:24	23:20	19:16	15:12	11:8	7:4		3:0
#define AD7606_RESULT_2() *(__IO uint16_t *)0x6D000000

//AD7606_3:   10 0000 x -> 01 0000 x
	//6			E			0			0			0			0			0			0
	//0110 	1110 	0000 	0000 	0000 	0000 	0000 	0000
	//31:28	27:24	23:20	19:16	15:12	11:8	7:4		3:0
#define AD7606_RESULT_3() *(__IO uint16_t *)0x6E000000

void AD7606_x3_init(void);
#ifdef AD7606_USE_BOARD_RANGE_SEL
void AD7606_SetInputRange(uint8_t _ucRange);
#endif
void AD7606_x3_Reset(void);
uint8_t AD7606x_IsReady(uint8_t ADC_port);
void AD7606x_StartConvst(uint8_t ADC_port);
void AD7606x_ReadNowAdc(uint8_t ADC_port, int16_t *rcvBuf, uint8_t num);


#endif