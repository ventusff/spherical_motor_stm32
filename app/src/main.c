/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2006-08-31     Bernard      first implementation
 * 2011-06-05     Bernard      modify for STM32F107 version
 */

#include <rthw.h>
#include <rtthread.h>
#include <board.h>
/**
 * @addtogroup STM32
 */

/*@{*/

int main(void)
{
	//wait system init.
	rt_thread_delay(DELAY_10MS(10));
	RCC_ClocksTypeDef  rcc_clocks;
	RCC_GetClocksFreq(&rcc_clocks);
	rt_kprintf("fHCLK = %d, fMAIN = %d, fPCLK1 = %d, fPCLK2 = %d. \n",rcc_clocks.HCLK_Frequency, rcc_clocks.SYSCLK_Frequency, rcc_clocks.PCLK1_Frequency, rcc_clocks.PCLK2_Frequency);
    /* user app entry */
    return 0;
}

/*@}*/
