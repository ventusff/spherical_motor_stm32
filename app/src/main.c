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

void show_sysclock_config(void)
{
	RCC_ClocksTypeDef  rcc_clocks;
	RCC_GetClocksFreq(&rcc_clocks);
	rt_kprintf("\n[Sysclock Config]:\n");
	rt_kprintf("fHCLK = %d, fMAIN = %d, fPCLK1 = %d, fPCLK2 = %d. \n",rcc_clocks.HCLK_Frequency, rcc_clocks.SYSCLK_Frequency, rcc_clocks.PCLK1_Frequency, rcc_clocks.PCLK2_Frequency);
}

int main(void)
{
	//wait system init.
	rt_thread_delay(DELAY_10MS(10));
	show_sysclock_config();
    /* user app entry */

	#ifdef RT_USING_ULOG
	// extern void ulog_example(void);
	// ulog_example();
	#endif
	
	#ifdef RT_USING_LOGTRACE
	// extern void log_trace_example(void);
	// log_trace_example();
	#endif

	#ifdef RT_USING_CAN
	extern void vCanRcvRcvPoll(void);
	vCanRcvRcvPoll();
	#endif

	while(1){
		rt_thread_delay(DELAY_S(1));
	}

    return 0;
}

/*@}*/
