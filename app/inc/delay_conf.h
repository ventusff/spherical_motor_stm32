/*********************************************************************************************************/
// Filename      : delay_config.h
// Version       : V1.00
// Programmer(s) : Liuqiuhu
// funcion		 : This file is used to configure the delay time
/*********************************************************************************************************/
#ifndef __DELAY_CONF_H__
#define __DELAY_CONF_H__

#include"rtconfig.h"

#if RT_TICK_PER_SECOND == 1
#define DELAY_1S			(RT_TICK_PER_SECOND)		
#define DELAY_S(X) 			(X*DELAY_1S)

#elif RT_TICK_PER_SECOND == 10				
#define DELAY_100MS(X)		(X)
#define DELAY_S(X)			(X*10)

#elif RT_TICK_PER_SECOND == 100		
#define	DELAY_10MS(X)		(X)
#define	DELAY_100MS(X)		(X*10)
#define DELAY_S(X)			(X*100)

#elif (RT_TICK_PER_SECOND == 1000)


#define DELAY_1MS           (RT_TICK_PER_SECOND/1000)		
#define DELAY_MS(X)         (X*DELAY_1MS)
#define DELAY_S(X)		    (X*1000*DELAY_1MS)
#define	DELAY_10MS(X)		DELAY_MS(10*X)

#elif (RT_TICK_PER_SECOND == 10000)
#define DELAY_100US(X)      (X*RT_TICK_PER_SECOND/10000)
#define DELAY_1MS           (RT_TICK_PER_SECOND/1000)		
#define DELAY_MS(X)         (X*DELAY_1MS)
#define DELAY_S(X)		    (X*1000*DELAY_1MS)
#define	DELAY_10MS(X)		DELAY_MS(10*X)

#elif (RT_TICK_PER_SECOND == 50000)
#define DELAY_100US(X)      (X*RT_TICK_PER_SECOND/10000)
#define DELAY_1MS           (RT_TICK_PER_SECOND/1000)		
#define DELAY_MS(X)         (X*DELAY_1MS)
#define DELAY_S(X)		    (X*1000*DELAY_1MS)
#define	DELAY_10MS(X)		DELAY_MS(10*X)

#endif

#define DELAY_SYS_RUN_LED			DELAY_MS(25)
#define DELAY_SYS_SLEEP_LED			DELAY_MS(1000)
#define DELAY_SYS_WAIT_INIT         DELAY_MS(1000)

#define FROM_TICK_TO_MS(x) (x * 1000.0 / RT_TICK_PER_SECOND)

#endif  

