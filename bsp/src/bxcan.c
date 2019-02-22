/*
 * File      : bxcan.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2015, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author            Notes
 * 2015-05-14     aubrcool@qq.com   first version
 */
#include <rthw.h>
#include <rtdevice.h>
#include <board.h>
#include <bxcan.h>

#if (defined (STM32F10X_LD_VL)) || (defined (STM32F10X_MD_VL)) || (defined (STM32F10X_HD_VL))
#undef RT_USING_CAN
#endif

#ifdef RT_USING_CAN

#ifndef STM32F4
#ifndef STM32F10X_CL
#define BX_CAN_FMRNUMBER 14
#define BX_CAN2_FMRSTART 7
#else
#define BX_CAN_FMRNUMBER 28
#define BX_CAN2_FMRSTART 14
#endif
#else
#define BX_CAN_FMRNUMBER 28     // the number of filter banks. for STM32F4, it's 28.

// commented by guojianfei 20190119. this can be DIYed. 
// just because CAN1 and CAN2 use the same set of filter banks(28 banks),
// here is just to allocate 28 banks to two CAN.
// we can change this to be bigger, if we are not using CAN2.
// #define BX_CAN2_FMRSTART 14  // orginal value
#define BX_CAN2_FMRSTART 24
#endif

#if (defined (STM32F10X_LD)) || (defined (STM32F10X_MD)) || (defined (STM32F10X_HD)) || (defined (STM32F10X_XL))
#undef USING_BXCAN2
#define CAN1_RX0_IRQn USB_LP_CAN1_RX0_IRQn
#define CAN1_TX_IRQn USB_HP_CAN1_TX_IRQn
#endif

#define BX_CAN_MAX_FILTERS (BX_CAN_FMRNUMBER * 4)       //the maximum number of filters.
#define BX_CAN_MAX_FILTER_MASKS BX_CAN_MAX_FILTERS
#define BX_CAN_FILTER_MAX_ARRAY_SIZE ((BX_CAN_MAX_FILTERS + 32 - 1) / 32)

struct stm_bxcanfiltermap
{
    rt_uint32_t id32mask_cnt;   // count of 32-bit mask filters. 1 filterbank = 1 filter
    rt_uint32_t id32bit_cnt;    // count of 32-bit list filters. 1 filterbank = 2 filters
    rt_uint32_t id16mask_cnt;   // count of 16-bit mask filters. 1 filterbank = 2 filters
    rt_uint32_t id16bit_cnt;    // count of 16-bit list filters. 1 filterbank = 4 filters
};
struct stm_bxcanfilter_masks
{
    rt_uint32_t id32maskm[BX_CAN_FILTER_MAX_ARRAY_SIZE];
    rt_uint32_t id32bitm[BX_CAN_FILTER_MAX_ARRAY_SIZE];
    rt_uint32_t id16maskm[BX_CAN_FILTER_MAX_ARRAY_SIZE];
    rt_uint32_t id16bitm[BX_CAN_FILTER_MAX_ARRAY_SIZE];
    rt_uint32_t id32maskshift[2];
    rt_uint32_t id32bitshift[2];
    rt_uint32_t id16maskshift[2];
    rt_uint32_t id16bitshift[2];
};
struct stm_bxcan
{
    CAN_TypeDef *reg;
    void *mfrbase;
    IRQn_Type sndirq;
    IRQn_Type rcvirq0;
    IRQn_Type rcvirq1;
    IRQn_Type errirq;
    struct stm_bxcanfilter_masks filtermask;
    rt_uint32_t alocmask[BX_CAN_FILTER_MAX_ARRAY_SIZE];

    const rt_uint32_t filtercnt;    //count of filterbanks used by CANx

    //before this value, filterbanks use FIFO0; after this value, filterbanks use FIFO1
    const rt_uint32_t fifo1filteroff;   

    //filtermap[0].xxx: filterbanks using FIFO0. filtermap[1].xxx: filterbanks using FIFO1
    const struct stm_bxcanfiltermap filtermap[2];       
};
struct stm_baud_rate_tab
{
	rt_uint32_t baud_rate;
	rt_uint32_t confdata;
};
static void calcfiltermasks(struct stm_bxcan *pbxcan);

/** by guojianfei 
 * @brief roughly allocate the registers of filterbanks to needed filters
 * 
 * @param can 
 */
static void bxcan1_filter_init(struct rt_can_device *can)
{
    rt_uint32_t i;
    rt_uint32_t mask;
    struct stm_bxcan *pbxcan = (struct stm_bxcan *) can->parent.user_data;
    for (i = 0; i < BX_CAN2_FMRSTART; i++)
    {
        CAN1->FMR |= FMR_FINIT;
        mask = 0x01 << (i + 0);

// commented by guojianfei 20190119.
// the meaning of the code below can be demonstrated in CAN_FilterInit

// CAN1->FS1R |= mask; means the x-filterbank is 32-bit scale
// CAN1->FS1R &=~ mask; means the x-filterbank is 16-bit scale

// CAN1->FM1R |= mask; means the x-filterbank is list mode
// CAN1->FM1R &=~ mask; means the x-filterbank is mask mode

// CAN1->FFA1R &= ~mask; means the x-filterbank uses FIFO0
// CAN1->FFA1R |= mask; means the x-filterbank uses FIFO1

// then, all the filter banks is arranged according to the sequence:

//: from 0 to fifo1filteroff - 1
// [filtermap[0]]
// 1. first, 32-bit mask mode, filterbanks count: filtermap[0].id32mask_cnt
// 2. then, 32-bit list mode, filterbanks count: filtermap[0].id32bit_cnt/2   (because 1 bank = 2 * 32-bit-list-filters)
// 3. then, 16-bit mask mode, filterbanks count: filtermap[0].id16mask_cnt/2  (because 1 bank = 2 * 16-bit-mask-filters)
// 4. last, 16-bit list mode, filterbanks count: filtermap[0].id16bit_cnt/4   (because 1 bank = 4 * 16-bit-list-filters)

//: from fifo1filteroff to BX_CAN2_FMRSTART - 1
// [filtermap[1]]
// 1. first, 32-bit mask mode, filterbanks count: filtermap[1].id32mask_cnt
// 2. then, 32-bit list mode, filterbanks count: filtermap[1].id32bit_cnt/2   (because 1 bank = 2 * 32-bit-list-filters)
// 3. then, 16-bit mask mode, filterbanks count: filtermap[1].id16mask_cnt/2  (because 1 bank = 2 * 16-bit-mask-filters)
// 4. last, 16-bit list mode, filterbanks count: filtermap[1].id16bit_cnt/4   (because 1 bank = 4 * 16-bit-list-filters)

//: from BX_CAN2_FMRSTART to all the remain filterbanks: (CAN2)
        if (i < pbxcan->fifo1filteroff)
        {
            if (pbxcan->filtermap[0].id32mask_cnt && i < pbxcan->filtermap[0].id32mask_cnt)
            {
                CAN1->FS1R |= mask;
                CAN1->FM1R &= ~mask;
                CAN1->FFA1R &= ~mask;
            }
            else if (pbxcan->filtermap[0].id32bit_cnt &&
                     i < pbxcan->filtermap[0].id32mask_cnt + pbxcan->filtermap[0].id32bit_cnt / 2)
            {
                CAN1->FS1R |= mask;
                CAN1->FM1R |= mask;
                CAN1->FFA1R &= ~mask;
            }
            else if (pbxcan->filtermap[0].id16mask_cnt &&
                     i < pbxcan->filtermap[0].id32mask_cnt + pbxcan->filtermap[0].id32bit_cnt / 2
                     + pbxcan->filtermap[0].id16mask_cnt / 2)
            {
                CAN1->FS1R &= ~mask;
                CAN1->FM1R &= ~mask;
                CAN1->FFA1R &= ~mask;
            }
            else if (pbxcan->filtermap[0].id16bit_cnt &&
                     i < pbxcan->filtermap[0].id32mask_cnt + pbxcan->filtermap[0].id32bit_cnt / 2
                     + pbxcan->filtermap[0].id16mask_cnt / 2 + pbxcan->filtermap[0].id16bit_cnt / 4
                    )
            {
                CAN1->FS1R &= ~mask;
                CAN1->FM1R |= mask;
                CAN1->FFA1R &= ~mask;
            }
        }
        else
        {
            if (pbxcan->filtermap[1].id32mask_cnt &&
                    i < pbxcan->filtermap[1].id32mask_cnt + pbxcan->fifo1filteroff)
            {
                CAN1->FS1R |= mask;
                CAN1->FM1R &= ~mask;
                CAN1->FFA1R |= mask;
            }
            else if (pbxcan->filtermap[1].id32bit_cnt &&
                     i < pbxcan->filtermap[1].id32mask_cnt + pbxcan->filtermap[1].id32bit_cnt / 2
                     + pbxcan->fifo1filteroff)
            {
                CAN1->FS1R |= mask;
                CAN1->FM1R |= mask;
                CAN1->FFA1R |= mask;
            }
            else if (pbxcan->filtermap[1].id16mask_cnt &&
                     i < pbxcan->filtermap[1].id32mask_cnt + pbxcan->filtermap[1].id32bit_cnt / 2
                     + pbxcan->filtermap[1].id16mask_cnt / 2 + pbxcan->fifo1filteroff)
            {
                CAN1->FS1R &= ~mask;
                CAN1->FM1R &= ~mask;
                CAN1->FFA1R |= mask;
            }
            else if (pbxcan->filtermap[1].id16bit_cnt &&
                     i < pbxcan->filtermap[1].id32mask_cnt + pbxcan->filtermap[1].id32bit_cnt / 2
                     + pbxcan->filtermap[1].id16mask_cnt / 2 + pbxcan->filtermap[1].id16bit_cnt / 4
                     + pbxcan->fifo1filteroff)
            {
                CAN1->FS1R &= ~mask;
                CAN1->FM1R |= mask;
                CAN1->FFA1R |= mask;
            }
        }
        CAN1->sFilterRegister[i].FR1 = 0xFFFFFFFF;
        CAN1->sFilterRegister[i].FR2 = 0xFFFFFFFF;
        CAN1->FMR &= ~FMR_FINIT;
    }
    calcfiltermasks(pbxcan);
}
#ifdef USING_BXCAN2
static void bxcan2_filter_init(struct rt_can_device *can)
{
    rt_uint32_t i;
    rt_uint32_t off;
    rt_uint32_t mask;
    CAN_SlaveStartBank(BX_CAN2_FMRSTART);
    struct stm_bxcan *pbxcan = (struct stm_bxcan *) can->parent.user_data;
    for (i = BX_CAN2_FMRSTART; i < BX_CAN_FMRNUMBER; i++)
    {
        CAN1->FMR |= FMR_FINIT;
        mask = 0x01 << (i + 0);
        off = i - BX_CAN2_FMRSTART;
        if (off < pbxcan->fifo1filteroff)
        {
            if (pbxcan->filtermap[0].id32mask_cnt && off < pbxcan->filtermap[0].id32mask_cnt)
            {
                CAN1->FS1R |= mask;
                CAN1->FM1R &= ~mask;
                CAN1->FFA1R &= ~mask;
            }
            else if (pbxcan->filtermap[0].id32bit_cnt &&
                     off < pbxcan->filtermap[0].id32mask_cnt + pbxcan->filtermap[0].id32bit_cnt / 2)
            {
                CAN1->FS1R |= mask;
                CAN1->FM1R |= mask;
                CAN1->FFA1R &= ~mask;
            }
            else if (pbxcan->filtermap[0].id16mask_cnt &&
                     off < pbxcan->filtermap[0].id32mask_cnt + pbxcan->filtermap[0].id32bit_cnt / 2
                     + pbxcan->filtermap[0].id16mask_cnt / 2)
            {
                CAN1->FS1R &= ~mask;
                CAN1->FM1R &= ~mask;
                CAN1->FFA1R &= ~mask;
            }
            else if (pbxcan->filtermap[0].id16bit_cnt &&
                     off < pbxcan->filtermap[0].id32mask_cnt + pbxcan->filtermap[0].id32bit_cnt / 2
                     + pbxcan->filtermap[0].id16mask_cnt / 2 + pbxcan->filtermap[0].id16bit_cnt / 4
                    )
            {
                CAN1->FS1R &= ~mask;
                CAN1->FM1R |= mask;
                CAN1->FFA1R &= ~mask;
            }
        }
        else
        {
            if (pbxcan->filtermap[1].id32mask_cnt &&
                    off < pbxcan->filtermap[1].id32mask_cnt + pbxcan->fifo1filteroff)
            {
                CAN1->FS1R |= mask;
                CAN1->FM1R &= ~mask;
                CAN1->FFA1R |= mask;
            }
            else if (pbxcan->filtermap[1].id32bit_cnt &&
                     off < pbxcan->filtermap[1].id32mask_cnt + pbxcan->filtermap[1].id32bit_cnt / 2
                     + pbxcan->fifo1filteroff)
            {
                CAN1->FS1R |= mask;
                CAN1->FM1R |= mask;
                CAN1->FFA1R |= mask;
            }
            else if (pbxcan->filtermap[1].id16mask_cnt &&
                     off < pbxcan->filtermap[1].id32mask_cnt + pbxcan->filtermap[1].id32bit_cnt / 2
                     + pbxcan->filtermap[1].id16mask_cnt / 2 + pbxcan->fifo1filteroff)
            {
                CAN1->FS1R &= ~mask;
                CAN1->FM1R &= ~mask;
                CAN1->FFA1R |= mask;
            }
            else if (pbxcan->filtermap[1].id16bit_cnt &&
                     off < pbxcan->filtermap[1].id32mask_cnt + pbxcan->filtermap[1].id32bit_cnt / 2
                     + pbxcan->filtermap[1].id16mask_cnt / 2 + pbxcan->filtermap[1].id16bit_cnt / 4
                     + pbxcan->fifo1filteroff)
            {
                CAN1->FS1R &= ~mask;
                CAN1->FM1R |= mask;
                CAN1->FFA1R |= mask;
            }
        }
        CAN1->sFilterRegister[i].FR1 = 0xFFFFFFFF;
        CAN1->sFilterRegister[i].FR2 = 0xFFFFFFFF;
        CAN1->FMR &= ~FMR_FINIT;
    }
    calcfiltermasks(pbxcan);
}
#endif

#define BS1SHIFT 16
#define BS2SHIFT 20
#define RRESCLSHIFT 0
#define SJWSHIFT 24
#define BS1MASK ( (0x0F) << BS1SHIFT )
#define BS2MASK ( (0x07) << BS2SHIFT )
#define RRESCLMASK ( 0x3FF << RRESCLSHIFT )
#define SJWMASK ( 0x3 << SJWSHIFT )

#define MK_BKCAN_BAUD(SJW,BS1,BS2,PRES) \
    ((SJW << SJWSHIFT) | (BS1 << BS1SHIFT) | (BS2 << BS2SHIFT) | (PRES << RRESCLSHIFT))

static const struct stm_baud_rate_tab bxcan_baud_rate_tab[] =
{
#if defined STM32F10X
#if defined STM32F10X_CL
    // 48 M
    {1000UL * 1000, MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_12tq, CAN_BS2_3tq, 3)},
    {1000UL * 800,  MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_6tq,  CAN_BS2_3tq, 6)},
    {1000UL * 500,  MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_12tq, CAN_BS2_3tq, 5)},
    {1000UL * 250,  MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_12tq, CAN_BS2_3tq, 11)},//1
    {1000UL * 125,  MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_12tq, CAN_BS2_3tq, 23)},
    {1000UL * 100,  MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_12tq, CAN_BS2_3tq, 29)},
    {1000UL * 50,   MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_12tq, CAN_BS2_3tq, 59)},
    {1000UL * 20,   MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_14tq, CAN_BS2_3tq, 149)},
    {1000UL * 10,   MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_16tq, CAN_BS2_8tq, 199)}
#else
	// 36 M
	{1000UL * 1000, MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_8tq,  CAN_BS2_3tq, 3)},
	{1000UL * 800,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_11tq, CAN_BS2_3tq, 3)},
	{1000UL * 500,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_9tq,  CAN_BS2_2tq, 6)},
	{1000UL * 250,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_13tq, CAN_BS2_2tq, 9)},//1
	{1000UL * 125,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_13tq, CAN_BS2_2tq, 18)},
	{1000UL * 100,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_9tq,  CAN_BS2_2tq, 30)},
	{1000UL * 50,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_13tq, CAN_BS2_2tq, 45)},
	{1000UL * 20,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_14tq, CAN_BS2_3tq, 100)},
	{1000UL * 10,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_14tq, CAN_BS2_3tq, 200)}
#endif

#elif defined STM32F40_41xxx
    // 1bit = (SYNC_SE(1tq) + CAN_BS1 + CAN_BS2) , 1 tq = 1/APB1_clock * prescaler
	// 42 M
	{1000UL * 1000, MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_10tq, CAN_BS2_3tq, 3)},      // 42 = 3 * 14
	{1000UL * 800,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_10tq, CAN_BS2_2tq, 4)},      // 42/0.8 = 52.5  (52 = 4*13)
	{1000UL * 500,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_8tq,  CAN_BS2_3tq, 7)},      // 42/0.5 = 84 = 7 * 12
	{1000UL * 250,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_8tq,  CAN_BS2_3tq, 13)},     // 42/0.25 = 168 = 14 * 12
	{1000UL * 125,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_8tq,  CAN_BS2_3tq, 28)},     // 42/0.125 = 28 * 12
	{1000UL * 100,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_10tq, CAN_BS2_3tq, 30)},     // 42/0.1 = 42 * 10 = 14 * 30
	{1000UL * 50,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_12tq, CAN_BS2_2tq, 56)},     // 42/0.05 = 42 * 20 = 56 * 15
	{1000UL * 20,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_16tq, CAN_BS2_3tq, 105)},    // 42/0.02 = 42 * 50 = 20 * 105
	{1000UL * 10,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_16tq, CAN_BS2_3tq, 210)}     // 42/0.01 = 42 * 100 = 20 * 210

#elif defined STM32F429_439xx
	// 45M
	{1000UL * 1000, MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_10tq, CAN_BS2_4tq, 3)},      // 45/1 = 45
	{1000UL * 800,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_10tq, CAN_BS2_3tq, 4)},      // 45/0.8 = 56.25 ~= 56 = 4 * 14
	{1000UL * 500,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_10tq, CAN_BS2_4tq, 6)},      // 45/0.5 = 90 = 6 * 15
	{1000UL * 250,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_10tq, CAN_BS2_4tq, 12)},     // 45/0.25 = 180 = 12 * 15
	{1000UL * 125,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_10tq, CAN_BS2_4tq, 24)},     // 45/0.125 = 360 =  24 * 15
	{1000UL * 100,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_10tq, CAN_BS2_4tq, 30)},     // 45/0.1 = 450 = 30 * 15
	{1000UL * 50,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_10tq, CAN_BS2_4tq, 60)},     // 45/0.05 = 900 = 60 * 15
	{1000UL * 20,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_10tq, CAN_BS2_4tq, 150)},    // 45/0.02 = 2250 = 150 * 15
	{1000UL * 10,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_10tq, CAN_BS2_4tq, 300)}     // 45/0.01 = 4500 = 300 * 15
#endif
};

#define BAUD_DATA(TYPE,NO) \
    ((bxcan_baud_rate_tab[NO].confdata & TYPE##MASK) >> TYPE##SHIFT)

static rt_uint32_t bxcan_get_baud_index(rt_uint32_t baud)
{
	rt_uint32_t len, index, default_index;
	
	len = sizeof(bxcan_baud_rate_tab)/sizeof(bxcan_baud_rate_tab[0]);
	default_index = len;

	for(index = 0; index < len; index++)
	{
		if(bxcan_baud_rate_tab[index].baud_rate == baud)
			return index;

		if(bxcan_baud_rate_tab[index].baud_rate == 1000UL * 250)
			default_index = index;
	}

	if(default_index != len)
		return default_index;

	return 0;	
}


static void bxcan_init(CAN_TypeDef *pcan, rt_uint32_t baud, rt_uint32_t mode)
{
    CAN_InitTypeDef        CAN_InitStructure;
	
	rt_uint32_t baud_index = bxcan_get_baud_index(baud);


    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 , ENABLE);

    // CAN_InitStructure.CAN_TTCM = DISABLE;
    // CAN_InitStructure.CAN_ABOM = ENABLE;
    // CAN_InitStructure.CAN_AWUM = DISABLE;
    // CAN_InitStructure.CAN_NART = DISABLE;
    // CAN_InitStructure.CAN_RFLM = DISABLE;
    // CAN_InitStructure.CAN_TXFP = ENABLE;


    CAN_InitStructure.CAN_TTCM = DISABLE;   //! this needs to be disabled. or the data will be interferred.
    CAN_InitStructure.CAN_ABOM = ENABLE;
    CAN_InitStructure.CAN_AWUM = ENABLE;
    CAN_InitStructure.CAN_NART = ENABLE;
    CAN_InitStructure.CAN_RFLM = ENABLE;
    CAN_InitStructure.CAN_TXFP = ENABLE;
    switch (mode)
    {
    case RT_CAN_MODE_NORMAL:
        CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
        break;
    case RT_CAN_MODE_LISEN:
        CAN_InitStructure.CAN_Mode = CAN_Mode_Silent;
        break;
    case RT_CAN_MODE_LOOPBACK:
        CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;
        break;
    case RT_CAN_MODE_LOOPBACKANLISEN:
        CAN_InitStructure.CAN_Mode = CAN_Mode_Silent_LoopBack;
        break;
    }
    CAN_InitStructure.CAN_SJW = BAUD_DATA(SJW, baud_index);
    CAN_InitStructure.CAN_BS1 = BAUD_DATA(BS1, baud_index);
    CAN_InitStructure.CAN_BS2 = BAUD_DATA(BS2, baud_index);
    CAN_InitStructure.CAN_Prescaler = BAUD_DATA(RRESCL, baud_index);

    // commented by guojianfei 20190131: bacause the error sysclosk settings, fAPB1 != 42MHZ, =13.5MHZ
    // fHCLK = 53760000, fMAIN = 53760000, fPCLK1 = 13440000, fPCLK2 = 26880000. 
    // this is the test result which could fit into the communication with chassis as 500k Baudrate 
    // CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    // CAN_InitStructure.CAN_BS1 = CAN_BS1_5tq;
    // CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
    // CAN_InitStructure.CAN_Prescaler = 3;

    CAN_Init(pcan, &CAN_InitStructure);
}


#if defined STM32F429_439xx
#define CAN1_PORT GPIOA

#define CAN1_RX_PIN GPIO_Pin_12
#define CAN1_RX_PIN_SOURCE GPIO_PinSource12

#define CAN1_TX_PIN GPIO_Pin_13
#define CAN1_TX_PIN_SOURCE GPIO_PinSource13
#else
#define CAN1_PORT GPIOA

#define CAN1_RX_PIN GPIO_Pin_11
#define CAN1_RX_PIN_SOURCE GPIO_PinSource11

#define CAN1_TX_PIN GPIO_Pin_12
#define CAN1_TX_PIN_SOURCE GPIO_PinSource12
#endif


//changed by guojianfei 20190119
static void bxcan1_hw_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_PinAFConfig(CAN1_PORT, CAN1_RX_PIN_SOURCE, GPIO_AF_CAN1);    //CAN1_RX
    GPIO_PinAFConfig(CAN1_PORT, CAN1_TX_PIN_SOURCE, GPIO_AF_CAN1);    //CAN1_TX

    //CAN1_RX
    GPIO_InitStructure.GPIO_Pin = CAN1_RX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(CAN1_PORT, &GPIO_InitStructure);

    //CAN1_TX
    GPIO_InitStructure.GPIO_Pin = CAN1_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(CAN1_PORT, &GPIO_InitStructure);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}
#ifdef USING_BXCAN2
static void bxcan2_hw_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX1_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = CAN2_TX_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}
#endif
rt_inline rt_err_t bxcan_enter_init(CAN_TypeDef *pcan)
{
    uint32_t wait_ack = 0x00000000;

    pcan->MCR |= CAN_MCR_INRQ ;

    while (((pcan->MSR & CAN_MSR_INAK) != CAN_MSR_INAK) && (wait_ack != INAK_TIMEOUT))
    {
        wait_ack++;
    }
    if ((pcan->MSR & CAN_MSR_INAK) != CAN_MSR_INAK)
    {
        return RT_ERROR;
    }
    return RT_EOK;
}
rt_inline rt_err_t bxcan_exit_init(CAN_TypeDef *pcan)
{
    uint32_t wait_ack = 0x00000000;

    pcan->MCR &= ~(uint32_t)CAN_MCR_INRQ;

    while (((pcan->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) && (wait_ack != INAK_TIMEOUT))
    {
        wait_ack++;
    }
    if ((pcan->MSR & CAN_MSR_INAK) != CAN_MSR_INAK)
    {
        return RT_ERROR;
    }
    return RT_EOK;
}
static rt_err_t bxcan_set_mode(CAN_TypeDef *pcan, rt_uint32_t mode)
{
    if (bxcan_enter_init(pcan) != RT_EOK)
    {
        return RT_ERROR;
    }
    pcan->BTR &= ~(uint32_t)((uint32_t)0x03 << 30);
    switch (mode)
    {
    case RT_CAN_MODE_NORMAL:
        mode = CAN_Mode_Normal;
        break;
    case RT_CAN_MODE_LISEN:
        mode = CAN_Mode_Silent;
        break;
    case RT_CAN_MODE_LOOPBACK:
        mode = CAN_Mode_LoopBack;
        break;
    case RT_CAN_MODE_LOOPBACKANLISEN:
        mode = CAN_Mode_Silent_LoopBack;
        break;
    }
    pcan->BTR |= ~(uint32_t)(mode << 30);
    if (bxcan_exit_init(pcan) != RT_EOK)
    {
        return RT_ERROR;
    }
    return RT_EOK;
}
static rt_err_t bxcan_set_privmode(CAN_TypeDef *pcan, rt_uint32_t mode)
{
    if (bxcan_enter_init(pcan) != RT_EOK)
    {
        return RT_ERROR;
    }
    if (mode == ENABLE)
    {
        pcan->MCR |= CAN_MCR_TXFP;
    }
    else
    {
        pcan->MCR &= ~(uint32_t)CAN_MCR_TXFP;
    }
    if (bxcan_exit_init(pcan) != RT_EOK)
    {
        return RT_ERROR;
    }
    return RT_EOK;
}
static rt_err_t bxcan_set_baud_rate(CAN_TypeDef *pcan, rt_uint32_t baud)
{
    rt_uint32_t mode;

	rt_uint32_t baud_index = bxcan_get_baud_index(baud);
	
    if (bxcan_enter_init(pcan) != RT_EOK)
    {
        return RT_ERROR;
    }
    pcan->BTR = 0;
    mode = pcan->BTR & ((rt_uint32_t)0x03 << 30);
    pcan->BTR = (mode                         | \
                 ((BAUD_DATA(SJW, baud_index)) << 24) | \
                 ((BAUD_DATA(BS1, baud_index)) << 16) | \
                 ((BAUD_DATA(BS2, baud_index)) << 20) | \
                 (BAUD_DATA(RRESCL, baud_index)));
    if (bxcan_exit_init(pcan) != RT_EOK)
    {
        return RT_ERROR;
    }
    return RT_EOK;
}

/**
 * @brief to get the number of filter bank and the filter's offset in the bank.
 * 
 * @param pbxcan    CANx object.
 * @param hdr       the number/index of the filter among all the filters used by CANx
 * @param pbase     @out the number/index of bank.
 * @param poff      @out the number/index of the filter in the specific bank.
 * @return rt_err_t 
 */
static rt_err_t bxcancalcbaseoff(struct stm_bxcan *pbxcan, rt_int32_t hdr,
                                 rt_int32_t *pbase, rt_int32_t   *poff)
{
    rt_uint32_t fifo0start, fifo0end;
    rt_uint32_t fifo1start, fifo1end;
    rt_uint32_t ptr;
    fifo0start = 0;
    fifo0end = pbxcan->filtermap[0].id32mask_cnt
               + pbxcan->filtermap[0].id32bit_cnt
               + pbxcan->filtermap[0].id16mask_cnt
               + pbxcan->filtermap[0].id16bit_cnt ;
    fifo1start = pbxcan->fifo1filteroff * 4;
    fifo1end = pbxcan->filtermap[1].id32mask_cnt
               + pbxcan->filtermap[1].id32bit_cnt
               + pbxcan->filtermap[1].id16mask_cnt
               + pbxcan->filtermap[1].id16bit_cnt ;
    if (hdr >= fifo0start && hdr < fifo0end)
    {
        *pbase = 0;
        ptr = 0;
    }
    else if (hdr >= fifo1start && hdr < fifo1end)
    {
        *pbase = pbxcan->fifo1filteroff;
        ptr = 1;
    }
    else
    {
        return RT_ERROR;
    }

    // below is the gradually decreasment of hdr to get the belonging of i (pbase).
    //      -> to see i belongs 32-mask, or 32-bit, or 16-mask, or 16-bit
    if (hdr > pbxcan->filtermap[ptr].id32mask_cnt)
    {
        hdr -= pbxcan->filtermap[ptr].id32mask_cnt;
        *pbase += pbxcan->filtermap[ptr].id32mask_cnt;
    }
    else
    {
        *pbase += hdr;
        *poff = 0;              //because 1 bank = 1 filter
        return RT_EOK;
    }

    if (hdr > pbxcan->filtermap[ptr].id32bit_cnt)
    {
        hdr -= pbxcan->filtermap[ptr].id32bit_cnt;
        *pbase += pbxcan->filtermap[ptr].id32bit_cnt / 2;
    }
    else
    {
        *pbase += hdr / 2;
        *poff = hdr % 2;        // because 1 bank = 2 filter
        return RT_EOK;
    }

    if (hdr > pbxcan->filtermap[ptr].id16mask_cnt)
    {
        hdr -= pbxcan->filtermap[ptr].id16mask_cnt;
        *pbase += pbxcan->filtermap[ptr].id16mask_cnt / 2;
    }
    else
    {
        *pbase += hdr / 2;
        *poff = hdr % 2;        // because 1 bank = 2 filter
        return RT_EOK;
    }

    if (hdr > pbxcan->filtermap[ptr].id16bit_cnt)
    {
        return RT_ERROR;
    }
    else
    {
        *pbase += hdr / 4;
        *poff = hdr % 4;        // because 1 bank = 4 filter
        return RT_EOK;
    }
}


static void calcandormask(rt_uint32_t *pmask, rt_uint32_t shift, rt_int32_t count)
{
    // rt_uint32_t tmpmask;
    // rt_uint32_t tmpmaskarray[BX_CAN_FILTER_MAX_ARRAY_SIZE] = {0,};
    // rt_int32_t i, tmp_shift, tmp_count = count;
    // i = 0;
    // while (tmp_count > 0)
    // {
    //     if (i >= 32)
    //     {
    //         tmpmaskarray[i] = 0xFFFFFFFF;
    //     }
    //     else
    //     {
    //         tmpmaskarray[i] = (0x01 << tmp_count) - 1;
    //     }
    //     tmp_count -= 32;
    //     i++;
    // };
    // tmp_count = i;
    // for (i = 0; i < tmp_count && i < BX_CAN_FILTER_MAX_ARRAY_SIZE; i++)
    // {
    //     tmpmask = tmpmaskarray[i];
    //     pmask[i] |= (rt_uint32_t)(tmpmask << shift);
    //     if (i < BX_CAN_FILTER_MAX_ARRAY_SIZE - 1)
    //     {
    //             tmp_shift = 32 - shift;
    //             if(tmp_shift < 0)
    //             {
    //                 //for smaller shift, this works.
    //                 //for bigger shift, this does not work.
    //                 // pmask[i + 1] |= (rt_uint32_t)(tmpmask << -tmp_shift);


    //                 pmask[i + 1] |= (rt_uint32_t)(tmpmask << -tmp_shift);
    //                 if(count - tmp_shift >= 32 && i < BX_CAN_FILTER_MAX_ARRAY_SIZE - 2)	//if some '1' is swallowed
    //                 {
    //                     //todo by guojianfei 20190119: should change to recuisive structure.
    //                     pmask[i + 2]|= (0x01 << (count - tmp_shift - 32)) - 1;
    //                     count = 0;
    //                 }

    //                 // wrong. when shift>32, the result is wrong.
    //                 // pmask[i + 1] |= (rt_uint32_t)(tmpmask >> (tmp_shift%32));
    //             }
    //             else
    //             {
    //                 pmask[i + 1] |= (rt_uint32_t)(tmpmask >> tmp_shift);
    //             }

    //             // wrong. when shift>32, the result is wrong.
    //             // pmask[i + 1] |= (rt_uint32_t)(tmpmask >> 32 - shift);


    //     }
    // }

    // changed by zhaoziqiu 20190130
    rt_uint8_t len = sizeof(rt_uint32_t) * 8;
    rt_uint32_t i;

    RT_ASSERT(shift + count <= len * BX_CAN_FILTER_MAX_ARRAY_SIZE);
        //throw out of range error
    {
        rt_uint32_t stRow,stCol,enRow,enCol;
        stRow = shift / len;
        stCol = shift % len;
        rt_uint32_t end = shift + count -1 ;
        enRow = end / len;
        enCol = end % len;
        if (stRow == enRow)
        {
            pmask[stRow] |= ((1<<(enCol+1))-1)&~((1<<(stCol))-1);
        }
        else
        {
            for( i=stRow+1 ; i<enRow ; i++){
                pmask[i] = 0xFFFFFFFF;
            }
            pmask[stRow] |= ~((1<<(stCol))-1);
            pmask[enRow] |= ((1<<(enCol+1))-1);
        }
    }
}


static void calcfiltermasks(struct stm_bxcan *pbxcan)
{
    rt_memset(&pbxcan->filtermask, 0, sizeof(pbxcan->filtermask));
    pbxcan->filtermask.id32maskshift[0] = 0;
    if (pbxcan->filtermap[0].id32mask_cnt)
    {
        calcandormask(pbxcan->filtermask.id32maskm, pbxcan->filtermask.id32maskshift[0],
                      pbxcan->filtermap[0].id32mask_cnt);
    }
    pbxcan->filtermask.id32maskshift[1] = pbxcan->fifo1filteroff * 4;
    if (pbxcan->filtermap[1].id32mask_cnt)
    {
        calcandormask(pbxcan->filtermask.id32maskm, pbxcan->filtermask.id32maskshift[1],
                      pbxcan->filtermap[1].id32mask_cnt);
    }
    pbxcan->filtermask.id32bitshift[0] = pbxcan->filtermask.id32maskshift[0] +
                                         pbxcan->filtermap[0].id32mask_cnt;
    if (pbxcan->filtermap[0].id32bit_cnt)
    {
        calcandormask(pbxcan->filtermask.id32bitm, pbxcan->filtermask.id32bitshift[0],
                      pbxcan->filtermap[0].id32bit_cnt);
    }
    pbxcan->filtermask.id32bitshift[1] = pbxcan->filtermask.id32maskshift[1] +
                                         pbxcan->filtermap[1].id32mask_cnt;
    if (pbxcan->filtermap[1].id32bit_cnt)
    {
        calcandormask(pbxcan->filtermask.id32bitm, pbxcan->filtermask.id32bitshift[1],
                      pbxcan->filtermap[1].id32bit_cnt);
    }
    pbxcan->filtermask.id16maskshift[0] = pbxcan->filtermask.id32bitshift[0] +
                                          pbxcan->filtermap[0].id32bit_cnt;
    if (pbxcan->filtermap[0].id16mask_cnt)
    {
        calcandormask(pbxcan->filtermask.id16maskm, pbxcan->filtermask.id16maskshift[0],
                      pbxcan->filtermap[0].id16mask_cnt);
    }
    pbxcan->filtermask.id16maskshift[1] = pbxcan->filtermask.id32bitshift[1] +
                                          pbxcan->filtermap[1].id32bit_cnt;
    if (pbxcan->filtermap[1].id16mask_cnt)
    {
        calcandormask(pbxcan->filtermask.id16maskm, pbxcan->filtermask.id16maskshift[1],
                      pbxcan->filtermap[1].id16mask_cnt);
    }
    pbxcan->filtermask.id16bitshift[0] = pbxcan->filtermask.id16maskshift[0] +
                                         pbxcan->filtermap[0].id16mask_cnt;
    if (pbxcan->filtermap[0].id16bit_cnt)
    {
        calcandormask(pbxcan->filtermask.id16bitm, pbxcan->filtermask.id16bitshift[0],
                      pbxcan->filtermap[0].id16bit_cnt);
    }
    pbxcan->filtermask.id16bitshift[1] = pbxcan->filtermask.id16maskshift[1] +
                                         pbxcan->filtermap[1].id16mask_cnt;
    if (pbxcan->filtermap[1].id16bit_cnt)
    {
        calcandormask(pbxcan->filtermask.id16bitm, pbxcan->filtermask.id16bitshift[1],
                      pbxcan->filtermap[1].id16bit_cnt);
    }
}

/**
 * @brief       get the filter index & base & off from the filter item
 *               , check it out in the stm32 registers to see whether it's already configured.
 * 
 * @param pbxcan 
 * @param pitem 
 * @param type 
 * @param base 
 * @param off 
 * @return rt_int32_t   return the filter index if found; returns -1 if not found.
 */
static rt_int32_t bxcanfindfilter(struct stm_bxcan *pbxcan, struct rt_can_filter_item *pitem,
                                  rt_int32_t type, rt_int32_t *base, rt_int32_t *off)
{
    rt_int32_t i;
    rt_uint32_t bits, thisid, thismask, shift, found;
    CAN_FilterRegister_TypeDef *pfilterreg;
    found = 0;
    switch (type)
    {
    case 3:
        shift = 3;
        for (i = 0; i < BX_CAN_MAX_FILTERS; i++)
        {
            bits = 0x01 << (i & 0x1F);
            // i 
            if (bits & (pbxcan->filtermask.id32maskm[i >> 5] & pbxcan->alocmask[i >> 5]))
            {
                bxcancalcbaseoff(pbxcan, i, base, off);
                pfilterreg = &((CAN_FilterRegister_TypeDef *)pbxcan->mfrbase)[*base];
                thisid = (rt_uint32_t)pitem->id << shift;
                thismask = (rt_uint32_t)pitem->mask << shift;
                if (pitem->ide)
                {
                    thisid |= CAN_ID_EXT;
                    thismask |= CAN_ID_EXT;
                }
                if (pitem->rtr)
                {
                    thisid |= CAN_RTR_REMOTE;
                    thismask |= CAN_RTR_REMOTE;
                }
                if (pfilterreg->FR1 == thisid && pfilterreg->FR2 == thismask)
                {
                    found = 1;
                    break;
                }
            }
        }
        break;
    case 2:
        shift = 3;
        for (i = 0; i < BX_CAN_MAX_FILTERS; i++)
        {
            bits = 0x01 << (i % 32);
            if (bits & (pbxcan->filtermask.id32bitm[i >> 5] & pbxcan->alocmask[i >> 5]))
            {
                bxcancalcbaseoff(pbxcan, i, base, off);
                pfilterreg = &((CAN_FilterRegister_TypeDef *)pbxcan->mfrbase)[*base];
                thisid = (rt_uint32_t)pitem->id << shift;
                if (pitem->ide)
                {
                    thisid |= CAN_ID_EXT;
                }
                if (pitem->rtr)
                {
                    thisid |= CAN_RTR_REMOTE;
                }
                if ((*off == 0 && pfilterreg->FR1 == thisid) ||
                        (*off == 1 && pfilterreg->FR2 == thisid)
                   )
                {
                    found = 1;
                    break;
                }
            }
        }
        break;
    case 1:
        shift = 5;
        for (i = 0; i < BX_CAN_MAX_FILTERS; i++)
        {
            bits = 0x01 << (i % 32);
            if (bits & (pbxcan->filtermask.id16maskm[i >> 5] & pbxcan->alocmask[i >> 5]))
            {
                bxcancalcbaseoff(pbxcan, i, base, off);
                pfilterreg = &((CAN_FilterRegister_TypeDef *)pbxcan->mfrbase)[*base];
                thisid = pitem->id << shift;
                if (pitem->rtr)
                {
                    thisid |= CAN_RTR_REMOTE << (shift - 2);
                }
                thismask = pitem->mask << shift;
                if (pitem->rtr)
                {
                    thismask |= CAN_RTR_REMOTE << (shift - 2);
                }
                if (*off == 0 && pfilterreg->FR1 == ((thisid & 0x0000FFFF) | ((thismask & 0x0000FFFF) << 16)) ||
                        *off == 1 && pfilterreg->FR2 == ((thisid & 0x0000FFFF) | ((thismask & 0x0000FFFF) << 16))
                   )
                {
                    found = 1;
                    break;
                }
            }
        }
        break;
    case 0:
        shift = 5;
        for (i = 0; i < BX_CAN_MAX_FILTERS; i++)
        {
            bits = 0x01 << (i % 32);
            if (bits & (pbxcan->filtermask.id16bitm[i >> 5] & pbxcan->alocmask[i >> 5]))
            {
                bxcancalcbaseoff(pbxcan, i, base, off);
                pfilterreg = &((CAN_FilterRegister_TypeDef *)pbxcan->mfrbase)[*base];
                thisid = pitem->id << shift;
                if (pitem->rtr)
                {
                    thisid |= CAN_RTR_REMOTE << (shift - 2);
                }
                if (*off < 2 && ((rt_uint16_t *)&pfilterreg->FR1)[*off & 0x01] == thisid ||
                        *off >= 2 && ((rt_uint16_t *)&pfilterreg->FR2)[*off & 0x01] == thisid)
                {
                    found = 1;
                    break;
                }
            }
        }
        break;
    }
    if (found)
    {
        return i;
    }
    return -1;
}
extern int __rt_ffs(int value);
static rt_err_t bxcanallocfilter(rt_uint32_t *pmask, rt_uint32_t *palocmask,
                                 rt_uint32_t count, rt_int32_t *hdr)
{
    rt_int32_t i;
    for (i = 0; i < count; i++)
    {
        rt_enter_critical();
        if ((pmask[i] & ~palocmask[i]) != 0)
        {
            *hdr = __rt_ffs(pmask[i] & ~palocmask[i]) - 1 + i * 32;
            palocmask[i] |= 0x01 << (*hdr % 0x1F);
            rt_exit_critical();
            return RT_EOK;
        }
        rt_exit_critical();
    }
    if (i >= count)
    {
        return RT_ENOMEM;
    }
    return RT_EOK;
}
static rt_err_t bxcanallocnewfilter(struct stm_bxcan *pbxcan, rt_int32_t actived,
                                    rt_int32_t type, rt_int32_t *hdr, rt_int32_t *base, rt_int32_t *off)
{
    rt_err_t res;
    *hdr = -1;
    switch (type)
    {
    case 0x03:
        res = bxcanallocfilter(pbxcan->filtermask.id32maskm, pbxcan->alocmask,
                               BX_CAN_FILTER_MAX_ARRAY_SIZE, hdr);
        break;
    case 0x02:
        res = bxcanallocfilter(pbxcan->filtermask.id32bitm, pbxcan->alocmask,
                               BX_CAN_FILTER_MAX_ARRAY_SIZE, hdr);
        break;
    case 0x01:
        res = bxcanallocfilter(pbxcan->filtermask.id16maskm, pbxcan->alocmask,
                               BX_CAN_FILTER_MAX_ARRAY_SIZE, hdr);
        break;
    case 0x00:
        res = bxcanallocfilter(pbxcan->filtermask.id16bitm, pbxcan->alocmask,
                               BX_CAN_FILTER_MAX_ARRAY_SIZE, hdr);
        break;
    }
    if (res != RT_EOK  || *hdr < 0)
    {
        return RT_ENOMEM;
    }
    bxcancalcbaseoff(pbxcan, *hdr, base, off);
    return RT_EOK;
}

/**
 * @brief    the only place that actually init filters
 * 
 * @param pbxcan 
 * @param pitem         the filter item to be set/changed
 * @param actived       
 * @return rt_err_t 
 */
static rt_err_t bxmodifyfilter(struct stm_bxcan *pbxcan, struct rt_can_filter_item *pitem, rt_uint32_t actived)
{
    rt_int32_t fcase;
    rt_err_t res;
    rt_int32_t hdr, fbase, foff;
    rt_uint32_t ID[2];
    rt_uint32_t shift;
    rt_uint32_t thisid;
    rt_uint32_t thismask;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    CAN_FilterRegister_TypeDef *pfilterreg;

    fcase = (pitem->mode | (pitem->ide << 1));

    // hdr = filter index among all. 
    //      if hdr == -1, not found among existing filters;
    //      otherwise, hdr == filter's index among all
    // fbase = filter bank index
    // foff = filter's offset in the bank

    hdr = bxcanfindfilter(pbxcan, pitem, fcase, &fbase, &foff);

    // if the filter item is not found among exsiting filters
    if (hdr < 0)
    {
        if (!actived)
        {
            return RT_EOK;
        }

        // if the filter is not found, and the hdr is also not set.
        // we allocate a new filter.
        else if (pitem->hdr == -1)
        {
            res = bxcanallocnewfilter(pbxcan, actived, fcase, &hdr, &fbase, &foff);
            if (res != RT_EOK)
            {
                return res;
            }
        }
        // if the filter is not found, but the hdr is already set.
        else if (pitem->hdr >= 0)
        {
            rt_enter_critical();
            res = bxcancalcbaseoff(pbxcan, pitem->hdr, &fbase, &foff);
            if (res != RT_EOK)
            {
                return res;
            }
            hdr = pitem->hdr;
            if (actived)
            {
                pbxcan->alocmask[hdr >> 5] |= 0x01 << (hdr % 0x1F);
            }
            rt_exit_critical();
        }
    }
    else
    {
        // if the filter is found, but we set it to non-actived
        if (!actived)
        {
            pitem->hdr = hdr;
        }

        // if the filter is found, and we still set it to actived.
        else if (hdr >= 0 && (pitem->hdr >= 0 || pitem->hdr == -1))
        {
            // re-align the hdr of the filter. (the index of the filter among all)
            pitem->hdr = hdr;
            return RT_EBUSY;
        }
    }

    // change the settings of STM32 registers according to the base and off.
    pitem->hdr =  hdr;
    pfilterreg = &((CAN_FilterRegister_TypeDef *)pbxcan->mfrbase)[fbase];
    ID[0] = pfilterreg->FR1;
    ID[1] = pfilterreg->FR2;
    CAN_FilterInitStructure.CAN_FilterNumber = (pfilterreg - &CAN1->sFilterRegister[0]);
    if (pitem->mode)
    {
        CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    }
    else
    {
        CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
    }
    if (pitem->ide)
    {
        CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    }
    else
    {
        CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;
    }
    switch (fcase)
    {
    case 0x03:
        if (actived)
        {
            shift = 3;
            thisid = (rt_uint32_t)pitem->id << shift;
            thismask = (rt_uint32_t)pitem->mask << shift;
            if (pitem->ide)
            {
                thisid |= CAN_ID_EXT;
                thismask |= CAN_ID_EXT;
            }
            if (pitem->rtr)
            {
                thisid |= CAN_RTR_REMOTE;
                thismask |= CAN_RTR_REMOTE;
            }
            ID[0] = thisid;
            ID[1] = thismask;
        }
        else
        {
            ID[0] = 0xFFFFFFFF;
            ID[1] = 0xFFFFFFFF;
        }
        break;
    case 0x02:
        if (actived)
        {
            shift = 3;
            thisid = (rt_uint32_t)pitem->id << shift;
            if (pitem->ide)
            {
                thisid |= CAN_ID_EXT;
            }
            if (pitem->rtr)
            {
                thisid |= CAN_RTR_REMOTE;
            }
            ID[foff] = thisid;
        }
        else
        {
            ID[foff] = 0xFFFFFFFF;
        }
        break;
    case 0x01:
        if (actived)
        {
            shift = 5;
            thisid = pitem->id << shift;
            if (pitem->rtr)
            {
                thisid |= CAN_RTR_REMOTE << (shift - 2);
            }
            thismask = pitem->mask << shift;
            if (pitem->rtr)
            {
                thismask |= CAN_RTR_REMOTE << (shift - 2);
            }
            ID[foff] = (thisid & 0x0000FFFF) | ((thismask & 0x0000FFFF) << 16);
        }
        else
        {
            ID[foff] = 0xFFFFFFFF;
        }
        break;
    case 0x00:
        if (actived)
        {
            shift = 5;
            thisid = pitem->id << shift;
            if (pitem->rtr)
            {
                thisid |= CAN_RTR_REMOTE << (shift - 2);
            }
            ((rt_uint16_t *) ID)[foff] = thisid;
        }
        else
        {
            ((rt_uint16_t *) ID)[foff] = 0xFFFF;
        }
        break;
    }
    if(pitem->ide)
    {
        CAN_FilterInitStructure.CAN_FilterIdHigh = (ID[0] & 0xFFFF0000) >> 16;
        CAN_FilterInitStructure.CAN_FilterIdLow = ID[0] & 0x0000FFFF;
        CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (ID[1] & 0xFFFF0000) >> 16;
        CAN_FilterInitStructure.CAN_FilterMaskIdLow = ((ID[1]) & 0x0000FFFF);
    }
    else
    {
        CAN_FilterInitStructure.CAN_FilterIdHigh = ((ID[1]) & 0x0000FFFF);
        CAN_FilterInitStructure.CAN_FilterIdLow = ID[0] & 0x0000FFFF;
        CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (ID[1] & 0xFFFF0000) >> 16;
        CAN_FilterInitStructure.CAN_FilterMaskIdLow = (ID[0] & 0xFFFF0000) >> 16;
    }
    if (fbase >= pbxcan->fifo1filteroff)
    {
        CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 1;
    }
    else
    {
        CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
    }
    if (ID[0] != 0xFFFFFFFF || ID[1] != 0xFFFFFFFF)
    {
        CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    }
    else
    {
        CAN_FilterInitStructure.CAN_FilterActivation = DISABLE;
    }
    if (!actived)
    {
        rt_enter_critical();
        pbxcan->alocmask[hdr >> 5] &= ~(0x01 << (hdr % 0x1F));
        rt_exit_critical();
    }

    //added by guojianfei 20190119. the only place actually initiallizing filters.
    CAN_FilterInit(&CAN_FilterInitStructure);
    return RT_EOK;
}
static rt_err_t setfilter(struct stm_bxcan *pbxcan, struct rt_can_filter_config *pconfig)
{
    struct rt_can_filter_item *pitem = pconfig->items;
    rt_uint32_t count = pconfig->count;
    rt_err_t res;
    while (count)
    {
        res = bxmodifyfilter(pbxcan, pitem, pconfig->actived);
        if (res != RT_EOK)
        {
            return res;
        }
        pitem++;
        count--;
    }
    return RT_EOK;
}
static rt_err_t configure(struct rt_can_device *can, struct can_configure *cfg)
{
    CAN_TypeDef *pbxcan;

    pbxcan = ((struct stm_bxcan *) can->parent.user_data)->reg;
    assert_param(IS_CAN_ALL_PERIPH(pbxcan));
    if (pbxcan == CAN1)
    {
        bxcan1_hw_init();
        bxcan_init(pbxcan, cfg->baud_rate, can->config.mode);
        bxcan1_filter_init(can);
    }
    else
    {
#ifdef USING_BXCAN2
        bxcan2_hw_init();
        bxcan_init(pbxcan, cfg->baud_rate, can->config.mode);
        bxcan2_filter_init(can);
#endif
    }
    return RT_EOK;
}
static rt_err_t control(struct rt_can_device *can, int cmd, void *arg)
{
    struct stm_bxcan *pbxcan;
    rt_uint32_t argval;
    NVIC_InitTypeDef  NVIC_InitStructure;

    pbxcan = (struct stm_bxcan *) can->parent.user_data;
    assert_param(pbxcan != RT_NULL);

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        argval = (rt_uint32_t) arg;
        if (argval == RT_DEVICE_FLAG_INT_RX)
        {
            NVIC_DisableIRQ(pbxcan->rcvirq0);
            NVIC_DisableIRQ(pbxcan->rcvirq1);
            CAN_ITConfig(pbxcan->reg, CAN_IT_FMP0 , DISABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_FF0 , DISABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_FOV0 , DISABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_FMP1 , DISABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_FF1 , DISABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_FOV1 , DISABLE);
        }
        else if (argval == RT_DEVICE_FLAG_INT_TX)
        {
            NVIC_DisableIRQ(pbxcan->sndirq);
            CAN_ITConfig(pbxcan->reg, CAN_IT_TME, DISABLE);
        }
        else if (argval == RT_DEVICE_CAN_INT_ERR)
        {
            CAN_ITConfig(pbxcan->reg, CAN_IT_BOF , DISABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_LEC , DISABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_ERR , DISABLE);
            NVIC_DisableIRQ(pbxcan->errirq);
        }
        break;
    case RT_DEVICE_CTRL_SET_INT:
        argval = (rt_uint32_t) arg;
        if (argval == RT_DEVICE_FLAG_INT_RX)
        {
            CAN_ITConfig(pbxcan->reg, CAN_IT_FMP0 , ENABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_FF0 , ENABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_FOV0 , ENABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_FMP1 , ENABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_FF1 , ENABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_FOV1 , ENABLE);
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
            NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
            NVIC_InitStructure.NVIC_IRQChannel = pbxcan->rcvirq0;
            NVIC_Init(&NVIC_InitStructure);
            NVIC_InitStructure.NVIC_IRQChannel = pbxcan->rcvirq1;
            NVIC_Init(&NVIC_InitStructure);
        }
        else if (argval == RT_DEVICE_FLAG_INT_TX)
        {
            CAN_ITConfig(pbxcan->reg, CAN_IT_TME, ENABLE);
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
            NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
            NVIC_InitStructure.NVIC_IRQChannel = pbxcan->sndirq;
            NVIC_Init(&NVIC_InitStructure);
        }
        else if (argval == RT_DEVICE_CAN_INT_ERR)
        {
            CAN_ITConfig(pbxcan->reg, CAN_IT_BOF , ENABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_LEC , ENABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_ERR , ENABLE);
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
            NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
            NVIC_InitStructure.NVIC_IRQChannel = pbxcan->errirq;
            NVIC_Init(&NVIC_InitStructure);
        }
        break;
    case RT_CAN_CMD_SET_FILTER:
        return setfilter(pbxcan, (struct rt_can_filter_config *) arg);
    case RT_CAN_CMD_SET_MODE:
        argval = (rt_uint32_t) arg;
        if (argval != RT_CAN_MODE_NORMAL ||
                argval != RT_CAN_MODE_LISEN ||
                argval != RT_CAN_MODE_LOOPBACK ||
                argval != RT_CAN_MODE_LOOPBACKANLISEN)
        {
            return RT_ERROR;
        }
        if (argval != can->config.mode)
        {
            can->config.mode = argval;
            return bxcan_set_mode(pbxcan->reg, argval);
        }
        break;
    case RT_CAN_CMD_SET_BAUD:
        argval = (rt_uint32_t) arg;
        if (argval != CAN1MBaud &&
                argval != CAN800kBaud &&
                argval != CAN500kBaud &&
                argval != CAN250kBaud &&
                argval != CAN125kBaud &&
                argval != CAN100kBaud &&
                argval != CAN50kBaud  &&
                argval != CAN20kBaud  &&
                argval != CAN10kBaud)
        {
            return RT_ERROR;
        }
        if (argval != can->config.baud_rate)
        {
            can->config.baud_rate = argval;
            return bxcan_set_baud_rate(pbxcan->reg, argval);
        }
        break;
    case RT_CAN_CMD_SET_PRIV:
        argval = (rt_uint32_t) arg;
        if (argval != RT_CAN_MODE_PRIV ||
                argval != RT_CAN_MODE_NOPRIV)
        {
            return RT_ERROR;
        }
        if (argval != can->config.privmode)
        {
            can->config.privmode = argval;
            return bxcan_set_privmode(pbxcan->reg, argval);
        }
        break;
    case RT_CAN_CMD_GET_STATUS:
    {
        rt_uint32_t errtype;
        errtype = pbxcan->reg->ESR;
        can->status.rcverrcnt = errtype >> 24;
        can->status.snderrcnt = (errtype >> 16 & 0xFF);
        can->status.errcode = errtype & 0x07;
        if (arg != &can->status)
        {
            rt_memcpy(arg, &can->status, sizeof(can->status));
        }
    }
    break;
    }

    return RT_EOK;
}
static int sendmsg(struct rt_can_device *can, const void *buf, rt_uint32_t boxno)
{
    CAN_TypeDef *pbxcan;
    struct rt_can_msg *pmsg = (struct rt_can_msg *) buf;

    pbxcan = ((struct stm_bxcan *) can->parent.user_data)->reg;
    assert_param(IS_CAN_ALL_PERIPH(pbxcan));

    pbxcan->sTxMailBox[boxno].TIR &= TMIDxR_TXRQ;
    if (pmsg->ide == RT_CAN_STDID)
    {
        assert_param(IS_CAN_STDID(pmsg->id));
        pbxcan->sTxMailBox[boxno].TIR |= ((pmsg->id << 21) | \
                                          (pmsg->rtr << 1));
    }
    else
    {
        assert_param(IS_CAN_EXTID(pmsg->id));
        pbxcan->sTxMailBox[boxno].TIR |= ((pmsg->id << 3) | \
                                          (pmsg->ide << 2) | \
                                          (pmsg->rtr << 1));
    }

    pmsg->len &= (uint8_t)0x0000000F;
    pbxcan->sTxMailBox[boxno].TDTR &= (uint32_t)0xFFFFFFF0;
    pbxcan->sTxMailBox[boxno].TDTR |= pmsg->len;

    pbxcan->sTxMailBox[boxno].TDLR = (((uint32_t)pmsg->data[3] << 24) |
                                      ((uint32_t)pmsg->data[2] << 16) |
                                      ((uint32_t)pmsg->data[1] << 8) |
                                      ((uint32_t)pmsg->data[0]));
    if (pmsg->len > 4)
    {
        pbxcan->sTxMailBox[boxno].TDHR = (((uint32_t)pmsg->data[7] << 24) |
                                          ((uint32_t)pmsg->data[6] << 16) |
                                          ((uint32_t)pmsg->data[5] << 8) |
                                          ((uint32_t)pmsg->data[4]));
    }
    pbxcan->sTxMailBox[boxno].TIR |= TMIDxR_TXRQ;

    return RT_EOK;
}
static int recvmsg(struct rt_can_device *can, void *buf, rt_uint32_t boxno)
{
    CAN_TypeDef *pbxcan;
    struct rt_can_msg *pmsg = (struct rt_can_msg *) buf;

    pbxcan = ((struct stm_bxcan *) can->parent.user_data)->reg;
    assert_param(IS_CAN_ALL_PERIPH(pbxcan));
    assert_param(IS_CAN_FIFO(boxno));
    pmsg->ide = ((uint8_t)0x04 & pbxcan->sFIFOMailBox[boxno].RIR) >> 2;
    if (pmsg->ide == CAN_Id_Standard)
    {
        pmsg->id = (uint32_t)0x000007FF & (pbxcan->sFIFOMailBox[boxno].RIR >> 21);
    }
    else
    {
        pmsg->id = (uint32_t)0x1FFFFFFF & (pbxcan->sFIFOMailBox[boxno].RIR >> 3);
    }

    pmsg->rtr = (uint8_t)((0x02 & pbxcan->sFIFOMailBox[boxno].RIR) >> 1);
    pmsg->len = (uint8_t)0x0F & pbxcan->sFIFOMailBox[boxno].RDTR;
    pmsg->data[0] = (uint8_t)0xFF & pbxcan->sFIFOMailBox[boxno].RDLR;
    pmsg->data[1] = (uint8_t)0xFF & (pbxcan->sFIFOMailBox[boxno].RDLR >> 8);
    pmsg->data[2] = (uint8_t)0xFF & (pbxcan->sFIFOMailBox[boxno].RDLR >> 16);
    pmsg->data[3] = (uint8_t)0xFF & (pbxcan->sFIFOMailBox[boxno].RDLR >> 24);
    if (pmsg->len > 4)
    {
        pmsg->data[4] = (uint8_t)0xFF & pbxcan->sFIFOMailBox[boxno].RDHR;
        pmsg->data[5] = (uint8_t)0xFF & (pbxcan->sFIFOMailBox[boxno].RDHR >> 8);
        pmsg->data[6] = (uint8_t)0xFF & (pbxcan->sFIFOMailBox[boxno].RDHR >> 16);
        pmsg->data[7] = (uint8_t)0xFF & (pbxcan->sFIFOMailBox[boxno].RDHR >> 24);
    }
    pmsg->hdr = (uint8_t)0xFF & (pbxcan->sFIFOMailBox[boxno].RDTR >> 8);
    if (boxno) pmsg->hdr += ((struct stm_bxcan *) can->parent.user_data)->fifo1filteroff * 4;
    return RT_EOK;
}

static const struct rt_can_ops canops =
{
    configure,
    control,
    sendmsg,
    recvmsg,
};
#ifdef USING_BXCAN1
static struct stm_bxcan bxcan1data =
{
    CAN1,

    // void *mfrbase; the base addr of 28 filterbanks registers. 
    // (each element of the array is consists of FR1 and FR2)
    (void *) &CAN1->sFilterRegister[0],     

    CAN1_TX_IRQn,
    CAN1_RX0_IRQn,
    CAN1_RX1_IRQn,
    CAN1_SCE_IRQn,

    {
        0,
    },                      //struct stm_bxcanfilter_masks filtermask;

    {0, 0, 0, 0},           //rt_uint32_t alocmask[BX_CAN_FILTER_MAX_ARRAY_SIZE];
    
    BX_CAN2_FMRSTART,       //count of filter banks used by CAN1 (FIFO0+FIFO1)
    BX_CAN2_FMRSTART/2,     //before this value, filter banks using FIFO0; after this value, filter banks using FIFO1


    {
        //! notice: the total count of banks below should not be greater than the count of all banks used by CANx
        //filter banks using FIFO0
        {
            0,      // = 0 banks.  count of 32-bit mask filters. 1 filterbank = 1 filter
            0,      // = 0 banks.  count of 32-bit list filters. 1 filterbank = 2 filters
            2,      // = 1 banks.  count of 16-bit mask filters. 1 filterbank = 2 filters
            44,     // = 11 banks.  count of 16-bit list filters. 1 filterbank = 4 filters
        },

        //filter banks using FIFO1
        {
            0,      // = 0 banks.  count of 32-bit mask filters. 1 filterbank = 1 filter
            0,      // = 0 banks.  count of 32-bit list filters. 1 filterbank = 2 filters
            2,      // = 1 banks.  count of 16-bit mask filters. 1 filterbank = 2 filters
            44,     // = 11 banks.  count of 16-bit list filters. 1 filterbank = 4 filters
        },
    },
};
struct rt_can_device bxcan1;
void CAN1_RX0_IRQHandler(void)
{
    rt_interrupt_enter();
    if (CAN1->RF0R & 0x03)
    {
        if ((CAN1->RF0R & CAN_RF0R_FOVR0) != 0)
        {
            CAN1->RF0R = CAN_RF0R_FOVR0;
            rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_RXOF_IND | 0 << 8);
        }
        else
        {
            rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_RX_IND | 0 << 8);
        }
        CAN1->RF0R |= CAN_RF0R_RFOM0;
    }
    rt_interrupt_leave();
}
void CAN1_RX1_IRQHandler(void)
{
    rt_interrupt_enter();
    if (CAN1->RF1R & 0x03)
    {
        if ((CAN1->RF1R & CAN_RF1R_FOVR1) != 0)
        {
            CAN1->RF1R = CAN_RF1R_FOVR1;
            rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_RXOF_IND | 1 << 8);
        }
        else
        {
            rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_RX_IND | 1 << 8);
        }
        CAN1->RF1R |= CAN_RF1R_RFOM1;
    }
    rt_interrupt_leave();
}
void CAN1_TX_IRQHandler(void)
{
    rt_uint32_t state;
    rt_interrupt_enter();
    if (CAN1->TSR & (CAN_TSR_RQCP0))
    {
        state =  CAN1->TSR & (CAN_TSR_RQCP0 | CAN_TSR_TXOK0 | CAN_TSR_TME0);
        CAN1->TSR |= CAN_TSR_RQCP0;
        if (state == (CAN_TSR_RQCP0 | CAN_TSR_TXOK0 | CAN_TSR_TME0))
        {
            rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_TX_DONE | 0 << 8);
        }
        else
        {
            rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_TX_FAIL | 0 << 8);
        }
    }
    if (CAN1->TSR & (CAN_TSR_RQCP1))
    {
        state =  CAN1->TSR & (CAN_TSR_RQCP1 | CAN_TSR_TXOK1 | CAN_TSR_TME1);
        CAN1->TSR |= CAN_TSR_RQCP1;
        if (state == (CAN_TSR_RQCP1 | CAN_TSR_TXOK1 | CAN_TSR_TME1))
        {
            rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_TX_DONE | 1 << 8);
        }
        else
        {
            rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_TX_FAIL | 1 << 8);
        }
    }
    if (CAN1->TSR & (CAN_TSR_RQCP2))
    {
        state =  CAN1->TSR & (CAN_TSR_RQCP2 | CAN_TSR_TXOK2 | CAN_TSR_TME2);
        CAN1->TSR |= CAN_TSR_RQCP2;
        if (state == (CAN_TSR_RQCP2 | CAN_TSR_TXOK2 | CAN_TSR_TME2))
        {
            rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_TX_DONE | 2 << 8);
        }
        else
        {
            rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_TX_FAIL | 2 << 8);
        }
    }
    rt_interrupt_leave();
}
void CAN1_SCE_IRQHandler(void)
{
    rt_uint32_t errtype, errLECtype;
    errtype = CAN1->ESR;
    rt_interrupt_enter();
    // if (errtype & 0x70 && bxcan1.status.lasterrtype == (errtype & 0x70))
    if (errtype & 0x70)
    {
    	errLECtype = (errtype & 0x70) >> 4;
        switch (errLECtype)
        {
        case RT_CAN_BUS_BIT_PAD_ERR:
            bxcan1.status.bitpaderrcnt++;
            break;
        case RT_CAN_BUS_FORMAT_ERR:
            bxcan1.status.formaterrcnt++;
            break;
        case RT_CAN_BUS_ACK_ERR:
            bxcan1.status.ackerrcnt++;
            break;
        case RT_CAN_BUS_IMPLICIT_BIT_ERR:
        case RT_CAN_BUS_EXPLICIT_BIT_ERR:
            bxcan1.status.biterrcnt++;
            break;
        case RT_CAN_BUS_CRC_ERR:
            bxcan1.status.crcerrcnt++;
            break;
        }
        bxcan1.status.lasterrtype = errtype & 0x70;
        CAN1->ESR &= ~0x70;
    }
    bxcan1.status.rcverrcnt = errtype >> 24;
    bxcan1.status.snderrcnt = (errtype >> 16 & 0xFF);
    bxcan1.status.errcode = errtype & 0x07;
    CAN1->MSR |= CAN_MSR_ERRI;
    rt_interrupt_leave();
}
#endif /*USING_BXCAN1*/

#ifdef USING_BXCAN2
static struct stm_bxcan bxcan2data =
{
    CAN2,
    (void *) &CAN1->sFilterRegister[BX_CAN2_FMRSTART],
    CAN2_TX_IRQn,
    CAN2_RX0_IRQn,
    CAN2_RX1_IRQn,
    CAN2_SCE_IRQn,
    {
        0,
    }
    {0, 0},
    BX_CAN_FMRNUMBER - BX_CAN2_FMRSTART,
    7,
    {
        {
            0,
            0,
            2,
            24,
        },
        {
            0,
            0,
            2,
            24,
        },
    },
};

struct rt_can_device bxcan2;
void CAN2_RX0_IRQHandler(void)
{
    rt_interrupt_enter();
    if (CAN2->RF0R & 0x03)
    {
        if ((CAN2->RF0R & CAN_RF0R_FOVR0) != 0)
        {
            CAN2->RF0R = CAN_RF0R_FOVR0;
            rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_RXOF_IND | 0 << 8);
        }
        else
        {
            rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_RX_IND | 0 << 8);
        }
        CAN2->RF0R |= CAN_RF0R_RFOM0;
    }
    rt_interrupt_leave();
}
void CAN2_RX1_IRQHandler(void)
{
    rt_interrupt_enter();
    if (CAN2->RF1R & 0x03)
    {
        if ((CAN2->RF1R & CAN_RF1R_FOVR1) != 0)
        {
            CAN2->RF1R = CAN_RF1R_FOVR1;
            rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_RXOF_IND | 1 << 8);
        }
        else
        {
            rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_RX_IND | 1 << 8);
        }
        CAN2->RF1R |= CAN_RF1R_RFOM1;
    }
    rt_interrupt_leave();
}
void CAN2_TX_IRQHandler(void)
{
    rt_uint32_t state;
    rt_interrupt_enter();
    if (CAN2->TSR & (CAN_TSR_RQCP0))
    {
        state =  CAN2->TSR & (CAN_TSR_RQCP0 | CAN_TSR_TXOK0 | CAN_TSR_TME0);
        CAN2->TSR |= CAN_TSR_RQCP0;
        if (state == (CAN_TSR_RQCP0 | CAN_TSR_TXOK0 | CAN_TSR_TME0))
        {
            rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_TX_DONE | 0 << 8);
        }
        else
        {
            rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_TX_FAIL | 0 << 8);
        }
    }
    if (CAN2->TSR & (CAN_TSR_RQCP1))
    {
        state =  CAN2->TSR & (CAN_TSR_RQCP1 | CAN_TSR_TXOK1 | CAN_TSR_TME1);
        CAN2->TSR |= CAN_TSR_RQCP1;
        if (state == (CAN_TSR_RQCP1 | CAN_TSR_TXOK1 | CAN_TSR_TME1))
        {
            rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_TX_DONE | 1 << 8);
        }
        else
        {
            rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_TX_FAIL | 1 << 8);
        }
    }
    if (CAN2->TSR & (CAN_TSR_RQCP2))
    {
        state =  CAN2->TSR & (CAN_TSR_RQCP2 | CAN_TSR_TXOK2 | CAN_TSR_TME2);
        CAN2->TSR |= CAN_TSR_RQCP2;
        if (state == (CAN_TSR_RQCP2 | CAN_TSR_TXOK2 | CAN_TSR_TME2))
        {
            rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_TX_DONE | 2 << 8);
        }
        else
        {
            rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_TX_FAIL | 2 << 8);
        }
    }
    rt_interrupt_leave();
}
void CAN2_SCE_IRQHandler(void)
{
    rt_uint32_t errtype;
    errtype = CAN2->ESR;
    rt_interrupt_enter();
    if (errtype & 0x70 && bxcan2.status.lasterrtype == (errtype & 0x70))
    {
        switch ((errtype & 0x70) >> 4)
        {
        case RT_CAN_BUS_BIT_PAD_ERR:
            bxcan2.status.bitpaderrcnt++;
            break;
        case RT_CAN_BUS_FORMAT_ERR:
            bxcan2.status.formaterrcnt++;
            break;
        case RT_CAN_BUS_ACK_ERR:
            bxcan2.status.ackerrcnt++;
            break;
        case RT_CAN_BUS_IMPLICIT_BIT_ERR:
        case RT_CAN_BUS_EXPLICIT_BIT_ERR:
            bxcan2.status.biterrcnt++;
            break;
        case RT_CAN_BUS_CRC_ERR:
            bxcan2.status.crcerrcnt++;
            break;
        }
        bxcan2.status.lasterrtype = errtype & 0x70;
        CAN2->ESR &= ~0x70;
    }
    bxcan2.status.rcverrcnt = errtype >> 24;
    bxcan2.status.snderrcnt = (errtype >> 16 & 0xFF);
    bxcan2.status.errcode = errtype & 0x07;
    CAN2->MSR |= CAN_MSR_ERRI;
    rt_interrupt_leave();
}
#endif /*USING_BXCAN2*/

int stm32_bxcan_init(void)
{

#ifdef USING_BXCAN1
    //changed by guojianfei 20190119 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 , ENABLE);
    CAN_DeInit(CAN1);
    bxcan1.config.baud_rate = CAN500kBaud;
    bxcan1.config.msgboxsz = 16;
    bxcan1.config.sndboxnumber = 3;
    bxcan1.config.mode = RT_CAN_MODE_NORMAL;
    bxcan1.config.privmode = 0;
    bxcan1.config.ticks = 50;   // commented by guojianfei 20190119: meant to config can driver timer, but not used.
#ifdef RT_CAN_USING_HDR
    bxcan1.config.maxhdr = BX_CAN2_FMRSTART * 4;
#endif
    rt_hw_can_register(&bxcan1, "bxcan1", &canops, &bxcan1data);
#endif

#ifdef USING_BXCAN2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
#ifndef USING_BXCAN1
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 , ENABLE);
#endif
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
    CAN_DeInit(CAN2);
    bxcan2.config.baud_rate = CAN1MBaud;
    bxcan2.config.msgboxsz = 16;
    bxcan2.config.sndboxnumber = 3;
    bxcan2.config.mode = RT_CAN_MODE_NORMAL;
    bxcan2.config.privmode = 0;
    bxcan2.config.ticks = 50;
#ifdef RT_CAN_USING_HDR
    bxcan2.config.maxhdr = (BX_CAN_FMRNUMBER - BX_CAN2_FMRSTART) * 4;
#endif
    rt_hw_can_register(&bxcan2, "bxcan2", &canops, &bxcan2data);
#endif
    return RT_EOK;
}
INIT_BOARD_EXPORT(stm32_bxcan_init);

#endif /*RT_USING_CAN*/
