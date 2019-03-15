#include "can_example.h"
#include <rtdevice.h>
#include <rtthread.h>

#include <finsh.h>

#define DONE (0)
#define ERR (-1)
#define JARVIS_CAN_HANLDER_NUM (20)

/* debug funciton */
#define JARVIS_CAN_DEBUG
#define JARVIS_CAN_DEBUG_PING
// #define JARVIS_CAN_DEBUG_THREAD
#define JARVIS_CAN_DEBUG_PING_THREAD_STACK_SIZE (512)
#define JARVIS_CAN_DEBUG_PING_THREAD_PRIO (22)


#ifdef JARVIS_CAN_DEBUG
#define LOG_TAG              "Can"      // log tag of this module.
#define LOG_LVL              LOG_LVL_DBG    // minimum log level of this module.
#include <ulog.h>
#endif

#if defined JARVIS_CAN_DEBUG && defined JARVIS_CAN_DEBUG_PING
#ifndef RT_USING_HEAP
static struct rt_semaphore PingSndLock;
#endif
static struct rt_semaphore* pPIngSndLock;
#endif

/* -----------------------  static variables  ----------------------- */
//! notice: static means local. Locally visible for this file, globally invisible for other files.
static rt_can_t pcan;
static rt_err_t can_rx_ind(rt_device_t dev, rt_size_t original_size);

static struct rt_can_msg rcv_msg;
static struct rt_can_msg snd_msg;

static struct rt_can_msg snd_ping_rp_msg;
static uint8_t ping_request_data[8] = {0xAA, 0xBB, 0x00, 0xA5, 0x00, 0x00, 0xCC, 0xDD};
static uint8_t ping_response_data[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0xAA, 0xBB, 0xCC, 0xDD};

#if defined JARVIS_CAN_DEBUG && defined JARVIS_CAN_DEBUG_THREAD
static struct rt_can_msg handler_msg;
static struct rt_messagequeue eCanMq;
static rt_uint8_t CANmqPoll[64];
#endif



static eCanRxNodeHandlerPair_t node_handlers[JARVIS_CAN_HANLDER_NUM];
static rt_uint32_t handlers_count = 0;

typedef enum {
    PING_KINCO_T = 0,
    PING_STM32_T = 1,
} ping_class;


typedef enum {
    PING_KINCO = 0x1 <<0,
    PING_STM32 = 0x1 <<1,
} ping_request_bit;

static struct ping_requet_t {
    rt_uint32_t flag;
    rt_tick_t delay[3];
} ping;

int xCanInit(void)
{
    #if defined JARVIS_CAN_DEBUG && defined JARVIS_CAN_DEBUG_THREAD
    rt_mq_init(&eCanMq, "CAN1 mq", &CANmqPoll, sizeof(struct rt_can_msg), sizeof(CANmqPoll), RT_IPC_FLAG_FIFO);
    #endif

    // extern struct rt_can_device bxcan1;
    pcan = (rt_can_t)rt_device_find("bxcan1");
    if(pcan == RT_NULL)
    {
    	return -1;
    }

    // pcan->config.baud_rate = CAN500kBaud;
    pcan->config.baud_rate = CAN500kBaud;
    pcan->config.mode = RT_CAN_MODE_NORMAL;
    pcan->config.msgboxsz = 2;
    // pcan->parent.init(&pcan->parent); // not ok!
    pcan->ops->configure(pcan, &pcan->config);
    
    if(rt_device_open(&pcan->parent, (RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX)) != RT_EOK )
    {
        // if open fails.
        return ERR;
    }
    else
    {
        pcan->parent.rx_indicate = can_rx_ind;
    }

    #if defined JARVIS_CAN_DEBUG && defined JARVIS_CAN_DEBUG_PING
    struct rt_can_filter_config cfg;
    struct rt_can_filter_item item;

    item.ide = 0;
    item.mode = 0;
    item.id = 0x599;
    item.rtr = 0;
    item.hdr = -1;

    cfg.items = &item;
    cfg.count = 1;
    cfg.actived = 1;

    pcan->ops->control(pcan, RT_CAN_CMD_SET_FILTER, &cfg);

    ping.flag = 0;
    
    #ifdef RT_USING_HEAP
    pPIngSndLock =  rt_sem_create("Jarvis Can Ping", 0, RT_IPC_FLAG_FIFO);
    #else
    rt_sem_init(&PingSndLock, "Jarvis Can Ping", 0, RT_IPC_FLAG_FIFO);
    pPIngSndLock = &PingSndLock;
    #endif

    #endif

    return DONE;
}
INIT_DEVICE_EXPORT(xCanInit);

static void _handle_msg(struct rt_can_msg* msg)
{
    rt_uint32_t id = msg->id;
    rt_uint8_t i = 0;
    rt_uint8_t ping_got_response = 0;
    /*-----------------------  ping functions  -----------------------*/
    // !notice:  just for ping, below is NOT the formal code !!!!! 
    #if defined JARVIS_CAN_DEBUG && defined JARVIS_CAN_DEBUG_PING
    if(id == 0x599)
    {
        if(ping.flag & PING_STM32) //if ping, calc delay
        {
        	ping.delay[PING_STM32_T] = rt_tick_get() - ping.delay[PING_STM32_T];
            rt_kprintf("\n===========================\nping stm32: \n");
            ping.flag &=~ PING_STM32;
            rt_kprintf("Delay: %.1f ms. \n",FROM_TICK_TO_MS(ping.delay[PING_STM32_T]));
            ping_got_response = 1;
        }
        else //if not pinging, send response.
        {
            snd_ping_rp_msg.len = 8;
            snd_ping_rp_msg.ide = 0;
            snd_ping_rp_msg.rtr = 0;
            snd_ping_rp_msg.id = 0x599;
            for(uint8_t i = 0; i < 8; i++)
            {
                snd_ping_rp_msg.data[i] = ping_response_data[i];
            }
            rt_sem_release(pPIngSndLock);
            return;
        }
    }
    else if((ping.flag & PING_KINCO) && (id == 0x581 || id == 0x582))
    {
    	ping.delay[PING_KINCO_T] = rt_tick_get() - ping.delay[PING_KINCO_T];
        rt_kprintf("\n===========================\nping kinco: \n");
        ping.flag &=~ PING_KINCO;
        rt_kprintf("Delay: %.1f ms. \n",FROM_TICK_TO_MS(ping.delay[PING_KINCO_T]));
        ping_got_response = 1;
    }
    
    if(ping_got_response)
    {
        rt_kprintf("CAN Rcv from id: %x \n", id);
        for(i = 0; i<8;i++)
        {
            rt_kprintf("[%x]", msg->data[i]);
        }
        rt_kprintf("\n");
        rt_kprintf("===========================\n\n");
        return;
    }
    #endif


    /*-----------------------  formal functions  -----------------------*/
    /* call the registered handler function */
    for(i = 0; i < handlers_count; i++)
    {
        if(id == node_handlers[i].id)
        {
            /* calls the registered can message handler  */
            node_handlers[i].handler(id, msg->len, msg->data);

            break;
        }
    }
}

static rt_err_t can_rx_ind(rt_device_t dev, rt_size_t original_size)
{
    rt_int32_t size = original_size;
    while(size--)
    {
        
        // size -= sizeof(struct rt_can_msg);
        pcan->parent.read(&pcan->parent,0, &rcv_msg, sizeof(struct rt_can_msg) );
        #if defined JARVIS_CAN_DEBUG && defined JARVIS_CAN_DEBUG_THREAD
        rt_mq_send(&eCanMq, (void*)&rcv_msg, sizeof(struct rt_can_msg));
        #else
        _handle_msg(&rcv_msg);
        #endif
    }
    return RT_EOK;
}



/*-----------------------  port for other stacks/packages  -----------------------*/
int xCanTransmit(struct rt_can_msg* msg)
{
    if(pcan->parent.write(&pcan->parent, 0, msg, sizeof(struct rt_can_msg)))
    {
        return DONE;
    }
    else
    {
        return ERR;
    }
}


int xCanNodeRegister(eCanNodeRegisterCfg_t* cfg)
{
    rt_uint8_t i  = 0;
    /*-----------------------  hanlders register  -----------------------*/
    for(i = 0; i<cfg->handler_cfg.nodes_count; i++)
    {
        if(handlers_count >= JARVIS_CAN_HANLDER_NUM)
        {
            return ERR;
        }
        node_handlers[handlers_count].id = cfg->handler_cfg.node_handlers[i].id;
        node_handlers[handlers_count].handler = cfg->handler_cfg.node_handlers[i].handler;
        handlers_count++;
    }
    
    /*-----------------------  filters register  -----------------------*/
    if(pcan->ops->control(pcan, RT_CAN_CMD_SET_FILTER, &cfg->filer_cfg)!=RT_EOK)
    {
        return ERR;
    }
    
    return DONE;
}

#if defined JARVIS_CAN_DEBUG && defined JARVIS_CAN_DEBUG_THREAD

void vCanRcvRcvPoll(void)
{
    while(1){
        rt_mq_recv(&eCanMq, (void*)&handler_msg, sizeof(struct rt_can_msg), RT_WAITING_FOREVER);
        _handle_msg(&handler_msg);
    }
}
#endif

#if defined JARVIS_CAN_DEBUG && defined JARVIS_CAN_DEBUG_PING

//! @notice: ventus: idle can not be suspended, for the system needs one thread ready minimum.
// static void CAN_idle_hook(void)
// {
//     if(_flag_to_send)
//     {
//         _flag_to_send = 0;
//         xCanTransmit(&snd_ping_rp_msg);
//     }
    
// }

void jarvis_can_ping_thread_entry(void* parameter)
{   
    while(1){
        if(rt_sem_take(pPIngSndLock, RT_WAITING_FOREVER) == RT_EOK)
        {
            xCanTransmit(&snd_ping_rp_msg);
        }
    }
}

#ifndef RT_USING_HEAP
static struct rt_thread thread_JarvisCanPing;
static U8 thread_JarvisCanPingStack[JARVIS_CAN_DEBUG_PING_THREAD_STACK_SIZE];
#endif

int jarvis_can_ping_thread_system_init(void)
{
    #ifdef RT_USING_HEAP
    rt_thread_t tid = rt_thread_create("Jarvis Can Ping",
                        jarvis_can_ping_thread_entry,
                        RT_NULL,
                        JARVIS_CAN_DEBUG_PING_THREAD_STACK_SIZE,
                        JARVIS_CAN_DEBUG_PING_THREAD_PRIO,
                        20);
    rt_thread_startup(tid);
    #else
    rt_thread_init(&thread_JarvisCanPing,
                "Jarvis Can PIng",
                jarvis_can_ping_thread_entry,
                RT_NULL,
                (void*)thread_JarvisCanPingStack,
                sizeof(thread_JarvisCanPingStack),
                JARVIS_CAN_DEBUG_PING_THREAD_PRIO,
                20);
    #endif
    return 0;
}
INIT_APP_EXPORT(jarvis_can_ping_thread_system_init)

void ping_kinco(void)
{
	uint8_t data[8] = {0x40, 0x63, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
	snd_msg.len = 8;
	snd_msg.ide = 0;
	snd_msg.rtr = 0;
	snd_msg.id = 0x601;
	for(uint8_t i = 0; i < 8; i++)
	{
		snd_msg.data[i] = data[i];
	}
    ping.flag |= PING_KINCO;
    ping.delay[PING_KINCO_T] = rt_tick_get();
	xCanTransmit(&snd_msg);
}
MSH_CMD_EXPORT(ping_kinco, test can when kinco chassis is on the bus.)

void ping_stm32(void)
{
	snd_msg.len = 8;
	snd_msg.ide = 0;
	snd_msg.rtr = 0;
	snd_msg.id = 0x599;
	for(uint8_t i = 0; i < 8; i++)
	{
		snd_msg.data[i] = ping_request_data[i];
	}
    ping.flag |= PING_STM32;
    ping.delay[PING_STM32_T] = rt_tick_get();
	xCanTransmit(&snd_msg);
}
MSH_CMD_EXPORT(ping_stm32, test can when another stm32 is on the bus.)

#endif