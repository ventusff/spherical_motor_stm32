#include "can_example.h"
#include <rtdevice.h>
#include <rtthread.h>

#include <finsh.h>

#define DONE (0)
#define ERR (-1)
#define JARVIS_CAN_HANLDER_NUM (20)

static rt_can_t pcan;
static rt_err_t can_rx_ind(rt_device_t dev, rt_size_t original_size);

static struct rt_can_msg rcv_msg;
static struct rt_can_msg snd_msg;
static struct rt_can_msg handler_msg;

static struct rt_messagequeue eCanMq;
static rt_uint8_t CANmqPoll[64];

static eCanRxHandlerPair_t handlers[JARVIS_CAN_HANLDER_NUM];
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
    rt_mq_init(&eCanMq, "CAN1 mq", &CANmqPoll, \ 
        sizeof(struct rt_can_msg), sizeof(CANmqPoll), RT_IPC_FLAG_FIFO);
    

    // extern struct rt_can_device bxcan1;
    pcan = (rt_can_t)rt_device_find("bxcan1");

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
    return DONE;
}
INIT_APP_EXPORT(xCanInit);

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

uint8_t ping_request_data[8] = {0xAA, 0xBB, 0x00, 0xA5, 0x00, 0x00, 0xCC, 0xDD};
uint8_t ping_response_data[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0xAA, 0xBB, 0xCC, 0xDD};
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

static rt_err_t can_rx_ind(rt_device_t dev, rt_size_t original_size)
{
    rt_int32_t size = original_size;
    while(size--)
    {
        
        // size -= sizeof(struct rt_can_msg);
        pcan->parent.read(&pcan->parent,0, &rcv_msg, sizeof(struct rt_can_msg) );
        rt_mq_send(&eCanMq, (void*)&rcv_msg, sizeof(struct rt_can_msg));
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


int xCanDeviceRegister(eCanDeviceRegisterCfg_t* cfg)
{
    rt_uint8_t i  = 0;
    /*-----------------------  hanlders register  -----------------------*/
    for(i = 0; i<cfg->handler_cfg.count; i++)
    {
        if(handlers_count >= JARVIS_CAN_HANLDER_NUM)
        {
            return ERR;
        }
        handlers[handlers_count].id = cfg->handler_cfg.pairs[i].id;
        handlers[handlers_count].handler = cfg->handler_cfg.pairs[i].handler;
        handlers_count++;
    }
    
    /*-----------------------  filters register  -----------------------*/
    if(pcan->ops->control(pcan, RT_CAN_CMD_SET_FILTER, &cfg->filer_cfg)!=RT_EOK)
    {
        return ERR;
    }
    
    return DONE;
}

void vCanRcvRcvPoll(void)
{
    rt_uint32_t id;
    rt_uint8_t i = 0;
    rt_uint8_t ping_got_response = 0;

    while(1){
        rt_mq_recv(&eCanMq, (void*)&handler_msg, sizeof(struct rt_can_msg), RT_WAITING_FOREVER);
        id = handler_msg.id;

        /*-----------------------  ping functions  -----------------------*/
        if(id == 0x599)
        {
            if(ping.flag & PING_STM32) //if ping, calc delay
            {
                rt_kprintf("\n===========================\nping stm32: \n");
                ping.flag &=~ PING_STM32;
                ping.delay[PING_STM32_T] = rt_tick_get() - ping.delay[PING_STM32_T];
                rt_kprintf("Delay: %.1f ms. \n",FROM_TICK_TO_MS(ping.delay[PING_STM32_T]));
                ping_got_response = 1;
            }
            else //if not pinging, send response.
            {
                snd_msg.len = 8;
                snd_msg.ide = 0;
                snd_msg.rtr = 0;
                snd_msg.id = 0x599;
                for(uint8_t i = 0; i < 8; i++)
                {
                    snd_msg.data[i] = ping_response_data[i];
                }
                xCanTransmit(&snd_msg);
                continue;
            }
        }

        if((ping.flag & PING_KINCO) && (id == 0x581 || id == 0x582))
        {
            rt_kprintf("\n===========================\nping kinco: \n");
            ping.flag &=~ PING_KINCO;
            ping.delay[PING_KINCO_T] = rt_tick_get() - ping.delay[PING_KINCO_T];
            rt_kprintf("Delay: %.1f ms. \n",FROM_TICK_TO_MS(ping.delay[PING_KINCO_T]));
            ping_got_response = 1;
        }

        if(ping_got_response)
        {
            rt_kprintf("CAN Rcv from id: %x \n", id);
            for(i = 0; i<8;i++)
            {
                rt_kprintf("[%x]", rcv_msg.data[i]);
            }
            rt_kprintf("\n");
            rt_kprintf("===========================\n\n");
            continue;
        }
        
        /*-----------------------  normal functions  -----------------------*/
        for(i = 0; i < handlers_count; i++)
        {
            if(id == handlers[i].id)
            {
                handlers[i].handler(id, rcv_msg.len, rcv_msg.data);
                break;
            }
        }
    }
}
