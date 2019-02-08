#ifndef __CAN_EXAMPLE__H
#define __CAN_EXAMPLE__H

#include <rtdevice.h>

typedef void (*vCanRxHandler_t) (rt_uint32_t id, rt_uint32_t len, rt_uint8_t* data);

typedef struct eCanRxHandlerPair_typedef {
    rt_uint32_t id;
    vCanRxHandler_t handler;
} eCanRxHandlerPair_t;

typedef struct eCanRxHandlerCfg_typedef {
    rt_uint32_t count;
    eCanRxHandlerPair_t* pairs;
} eCanRxHandlerCfg_t;

typedef struct eCanDeviceRegisterCfg_typedef {
    struct rt_can_filter_config filer_cfg;
    eCanRxHandlerCfg_t handler_cfg;
} eCanDeviceRegisterCfg_t;

int xCanInit(void);
int xCanTransmit(struct rt_can_msg* msg);
int xCanDeviceRegister(eCanDeviceRegisterCfg_t* cfg);


#endif
