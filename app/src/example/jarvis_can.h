#ifndef __JARVIS_CAN__H
#define __JARVIS_CAN__H

#include <rtdevice.h>

typedef void (*vJVCanRxHandler_t) (rt_uint32_t id, rt_uint32_t len, rt_uint8_t* data);

typedef struct eJVCanRxHandlerPair_typedef {
    rt_uint32_t id;
    vJVCanRxHandler_t handler;
} eJVCanRxHandlerPair_t;

typedef struct eJVCanRxHandlerCfg_typedef {
    rt_uint32_t count;
    eJVCanRxHandlerPair_t* pairs;
} eJVCanRxHandlerCfg_t;

typedef struct eJVCanDeviceRegisterCfg_typedef {
    struct rt_can_filter_config filer_cfg;
    eJVCanRxHandlerCfg_t handler_cfg;
} eJVCanDeviceRegisterCfg_t;

int xJVCanInit(void);
int xJVCanTransmit(struct rt_can_msg* msg);
int xJVCanDeviceRegister(eJVCanDeviceRegisterCfg_t* cfg);


#endif
