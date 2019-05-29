/**
 * @file A9800.c
 * @author Jianfei (Ventus) Guo (ffventus@gmail.com)
 * @brief
 * @version 0.1
 * @date 2019-05-26
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "A9800.h"
#include <rt_spi.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <configuration.h>

static struct rt_spi_message msg;
static struct rt_spi_device* pmouse1;
static struct rt_spi_device* pmouse2;
static rt_uint8_t zero_dummy = 0;

/**
 * @brief Read from the registers of A9800
 *
 * @param uAddr register address
 * @param chip chip number
 * @return uint8_t
 */
uint8_t A9800xReadReg(uint8_t uAddr,uint8_t chip)
{
    //todo: mutex? lock? !necessary
    uint8_t   uVal;
    struct rt_spi_device* pmouse;
    if(chip == ADNS9800_CHIP1)
    {
        pmouse = pmouse1;
    }else if(chip == ADNS9800_CHIP2)
    {
        pmouse = pmouse2;
    }

    /* NCS & write address */
    msg.send_buf = &uAddr;
    msg.recv_buf = RT_NULL;
    msg.length = 1;
    msg.cs_release = 0;
    msg.cs_take = 1;
    rt_spi_transfer_message(pmouse, &msg);

    /* tSRAD = 100us */
    rt_thread_delay(5);

    /* read & rise NCS*/
    msg.send_buf = &zero_dummy;
    msg.recv_buf = &uVal;
    msg.length = 1;
    msg.cs_release = 1;
    msg.cs_take = 0;
    // After read, we only need to wait 120ns until NCS rise.
    rt_spi_transfer_message(pmouse, &msg);

    // tSRW = tSRR = 20us.
    rt_thread_delay(1);
	return uVal;
}

/**
 * @brief Write to registers of A9800
 *
 * @param uAddr register address
 * @param uVal the value to write, Byte
 * @param chip chip number
 */
void A9800xWriteReg(uint8_t uAddr, uint8_t uVal,uint8_t chip)
{
    //todo: mutex? lock? !necessary
    struct rt_spi_device* pmouse;
    if(chip == ADNS9800_CHIP1)
    {
        pmouse = pmouse1;
    }else if(chip == ADNS9800_CHIP2)
    {
        pmouse = pmouse2;
    }

    /* NCS & write address */
    uAddr |= 0x80;
    msg.send_buf = &uAddr;
    msg.recv_buf = RT_NULL;
    msg.length = 1;
    msg.cs_release = 0;
    msg.cs_take = 1;
    rt_spi_transfer_message(pmouse, &msg);

    /* write value & NCS */
    msg.send_buf = &uVal;
    msg.recv_buf = RT_NULL;
    msg.length = 1;
    msg.cs_release = 1;
    msg.cs_take = 0;
    rt_spi_transfer_message(pmouse, &msg);

    /* tSWR = tsWW = 120us */
    rt_thread_delay(6);
}

/**
 * @brief
 *
 * @param puBuf
 * @param uNum
 * @param chip
 */
void A9800xBurstReadMoveReg(uint8_t * puBuf, uint8_t uNum, uint8_t chip)
{
    //todo: mutex? lock? !necessary
    struct rt_spi_device* pmouse;
    if(chip == ADNS9800_CHIP1)
    {
        pmouse = pmouse1;
    }else if(chip == ADNS9800_CHIP2)
    {
        pmouse = pmouse2;
    }

    /* NCS & write addr */
    rt_uint8_t uAddr = A9800_MOTION_BURST_ADDR;
    msg.send_buf = &uAddr;
    msg.recv_buf = RT_NULL;
    msg.length = 1;
    msg.cs_release = 0;
    msg.cs_take = 1;
    rt_spi_transfer_message(pmouse, &msg);

    /* tSRAD = 100us */
    rt_thread_delay(5);

    /* read in burst mode */
    msg.send_buf = &zero_dummy;
    msg.recv_buf = puBuf;
    msg.length = uNum;
    msg.cs_release = 0;
    msg.cs_take = 0;
    rt_spi_transfer_message(pmouse, &msg);

    /* release NCS */
    rt_spi_release(pmouse);

    // tSRW = tSRR = 20us.
    rt_thread_delay(1);
}

/**
 * @brief write CPI into register
 *
 * @param uCpi CPI code, range:[1~164], 50CPI/1, register default: 68 = 3400CPI
 * @param chip
 */
void A9800xWriteCpi(uint8_t uCpi,uint8_t chip)
{
    A9800xWriteReg(A9800_CONFIGURATION_I_ADDR, uCpi,chip);
}

/**
 * @brief write Lift Threshold into register
 *
 * @param uLift Lift threshold, range:[1~31], register default: 16.
 * @param chip
 */
void A9800xWriteLift(uint8_t uLift,uint8_t chip)
{
    A9800xWriteReg(A9800_LIFT_DETECTION_THR_ADDR, uLift,chip);
}

/**
 * @brief check ID
 *
 * @param chip
 */
void A9800xInitCheckId(uint8_t chip)
{
    A9800xReadReg(A9800_PRODUCTID_ADDR,chip);
    A9800xReadReg(A9800_REVISIONID_ADDR,chip);
    A9800xReadReg(A9800_INVERSE_PRODUCT_ID_ADDR,chip);
}

/**
 * @brief PowerUp
 *
 * @param chip
 */
void A9800xInitPowerUp(uint8_t chip)
{
    /* PowerUP */
    A9800xWriteReg(A9800_POWER_UP_RESET_ADDR, A9800_POWER_UP_RESET, chip);

    /* delay 50ms */
    rt_thread_mdelay(50);

    A9800xReadReg(A9800_MOTION_ADDR, chip);
    A9800xReadReg(A9800_DELTAX_L_ADDR, chip);
    A9800xReadReg(A9800_DELTAX_H_ADDR, chip);
    A9800xReadReg(A9800_DELTAY_L_ADDR, chip);
    A9800xReadReg(A9800_DELTAY_H_ADDR, chip);
}

/**
 * @brief Load SROM into chip
 *
 * @param chip
 */
void A9800xLoadSrom(uint8_t chip)
{
    rt_kprintf("Size of SROM: %d.\n",sizeof(c_auA9800SromData));
    rt_kprintf("Uploading ADNS9800 firmware...\n");

    struct rt_spi_device* pmouse;
    if(chip == ADNS9800_CHIP1)
    {
        pmouse = pmouse1;
    }else if(chip == ADNS9800_CHIP2)
    {
        pmouse = pmouse2;
    }

    /* Select 3k SROM size */
    A9800xWriteReg(A9800_CONFIGURATION_IV_ADDR, A9800_SROM_SIZE_3_0K, chip);
    rt_thread_mdelay(1); //delay 1ms

    A9800xWriteReg(A9800_SROM_ENABLE_ADDR, 0x1D, chip);

    /* Wait 1 frame period */
    rt_thread_mdelay(10); //delay 10ms

    A9800xWriteReg(A9800_SROM_ENABLE_ADDR, 0x18, chip);
    rt_thread_delay(6); //delay 120us

    // rt_spi_take(pmouse);
    // A9800DelayUs(10);

    /* write address to enter SROM load mode. */
    rt_uint8_t uAddr = A9800_SROM_LOAD_BURST_ADDR | 0x80;
    msg.send_buf = &uAddr;
    msg.recv_buf = RT_NULL;
    msg.length = 1;
    msg.cs_release = 0;
    msg.cs_take = 1;
    rt_spi_transfer_message(pmouse, &msg);
    rt_thread_mdelay(10);   //?delay 10ms. on user manual of A9800, it's 15us

    /* write the whole SROM data byte by byte */
    msg.cs_take = 0;
    msg.cs_release = 0;
    msg.length = 1;
    msg.recv_buf = RT_NULL;
    for (uint16_t i = 0; i < sizeof(c_auA9800SromData); i++)
    {
        msg.send_buf = &c_auA9800SromData[i];
        rt_spi_transfer_message(pmouse, &msg);
        rt_thread_delay(2); //?delay 100us. on user manual of A9800, it's 15us
    }
    rt_spi_release(pmouse);
    rt_kprintf("ADNS9800 firmware upload done.\n");
}

/**
 * @brief Check SROM
 *
 * @param chip
 * @return OP result of the operation
 */
OP A9800xCheckSrom(uint8_t chip)
{
    uint8_t auBuf[10];

    rt_thread_delay(10);    // should be delay 200us

    auBuf[0] = A9800xReadReg(A9800_SROM_ID_ADDR, chip);                    // Check SROM version
    rt_thread_delay(2); // 40us. should be delay 10us

    auBuf[1] = A9800xReadReg(A9800_MOTION_ADDR, chip);                     // Check Fault
    rt_thread_delay(2); // 40us. should be delay 10us
    if ((auBuf[1] & A9800_MOTION_FAULT) == A9800_MOTION_FAULT)
    {
		rt_kprintf("FAULT\n");
        return OP_FAIL;
    }

    A9800xWriteReg(A9800_SROM_ENABLE_ADDR, 0x15, chip);                    // Check CRC Test
    rt_thread_mdelay(50);   //delay 50ms

    auBuf[2] = A9800xReadReg(A9800_DATA_OUT_LOWER_ADDR, chip);
    rt_thread_delay(2); // 40us. should be delay 10us
    auBuf[3] = A9800xReadReg(A9800_DATA_OUT_UPPER_ADDR, chip);
    rt_thread_delay(2); // 40us. should be delay 10us
    if ((auBuf[2] != 0xEF) || (auBuf[3] != 0xBE))
    {
		rt_kprintf("DATA_OUT_LOWER/UPPER_ADDR\n");
        return OP_FAIL;
    }

    A9800xWriteReg(A9800_OBSERVATION_ADDR, 0x00, chip);                    // Clear Observation Register
    rt_thread_delay(2); // 40us. should be delay 1us
    auBuf[4] = A9800xReadReg(A9800_OBSERVATION_ADDR, chip);                // Check if SROM is running
    if ((auBuf[4] & 0x40) == 0)
    {
		rt_kprintf("OBSERVATION_ADDR\n");
        return OP_FAIL;
    }

    return OP_SUCCESS;
}

/**
 * @brief Enable laser power.
 *
 * @param chip
 */
void A9800xEnLaserPower(uint8_t chip)
{
    uint8_t uData;
    uData = A9800xReadReg(A9800_LASER_CTRL0_ADDR, chip);
    A9800xWriteReg(A9800_LASER_CTRL0_ADDR, (uData & 0xFE), chip);                // Enable Laser Power
}

/**
 * @brief Initialize SROM and enable laser
 *
 * @param chip
 */
void A9800xInitSrom(uint8_t chip)
{
    A9800xLoadSrom(chip);
    rt_thread_mdelay(1);    //delay 1ms

    A9800xReadReg(0x2A, chip);
    OP result = A9800xCheckSrom(chip);
    A9800xEnLaserPower(chip);
	if(result == OP_SUCCESS)
    {
        rt_kprintf("SROM check success.\n");
    }
    else if(result == OP_FAIL)
    {
        rt_kprintf("SROM check failed.\n");
    }
}

/**
 * @brief display register info
 *
 * @param chip
 */
void A9800xdispRegisters(uint8_t chip){

    uint16_t oreg[7] = {
    0x00,0x3F,0x2A,0x02  };
    uint8_t * oregname[] = {
    "Product_ID","Inverse_Product_ID","SROM_Version","Motion"  };
    uint8_t regres;
    uint8_t rctr=0;
    rt_kprintf("ANDS9800 - %d Info\n",chip + 1);
    for(rctr=0; rctr<4; rctr++){
        regres = A9800xReadReg(oreg[rctr], chip);
        rt_kprintf("%s:%x\n",oregname[rctr],regres);
    }
}

/**
 * @brief initialize the A9800s. attach the device to the SPI bus.
 *
 * @param chip
 */
void A9800xInit(uint8_t chip)
{
    rt_kprintf("----------- ADNS9800 - %d Initialization -----------\n", chip + 1);
    uint8_t auBuf[10];

    A9800xInitCheckId(chip);
    A9800xInitPowerUp(chip);
    A9800xInitSrom(chip);
    A9800xdispRegisters(chip);

    //A9800xWriteCpi(g_uA9800Cpi, chip);
    A9800xWriteCpi((uint8_t)(_cfg_ADNS9800_CPI/50), chip);

    // Ventus: tests proved that there should be a interval between write CPI and read after.
    //          1ms would be enough to ensure the read CPI data is correct.
    rt_thread_mdelay(1);

    uint8_t CPI_Byte = A9800xReadReg(A9800_CONFIGURATION_I_ADDR, chip);
    rt_kprintf("ANDS9800 - %d CPI set to:%d.\n",chip + 1,((uint16_t)CPI_Byte)*50);

    A9800xWriteLift(_cfg_ADNS9800_LIFT, chip);

    auBuf[0] = A9800xReadReg(A9800_MOTION_ADDR, chip);
    auBuf[1] = A9800xReadReg(A9800_LASER_CTRL0_ADDR, chip);
    auBuf[2] = A9800xReadReg(A9800_LASER_CTRL1_ADDR, chip);
    auBuf[3] = A9800xReadReg(A9800_LP_CFG0_ADDR, chip);
    auBuf[4] = A9800xReadReg(A9800_LP_CFG1_ADDR, chip);
    if (auBuf[0] == 0)
    {
        ;
    }
}

void A9800xGetDeltaXY(uint8_t chip, int16_t* delta_ix, int16_t* delta_iy)
{
    uint8_t           auBuf[6];
    // int16_t          iMoveX;
    // int16_t          iMoveY;
    
    A9800xBurstReadMoveReg(auBuf, 6, chip);

    *delta_ix = auBuf[3];
    *delta_ix <<= 8;
    *delta_ix |= auBuf[2];

    *delta_iy = auBuf[5];
    *delta_iy <<= 8;
    *delta_iy |= auBuf[4];
}

static const struct rt_spi_configuration A9800_config = 
{
    RT_SPI_CPOL | RT_SPI_CPHA | RT_SPI_MSB,
    8,
    0,
    2000000,
};

int mouse_sensor_init(void)
{
    rt_kprintf("\nInitializing mouse sensors...\n");
    pmouse1 = (struct rt_spi_device *)rt_device_find("mouse1");
    pmouse2 = (struct rt_spi_device *)rt_device_find("mouse2");
    pmouse1->config = A9800_config;
    pmouse2->config = A9800_config;
    A9800xInit(ADNS9800_CHIP1);
    A9800xInit(ADNS9800_CHIP2);
    return 0;
}
INIT_APP_EXPORT(mouse_sensor_init)

void list_mouse_xy(void)
{
    int16_t delta_ix1, delta_iy1, delta_ix2, delta_iy2;
    A9800xGetDeltaXY(ADNS9800_CHIP1, &delta_ix1, &delta_iy1);
    A9800xGetDeltaXY(ADNS9800_CHIP2, &delta_ix2, &delta_iy2);
    rt_kprintf("Got mouse sensor movement:\n");
    rt_kprintf("Mouse1: dx: %8d, dy: %8d.\n", delta_ix1, delta_iy1);
    rt_kprintf("Mouse2: dx: %8d, dy: %8d.\n", delta_ix2, delta_iy2);
}
MSH_CMD_EXPORT(list_mouse_xy, list mouse sensor movement data.)
