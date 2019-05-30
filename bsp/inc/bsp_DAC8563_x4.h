#ifndef __BSP__DAC8563__INCLUDED__H
#define __BSP__DAC8563__INCLUDED__H

#define __CST_DAC8563_OUTPUT_ZERO    32767
#define __CST_DAC8563_OUTPUT_MAX     65536

#define __CST_DAC_1_BIT_TO_V        0.0003051758f    //20V/65536
#define __CST_DAC_1_V_TO_BIT        3276.8f          //65536/20V

void DAC8563_x4_ZeroData_Parallel(void);
void DAC8563_x4_SetData_Parallel_Raw(uint16_t _dac11, uint16_t _dac12, uint16_t _dac21, uint16_t _dac22, uint16_t _dac31, uint16_t _dac32, uint16_t _dac41, uint16_t _dac42);
void DAC8563_x4_SetData_Parallel_RawArray(uint16_t * data_x8);
void DAC8563_x4_SetData_Parallel_VoltageArray(float * data_x8);
void DAC8563_x4_SetData_Parallel_Voltage(float _dac11, float _dac12, float _dac21, float _dac22, float _dac31, float _dac32, float _dac41, float _dac42);

#endif 
