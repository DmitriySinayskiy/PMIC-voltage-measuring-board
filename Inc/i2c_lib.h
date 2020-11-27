#ifndef i2c_lib_H
#define i2c_lib_H

#include "stm32f1xx_hal.h"

int I2C1_Read_Byte(uint8_t addr,uint8_t data);
void I2C1_Write_Byte(uint8_t addr,uint8_t data);
void I2C1_Write_Array(uint8_t addr,uint8_t data[],uint8_t size);
int  I2C1_Read_Array(uint8_t addr,uint8_t data[],uint8_t size);

#endif
