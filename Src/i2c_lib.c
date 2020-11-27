#include "i2c_lib.h"

extern I2C_HandleTypeDef hi2c1;
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

//=====================================================================================================
// Write data byte to I2C1 bus
void I2C1_Write_Byte(uint8_t addr,uint8_t data)
{
	addr=addr<<1;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)0x90,&data,1,1000)!=HAL_OK)
	{
		if(HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
		{
			uint8_t i2c_error[] = "I2C write byte error. Check ADC board connection then reset";
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET);
			HAL_Delay(500);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_RESET);
			HAL_Delay(500);
			
			CDC_Transmit_FS(i2c_error,sizeof(i2c_error));
		}
	}
}

//=====================================================================================================
// Read data byte from I2C1 bus
int I2C1_Read_Byte(uint8_t addr,uint8_t data)
{

	addr=(addr<<1)+1;
	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)0x91,&data,1,1000)!=HAL_OK)
	{
		if(HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
		{
			uint8_t i2c_error[] = "I2C read byte error.Check ADC board connection then reset";
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_RESET);
			HAL_Delay(100);
			CDC_Transmit_FS(i2c_error,sizeof(i2c_error));
		}
	}
	return data;
}
//=======================================================================================================
//Write data array to I2C1 bus
void I2C1_Write_Array(uint8_t addr,uint8_t data[],uint8_t size)
{
	addr=addr<<1;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)addr,data,size,1000)!=HAL_OK)
	{
		if(HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
		{
			uint8_t i2c_error[] = "I2C write array error.Check PCA9532 assembly then reset";
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET);
			CDC_Transmit_FS(i2c_error,sizeof(i2c_error));
			//ADD sending to uart
		}
	}
}
//=======================================================================================================
//Read data array from I2C bus
/*int I2C1_Read_Array(uint8_t addr,uint8_t data[],uint8_t size)
{
	addr=(addr<<1)+1;
	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)addr,data,size,1000)!=HAL_OK)
	{
		if(HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
		{
			sprintf(str, "I2C read array error");
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET);
			//ADD sending to uart
		}
	}
	return data;
}
*/
//====================================================================================================
