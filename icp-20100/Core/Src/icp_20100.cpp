/*
 *
 */
#include <icp_20100.hpp>
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"

ICP_20100::ICP_20100(I2C_HandleTypeDef *hi2c){
	this->hi2c = hi2c;
	// idk what the other this-> registers are so i'm leaving it out for now
}


void ICP_20100::ICP20100_Init(void){
	uint8_t data;

	data = MODE_SELECT_CONTINUOUS;
	HAL_I2C_Mem_Write(&hi2c1, ICP20100_ADDR, MODE_SELECT_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);


	data = INTERRUPT_MASK_PRESSURE_READY;
	HAL_I2C_Mem_Write(&hi2c1, ICP20100_ADDR, INTERRUPT_MASK_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

uint16_t ICP_20100::ICP20100_ReadPressure(void){
	uint8_t pressure_data[3];  // Buffer to hold 3 bytes of pressure data

	HAL_I2C_Mem_Read(&hi2c1, ICP20100_ADDR, PRESS_ABS_LSB_REG, I2C_MEMADD_SIZE_8BIT, pressure_data, 3, HAL_MAX_DELAY);


	uint32_t pressure = (pressure_data[0] << 16) | (pressure_data[1] << 8) | pressure_data[2];

	return pressure;
}
