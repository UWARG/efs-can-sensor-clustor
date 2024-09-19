/*
 * mlx90393_i2c.c
 *
 *  Created on: Sep 13, 2024
 *      Author: Henry
 */
#include "mlx90393_i2c.h"
#define I2C_ADDRESS 0x0F << 1 //Determined by A0 A1 connection
#define CS GPIO_PIN_5

void mlx90393_i2c_init(void)
{
	HAL_GPIO_WritePin(GPIOB, CS, GPIO_PIN_SET);
}

void mlx90393_i2c_transmit(uint8_t *tx_data, uint8_t *rx_data, uint16_t tx_data, uint16_t rx_data)
{
	HAL_I2C_Master_Transmit(&hi2c3, I2C_ADDRESS, tx_data, tx_size, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c3, I2C_ADDRESS + 1, rx_data, rx_size, HAL_MAX_DELAY);
}

void mlx90393_i2c_SM(uint8_t *rx_data, uint8_t zyxt)
{
	uint8_t tx_data = (0x30)| zyxt;
	mlx90393_i2c_transmit(tx_data, rx_data, 1, 1);
}

void mlx90393_i2c_RM(uint8_t *rx_data, uint8_t zyxt){
	int rx_size = zyxt_set_bits(zyxt);
	uint8_t tx_data = (0x40) | zyxt;
	mlx90393_i2c_transmit(tx_data, rx_data, 1, 2 * rx_size + 1);
}

int zyxt_set_bits(uint8_t zyxt){
	int count = 0;
	while(zyxt){
		if(zyxt & 0x1){
			count++;
		}
		zyxt >> 1;
	}
	return count;
}
