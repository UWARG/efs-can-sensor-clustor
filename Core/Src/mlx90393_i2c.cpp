/*
 * mlx90393_i2c.c
 *
 *  Created on: Sep 13, 2024
 *      Author: Henry
 */
#include <mlx90393_i2c.hpp>

MLX90393::MLX90393(){
	HAL_GPIO_WritePin(GPIOB, CS, GPIO_PIN_SET);
	uint16_t mlx90393_t = 0;
	uint16_t mlx90393_x = 0;
	uint16_t mlx90393_y = 0;
	uint16_t mlx90393_z = 0;
}

void MLX90393::mlx90393_i2c_transmit(uint8_t *tx_data, uint8_t *rx_data, uint16_t tx_size, uint16_t rx_size)
{
	uint8_t rx_buf[rx_size + 2];
	HAL_I2C_Master_Transmit(&hi2c3, DEFAULT_I2C_ADDRESS, tx_data, tx_size, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c3, DEFAULT_I2C_ADDRESS + 1, rx_buf, rx_size + 1, HAL_MAX_DELAY);
	status = rx_data[0];
	for(int i = 0; i < rx_size; i++){
		rx_data[i] = rx_buf[i + 1];
	}
}

bool MLX90393::mlx90393_i2c_SM(uint8_t *rx_data, uint8_t zyxt)
{
	uint8_t tx_data = (0x30)| zyxt;
	mlx90393_i2c_transmit(tx_data, rx_data, 1, 1);
	if((status == MLX90393_STATUS_OK) || (status == MLX90393_STATUS_SMMODE)){
		return true;
	}
	return false;
}

bool MLX90393::mlx90393_i2c_RM(uint8_t *rx_data, uint8_t zyxt){
	int rx_size = zyxt_set_bits(zyxt) + 1;
	uint8_t tx_data = (0x40) | zyxt;
	mlx90393_i2c_transmit(tx_data, rx_data, 1, 2 * rx_size + 1);
	if(status != MLX90393_STATUS_OK){
		return false;
	}
	mlx90393_decode(rx_data, zyxt);
	//Need to adjust with resolution and gain
	return true;
}

int MLX90393::mlx90393_zyxt_set_bits(uint8_t zyxt){
	int count = 0;
	while(zyxt){
		if(zyxt & 0x1){
			count++;
		}
		zyxt >> 1;
	}
	return count;
}

void MLX90393::mlx90393_decode(uint8_t *rx_data, u8int_t zyxt){
	uint8_t *cursor = rx_data;
	status = rx_data[0];
	cursor += 1; //Skip status byte
	if(zyxt & MLX90393_T){
		mlx90393_t = mlx90393_decode_helper(cursor);
		cursor += 2;
	}
	if(zyxt & MLX90393_X){
		mlx90393_x = mlx90393_decode_helper(cursor);
		cursor += 2;
	}
	if(zyxt & MLX90393_Y){
		mlx90393_y = mlx90393_decode_helper(cursor);
		cursor += 2;
	}
	if(zyxt & MLX90393_Z){
		mlx90393_z = mlx90393_decode_helper(cursor);
		cursor += 2;
	}
}

uint16_t MLX90393::mlx90393_decode_helper(uint8_t *data){
	return (data[0] << 8 + data[1]);
}
