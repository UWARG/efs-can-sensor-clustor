/*
 * mlx90393_i2c.c
 *
 *  Created on: Sep 13, 2024
 *      Author: Henry
 */
#include <mlx90393_i2c.hpp>
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"

MLX90393::MLX90393(I2C_HandleTypeDef *hi2c):hi2c(hi2c){
	HAL_GPIO_WritePin(GPIOB, CS, GPIO_PIN_SET);
	mlx90393_i2c_EX();
	mlx90393_i2c_RT();
	reg.gain = MLX90393_GAIN_1X;
	reg.x_res = MLX90393_RES_15;
	reg.y_res = MLX90393_RES_15;
	reg.z_res = MLX90393_RES_15;
	reg.hallconf = 0x0C;
	reg.tcmp_en = 0x00;
	reg.filter = 0x00;
	reg.osr = 0x00;
}

HAL_StatusTypeDef MLX90393::mlx90393_i2c_transceive(uint8_t *tx_data, uint8_t *rx_data, uint16_t tx_size, uint16_t rx_size)
{
	uint8_t rx_buf[rx_size + 2];
	HAL_StatusTypeDef status;
	status = HAL_I2C_Master_Transmit(hi2c, DEFAULT_I2C_ADDRESS, tx_data, tx_size, HAL_MAX_DELAY);
	if(status != HAL_OK){
		return status;
	}
	status = HAL_I2C_Master_Receive(hi2c, DEFAULT_I2C_ADDRESS + 1, rx_buf, rx_size + 1, HAL_MAX_DELAY);
	return status;
}

bool MLX90393::mlx90393_i2c_SM(uint8_t *rx_data, uint8_t zyxt)
{
	uint8_t* tx_data;
	*tx_data = (uint8_t)CMD_START_MEASUREMENT | zyxt;
	HAL_StatusTypeDef status = mlx90393_i2c_transceive(tx_data, rx_data, 1, 1);
	return status == HAL_OK ? 0 : 1;
}

bool MLX90393::mlx90393_i2c_RM(uint8_t *rx_data, uint8_t zyxt){
	int rx_size = mlx90393_zyxt_set_bits(zyxt) + 1;
	uint8_t* tx_data;
	*tx_data = (uint8_t)CMD_READ_MEASUREMENT | zyxt;
	HAL_StatusTypeDef status = mlx90393_i2c_transceive(tx_data, rx_data, 1, 2 * rx_size + 1);
	mlx90393_decode(rx_data, zyxt);
	mlx90393_convert();
	return status == HAL_OK ? 0 : 1;
}

bool MLX90393::mlx90393_i2c_EX(){
	uint8_t* tx_data;
	*tx_data = CMD_EXIT;
	HAL_StatusTypeDef status = mlx90393_i2c_transceive(tx_data, nullptr, 1, 0);
	return status == HAL_OK ? 0 : 1;
}

bool MLX90393::mlx90393_i2c_RT(){
	uint8_t* tx_data;
	*tx_data = CMD_RESET;
	HAL_StatusTypeDef status = mlx90393_i2c_transceive(tx_data, nullptr, 1, 0);
	return status == HAL_OK ? 0 : 1;
}

bool MLX90393::mlx90393_i2c_WR(uint8_t regNum, uint8_t *tx_data){
	if(HAL_I2C_Mem_Write(hi2c, DEFAULT_I2C_ADDRESS, regNum << 2, I2C_MEMADD_SIZE_16BIT, tx_data, 1, HAL_MAX_DELAY) != HAL_OK){
		return false;
	}
	return true;
}
bool MLX90393::mlx90393_i2c_RR(uint8_t regNum){
	uint8_t regVal[2];
	if(HAL_I2C_Mem_Read(hi2c, DEFAULT_I2C_ADDRESS, regNum << 2, I2C_MEMADD_SIZE_16BIT, regVal, 1, HAL_MAX_DELAY) != HAL_OK){
		return false;
	}
	reg.val = regVal[0] << 8 | regVal[1];
	return true;
}

bool MLX90393::mlx90393_set_gain(uint8_t gain){
	if(!mlx90393_i2c_RR(MLX90393_CONF1)){
		return false;
	}
	uint16_t data = reg.val & ~MLX90393_GAIN_MASK;
	data |= gain << MLX90393_GAIN_SHIFT;
	reg.gain = gain;
	uint8_t buf[2];
	buf[0] = data >> 8 & 0xFF;
	buf[1] = data & 0xff;
	return(mlx90393_i2c_WR(MLX90393_CONF1, buf));
}

bool MLX90393::mlx90393_get_gain(){
	if(!mlx90393_i2c_RR(MLX90393_CONF1)){
		return false;
	}
	uint16_t data = reg.val & MLX90393_GAIN_MASK;
	reg.gain = data >> MLX90393_GAIN_SHIFT;
	return true;
}

bool MLX90393::mlx90393_set_resolution(uint8_t x_res, uint8_t y_res, uint8_t z_res){
	if(!mlx90393_i2c_RR(MLX90393_CONF3)){
		return false;
	}
	//Res 2 and 3 not allowed if temperature compensation enabled. See 16.2.10
	if(reg.tcmp_en == 1){
		if(x_res == 2 || x_res == 3 || y_res == 2 || y_res == 3 || z_res == 2 || z_res == 3){
			return false;
		}
	}
	reg.x_res = x_res;
	reg.y_res = y_res;
	reg.z_res = z_res;
	uint16_t data = reg.val;
	if(x_res != 0){
		data = (data & ~MLX90393_X_RES_MASK) | (x_res << MLX90393_X_RES_SHIFT);
	}
	if(y_res != 0){
		data = (data & ~MLX90393_Y_RES_MASK) | (y_res << MLX90393_Y_RES_SHIFT);
	}
	if(z_res != 0){
		data = (data & ~MLX90393_Z_RES_MASK) | (z_res << MLX90393_Z_RES_SHIFT);
	}
	uint8_t buf[2];
	buf[0] = data >> 8 & 0xFF;
	buf[1] = data & 0xff;
	return(mlx90393_i2c_WR(MLX90393_CONF3, buf));
}


bool MLX90393::mlx90393_get_resolution(){
	if(mlx90393_i2c_RR(MLX90393_CONF3)){
		return false;
	}
	uint16_t data = reg.val;
	reg.x_res = (data & MLX90393_X_RES_MASK) >> MLX90393_X_RES_SHIFT;
	reg.y_res = (data & MLX90393_Y_RES_MASK) >> MLX90393_Y_RES_SHIFT;
	reg.z_res = (data & MLX90393_Z_RES_MASK) >> MLX90393_Z_RES_SHIFT;
	return true;
}

bool MLX90393::mlx90393_set_filter(uint8_t filter){
	if(!mlx90393_i2c_RR(MLX90393_CONF3)){
		return false;
	}
	//Not permitted settings see 16.2.5
	if(reg.hallconf == 0x0C){
		if(reg.osr == 0x00){
			if(filter == 0 || filter == 1){
				return false;
			}
		}
		if(reg.osr == 0x01){
			if(filter == 1){
				return false;
			}
		}
	}
	reg.filter = filter;
	uint16_t data = reg.val;
	data = (data & ~MLX90393_FILTER_MASK) | (filter << MLX90393_FILTER_SHIFT);
	uint8_t buf[2];
	buf[0] = data >> 8 & 0xFF;
	buf[1] = data & 0xff;
	return(mlx90393_i2c_WR(MLX90393_CONF3, buf));

}
bool MLX90393::mlx90393_get_filter(){
	if(!mlx90393_i2c_RR(MLX90393_CONF3)){
		return false;
	}
	uint16_t data = reg.val;
	reg.filter = (data & MLX90393_FILTER_MASK) >> MLX90393_FILTER_SHIFT;
	return true;
}
bool MLX90393::mlx90393_set_oversampling(uint8_t osr){
	if(!mlx90393_i2c_RR(MLX90393_CONF3)){
		return false;
	}
	//Not permitted settings see 16.2.5
	if(reg.hallconf == 0x0C){
		if(reg.filter == 0x00){
			if(osr == 0 || osr == 1){
				return false;
			}
		}
		if(reg.hallconf == 0x01){
			if(osr == 1){
				return false;
			}
		}
	}
	reg.osr = osr;
	uint16_t data = reg.val;
	data = (data & ~MLX90393_OSR_MASK) | (reg.filter << MLX90393_OSR_SHIFT);
	uint8_t buf[2];
	buf[0] = data >> 8 & 0xFF;
	buf[1] = data & 0xff;
	return(mlx90393_i2c_WR(MLX90393_CONF3, buf));
}

bool MLX90393::mlx90393_get_oversampling(){
	if(!mlx90393_i2c_RR(MLX90393_CONF3)){
		return false;
	}
	uint16_t data = reg.val;
	reg.osr = (data & MLX90393_OSR_MASK) >> MLX90393_OSR_SHIFT;
	return true;
}

int MLX90393::mlx90393_zyxt_set_bits(uint8_t zyxt){
	int count = 0;
	while(zyxt){
		if(zyxt & 0x1){
			count++;
		}
		zyxt >>= 1;
	}
	return count;
}

void MLX90393::mlx90393_decode(uint8_t *rx_data, uint8_t zyxt){
	uint8_t *cursor = rx_data;
	reg.stat = rx_data[0];
	cursor += 1; //Skip status byte
	if(zyxt & MLX90393_T){
		raw.t = mlx90393_decode_helper(cursor);
		cursor += 2;
	}
	if(zyxt & MLX90393_X){
		raw.x = mlx90393_decode_helper(cursor);
		cursor += 2;
	}
	if(zyxt & MLX90393_Y){
		raw.y = mlx90393_decode_helper(cursor);
		cursor += 2;
	}
	if(zyxt & MLX90393_Z){
		raw.z = mlx90393_decode_helper(cursor);
		cursor += 2;
	}
}

uint16_t MLX90393::mlx90393_decode_helper(uint8_t *data){
	return (data[0] << 8 | data[1]);
}

void MLX90393::mlx90393_convert(){
	//Only when tcmp_en = 0
	//Remove sign bit for unsigned res settings
	if(reg.x_res == 2){
		raw.x -= MLX90393_RES_17;
	}
	if(reg.x_res == 3){
		raw.x -= MLX90393_RES_18;
	}
	if(reg.y_res == 2){
		raw.y -= MLX90393_RES_17;
	}
	if(reg.y_res == 3){
		raw.y -= MLX90393_RES_18;
	}
	if(reg.z_res == 2){
		raw.z -= MLX90393_RES_17;
	}
	if(reg.x_res == 2){
		raw.z -= MLX90393_RES_18;
	}

	//Check if temperature compensation is enabled. See 16.2.10
	//Convert raw data base on sensitivity
	if(reg.tcmp_en == 0){
		converted.x = (float)raw.x * sens_lookup_0xC[reg.gain][reg.x_res][0];
		converted.y = (float)raw.y * sens_lookup_0xC[reg.gain][reg.y_res][0];
		converted.z = (float)raw.z * sens_lookup_0xC[reg.gain][reg.z_res][1];
	}else{
		converted.x = (float)((uint16_t)raw.x * sens_lookup_0xC[reg.gain][reg.x_res][0]);
		converted.y = (float)((uint16_t)raw.y * sens_lookup_0xC[reg.gain][reg.y_res][0]);
		converted.z = (float)((uint16_t)raw.z * sens_lookup_0xC[reg.gain][reg.z_res][1]);
	}

	//Check hall config, see 16.2.4
	if(reg.hallconf == 0x0){
		converted.x = converted.x * 98 / 75;
		converted.y = converted.y * 98 / 75;
		converted.z = converted.z * 98 / 75;
	}
}