/*
 * mlx90393_i2c.c
 *
 *  Created on: Sep 13, 2024
 *      Author: Henry
 */
#include <mlx90393_i2c.hpp>
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"

MLX90393::MLX90393(I2C_HandleTypeDef *hi2c){
	this->hi2c = hi2c;
	HAL_GPIO_WritePin(GPIOB, CS, GPIO_PIN_SET);
	mlx90393_i2c_EX();
	mlx90393_i2c_RT();
	this->reg.gain = MLX90393_GAIN_1X;
	this->reg.x_res = MLX90393_RES_15;
	this->reg.y_res = MLX90393_RES_15;
	this->reg.z_res = MLX90393_RES_15;
	this->reg.hallconf = 0x0C;
	this->reg.tcmp_en = 0x00;
	this->reg.filter = 0x00;
	this->reg.osr = 0x00;
	this->raw.t = 0;
	this->raw.x = 0;
	this->raw.y = 0;
	this->raw.z = 0;
	this->converted.t = 0;
	this->converted.x = 0;
	this->converted.y = 0;
	this->converted.z = 0;
	zyxt = 0x0E;
}

HAL_StatusTypeDef MLX90393::mlx90393_i2c_transceive(uint8_t *tx_data, uint8_t *rx_data, uint16_t tx_size, uint16_t rx_size)
{
	HAL_StatusTypeDef status;
	status = HAL_I2C_Master_Transmit(hi2c, DEFAULT_I2C_ADDRESS, tx_data, tx_size, HAL_MAX_DELAY);
	if(status != HAL_OK){
		return status;
	}
	status = HAL_I2C_Master_Receive(hi2c, DEFAULT_I2C_ADDRESS, rx_data, rx_size, HAL_MAX_DELAY);
	return status;
}

bool MLX90393::mlx90393_i2c_SM()
{
	uint8_t tx_data = (uint8_t)CMD_START_MEASUREMENT | this->zyxt;
	uint8_t buf = 0x00;
	HAL_StatusTypeDef status = mlx90393_i2c_transceive(&tx_data, &buf, 1, 1);
	this->reg.stat = buf;
	return status == HAL_OK;
}

bool MLX90393::mlx90393_i2c_RM(){
	int rx_size = 2 * mlx90393_zyxt_set_bits() + 1;
	uint8_t rx_data[rx_size];
	uint8_t tx_data = (uint8_t)CMD_READ_MEASUREMENT | this->zyxt;
	HAL_StatusTypeDef status = mlx90393_i2c_transceive(&tx_data, rx_data, 1, rx_size);
	mlx90393_decode(rx_data);
	mlx90393_convert();
	return status == HAL_OK;
}

bool MLX90393::mlx90393_i2c_EX(){
	uint8_t tx_data = CMD_EXIT;
	uint8_t buf = 0x00;
	HAL_StatusTypeDef status = mlx90393_i2c_transceive(&tx_data, &buf, 1, 1);
	this->reg.stat = buf;
	return status == HAL_OK;
}

bool MLX90393::mlx90393_i2c_RT(){
	uint8_t tx_data = CMD_RESET;
	uint8_t buf = 0x00;
	HAL_StatusTypeDef status = mlx90393_i2c_transceive(&tx_data, &buf, 1, 1);
	return status == HAL_OK;
}

bool MLX90393::mlx90393_i2c_WR(uint8_t regNum, uint8_t *tx_data){
	if(HAL_I2C_Mem_Write(this->hi2c, DEFAULT_I2C_ADDRESS, regNum << 2, I2C_MEMADD_SIZE_16BIT, tx_data, 1, HAL_MAX_DELAY) != HAL_OK){
		return false;
	}
	return true;
}
bool MLX90393::mlx90393_i2c_RR(uint8_t regNum){
	uint8_t regVal[2];
	if(HAL_I2C_Mem_Read(this->hi2c, DEFAULT_I2C_ADDRESS, regNum << 2, I2C_MEMADD_SIZE_16BIT, regVal, 1, HAL_MAX_DELAY) != HAL_OK){
		return false;
	}
	this->reg.val = regVal[0] << 8 | regVal[1];
	return true;
}

bool MLX90393::mlx90393_i2c_set_gain(uint8_t gain){
	if(!mlx90393_i2c_RR(MLX90393_CONF1)){
		return false;
	}
	uint16_t data = this->reg.val & ~MLX90393_GAIN_MASK;
	data |= gain << MLX90393_GAIN_SHIFT;
	this->reg.gain = gain;
	uint8_t buf[2];
	buf[0] = data >> 8 & 0xFF;
	buf[1] = data & 0xff;
	return(mlx90393_i2c_WR(MLX90393_CONF1, buf));
}

bool MLX90393::mlx90393_i2c_get_gain(){
	if(!mlx90393_i2c_RR(MLX90393_CONF1)){
		return false;
	}
	uint16_t data = reg.val & MLX90393_GAIN_MASK;
	this->reg.gain = data >> MLX90393_GAIN_SHIFT;
	return true;
}

bool MLX90393::mlx90393_i2c_set_resolution(uint8_t x_res, uint8_t y_res, uint8_t z_res){
	if(!mlx90393_i2c_RR(MLX90393_CONF3)){
		return false;
	}
	//Res 2 and 3 not allowed if temperature compensation enabled. See 16.2.10
	if(this->reg.tcmp_en == 1){
		if(x_res == 2 || x_res == 3 || y_res == 2 || y_res == 3 || z_res == 2 || z_res == 3){
			return false;
		}
	}
	this->reg.x_res = x_res;
	this->reg.y_res = y_res;
	this->reg.z_res = z_res;
	uint16_t data = this->reg.val;
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


bool MLX90393::mlx90393_i2c_get_resolution(){
	if(mlx90393_i2c_RR(MLX90393_CONF3)){
		return false;
	}
	uint16_t data = this->reg.val;
	this->reg.x_res = (data & MLX90393_X_RES_MASK) >> MLX90393_X_RES_SHIFT;
	this->reg.y_res = (data & MLX90393_Y_RES_MASK) >> MLX90393_Y_RES_SHIFT;
	this->reg.z_res = (data & MLX90393_Z_RES_MASK) >> MLX90393_Z_RES_SHIFT;
	return true;
}

bool MLX90393::mlx90393_i2c_set_filter(uint8_t filter){
	if(!mlx90393_i2c_RR(MLX90393_CONF3)){
		return false;
	}
	//Not permitted settings see 16.2.5
	if(this->reg.hallconf == 0x0C){
		if(this->reg.osr == 0x00){
			if(filter == 0 || filter == 1){
				return false;
			}
		}
		if(this->reg.osr == 0x01){
			if(filter == 1){
				return false;
			}
		}
	}
	this->reg.filter = filter;
	uint16_t data = this->reg.val;
	data = (data & ~MLX90393_FILTER_MASK) | (filter << MLX90393_FILTER_SHIFT);
	uint8_t buf[2];
	buf[0] = data >> 8 & 0xFF;
	buf[1] = data & 0xFF;
	return(mlx90393_i2c_WR(MLX90393_CONF3, buf));

}
bool MLX90393::mlx90393_i2c_get_filter(){
	if(!mlx90393_i2c_RR(MLX90393_CONF3)){
		return false;
	}
	uint16_t data = this->reg.val;
	this->reg.filter = (data & MLX90393_FILTER_MASK) >> MLX90393_FILTER_SHIFT;
	return true;
}
bool MLX90393::mlx90393_i2c_set_oversampling(uint8_t osr){
	if(!mlx90393_i2c_RR(MLX90393_CONF3)){
		return false;
	}
	//Not permitted settings see 16.2.5
	if(this->reg.hallconf == 0x0C){
		if(this->reg.filter == 0x00){
			if(osr == 0 || osr == 1){
				return false;
			}
		}
		if(this->reg.hallconf == 0x01){
			if(osr == 1){
				return false;
			}
		}
	}
	this->reg.osr = osr;
	uint16_t data = this->reg.val;
	data = (data & ~MLX90393_OSR_MASK) | (this->reg.filter << MLX90393_OSR_SHIFT);
	uint8_t buf[2];
	buf[0] = data >> 8 & 0xFF;
	buf[1] = data & 0xFF;
	return(mlx90393_i2c_WR(MLX90393_CONF3, buf));
}

bool MLX90393::mlx90393_i2c_get_oversampling(){
	if(!mlx90393_i2c_RR(MLX90393_CONF3)){
		return false;
	}
	uint16_t data = this->reg.val;
	this->reg.osr = (data & MLX90393_OSR_MASK) >> MLX90393_OSR_SHIFT;
	return true;
}

bool MLX90393::mlx90393_i2c_has_error(){
	return (this->reg.stat & ERROR_BIT) != 0;
}

bool MLX90393::mlx90393_i2c_read_data(){
	if(!mlx90393_i2c_SM()){
		return false;
	}
	HAL_Delay(mlx90393_tconv[this->reg.filter][this->reg.osr] + 10);
	if(!mlx90393_i2c_RM()){
		return false;
	}
	return true;
}

int MLX90393::mlx90393_zyxt_set_bits(){
	int count = 0;
	uint8_t temp = this->zyxt;
	while(temp){
		if(temp & 0x1){
			count++;
		}
		temp >>= 1;
	}
	return count;
}

void MLX90393::mlx90393_decode(uint8_t *rx_data){
	uint8_t *cursor = rx_data;
	this->reg.stat = rx_data[0];
	cursor += 1; //Skip status byte
	if(this->zyxt & MLX90393_T){
		this->raw.t = mlx90393_decode_helper(cursor);
		cursor += 2;
	}
	if(this->zyxt & MLX90393_X){
		this->raw.x = mlx90393_decode_helper(cursor);
		cursor += 2;
	}
	if(this->zyxt & MLX90393_Y){
		this->raw.y = mlx90393_decode_helper(cursor);
		cursor += 2;
	}
	if(this->zyxt & MLX90393_Z){
		this->raw.z = mlx90393_decode_helper(cursor);
		cursor += 2;
	}
}

int16_t MLX90393::mlx90393_decode_helper(uint8_t *data){
	return (data[0] << 8 | data[1]);
}

void MLX90393::mlx90393_convert(){
	//Only when tcmp_en = 0
	//Remove sign bit for unsigned res settings
	if(this->reg.x_res == 2){
		this->raw.x &= ~MLX90393_RES_17;
	}
	if(this->reg.x_res == 3){
		this->raw.x &= ~MLX90393_RES_18;
	}
	if(this->reg.y_res == 2){
		this->raw.y &= ~MLX90393_RES_17;
	}
	if(this->reg.y_res == 3){
		this->raw.y &= ~MLX90393_RES_18;
	}
	if(this->reg.z_res == 2){
		this->raw.z &= ~MLX90393_RES_17;
	}
	if(this->reg.x_res == 2){
		this->raw.z &= ~MLX90393_RES_18;
	}

	//Check if temperature compensation is enabled. See 16.2.10
	//Convert raw data base on sensitivity
	if(this->reg.tcmp_en == 0){
		this->converted.x = (float)this->raw.x * sens_lookup_0xC[this->reg.gain][this->reg.x_res][0];
		this->converted.y = (float)this->raw.y * sens_lookup_0xC[this->reg.gain][this->reg.y_res][0];
		this->converted.z = (float)this->raw.z * sens_lookup_0xC[this->reg.gain][this->reg.z_res][1];
	}else{
		this->converted.x = (float)((uint16_t)this->raw.x * sens_lookup_0xC[this->reg.gain][this->reg.x_res][0]);
		this->converted.y = (float)((uint16_t)this->raw.y * sens_lookup_0xC[this->reg.gain][this->reg.y_res][0]);
		this->converted.z = (float)((uint16_t)this->raw.z * sens_lookup_0xC[this->reg.gain][this->reg.z_res][1]);
	}

	//Check hall config, see 16.2.4
	if(this->reg.hallconf == 0x0){
		this->converted.x = this->converted.x * 98 / 75;
		this->converted.y = this->converted.y * 98 / 75;
		this->converted.z = this->converted.z * 98 / 75;
	}
}

void MLX90393::mlx90393_set_zyxt(uint8_t set_zyxt){
	this->zyxt = set_zyxt;
}
