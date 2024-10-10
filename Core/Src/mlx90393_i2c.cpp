/*
 * mlx90393_i2c.c
 *
 *  Created on: Sep 13, 2024
 *      Author: Henry
 */
#include <mlx90393_i2c.hpp>

MLX90393::MLX90393(){
	HAL_GPIO_WritePin(GPIOB, CS, GPIO_PIN_SET);
}

void MLX90393::mlx90393_i2c_transmit(uint8_t *tx_data, uint8_t *rx_data, uint16_t tx_size, uint16_t rx_size)
{
	uint8_t rx_buf[rx_size + 2];
	HAL_I2C_Master_Transmit(&hi2c3, DEFAULT_I2C_ADDRESS, tx_data, tx_size, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c3, DEFAULT_I2C_ADDRESS + 1, rx_buf, rx_size + 1, HAL_MAX_DELAY);
	reg.stat = rx_buf[0];
	for(int i = 0; i < rx_size; i++){
		rx_data[i] = rx_buf[i + 1];
	}
}

bool MLX90393::mlx90393_i2c_SM(uint8_t *rx_data, uint8_t zyxt)
{
	uint8_t tx_data = CMD_START_MEASUREMENT | zyxt;
	mlx90393_i2c_transmit(tx_data, rx_data, 1, 1);
	return !(mlx_90393_has_error() || (reg.stat & POLLING_MODE_BIT) == 0);
}

bool MLX90393::mlx90393_i2c_RM(uint8_t *rx_data, uint8_t zyxt){
	int rx_size = zyxt_set_bits(zyxt) + 1;
	uint8_t tx_data = CMD_READ_MEASUREMENT | zyxt;
	mlx90393_i2c_transmit(tx_data, rx_data, 1, 2 * rx_size + 1);
	mlx90393_decode(rx_data, zyxt);
	mlx90393_convert();
	return !mlx_90393_has_error();
}

bool MLX90393::mlx90393_i2c_EX(){
	mlx90393_i2c_transmit(CMD_EXIT, nullptr, 1, 0);
	return !mlx90393_has_error();
}

bool MLX90393::mlx90393_i2c_RT(){
	mlx90393_i2c_transmit(CMD_RESET, nullptr, 1, 0);
	return !(mlx90393_has_error() || (mlx90393_stat & RESET_BIT) == 0);
}

bool MLX90393::mlx90393_i2c_WR(uint8_t reg, uint8_t *tx_data){
	if(HAL_I2C_Mem_Write(&hi2c3, DEFAULT_I2C_ADDRESS, 0x00 << 2, I2C_MEMADD_SIZE_16BIT, tx_data, 1, HAL_MAX_DELAY) != HAL_OK){
		return false;
	}
	return true;
}
bool MLX90393::mlx90393_i2c_RR(uint8_t reg){
	if(HAL_I2C_Mem_Write(&hi2c3, DEFAULT_I2C_ADDRESS, reg << 2, I2C_MEMADD_SIZE_16BIT, &reg.val, 1, HAL_MAX_DELAY) != HAL_OK){
		return false;
	}
	return true;
}

bool mlx90393_set_gain(uint8_t gain){
	if(!mlx90393_i2c_RR(MLX90393_CONF1)){
		return false;
	}
	uint16_t data = reg.val & MLX90393_GAIN_MASK;
	data |= gain << MLX90393_GAIN_SHIFT;
	reg.gain = gain;
	return(mlx90393_i2c_WR(MLX90393_CONF1, &data));
}

bool mlx90393_get_gain(){
	if(!mlx90393_i2c_RR(MLX90393_CONF1)){
		return false;
	}
	uint16_t data = reg.val & MLX90393_GAIN_MASK;
	reg.gain = data >> MLX90393_GAIN_SHIFT;
	return true;
}

bool mlx90393_set_resolution(uint8_t x_res, uint8_t y_res, uint8_t z_res){
	if(!mlx90393_i2c_RR(MLX90393_CONF3)){
		return false;
	}
	reg.x_res = x_res;
	reg.y_res = y_res;
	reg.z_res = z_res;
	uint16_t data = mlx90393_reg_val;
	if(x_res != 0){
		data = (data & MLX90393_X_RES_MASK) | (x_res << MLX90393_X_RES_SHIFT);
	}
	if(y_res != 0){
		data = (data & MLX90393_Y_RES_MASK) | (y_res << MLX90393_Y_RES_SHIFT);
	}
	if(z_res != 0){
		data = (data & MLX90393_Z_RES_MASK) | (z_res << MLX90393_Z_RES_SHIFT);
	}
	return(mlx90393_i2c_WR(MLX90393_CONF3, &data));
}


bool mlx90393_get_resolution(){
	if(mlx90393_i2c_RR(MLX90393_CONF3)){
		return false;
	}
	uint16_t data = reg.val;
	reg.x_res = (data & MLX90393_X_RES_MASK) >> MLX90393_X_RES_SHIFT;
	reg.y_res = (data & MLX90393_Y_RES_MASK) >> MLX90393_Y_RES_SHIFT;
	reg.z_res = (data & MLX90393_Z_RES_MASK) >> MLX90393_Z_RES_SHIFT;
	return true;
}

bool MLX90393::mlx90393_has_error(){
	return (reg.stat & ERROR_BIT) != 0;
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
	//Convert raw data base on sensitivity
	converted.x = (float)raw.x * sens_lookup_0xC[reg.gain][reg.rex_x][0];
	converted.y = (float)raw.y * sens_lookup_0xC[reg.gain][reg.rex_y][0];
	converted.z = (float)raw.z * sens_lookup_0xC[reg.gain][reg.rex_z][1];
}

void MLX90393::mlx90393_convert_raw(){

}
