/*
 * i2c_magnetometer.h
 *
 *  Created on: Sep 14, 2024
 *      Author: henry
 */

#ifndef INC_MLX90393_I2C_H_
#define INC_MLX90393_I2C_H_

#define DEFAULT_I2C_ADDRESS 0x0F

class MLX90393
{
	public:
		MLX90393();
		bool begin_I2C(uint8_t i2c_addr = DEFAULT_I2C_ADDRESS);

	private:

};

void mlx90393_i2c_init(void);

void mlx90393_i2c_transmit(uint8_t *tx_data, uint8_t *rx_data, uint16_t tx_size, uint16_t rx_size);

void mlx90393_i2c_SM(uint8_t *rx_data, uint8_t zyxt);

void mlx90393_i2c_RM(uint8_t *rx_data, uint8_t zyxt);

int zyxt_set_bits(uint8_t zyxt);


#endif /* INC_MLX90393_I2C_H_ */
