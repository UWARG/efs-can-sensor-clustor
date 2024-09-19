/*
 * i2c_magnetometer.h
 *
 *  Created on: Sep 14, 2024
 *      Author: henry
 */

#ifndef INC_MLX90393_I2C_H_
#define INC_MLX90393_I2C_H_

#define DEFAULT_I2C_ADDRESS 0x0F << 1 //Set I2C address
#define CS GPIO_PIN_5 //Set CS pin
// Flags to use with "zyxt" variables.
#define MLX90393_T  0x01  /* Temperature */
#define MLX90393_X  0x02  /* X-axis */
#define MLX90393_Y  0x04  /* Y-axis */
#define MLX90393_Z  0x08  /* Z-axis */


class MLX90393{
	public:
		MLX90393();
		void mlx90393_i2c_SM(uint8_t *rx_data, uint8_t zyxt); //Single measurement
		void mlx90393_i2c_RM(uint8_t *rx_data, uint8_t zyxt); //Read measurement
		int mlx90393_zyxt_set_bits(uint8_t zyxt);
		uint16_t mlx90393_decode_helper(uint8_t *data);


	private:
		uint16_t mlx90393_t;
		uint16_t mlx90393_x;
		uint16_t mlx90393_y;
		uint16_t mlx90393_z;

		void mlx90393_i2c_transmit(uint8_t *tx_data, uint8_t *rx_data, uint16_t tx_size, uint16_t rx_size);
		void mlx90393_decode(uint8_t *rx_data, uint8_t zyxt);

};









#endif /* INC_MLX90393_I2C_H_ */
