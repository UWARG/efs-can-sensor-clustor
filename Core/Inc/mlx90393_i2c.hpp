/*
 * i2c_magnetometer.h
 *
 *  Created on: Sep 14, 2024
 *      Author: henry
 */

#ifndef INC_MLX90393_I2C_HPP_
#define INC_MLX90393_I2C_HPP_

#define DEFAULT_I2C_ADDRESS 0x0F << 1 //Set I2C address
#define CS GPIO_PIN_5 //Set CS pin
// Flags to use with "zyxt" variables.
#define MLX90393_T  0x01  /* Temperature */
#define MLX90393_X  0x02  /* X-axis */
#define MLX90393_Y  0x04  /* Y-axis */
#define MLX90393_Z  0x08  /* Z-axis */
#define MLX90393_CONF1 0x00         /**< Gain */
#define MLX90393_CONF2 0x01         /**< Burst, comm mode */
#define MLX90393_CONF3 0x02         /**< Oversampling, filter, res. */
#define MLX90393_CONF4 0x03         /**< Sensitivty drift. */
#define MLX90393_GAIN_SHIFT 4       /**< Left-shift for gain bits. */
#define MLX90393_HALL_CONF 0x0C     /**< Hall plate spinning rate adj. */
#define MLX90393_STATUS_OK 0x00     /**< OK value for status response. */
#define MLX90393_STATUS_SMMODE 0x08 /**< SM Mode status response. */
#define MLX90393_STATUS_RESET 0x01  /**< Reset value for status response. */
#define MLX90393_STATUS_ERROR 0xFF  /**< OK value for status response. */
#define MLX90393_STATUS_MASK 0xFC   /**< Mask for status OK checks. */

class MLX90393{
	public:
		MLX90393();
		bool mlx90393_i2c_SM(uint8_t *rx_data, uint8_t zyxt); //Start single measurement
		bool mlx90393_i2c_RM(uint8_t *rx_data, uint8_t zyxt); //Read measurement
		int mlx90393_zyxt_set_bits(uint8_t zyxt);
		bool mlx90393_set_gain();
		bool mlx90393_set_resolution();
		bool mlx90393_set_filter();
		bool mlx90393_set_oversampling();
		bool mlx90393_set_trig_int();
		uint16_t mlx90393_decode_helper(uint8_t *data);



	private:
		uint16_t mlx90393_t;
		uint16_t mlx90393_x;
		uint16_t mlx90393_y;
		uint16_t mlx90393_z;
		uint8_t status;



		void mlx90393_i2c_transmit(uint8_t *tx_data, uint8_t *rx_data, uint16_t tx_size, uint16_t rx_size);
		void mlx90393_decode(uint8_t *rx_data, uint8_t zyxt);

};









#endif /* INC_MLX90393_I2C_HPP_ */
