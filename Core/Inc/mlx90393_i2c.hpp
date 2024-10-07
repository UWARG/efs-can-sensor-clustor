/*
 * i2c_magnetometer.h
 *
 *  Created on: Sep 14, 2024
 *      Author: henry
 */

#ifndef INC_MLX90393_I2C_HPP_
#define INC_MLX90393_I2C_HPP_

#define DEFAULT_I2C_ADDRESS 0x0C << 1 //Set I2C address
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
#define MLX90393_GAIN_MASK 0x0070	// Mask to clear bits 4-6
#define MLX90393_X_RES_MASK 0x00C0  // Mask to clear bits 5-6
#define MLX90393_Y_RES_MASK 0x0180  // Mask to clear bits 7-8
#define MLX90393_Z_RES_MASK 0x0600  // Mask to clear bits 9-10
#define MLX90393_X_RES_SHIFT 5       /**< Left-shift for x_res bits. */
#define MLX90393_Y_RES_SHIFT 7       /**< Left-shift for y_res bits. */
#define MLX90393_Z_RES_SHIFT 9       /**< Left-shift for z_res bits. */
#define MLX90393_HALL_CONF 0x0C     /**< Hall plate spinning rate adj. */

class MLX90393{
	public:
		MLX90393();
		bool mlx90393_i2c_SM(uint8_t *rx_data, uint8_t zyxt); //Start single measurement
		bool mlx90393_i2c_RM(uint8_t *rx_data, uint8_t zyxt); //Read measurement
		bool mlx90393_i2c_EX();
		bool mlx90393_i2c_RT();
		bool mlx90393_i2c_WR(uint8_t reg, uint8_t *tx_data);
		bool mlx90393_i2c_RR(uint8_t reg);
		bool mlx90393_has_error();
		int mlx90393_zyxt_set_bits(uint8_t zyxt);
		bool mlx90393_set_gain(uint8_t gain);
		bool mlx90393_get_gain();
		bool mlx90393_set_resolution(uint8_t x_res, uint8_t y_res, uint8_t z_res);
		bool mlx90393_get_resolution();
		bool mlx90393_set_filter();
		bool mlx90393_set_oversampling();
		bool mlx90393_set_trig_int();
		uint16_t mlx90393_decode_helper(uint8_t *data);
		void mlx90393_convert_raw();

		enum { BURST_MODE_BIT = 0x80, WAKE_ON_CHANGE_BIT = 0x40,
		       POLLING_MODE_BIT = 0x20, ERROR_BIT = 0x10, EEC_BIT = 0x08,
		       RESET_BIT = 0x04, D1_BIT = 0x02, D0_BIT = 0x01 };

		enum {
		  CMD_NOP = 0x00,
		  CMD_EXIT = 0x80,
		  CMD_START_BURST = 0x10,
		  CMD_WAKE_ON_CHANGE = 0x20,
		  CMD_START_MEASUREMENT = 0x30,
		  CMD_READ_MEASUREMENT = 0x40,
		  CMD_READ_REGISTER = 0x50,
		  CMD_WRITE_REGISTER = 0x60,
		  CMD_MEMORY_RECALL = 0xd0,
		  CMD_MEMORY_STORE = 0xe0,
		  CMD_RESET = 0xf0
		};

	private:
		uint16_t mlx90393_t;
		uint16_t mlx90393_x;
		uint16_t mlx90393_y;
		uint16_t mlx90393_z;
		uint8_t mlx90393_status;
		uint16_t mlx90393_reg_val;
		uint8_t mlx90393_gain;
		uint8_t mlx90393_x_res;
		uint8_t mlx90393_y_res;
		uint8_t mlx90393_z_res;

		void mlx90393_i2c_transmit(uint8_t *tx_data, uint8_t *rx_data, uint16_t tx_size, uint16_t rx_size);
		void mlx90393_decode(uint8_t *rx_data, uint8_t zyxt);

};









#endif /* INC_MLX90393_I2C_HPP_ */
