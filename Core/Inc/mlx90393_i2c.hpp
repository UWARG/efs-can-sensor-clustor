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
#define MLX90393_T  0x01  // Temperature
#define MLX90393_X  0x02  // X-axis
#define MLX90393_Y  0x04  // Y-axis
#define MLX90393_Z  0x08  // Z-axis

#define MLX90393_CONF1 0x00         // Gain
#define MLX90393_CONF2 0x01         // Burst, comm mode
#define MLX90393_CONF3 0x02         // Oversampling, filter, res
#define MLX90393_CONF4 0x03         // Sensitivty drift
//Shifts and masks
#define MLX90393_GAIN_SHIFT 4       // Left-shift for gain bits
#define MLX90393_GAIN_MASK 0x0070	// Mask to clear bits 4-6
#define MLX90393_X_RES_MASK 0x00C0  // Mask to clear bits 5-6
#define MLX90393_Y_RES_MASK 0x0180  // Mask to clear bits 7-8
#define MLX90393_Z_RES_MASK 0x0600  // Mask to clear bits 9-10
#define MLX90393_X_RES_SHIFT 5       // Left-shift for x_res bits
#define MLX90393_Y_RES_SHIFT 7       // Left-shift for y_res bits
#define MLX90393_Z_RES_SHIFT 9       // Left-shift for z_res bits
#define MLX90393_HALL_CONF 0x0C     // Hall plate spinning rate adj
#define MLX90393_RES_17 0x8000		// Mask to remove sign bit for res = 2 (MSB = bit 17)
#define MLX90393_RES_18 0x4000		// Mask to remove sign bit for res = 3 (MSB = bit 18)
//Commands
#define CMD_NOP 0x00
#define	CMD_EXIT 0x80
#define	CMD_START_BURST 0x10
#define	CMD_WAKE_ON_CHANGE 0x20
#define	CMD_START_MEASUREMENT 0x30
#define	CMD_READ_MEASUREMENT 0x40
#define	CMD_READ_REGISTER 0x50
#define	CMD_WRITE_REGISTER 0x60
#define	CMD_MEMORY_RECALL 0xD0
#define	CMD_MEMORY_STORE 0xE0
#define	CMD_RESET 0xF0
//Status bits
#define BURST_MODE_BIT 0x80
#define WAKE_ON_CHANGE_BIT 0x40
#define POLLING_MODE_BIT 0x20
#define ERROR_BIT 0x10
#define EEC_BIT 0x08,
#define RESET_BIT 0x04
#define D1_BIT 0x02
#define D0_BIT 0x01

class MLX90393{
	public:
		MLX90393();
		bool mlx90393_i2c_SM(uint8_t *rx_data, uint8_t zyxt); //Start single measurement mode cmd
		bool mlx90393_i2c_RM(uint8_t *rx_data, uint8_t zyxt); //Read measurement cmd
		bool mlx90393_i2c_EX(); //Exit mode cmd
		bool mlx90393_i2c_RT(); //Reset cmd
		bool mlx90393_i2c_WR(uint8_t reg, uint8_t *tx_data); //Write register cmd
		bool mlx90393_i2c_RR(uint8_t reg); // Read register cmd
		bool mlx90393_has_error(); //Check status bit error
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

	private:
		typedef struct raw{
		    uint16_t t;
		    uint16_t x;
		    uint16_t y;
		    uint16_t z;
		} raw;

		typedef struct converted{
		    float t;
		    float x;
		    float y;
		    float z;
		} converted;

		typedef struct reg{
			uint8_t stat;
			uint16_t val;
			uint8_t gain;
			uint8_t x_res;
			uint8_t y_res;
			uint8_t z_res;
			uint8_t hallconf;
			uint8_t tcmp_en;
		}reg;

		const float sens_lookup_0xC[8][4][2] = {
	        /* GAIN_SEL = 0, 5x gain */
	        {{0.751, 1.210}, {1.502, 2.420}, {3.004, 4.840}, {6.009, 9.680}},
	        /* GAIN_SEL = 1, 4x gain */
	        {{0.601, 0.968}, {1.202, 1.936}, {2.403, 3.872}, {4.840, 7.744}},
	        /* GAIN_SEL = 2, 3x gain */
	        {{0.451, 0.726}, {0.901, 1.452}, {1.803, 2.904}, {3.605, 5.808}},
	        /* GAIN_SEL = 3, 2.5x gain */
	        {{0.376, 0.605}, {0.751, 1.210}, {1.502, 2.420}, {3.004, 4.840}},
	        /* GAIN_SEL = 4, 2x gain */
	        {{0.300, 0.484}, {0.601, 0.968}, {1.202, 1.936}, {2.403, 3.872}},
	        /* GAIN_SEL = 5, 1.667x gain */
	        {{0.250, 0.403}, {0.501, 0.807}, {1.001, 1.613}, {2.003, 3.227}},
	        /* GAIN_SEL = 6, 1.333x gain */
	        {{0.200, 0.323}, {0.401, 0.645}, {0.801, 1.291}, {1.602, 2.581}},
	        /* GAIN_SEL = 7, 1x gain */
	        {{0.150, 0.242}, {0.300, 0.484}, {0.601, 0.968}, {1.202, 1.936}},
		};

		void mlx90393_i2c_transmit(uint8_t *tx_data, uint8_t *rx_data, uint16_t tx_size, uint16_t rx_size);
		void mlx90393_decode(uint8_t *rx_data, uint8_t zyxt);
		void mlx90393_convert();

};









#endif /* INC_MLX90393_I2C_HPP_ */
