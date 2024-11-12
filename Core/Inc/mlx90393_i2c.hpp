/*
 * i2c_magnetometer.h
 *
 *  Created on: Sep 14, 2024
 *      Author: henry
 */

#ifndef INC_MLX90393_I2C_HPP_
#define INC_MLX90393_I2C_HPP_

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include <cstdint>

#define DEFAULT_I2C_ADDRESS 0x18 << 1 //Set I2C address
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
#define MLX90393_X_RES_MASK 0x0060  // Mask to clear bits 5-6
#define MLX90393_Y_RES_MASK 0x0180  // Mask to clear bits 7-8
#define MLX90393_Z_RES_MASK 0x0600  // Mask to clear bits 9-10
#define MLX90393_X_RES_SHIFT 5       // Left-shift for x_res bits
#define MLX90393_Y_RES_SHIFT 7       // Left-shift for y_res bits
#define MLX90393_Z_RES_SHIFT 9       // Left-shift for z_res bits
#define MLX90393_RES_17_MASK 0x8000		// Mask to remove sign bit for res = 2 (MSB = bit 17)
#define MLX90393_RES_18_MASK 0x4000		// Mask to remove sign bit for res = 3 (MSB = bit 18)
#define MLX90393_FILTER_MASK 0x001C
#define MLX90393_FILTER_SHIFT 2
#define MLX90393_OSR_MASK 0x1800
#define MLX90393_OSR_SHIFT 11
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
//Gain selection
#define MLX90393_GAIN_5X 0x00
#define MLX90393_GAIN_4X 0x01
#define MLX90393_GAIN_3X 0x02
#define MLX90393_GAIN_2_5X 0x03
#define MLX90393_GAIN_2X 0x04
#define MLX90393_GAIN_1_67X 0x05
#define MLX90393_GAIN_1_33X 0x06
#define MLX90393_GAIN_1X 0x07
//Res selection
#define MLX90393_RES_15 0x00
#define MLX90393_RES_16 0x01
#define MLX90393_RES_17 0x02
#define MLX90393_RES_18 0x03
//OSR selection
#define MLX90393_OSR_0 0x00
#define MLX90393_OSR_1 0x01
#define MLX90393_OSR_2 0x02
#define MLX90393_OSR_3 0x03
//Filter Selection
#define MLX90393_FILTER_0 0x00
#define MLX90393_FILTER_1 0x01
#define MLX90393_FILTER_2 0x02
#define MLX90393_FILTER_3 0x03
#define MLX90393_FILTER_4 0x04
#define MLX90393_FILTER_5 0x05
#define MLX90393_FILTER_6 0x06
#define MLX90393_FILTER_7 0x07

//See 16.2.4
const float sens_lookup_0xC[8][4][2] = {
	/* GAIN_SEL = 0, 5x gain */
	{ { 0.751, 1.210 }, { 1.502, 2.420 }, { 3.004, 4.840 }, { 6.009, 9.680 } },
	/* GAIN_SEL = 1, 4x gain */
	{ { 0.601, 0.968 }, { 1.202, 1.936 }, { 2.403, 3.872 }, { 4.840, 7.744 } },
	/* GAIN_SEL = 2, 3x gain */
	{ { 0.451, 0.726 }, { 0.901, 1.452 }, { 1.803, 2.904 }, { 3.605, 5.808 } },
	/* GAIN_SEL = 3, 2.5x gain */
	{ { 0.376, 0.605 }, { 0.751, 1.210 }, { 1.502, 2.420 }, { 3.004, 4.840 } },
	/* GAIN_SEL = 4, 2x gain */
	{ { 0.300, 0.484 }, { 0.601, 0.968 }, { 1.202, 1.936 }, { 2.403, 3.872 } },
	/* GAIN_SEL = 5, 1.667x gain */
	{ { 0.250, 0.403 }, { 0.501, 0.807 }, { 1.001, 1.613 }, { 2.003, 3.227 } },
	/* GAIN_SEL = 6, 1.333x gain */
	{ { 0.200, 0.323 }, { 0.401, 0.645 }, { 0.801, 1.291 }, { 1.602, 2.581 } },
	/* GAIN_SEL = 7, 1x gain */
	{ { 0.150, 0.242 }, { 0.300, 0.484 }, { 0.601, 0.968 }, { 1.202, 1.936 } },
};

const float mlx90393_tconv[8][4] = {
    /* DIG_FILT = 0 */
    {1.27, 1.84, 3.00, 5.30},
    /* DIG_FILT = 1 */
    {1.46, 2.23, 3.76, 6.84},
    /* DIG_FILT = 2 */
    {1.84, 3.00, 5.30, 9.91},
    /* DIG_FILT = 3 */
    {2.61, 4.53, 8.37, 16.05},
    /* DIG_FILT = 4 */
    {4.15, 7.60, 14.52, 28.34},
    /* DIG_FILT = 5 */
    {7.22, 13.75, 26.80, 52.92},
    /* DIG_FILT = 6 */
    {13.36, 26.04, 51.38, 102.07},
    /* DIG_FILT = 7 */
    {25.65, 50.61, 100.53, 200.37},
};

class MLX90393{
	public:
		MLX90393(I2C_HandleTypeDef *hi2c, bool *flag);
		bool i2c_SM(); //Start single measurement mode cmd
		bool i2c_RM(); //Read measurement cmd
		bool i2c_EX(); //Exit mode cmd
		bool i2c_RT(); //Reset cmd
		bool i2c_WR(uint8_t reg, uint16_t tx_data); //Write register cmd
		bool i2c_RR(uint8_t reg); // Read register cmd
		bool i2c_set_gain(uint8_t gain);
		bool i2c_get_gain();
		bool i2c_set_resolution(uint8_t x_res, uint8_t y_res, uint8_t z_res);
		bool i2c_get_resolution();
		bool i2c_set_filter(uint8_t filter);
		bool i2c_get_filter();
		bool i2c_set_oversampling(uint8_t osr);
		bool i2c_get_oversampling();
		bool i2c_has_error();
		bool i2c_read_data();

	private:
		struct RawData{
		    int16_t t;
		    int16_t x;
		    int16_t y;
		    int16_t z;
		} ;

		struct ConvertedData{
		    float t;
		    float x;
		    float y;
		    float z;
		};

		struct RegVal{
			uint8_t stat;
			uint16_t val;
			uint8_t gain;
			uint8_t x_res;
			uint8_t y_res;
			uint8_t z_res;
			uint8_t hallconf;
			uint8_t tcmp_en;
			uint8_t filter;
			uint8_t osr;
		};

		volatile struct RawData raw;
		volatile struct ConvertedData converted;
		volatile struct RegVal reg;
		I2C_HandleTypeDef *hi2c;
		uint8_t zyxt;
		bool *flag;

		HAL_StatusTypeDef i2c_transceive(uint8_t *tx_data, uint8_t *rx_data, uint16_t tx_size, uint16_t rx_size);
		void decode(uint8_t *rx_data);
		void convert();
		int16_t decode_helper(uint8_t *data);
		int zyxt_set_bits();
		void set_zyxt(uint8_t set_zyxt);
};

#endif /* INC_MLX90393_I2C_HPP_ */
