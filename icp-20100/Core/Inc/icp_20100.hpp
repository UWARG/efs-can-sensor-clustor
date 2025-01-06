/*
 *
 */

#ifndef INC_ICP_20100_HPP_
#define INC_ICP_20100_HPP_

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
//#include <cstdint>

#define ICP20100_ADDR 0x0C << 1 // shifted left for 7-bit addressing
//do i use reset address or another address?

// Register addresses
#define MODE_SELECT_REG 0xC0
#define INTERRUPT_MASK_REG 0xC2
#define PRESS_ABS_LSB_REG 0xC7

// Mode and Interrupt configurations
#define MODE_SELECT_CONTINUOUS 0x8B // 10001011 in binary
#define INTERRUPT_MASK_PRESSURE_READY 0x47 // 10001111 in binary

class ICP_20100{
	public:
		ICP_20100(I2C_HandleTypeDef *hi2c);
		void ICP20100_Init(void);
		uint16_t ICP20100_ReadPressure(void);
};

#endif /* INC_ICP_20100_HPP_ */
