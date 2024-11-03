/*
 * epc611.h
 *
 *  Created on: Sep 29, 2024
 *      Author: Playdream
 */

#ifndef INC_EPC611_HPP_
#define INC_EPC611_HPP_

const uint16_t EPC_NOP = 0x0;
const uint16_t EPC_QUIT = 0x6000;
const uint16_t EPC_RESET = 0xC000;

// EPC Command Flags
const uint16_t EPC_READ = 0x2000;
const uint16_t EPC_WRITE = 0x4000;
const uint16_t EPC_PAGE_SELECT = 0x8000;

// EPC Response Codes
const uint16_t EPC_IDLE = 0x0;
const uint16_t EPC_SPI_NOT_READY = 0xFFFF;  // Poll with NOP until you get NOP
const uint16_t EPC_SYS_NOT_READY = 0xEBFF;  // Poll with NOP until you get NOP
const uint16_t EPC_ERROR = 0xF58E;          // You messed up something
const uint16_t EPC_QUIT_RESPONSE = 0xE38E;
const uint16_t EPC_WRITE_NOT_DONE = 0xCCCC; // Poll with NOP until you get WRITE_DONE
const uint16_t EPC_READ_NOT_DONE = 0x7333;  // Poll with NOP until you get READ_DONE

// EPC Response Flags
const uint16_t EPC_FLAG = 0xE000; // 1110 0000 0000 0000
const uint16_t EPC_READ_DONE = 0x2000;
const uint16_t EPC_WRITE_DONE = 0x4000;
const uint16_t EPC_PAGE_RESPONSE = 0x8000;

const uint32_t SPI_TIMEOUT = 10; // ms

class EPC611 {
	public:

		EPC611(SPI_HandleTypeDef *spi, SPI_HandleTypeDef *nss_gpio_type, uint16_t nss_pin_out, SPI_HandleTypeDef *data_rdy_gpio_type, uint16_t data_rdy_pin_out);

		uint8_t sendRecv(uint16_t data);
		uint16_t write(uint8_t data,uint8_t address);
		uint16_t read(uint8_t address);
		uint16_t pageSelect(uint8_t page);
		uint16_t reset();
		uint16_t quit();
		uint16_t nop();
		uint16_t poll(uint16_t last_response, int attempts);
		void startTIM();

	private:
		SPI_HandleTypeDef *epc_spi;

		GPIO_TypeDef *gpio_nss;
		uint16_t nss_pin;

		GPIO_TypeDef *gpio_data_rdy;
		uint16_t data_rdy_pin;

};

#endif /* INC_EPC611_HPP_ */
