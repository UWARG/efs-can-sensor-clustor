/*
 * epc611.cpp
 *
 *  Created on: Sep 29, 2024
 *      Author: Playdream
 */

#include "epc611.hpp"
#include "stm32l4xx_hal_conf.h"

EPC611::EPC611(SPI_HandleTypeDef *spi, SPI_HandleTypeDef *nss_gpio_type, uint16_t nss_pin_out, SPI_HandleTypeDef *data_rdy_gpio_type, uint16_t data_rdy_pin_out) {
	epc_spi = spi;
	gpio_nss = nss_gpio_type;
	nss_pin = nss_pin_out;
	gpio_data_rdy = data_rdy_gpio_type;
	data_rdy_pin = data_rdy_pin_out;
}

EPC611::sendRecv(uint16_t data) {
	HAL_GPIO_WritePin(gpio_nss,nss_pin,GPIO_PIN_RESET);
	uint8_t send[2] = {data>>8, data&0x00FF}; // MSB first
	uint8_t received[2] = {0};
	HAL_SPI_TransmitReceive(&hspi1, send, received, 2, SPI_TIMEOUT);
	return received[0] | (received[1]<<8);
	HAL_GPIO_WritePin(gpio_nss,nss_pin,GPIO_PIN_SET);
}

uint16_t EPC611::write(uint8_t data,uint8_t address) {
	uint16_t write = EPC_WRITE | data | ((address&0x1F) << 8);
	return EPC611::epcSendRecv(write);
}
uint16_t EPC611::read(uint8_t address) {
	uint16_t write = EPC_READ | ((address&0x1F) << 8);
	return EPC611::epcSendRecv(write);
}
uint16_t EPC611::pageSelect(uint8_t page) {
	uint16_t write = EPC_PAGE_SELECT | ((page&0x07) << 8);
	return EPC611::epcSendRecv(write);
}
uint16_t EPC611::reset() {
	return EPC611::epcSendRecv(EPC_RESET);
}
uint16_t EPC611::quit() {
	return EPC611::epcSendRecv(EPC_QUIT);
}
uint16_t EPC611::nop() {
	return EPC611::epcSendRecv(EPC_NOP);
}

// For polling read and write (Done after an epcRead or epcWrite)
uint16_t EPC611::poll(uint16_t last_response, int attempts) {
	uint16_t new_response = last_response;
	if(last_response == EPC_WRITE_NOT_DONE || last_response == EPC_READ_NOT_DONE) {
		for(int i=0;i<attempts && (new_response == EPC_WRITE_NOT_DONE || new_response == EPC_READ_NOT_DONE);i++) {
			new_response = EPC611::epcNop();
		}
	}
	return new_response;
}

void EPC611::startTIM() {
	// Startup
	while(EPC611::nop() != EPC_IDLE);


}
