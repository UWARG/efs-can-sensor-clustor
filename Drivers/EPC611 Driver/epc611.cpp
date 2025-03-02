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

uint16_t EPC611::sendRecv(uint16_t data) {
	HAL_GPIO_WritePin(gpio_nss,nss_pin,GPIO_PIN_RESET);
	uint8_t send[2] = {data>>8, data&0x00FF}; // MSB first
	uint8_t received[2] = {0};
	HAL_SPI_TransmitReceive(&hspi1, send, received, 2, SPI_TIMEOUT);
	return received[0] <<8 | (received[1]);
	HAL_GPIO_WritePin(gpio_nss,nss_pin,GPIO_PIN_SET);
}

uint16_t EPC611::write(uint8_t data,uint8_t address) {
	uint16_t write = EPC_WRITE | data | ((address&0x1F) << 8);
	return EPC611::poll(write);
}
uint16_t EPC611::read(uint8_t address) {
	uint16_t write = EPC_READ | ((address&0x1F) << 8);
	return EPC611::poll(write);
}
uint16_t EPC611::pageSelect(uint8_t page) {
	uint16_t write = EPC_PAGE_SELECT | ((page&0x07) << 8);
	return EPC611::poll(write);
}
uint16_t EPC611::reset() {
	return EPC611::poll(EPC_RESET);
}
uint16_t EPC611::quit() {
	return EPC611::poll(EPC_QUIT);
}
uint16_t EPC611::nop() {
	return EPC611::poll(EPC_NOP);
}

// For polling read and write (Done after an epcRead or epcWrite)
/*uint16_t EPC611::poll(uint16_t last_response, int attempts) {
	uint16_t new_response = last_response;
	if(last_response == EPC_WRITE_NOT_DONE || last_response == EPC_READ_NOT_DONE) {
		for(int i=0;i<attempts && (new_response == EPC_WRITE_NOT_DONE || new_response == EPC_READ_NOT_DONE);i++) {
			new_response = EPC611::epcNop();
		}
	}
	return new_response;
}*/
uint16_t EPC611::poll(uint16_t data)
{
	uint16_t status = EPC611::sendRecv(data);
	while(status == EPC_WRITE_NOT_DONE || status == EPC_READ_NOT_DONE)
	{
		status = EPC611::sendRecv(data);
	}
	return status;
}

void EPC611::startTIM() {
	// Startup
	while(EPC611::nop() != EPC_IDLE);

}

void EPC611:startUHD()
{
	// Set mode
	pageSelect(4);
	write(0x38, 0x12);
	write(0x2F, 0x15); // Set 4 DCS Ultra High-Dynamic (UHD) mode

	// Set frequency
	write(0x01, 0x05); // Set frequency to 10 MHz

	// Set integration time
	pageSelect(5);
	write(0x01, 0x00);
	write(0xFF, 0x01);
	write(0x03, 0x02);
	write(0xFF, 0x03);
	write(0x07, 0x04);
	write(0xFF, 0x05);
	write(0x0F, 0x06);
	write(0xFF, 0x07);
	write(0x1F, 0x08);
	write(0xFF, 0x09);
	write(0x3F, 0x0A);
	write(0xFF, 0x0B);
	write(0x7F, 0x0C);
	write(0xFF, 0x0D);
	write(0xFF, 0x0E);
	write(0xFF, 0x0F);

	nop();

	// Start measurement
	pageSelect(2);
	write(0x01, 0x18); // Set trigger
	nop();
}

// Returns 8 rows of sums for the 8x8 TOF sensor
void EPC611::getFrameUHD(uint16_t DCS_frames[][]) // Gets 4 frames because that's what the chip does for some reason
{
	// Start frame measurement
	pageSelect(2);
	for(int j=0; j<4; j++) { // repeat for DCS0 - DCS3
		for(int i=0; i<4;i++) { // repeat for 4 double-rows
			uint16_t status = read(0x15);

			// Wait for data
			while(!dataReady())
			{
				status = read(0x15);
				nop();
			}
			uint16_t data1 = read(0x14);
			uint16_t data2 = read(0x14);
			uint16_t data3 = read(0x14);
			uint16_t data4 = read(0x14);
			nop();
			DCS_frames[j][UHD_READOUT[i*2]] = data1 << 8 | data2;
			DCS_frames[j][UHD_READOUT[i*2+1]] = data3 << 8 | data4;
		}
	}
}

bool EPC611::dataReady()
{
	return (HAL_GPIO_WritePin(gpio_data_rdy,data_rdy_pin) == GPIO_PIN_SET);
}
