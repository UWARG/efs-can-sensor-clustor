/*
 * range_sensor.hpp
 *
 *  Created on: Sep 16, 2024
 *      Author: praka
 */

#ifndef INC_RANGE_SENSOR_HPP_
#define INC_RANGE_SENSOR_HPP_

#include <cstdint>

typedef enum
{
	DISTANCE_PRESET_CONFIG_NONE = 0,
	DISTANCE_PRESET_CONFIG_BALANCED,
	DISTANCE_PRESET_CONFIG_HIGH_ACCURACY,
} distance_preset_config_t;

typedef struct
{
	acc_sensor_t                      *sensor;
	acc_detector_distance_config_t    *config;
	acc_detector_distance_handle_t    *handle;
	void                              *buffer;
	uint32_t                          buffer_size;
	uint8_t                           *detector_cal_result_static;
	uint32_t                          detector_cal_result_static_size;
	acc_detector_cal_result_dynamic_t detector_cal_result_dynamic;
} distance_detector_resources_t;

#define SENSOR_ID (1U)
// 2 seconds should be enough even for long ranges and high signal quality
#define SENSOR_TIMEOUT_MS (2000U)

class Rangefinder {
	public:
	Rangefinder(); //SPI_HandleTypeDef* spi, int chip_select_pin);

	int get_distance(); // isnt this accomplished in get_next
	private:
	distance_detector_resources_t resources_;
	acc_cal_result_t sensor_cal_result_;
	acc_detector_distance_result_t result_;

	void cleanup();
	void set_config(acc_detector_distance_config_t *detector_config, distance_preset_config_t preset);
	bool initialize_detector_resources();
	bool do_sensor_calibration(acc_sensor_t     *sensor,
									  void             *buffer,
									  uint32_t         buffer_size);
	bool do_full_detector_calibration();
	bool do_detector_calibration_update();
	bool do_detector_get_next(acc_detector_distance_result_t *result);
	void print_distance_result(const acc_detector_distance_result_t *result);


};

#endif /* INC_RANGE_SENSOR_HPP_ */
