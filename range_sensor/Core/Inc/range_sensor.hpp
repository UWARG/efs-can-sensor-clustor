/*
 * range_sensor.hpp
 *
 *  Created on: Sep 16, 2024
 *      Author: praka
 */

#ifndef INC_RANGE_SENSOR_HPP_
#define INC_RANGE_SENSOR_HPP_

#include <cstdint>

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

void read_radar (void);

class Rangefinder {
	public:

	void set_config(acc_detector_distance_config_t *detector_config, distance_preset_config_t preset);
	bool initialize_detector_resources(distance_detector_resources_t *resources);

};

#endif /* INC_RANGE_SENSOR_HPP_ */
