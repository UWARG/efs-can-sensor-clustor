/*
 * range_sensor.cpp
 *
 *  Created on: Sep 16, 2024
 *      Author: praka
 */
// Copyright (c) Acconeer AB, 2022-2024
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "acc_definitions_a121.h"
#include "acc_detector_distance.h"
#include "acc_hal_definitions_a121.h"
#include "acc_hal_integration_a121.h"
#include "acc_integration.h"
#include "acc_integration_log.h"
#include "acc_rss_a121.h"
#include "acc_sensor.h"
#include "acc_version.h"
#include "range_sensor.hpp"

/** \example example_detector_distance.c
 * @brief This is an example on how the Detector Distance API can be used
 * @n
 * This example executes as follows:
 *   - Retrieve HAL integration
 *   - Initialize distance detector resources:
 *     + Create distance detector configuration
 *     + Update configuration settings
 *     + Create Distance detector handle
 *     + Create buffer for detector calibration data
 *     + Create buffer for sensor data
 *   - Create and calibrate the sensor
 *   - Calibrate the detector
 *   - Measure distances with the detector (loop):
 *     + Prepare sensor with the detector
 *     + Measure and wait until a read can be done
 *     + Process measurement and print the result
 *     + Handle "calibration_needed" indication
 *   - Cleanup:
 *     + Destroy detector configuration
 *     + Destroy detector handle
 *     + Destroy sensor data buffer
 *     + Destroy detector calibration data buffer
 */


void Rangefinder::cleanup()
{
	acc_hal_integration_sensor_disable(SENSOR_ID);
	acc_hal_integration_sensor_supply_off(SENSOR_ID);

	acc_detector_distance_config_destroy(resources_.config);
	acc_detector_distance_destroy(resources_.handle);

	acc_integration_mem_free(resources_.buffer);
	acc_integration_mem_free(resources_.detector_cal_result_static);

	if (resources_.sensor != NULL)
	{
		acc_sensor_destroy(resources_.sensor);
	}
}

void Rangefinder::set_config(acc_detector_distance_config_t *detector_config, distance_preset_config_t preset)
{
	// Add configuration of the detector here
	switch (preset)
	{
		case DISTANCE_PRESET_CONFIG_NONE:
			// Add configuration of the detector here
			break;

		case DISTANCE_PRESET_CONFIG_BALANCED:
			acc_detector_distance_config_start_set(detector_config, 0.25f);
			acc_detector_distance_config_end_set(detector_config, 3.0f);
			acc_detector_distance_config_max_step_length_set(detector_config, 0U);
			acc_detector_distance_config_max_profile_set(detector_config, ACC_CONFIG_PROFILE_5);
			acc_detector_distance_config_reflector_shape_set(detector_config, ACC_DETECTOR_DISTANCE_REFLECTOR_SHAPE_GENERIC);
			acc_detector_distance_config_peak_sorting_set(detector_config, ACC_DETECTOR_DISTANCE_PEAK_SORTING_STRONGEST);
			acc_detector_distance_config_threshold_method_set(detector_config, ACC_DETECTOR_DISTANCE_THRESHOLD_METHOD_CFAR);
			acc_detector_distance_config_threshold_sensitivity_set(detector_config, 0.5f);
			acc_detector_distance_config_signal_quality_set(detector_config, 15.0f);
			acc_detector_distance_config_close_range_leakage_cancellation_set(detector_config, false);
			break;

		case DISTANCE_PRESET_CONFIG_HIGH_ACCURACY:
			acc_detector_distance_config_start_set(detector_config, 0.25f);
			acc_detector_distance_config_end_set(detector_config, 3.0f);
			acc_detector_distance_config_max_step_length_set(detector_config, 2U);
			acc_detector_distance_config_max_profile_set(detector_config, ACC_CONFIG_PROFILE_3);
			acc_detector_distance_config_reflector_shape_set(detector_config, ACC_DETECTOR_DISTANCE_REFLECTOR_SHAPE_GENERIC);
			acc_detector_distance_config_peak_sorting_set(detector_config, ACC_DETECTOR_DISTANCE_PEAK_SORTING_STRONGEST);
			acc_detector_distance_config_threshold_method_set(detector_config, ACC_DETECTOR_DISTANCE_THRESHOLD_METHOD_CFAR);
			acc_detector_distance_config_threshold_sensitivity_set(detector_config, 0.5f);
			acc_detector_distance_config_signal_quality_set(detector_config, 20.0f);
			acc_detector_distance_config_close_range_leakage_cancellation_set(detector_config, false);
			break;
	}
}


bool Rangefinder::initialize_detector_resources()
{
	resources_.handle = acc_detector_distance_create(resources_.config);
	if (resources_.handle == NULL)
	{
		printf("acc_detector_distance_create() failed\n");
		return false;
	}

	if (!acc_detector_distance_get_sizes(resources_.handle, &(resources_.buffer_size), &(resources_.detector_cal_result_static_size)))
	{
		printf("acc_detector_distance_get_sizes() failed\n");
		return false;
	}

	resources_.buffer = acc_integration_mem_alloc(resources_.buffer_size);
	if (resources_.buffer == NULL)
	{
		printf("sensor buffer allocation failed\n");
		return false;
	}

	resources_.detector_cal_result_static = (uint8_t*) acc_integration_mem_alloc(resources_.detector_cal_result_static_size);
	if (resources_.detector_cal_result_static == NULL)
	{
		printf("calibration buffer allocation failed\n");
		return false;
	}

	return true;
}


bool Rangefinder::do_sensor_calibration(acc_sensor_t     *sensor,
                                  void             *buffer,
                                  uint32_t         buffer_size)
{
	bool           status              = false;
	bool           cal_complete        = false;
	const uint16_t calibration_retries = 1U;

	// Random disturbances may cause the calibration to fail. At failure, retry at least once.
	for (uint16_t i = 0; !status && (i <= calibration_retries); i++)
	{
		// Reset sensor before calibration by disabling/enabling it
		acc_hal_integration_sensor_disable(SENSOR_ID);
		acc_hal_integration_sensor_enable(SENSOR_ID);

		do
		{
			status = acc_sensor_calibrate(sensor, &cal_complete, &sensor_cal_result_, buffer, buffer_size);

			if (status && !cal_complete)
			{
				status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
			}
			// vTaskDelay() this!
		} while (status && !cal_complete);
	}

	if (status)
	{
		/* Reset sensor after calibration by disabling/enabling it */
		acc_hal_integration_sensor_disable(SENSOR_ID);
		acc_hal_integration_sensor_enable(SENSOR_ID);
	}

	return status;
}


bool Rangefinder::do_full_detector_calibration()
{
	bool done = false;
	bool status;

	do
	{
		status = acc_detector_distance_calibrate(resources_.sensor,
		                                         resources_.handle,
		                                         &sensor_cal_result_,
		                                         resources_.buffer,
		                                         resources_.buffer_size,
		                                         resources_.detector_cal_result_static,
		                                         resources_.detector_cal_result_static_size,
		                                         &resources_.detector_cal_result_dynamic,
		                                         &done);

		if (status && !done)
		{
			status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
		}
		// vTaskDelay() this!
	} while (status && !done);

	return status;
}


bool Rangefinder::do_detector_calibration_update()
{
	bool done = false;
	bool status;

	do
	{
		status = acc_detector_distance_update_calibration(resources_.sensor,
		                                                  resources_.handle,
														  &sensor_cal_result_,
		                                                  resources_.buffer,
		                                                  resources_.buffer_size,
		                                                  &resources_.detector_cal_result_dynamic,
		                                                  &done);

		if (status && !done)
		{
			status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
		}
		// vTaskDelay() this!
	} while (status && !done);

	return status;
}


bool Rangefinder::do_detector_get_next(acc_detector_distance_result_t *result)
{
	bool result_available = false;

	do
	{
		if (!acc_detector_distance_prepare(resources_.handle, resources_.config, resources_.sensor, &sensor_cal_result_, resources_.buffer,
		                                   resources_.buffer_size))
		{
			printf("acc_detector_distance_prepare() failed\n");
			return false;
		}

		if (!acc_sensor_measure(resources_.sensor))
		{
			printf("acc_sensor_measure() failed\n");
			return false;
		}

		if (!acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS))
		{
			printf("Sensor interrupt timeout\n");
			return false;
		}

		if (!acc_sensor_read(resources_.sensor, resources_.buffer, resources_.buffer_size))
		{
			printf("acc_sensor_read() failed\n");
			return false;
		}

		if (!acc_detector_distance_process(resources_.handle, resources_.buffer, resources_.detector_cal_result_static,
		                                   &resources_.detector_cal_result_dynamic,
		                                   &result_available, result))
		{
			printf("acc_detector_distance_process() failed\n");
			return false;
		}
		// vTaskDelay() this!
	} while (!result_available);

	return true;
}


void Rangefinder::print_distance_result(const acc_detector_distance_result_t *result)
{
	printf("%d detected distances", result->num_distances);
	if (result->num_distances > 0)
	{
		printf(": ");

		for (uint8_t i = 0; i < result->num_distances; i++)
		{
			printf("%" PRIfloat " m ", ACC_LOG_FLOAT_TO_INTEGER(result->distances[i]));
		}
	}

	printf("\n");
}

Rangefinder::Rangefinder():
		resources_ {0},
		sensor_cal_result_{0},
		result_{0}
		{
		printf("Acconeer software version %s\n", acc_version_get());

		const acc_hal_a121_t *hal = acc_hal_rss_integration_get_implementation();

		if (!acc_rss_hal_register(hal))
		{
			exit(1);
		}

		resources_.config = acc_detector_distance_config_create();
		if (resources_.config == NULL)
		{
			printf("acc_detector_distance_config_create() failed\n");
			cleanup();
			exit(1);
		}

		set_config(resources_.config, DISTANCE_PRESET_CONFIG_BALANCED);

		if (!initialize_detector_resources())
		{
			printf("Initializing detector resources failed\n");
			cleanup();
			exit(1);
		}

		// Print the configuration
		acc_detector_distance_config_log(resources_.handle, resources_.config);

		/* Turn the sensor on */
		acc_hal_integration_sensor_supply_on(SENSOR_ID);
		acc_hal_integration_sensor_enable(SENSOR_ID);

		resources_.sensor = acc_sensor_create(SENSOR_ID);
		if (resources_.sensor == NULL)
		{
			printf("acc_sensor_create() failed\n");
			cleanup();
			exit(1);
		}

		if (!do_sensor_calibration(resources_.sensor, resources_.buffer, resources_.buffer_size))
		{
			printf("Sensor calibration failed\n");
			cleanup();
			exit(1);
		}

		if (!do_full_detector_calibration())
		{
			printf("Detector calibration failed\n");
			cleanup();
			exit(1);
		}
}

int Rangefinder::get_distance_task() {
	while (1) {
		acc_detector_distance_result_t result = { 0 };

		if (!do_detector_get_next(&result))
		{
			printf("Could not get next result\n");
			cleanup(&resources);
			return EXIT_FAILURE;
		}

		/* If "calibration needed" is indicated, the sensor needs to be recalibrated and the detector calibration updated */
		if (result.calibration_needed)
		{
			printf("Sensor recalibration and detector calibration update needed ... \n");

			if (!do_sensor_calibration(resources.sensor, &sensor_cal_result, resources.buffer, resources.buffer_size))
			{
				printf("Sensor calibration failed\n");
				cleanup(&resources);
				return EXIT_FAILURE;
			}

			/* Once the sensor is recalibrated, the detector calibration should be updated and measuring can continue. */
			if (!do_detector_calibration_update(&resources))
			{
				printf("Detector calibration update failed\n");
				cleanup(&resources);
				return EXIT_FAILURE;
			}

			printf("Sensor recalibration and detector calibration update done!\n");
		}
		else
		{
			print_distance_result(&result);
		}

		// vTaskDelay() this!
	}
}

