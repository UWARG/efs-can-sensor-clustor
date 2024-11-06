// Copyright (c) Acconeer AB, 2024
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "acc_config.h"
#include "acc_hal_definitions_a121.h"
#include "acc_hal_integration_a121.h"
#include "acc_integration.h"
#include "acc_processing.h"
#include "acc_rss_a121.h"
#include "acc_sensor.h"
#include "acc_version.h"
#include "example_hand_motion_detection.h"
#include "example_hand_motion_detection_main.h"

#define SENSOR_ID         (1U)
#define SENSOR_TIMEOUT_MS (1000U)

/**
 * @brief Frees any allocated resources
 */
static void cleanup(acc_sensor_t *sensor, void *buffer, hand_motion_detection_config_t *config, hand_motion_detection_handle_t *handle);

/**
 * @brief Performs sensor calibration (with retry)
 */
static bool do_sensor_calibration(acc_sensor_t *sensor, acc_cal_result_t *cal_result, void *buffer, uint32_t buffer_size);

/**
 * @brief Handle sensor indications
 */
static bool handle_indications(hand_motion_detection_handle_t *handle,
                               hand_motion_detection_result_t *result,
                               acc_sensor_t                   *sensor,
                               acc_cal_result_t               *cal_result,
                               void                           *buffer,
                               uint32_t                        buffer_size,
                               bool                           *data_reliable);

/**
 * @brief Print a processor result in a human-readable format
 */
static void print_hand_motion_detection_result(const hand_motion_detection_result_t *result);

int acc_example_hand_motion_detection_main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
	acc_sensor_t    *sensor = NULL;
	acc_cal_result_t cal_result;
	void            *buffer      = NULL;
	uint32_t         buffer_size = 0;

	hand_motion_detection_config_t *hand_motion_detection_config = NULL;
	hand_motion_detection_handle_t *hand_motion_detection_handle = NULL;
	hand_motion_detection_result_t  hand_motion_detection_result = {0};

	printf("Acconeer software version %s\n", acc_version_get());

	const acc_hal_a121_t *hal = acc_hal_rss_integration_get_implementation();

	if (!acc_rss_hal_register(hal))
	{
		return EXIT_FAILURE;
	}

	hand_motion_detection_config = hand_motion_detection_config_create();

	if (hand_motion_detection_config == NULL)
	{
		printf("hand_motion_detection_config_create() failed\n");
		cleanup(sensor, buffer, hand_motion_detection_config, hand_motion_detection_handle);
		return EXIT_FAILURE;
	}

	hand_motion_detection_set_config(HAND_MOTION_DETECTION_PRESET_FAUCET, hand_motion_detection_config);

	hand_motion_detection_handle = hand_motion_detection_handle_create(hand_motion_detection_config, SENSOR_ID);
	if (hand_motion_detection_handle == NULL)
	{
		printf("hand_motion_detection_handle_create() failed\n");
		cleanup(sensor, buffer, hand_motion_detection_config, hand_motion_detection_handle);
		return EXIT_FAILURE;
	}

	hand_motion_detection_config_log(hand_motion_detection_handle);

	if (!hand_motion_detection_get_buffer_size(hand_motion_detection_handle, &buffer_size))
	{
		printf("acc_rss_get_buffer_size() failed\n");
		cleanup(sensor, buffer, hand_motion_detection_config, hand_motion_detection_handle);
		return EXIT_FAILURE;
	}

	buffer = acc_integration_mem_alloc(buffer_size);
	if (buffer == NULL)
	{
		printf("buffer allocation failed\n");
		cleanup(sensor, buffer, hand_motion_detection_config, hand_motion_detection_handle);
		return EXIT_FAILURE;
	}

	acc_hal_integration_sensor_supply_on(SENSOR_ID);
	acc_hal_integration_sensor_enable(SENSOR_ID);

	sensor = acc_sensor_create(SENSOR_ID);
	if (sensor == NULL)
	{
		printf("acc_sensor_create() failed\n");
		cleanup(sensor, buffer, hand_motion_detection_config, hand_motion_detection_handle);
		return EXIT_FAILURE;
	}

	if (!do_sensor_calibration(sensor, &cal_result, buffer, buffer_size))
	{
		printf("do_sensor_calibration() failed\n");
		acc_sensor_status(sensor);
		cleanup(sensor, buffer, hand_motion_detection_config, hand_motion_detection_handle);
		return EXIT_FAILURE;
	}

	while (true)
	{
		if (!hand_motion_detection_prepare(hand_motion_detection_handle, sensor, &cal_result, buffer, buffer_size, false))
		{
			printf("hand_motion_detection_prepare() failed\n");
			cleanup(sensor, buffer, hand_motion_detection_config, hand_motion_detection_handle);
			return EXIT_FAILURE;
		}

		if (!acc_sensor_measure(sensor))
		{
			printf("acc_sensor_measure failed\n");
			acc_sensor_status(sensor);
			cleanup(sensor, buffer, hand_motion_detection_config, hand_motion_detection_handle);
			return EXIT_FAILURE;
		}

		if (!acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS))
		{
			printf("Sensor interrupt timeout\n");
			acc_sensor_status(sensor);
			cleanup(sensor, buffer, hand_motion_detection_config, hand_motion_detection_handle);
			return EXIT_FAILURE;
		}

		if (!acc_sensor_read(sensor, buffer, buffer_size))
		{
			printf("acc_sensor_read failed\n");
			acc_sensor_status(sensor);
			cleanup(sensor, buffer, hand_motion_detection_config, hand_motion_detection_handle);
			return EXIT_FAILURE;
		}

		hand_motion_detection_process(hand_motion_detection_handle, buffer, &hand_motion_detection_result);

		bool data_reliable;

		if (!handle_indications(
		        hand_motion_detection_handle, &hand_motion_detection_result, sensor, &cal_result, buffer, buffer_size, &data_reliable))
		{
			printf("handle_indications() failed\n");
			cleanup(sensor, buffer, hand_motion_detection_config, hand_motion_detection_handle);
			return EXIT_FAILURE;
		}

		if (data_reliable)
		{
			print_hand_motion_detection_result(&hand_motion_detection_result);
		}
	}

	cleanup(sensor, buffer, hand_motion_detection_config, hand_motion_detection_handle);

	printf("Application finished OK\n");

	return EXIT_SUCCESS;
}

static void cleanup(acc_sensor_t *sensor, void *buffer, hand_motion_detection_config_t *config, hand_motion_detection_handle_t *handle)
{
	acc_hal_integration_sensor_disable(SENSOR_ID);
	acc_hal_integration_sensor_supply_off(SENSOR_ID);

	if (sensor != NULL)
	{
		acc_sensor_destroy(sensor);
	}

	if (buffer != NULL)
	{
		acc_integration_mem_free(buffer);
	}

	if (config != NULL)
	{
		hand_motion_detection_config_destroy(config);
	}

	if (handle != NULL)
	{
		hand_motion_detection_handle_destroy(handle);
	}
}

static bool do_sensor_calibration(acc_sensor_t *sensor, acc_cal_result_t *cal_result, void *buffer, uint32_t buffer_size)
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
			status = acc_sensor_calibrate(sensor, &cal_complete, cal_result, buffer, buffer_size);

			if (status && !cal_complete)
			{
				status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
			}
		} while (status && !cal_complete);
	}

	if (status)
	{
		// Reset sensor after calibration by disabling/enabling it
		acc_hal_integration_sensor_disable(SENSOR_ID);
		acc_hal_integration_sensor_enable(SENSOR_ID);
	}

	return status;
}

static bool handle_indications(hand_motion_detection_handle_t *handle,
                               hand_motion_detection_result_t *result,
                               acc_sensor_t                   *sensor,
                               acc_cal_result_t               *cal_result,
                               void                           *buffer,
                               uint32_t                        buffer_size,
                               bool                           *data_reliable)
{
	bool status = true;

	*data_reliable = true;

	if (result->proc_result.data_saturated)
	{
		*data_reliable = false;

		printf("Data saturated. Try to reduce the sensor gain.\n");
	}

	if (result->proc_result.frame_delayed)
	{
		printf("Frame delayed. Could not read data fast enough.\n");
		printf("Try lowering the frame rate or call 'acc_sensor_read' more frequently.\n");
	}

	if (result->proc_result.calibration_needed)
	{
		printf("The current calibration is not valid for the current temperature.\n");
		printf("Re-calibrating sensor...\n");

		if (status)
		{
			if (!do_sensor_calibration(sensor, cal_result, buffer, buffer_size))
			{
				printf("do_sensor_calibration() failed\n");
				acc_sensor_status(sensor);
				status = false;
			}
			else
			{
				printf("Sensor recalibration done!\n");
			}
		}

		if (status)
		{
			if (!hand_motion_detection_prepare(handle, sensor, cal_result, buffer, buffer_size, true))
			{
				printf("hand_motion_detection_prepare() failed\n");
				status = false;
			}
		}

		*data_reliable = false;

		printf("The sensor was successfully re-calibrated.\n");
	}

	return status;
}

static void print_hand_motion_detection_result(const hand_motion_detection_result_t *result)
{
	printf("App mode: %s\n", result->app_mode == HAND_MOTION_DETECTION_APP_MODE_PRESENCE ? "presence" : "handmotion");

	if (result->algo_result_available)
	{
		printf("Detection state: ");

		switch (result->algo_result.detection_state)
		{
			case HAND_MOTION_DETECTION_STATE_NO_DETECTION:
				printf("No detection\n");
				break;
			case HAND_MOTION_DETECTION_STATE_DETECTION:
				printf("Detection\n");
				break;
			case HAND_MOTION_DETECTION_STATE_RETENTION:
				printf("Retention\n");
				break;
		}
	}
}
