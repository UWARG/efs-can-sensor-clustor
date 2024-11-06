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
#include "acc_integration_log.h"
#include "acc_processing.h"
#include "acc_rss_a121.h"
#include "acc_sensor.h"
#include "acc_version.h"
#include "example_waste_level.h"
#include "example_waste_level_main.h"

#define SENSOR_ID         (1U)
#define SENSOR_TIMEOUT_MS (1000U)

/**
 * @brief Frees any allocated resources
 */
static void cleanup(acc_processing_t         *processing,
                    acc_sensor_t             *sensor,
                    void                     *buffer,
                    waste_level_handle_t     *handle,
                    waste_level_app_config_t *app_config);

/**
 * @brief Performs sensor calibration (with retry) and sensor prepare
 */
static bool do_sensor_calibration_and_prepare(acc_sensor_t *sensor, acc_config_t *config, void *buffer, uint32_t buffer_size);

/**
 * @brief Print a processor result in a human-readable format
 */
static void print_waste_level_result(const waste_level_result_t *result);

int acc_example_waste_level_main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
	acc_processing_t         *processing  = NULL;
	acc_sensor_t             *sensor      = NULL;
	void                     *buffer      = NULL;
	uint32_t                  buffer_size = 0;
	acc_processing_metadata_t proc_meta;
	acc_processing_result_t   proc_result;

	waste_level_handle_t     *waste_level_handle = NULL;
	waste_level_app_config_t *app_config         = NULL;

	waste_level_result_t waste_level_result = {0};

	printf("Acconeer software version %s\n", acc_version_get());

	const acc_hal_a121_t *hal = acc_hal_rss_integration_get_implementation();

	if (!acc_rss_hal_register(hal))
	{
		return EXIT_FAILURE;
	}

	app_config = waste_level_app_config_create();
	if (app_config == NULL)
	{
		printf("waste_level_app_config_create() failed\n");
		cleanup(processing, sensor, buffer, waste_level_handle, app_config);
		return EXIT_FAILURE;
	}

	waste_level_app_config_set_preset(WASTE_LEVEL_PRESET_PLASTIC_WASTE_BIN, app_config);

	waste_level_processing_config_log(&app_config->processing_config);

	acc_config_log(app_config->sensor_config);

	waste_level_handle = waste_level_handle_create(app_config);
	if (waste_level_handle == NULL)
	{
		printf("waste_level_handle_create() failed\n");
		cleanup(processing, sensor, buffer, waste_level_handle, app_config);
		return EXIT_FAILURE;
	}

	processing = acc_processing_create(app_config->sensor_config, &proc_meta);
	if (processing == NULL)
	{
		printf("acc_processing_create() failed\n");
		cleanup(processing, sensor, buffer, waste_level_handle, app_config);
		return EXIT_FAILURE;
	}

	if (!acc_rss_get_buffer_size(app_config->sensor_config, &buffer_size))
	{
		printf("acc_rss_get_buffer_size() failed\n");
		cleanup(processing, sensor, buffer, waste_level_handle, app_config);
		return EXIT_FAILURE;
	}

	buffer = acc_integration_mem_alloc(buffer_size);
	if (buffer == NULL)
	{
		printf("buffer allocation failed\n");
		cleanup(processing, sensor, buffer, waste_level_handle, app_config);
		return EXIT_FAILURE;
	}

	acc_hal_integration_sensor_supply_on(SENSOR_ID);
	acc_hal_integration_sensor_enable(SENSOR_ID);

	sensor = acc_sensor_create(SENSOR_ID);
	if (sensor == NULL)
	{
		printf("acc_sensor_create() failed\n");
		cleanup(processing, sensor, buffer, waste_level_handle, app_config);
		return EXIT_FAILURE;
	}

	if (!do_sensor_calibration_and_prepare(sensor, app_config->sensor_config, buffer, buffer_size))
	{
		printf("do_sensor_calibration_and_prepare() failed\n");
		acc_sensor_status(sensor);
		cleanup(processing, sensor, buffer, waste_level_handle, app_config);
		return EXIT_FAILURE;
	}

	while (true)
	{
		if (!acc_sensor_measure(sensor))
		{
			printf("acc_sensor_measure failed\n");
			acc_sensor_status(sensor);
			cleanup(processing, sensor, buffer, waste_level_handle, app_config);
			return EXIT_FAILURE;
		}

		if (!acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS))
		{
			printf("Sensor interrupt timeout\n");
			acc_sensor_status(sensor);
			cleanup(processing, sensor, buffer, waste_level_handle, app_config);
			return EXIT_FAILURE;
		}

		if (!acc_sensor_read(sensor, buffer, buffer_size))
		{
			printf("acc_sensor_read failed\n");
			acc_sensor_status(sensor);
			cleanup(processing, sensor, buffer, waste_level_handle, app_config);
			return EXIT_FAILURE;
		}

		acc_processing_execute(processing, buffer, &proc_result);

		if (proc_result.calibration_needed)
		{
			printf("The current calibration is not valid for the current temperature.\n");
			printf("The sensor needs to be re-calibrated.\n");

			if (!do_sensor_calibration_and_prepare(sensor, app_config->sensor_config, buffer, buffer_size))
			{
				printf("do_sensor_calibration_and_prepare() failed\n");
				acc_sensor_status(sensor);
				cleanup(processing, sensor, buffer, waste_level_handle, app_config);
				return EXIT_FAILURE;
			}

			printf("The sensor was successfully re-calibrated.\n");
		}
		else if (proc_result.data_saturated)
		{
			printf("Data is saturated. Try to reduce the sensor gain\n");
		}
		else
		{
			waste_level_process(waste_level_handle, app_config, &proc_meta, proc_result.frame, &waste_level_result);

			print_waste_level_result(&waste_level_result);
		}
	}

	cleanup(processing, sensor, buffer, waste_level_handle, app_config);

	printf("Application finished OK\n");

	return EXIT_SUCCESS;
}

static void cleanup(acc_processing_t         *processing,
                    acc_sensor_t             *sensor,
                    void                     *buffer,
                    waste_level_handle_t     *handle,
                    waste_level_app_config_t *app_config)
{
	acc_hal_integration_sensor_disable(SENSOR_ID);
	acc_hal_integration_sensor_supply_off(SENSOR_ID);

	if (sensor != NULL)
	{
		acc_sensor_destroy(sensor);
	}

	if (processing != NULL)
	{
		acc_processing_destroy(processing);
	}

	if (buffer != NULL)
	{
		acc_integration_mem_free(buffer);
	}

	if (app_config != NULL)
	{
		waste_level_app_config_destroy(app_config);
	}

	if (handle != NULL)
	{
		waste_level_handle_destroy(handle);
	}
}

static bool do_sensor_calibration_and_prepare(acc_sensor_t *sensor, acc_config_t *config, void *buffer, uint32_t buffer_size)
{
	bool             status       = false;
	bool             cal_complete = false;
	acc_cal_result_t cal_result;
	const uint16_t   calibration_retries = 1U;

	// Random disturbances may cause the calibration to fail. At failure, retry at least once.
	for (uint16_t i = 0; !status && (i <= calibration_retries); i++)
	{
		// Reset sensor before calibration by disabling/enabling it
		acc_hal_integration_sensor_disable(SENSOR_ID);
		acc_hal_integration_sensor_enable(SENSOR_ID);

		do
		{
			status = acc_sensor_calibrate(sensor, &cal_complete, &cal_result, buffer, buffer_size);

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

		status = acc_sensor_prepare(sensor, config, &cal_result, buffer, buffer_size);
	}

	return status;
}

static void print_waste_level_result(const waste_level_result_t *result)
{
	if (result->level_found)
	{
		printf("Fill level %" PRIu8 "%% (%" PRIfloat "m)\n", result->level_percent, ACC_LOG_FLOAT_TO_INTEGER(result->level_m));
	}
	else
	{
		printf("No level found\n");
	}
}
