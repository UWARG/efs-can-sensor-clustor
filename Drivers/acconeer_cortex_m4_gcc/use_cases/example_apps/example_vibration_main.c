// Copyright (c) Acconeer AB, 2023-2024
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <float.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "acc_alg_basic_utils.h"
#include "acc_algorithm.h"
#include "acc_config.h"
#include "acc_definitions_a121.h"
#include "acc_definitions_common.h"
#include "acc_hal_definitions_a121.h"
#include "acc_hal_integration_a121.h"
#include "acc_integration.h"
#include "acc_integration_log.h"
#include "acc_processing.h"
#include "acc_rss_a121.h"
#include "acc_sensor.h"
#include "acc_version.h"

#include "example_vibration.h"

/** \example example_vibration_main.c
 * @brief This is an example on how the Service API can be used to measure
 *        vibrations
 * @n
 * The example executes as follows:
 *   - Set vibration configuration
 *   - Create vibration processing handle using config from previous step
 *   - Create a processing instance using an internally translated config from previous step
 *   - Allocate sensor buffer
 *   - Create a sensor instance
 *   - Calibrate & prepare the sensor
 *   - Loop:
 *     - Perform a sensor measurement and read out the data
 *     - Check & handle the 'calibration_needed' indication
 *     - Process the measurement and print vibration result
 *   - Destroy the sensor instance
 *   - Destroy the processing instance
 *   - Destroy the configuration
 */

#define SENSOR_ID         (1U)
#define SENSOR_TIMEOUT_MS (1000U)

#define DISPLACEMENT_HISTORY_COLUMN_WIDTH (16U)

static bool do_sensor_calibration_and_prepare(acc_sensor_t *sensor, void *buffer, uint32_t buffer_size, const acc_config_t *sensor_config);

static void print_result(acc_vibration_handle_t *handle, acc_vibration_result_t *result);

static void cleanup(acc_sensor_t *sensor, acc_processing_t *processing, void *buffer, acc_vibration_handle_t *handle);

int acc_example_vibration_main(int argc, char *argv[]);

int acc_example_vibration_main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
	acc_sensor_t             *sensor      = NULL;
	void                     *buffer      = NULL;
	uint32_t                  buffer_size = 0U;
	acc_processing_t         *processing  = NULL;
	acc_processing_metadata_t proc_meta   = {0};
	acc_processing_result_t   proc_result = {0};

	acc_vibration_handle_t *handle = NULL;
	acc_vibration_result_t  result = {0};
	acc_vibration_config_t  config = {0};

	/** Vibration example configuration **/
	// Use a preset as a starting point. The call below will also initialize the config struct.
	acc_vibration_preset_set(&config, ACC_VIBRATION_PRESET_LOW_FREQUENCY);

	// If needed, adjust configuration parameters before creating a vibration handle.
	// Refer to the config struct definition in 'example_vibration.h' to see what parameters are available.

	printf("Acconeer software version %s\n", acc_version_get());

	const acc_hal_a121_t *hal = acc_hal_rss_integration_get_implementation();

	if (!acc_rss_hal_register(hal))
	{
		return EXIT_FAILURE;
	}

	acc_vibration_config_log(&config);

	/*
	 * The handle will use the config passed to it here
	 * for its lifetime. Make sure any changes to
	 * 'config' happens before this point.
	 */
	handle = acc_vibration_handle_create(&config);
	if (handle == NULL)
	{
		printf("acc_vibration_handle_create() failed\n");
		cleanup(sensor, processing, buffer, handle);
		return EXIT_FAILURE;
	}

	acc_config_log(acc_vibration_handle_sensor_config_get(handle));

	processing = acc_processing_create(acc_vibration_handle_sensor_config_get(handle), &proc_meta);
	if (processing == NULL)
	{
		printf("acc_processing_create() failed\n");
		return EXIT_FAILURE;
	}

	if (!acc_rss_get_buffer_size(acc_vibration_handle_sensor_config_get(handle), &buffer_size))
	{
		printf("acc_rss_get_buffer_size() failed\n");
		cleanup(sensor, processing, buffer, handle);
		return EXIT_FAILURE;
	}

	buffer = acc_integration_mem_alloc(buffer_size);
	if (buffer == NULL)
	{
		printf("buffer allocation failed\n");
		cleanup(sensor, processing, buffer, handle);
		return EXIT_FAILURE;
	}

	acc_hal_integration_sensor_supply_on(SENSOR_ID);
	acc_hal_integration_sensor_enable(SENSOR_ID);

	sensor = acc_sensor_create(SENSOR_ID);
	if (sensor == NULL)
	{
		printf("acc_sensor_create() failed\n");
		return EXIT_FAILURE;
	}

	if (!do_sensor_calibration_and_prepare(sensor, buffer, buffer_size, acc_vibration_handle_sensor_config_get(handle)))
	{
		printf("do_sensor_calibration_and_prepare() failed\n");
		acc_sensor_status(sensor);
		cleanup(sensor, processing, buffer, handle);
		return EXIT_FAILURE;
	}

	while (true)
	{
		if (!acc_sensor_measure(sensor))
		{
			printf("acc_sensor_measure failed\n");
			acc_sensor_status(sensor);
			cleanup(sensor, processing, buffer, handle);
			return EXIT_FAILURE;
		}

		if (!acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS))
		{
			printf("Sensor interrupt timeout\n");
			acc_sensor_status(sensor);
			cleanup(sensor, processing, buffer, handle);
			return EXIT_FAILURE;
		}

		if (!acc_sensor_read(sensor, buffer, buffer_size))
		{
			printf("acc_sensor_read failed\n");
			acc_sensor_status(sensor);
			cleanup(sensor, processing, buffer, handle);
			return EXIT_FAILURE;
		}

		acc_processing_execute(processing, buffer, &proc_result);

		if (proc_result.calibration_needed)
		{
			printf("The current calibration is not valid for the current temperature.\n");
			printf("The sensor needs to be re-calibrated.\n");

			if (!do_sensor_calibration_and_prepare(sensor, buffer, buffer_size, acc_vibration_handle_sensor_config_get(handle)))
			{
				printf("do_sensor_calibration_and_prepare() failed\n");
				acc_sensor_status(sensor);
				cleanup(sensor, processing, buffer, handle);
				return EXIT_FAILURE;
			}

			printf("The sensor was successfully re-calibrated.\n");
		}
		else
		{
			acc_vibration_process(&proc_result, handle, &config, &result);
			print_result(handle, &result);
		}
	}

	cleanup(sensor, processing, buffer, handle);

	printf("Application finished OK\n");

	return EXIT_SUCCESS;
}

static bool do_sensor_calibration_and_prepare(acc_sensor_t *sensor, void *buffer, uint32_t buffer_size, const acc_config_t *sensor_config)
{
	bool             status       = false;
	bool             cal_complete = false;
	acc_cal_result_t cal_result   = {0};

	const uint16_t calibration_retries = 1U;

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

		status = acc_sensor_prepare(sensor, sensor_config, &cal_result, buffer, buffer_size);
	}

	return status;
}

static void print_result(acc_vibration_handle_t *handle, acc_vibration_result_t *result)
{
	char   buf[80]  = "";
	size_t buf_size = sizeof(buf);

	if (result->max_displacement != FLT_MAX)
	{
		snprintf(buf,
		         buf_size,
		         ", max_displacement=%" PRIfloat " um, max_displacement_freq=%" PRIfloat " Hz",
		         ACC_LOG_FLOAT_TO_INTEGER(result->max_displacement),
		         ACC_LOG_FLOAT_TO_INTEGER(result->max_displacement_freq));
	}

	printf("Vibration: max_sweep_amplitude=%" PRIfloat " um%s\n", ACC_LOG_FLOAT_TO_INTEGER(result->max_sweep_amplitude), buf);

	bool continuous_data_acquisition = true;
	if (acc_vibration_handle_continuous_data_acquisition_get(handle, &continuous_data_acquisition) && continuous_data_acquisition)
	{
		return;
	}

	uint16_t     num_elems            = 0U;
	const float *displacement_history = acc_vibration_handle_displacement_history_get(handle, &num_elems);

	if (displacement_history != NULL)
	{
		printf("\nDisplacement history:\n");

		for (uint16_t i = 0; i < num_elems; i++)
		{
			printf("%" PRIfloat " ", ACC_LOG_FLOAT_TO_INTEGER(displacement_history[i]));
			if ((i % DISPLACEMENT_HISTORY_COLUMN_WIDTH) == 0)
			{
				printf("\n");
			}
		}

		printf("\n\n");
	}
}

static void cleanup(acc_sensor_t *sensor, acc_processing_t *processing, void *buffer, acc_vibration_handle_t *handle)
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

	if (handle != NULL)
	{
		acc_vibration_handle_destroy(handle);
	}
}
