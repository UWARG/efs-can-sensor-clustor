// Copyright (c) Acconeer AB, 2024
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "acc_hal_definitions_a121.h"
#include "acc_hal_integration_a121.h"
#include "acc_integration.h"
#include "acc_rss_a121.h"
#include "acc_version.h"
#include "ref_app_parking.h"

#define SENSOR_ID (1U)

int acc_ref_app_parking_main(int argc, char *argv[]);

int acc_ref_app_parking_main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	ref_app_parking_config_t  parking_config = {0};
	ref_app_parking_handle_t *handle         = NULL;

	printf("Acconeer software version %s\n", acc_version_get());

	const acc_hal_a121_t *hal = acc_hal_rss_integration_get_implementation();

	if (!acc_rss_hal_register(hal))
	{
		return EXIT_FAILURE;
	}

	ref_app_parking_set_config(PARKING_PRESET_GROUND, &parking_config);

	handle = ref_app_parking_handle_create(&parking_config, SENSOR_ID);
	if (handle == NULL)
	{
		printf("ref_app_parking_handle_create() failed\n");
		ref_app_parking_handle_destroy(handle);
		return EXIT_FAILURE;
	}

	if (!ref_app_parking_sensor_calibration(handle))
	{
		printf("ref_app_parking_sensor_calibration() failed\n");
		ref_app_parking_handle_destroy(handle);
		return EXIT_FAILURE;
	}

	if (!ref_app_parking_noise_calibration(handle))
	{
		printf("ref_app_parking_noise_calibration() failed\n");
		ref_app_parking_handle_destroy(handle);
		return EXIT_FAILURE;
	}

	if (parking_config.obstruction_detection_enabled)
	{
		if (!ref_app_parking_obstruction_calibration(handle))
		{
			printf("ref_app_parking_obstruction_calibration() failed\n");
			ref_app_parking_handle_destroy(handle);
			return EXIT_FAILURE;
		}
	}

	if (!ref_app_parking_sensor_prepare(handle))
	{
		printf("ref_app_parking_sensor_prepare() failed\n");
		ref_app_parking_handle_destroy(handle);
		return EXIT_FAILURE;
	}

	bool obstruction_detected = false;
	bool car_detected         = false;

	while (true)
	{
		if (!ref_app_parking_measure(handle, parking_config.frame_rate_app_driven))
		{
			printf("ref_app_parking_measure() failed\n");
			ref_app_parking_handle_destroy(handle);
			return EXIT_FAILURE;
		}

		bool data_reliable;

		if (!ref_app_parking_handle_indications(handle, &data_reliable))
		{
			printf("ref_app_parking_handle_indications() failed\n");
			ref_app_parking_handle_destroy(handle);
			return EXIT_FAILURE;
		}

		if (data_reliable)
		{
			if (parking_config.obstruction_detection_enabled)
			{
				ref_app_parking_obstruction_process(handle, &obstruction_detected);

				printf("obstruction_detected: %s\n", obstruction_detected ? "true" : "false");
			}

			ref_app_parking_process(handle, &car_detected);

			printf("car_detected: %s\n", car_detected ? "true" : "false");
		}
	}

	ref_app_parking_handle_destroy(handle);

	printf("Application finished OK\n");

	return EXIT_SUCCESS;
}
