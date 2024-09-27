/*
 * range_sensor.cpp
 *
 *  Created on: Sep 16, 2024
 *      Author: praka
 */

static void set_config(acc_detector_distance_config_t *detector_config, distance_preset_config_t preset)
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


static bool initialize_detector_resources(distance_detector_resources_t *resources)
{
	resources->handle = acc_detector_distance_create(resources->config);
	if (resources->handle == NULL)
	{
		printf("acc_detector_distance_create() failed\n");
		return false;
	}

	if (!acc_detector_distance_get_sizes(resources->handle, &(resources->buffer_size), &(resources->detector_cal_result_static_size)))
	{
		printf("acc_detector_distance_get_sizes() failed\n");
		return false;
	}

	resources->buffer = acc_integration_mem_alloc(resources->buffer_size);
	if (resources->buffer == NULL)
	{
		printf("sensor buffer allocation failed\n");
		return false;
	}

	resources->detector_cal_result_static = acc_integration_mem_alloc(resources->detector_cal_result_static_size);
	if (resources->detector_cal_result_static == NULL)
	{
		printf("calibration buffer allocation failed\n");
		return false;
	}

	return true;
}


static bool do_sensor_calibration(acc_sensor_t     *sensor,
                                  acc_cal_result_t *sensor_cal_result,
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
			status = acc_sensor_calibrate(sensor, &cal_complete, sensor_cal_result, buffer, buffer_size);

			if (status && !cal_complete)
			{
				status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
			}
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


static bool do_full_detector_calibration(distance_detector_resources_t *resources,
                                         const acc_cal_result_t        *sensor_cal_result)
{
	bool done = false;
	bool status;

	do
	{
		status = acc_detector_distance_calibrate(resources->sensor,
		                                         resources->handle,
		                                         sensor_cal_result,
		                                         resources->buffer,
		                                         resources->buffer_size,
		                                         resources->detector_cal_result_static,
		                                         resources->detector_cal_result_static_size,
		                                         &resources->detector_cal_result_dynamic,
		                                         &done);

		if (status && !done)
		{
			status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
		}
	} while (status && !done);

	return status;
}


static bool do_detector_calibration_update(distance_detector_resources_t *resources,
                                           const acc_cal_result_t        *sensor_cal_result)
{
	bool done = false;
	bool status;

	do
	{
		status = acc_detector_distance_update_calibration(resources->sensor,
		                                                  resources->handle,
		                                                  sensor_cal_result,
		                                                  resources->buffer,
		                                                  resources->buffer_size,
		                                                  &resources->detector_cal_result_dynamic,
		                                                  &done);

		if (status && !done)
		{
			status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
		}
	} while (status && !done);

	return status;
}


static bool do_detector_get_next(distance_detector_resources_t  *resources,
                                 const acc_cal_result_t         *sensor_cal_result,
                                 acc_detector_distance_result_t *result)
{
	bool result_available = false;

	do
	{
		if (!acc_detector_distance_prepare(resources->handle, resources->config, resources->sensor, sensor_cal_result, resources->buffer,
		                                   resources->buffer_size))
		{
			printf("acc_detector_distance_prepare() failed\n");
			return false;
		}

		if (!acc_sensor_measure(resources->sensor))
		{
			printf("acc_sensor_measure() failed\n");
			return false;
		}

		if (!acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS))
		{
			printf("Sensor interrupt timeout\n");
			return false;
		}

		if (!acc_sensor_read(resources->sensor, resources->buffer, resources->buffer_size))
		{
			printf("acc_sensor_read() failed\n");
			return false;
		}

		if (!acc_detector_distance_process(resources->handle, resources->buffer, resources->detector_cal_result_static,
		                                   &resources->detector_cal_result_dynamic,
		                                   &result_available, result))
		{
			printf("acc_detector_distance_process() failed\n");
			return false;
		}
	} while (!result_available);

	return true;
}


static void print_distance_result(const acc_detector_distance_result_t *result)
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

