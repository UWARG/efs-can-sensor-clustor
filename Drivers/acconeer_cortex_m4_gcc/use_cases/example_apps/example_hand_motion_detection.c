// Copyright (c) Acconeer AB, 2024
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <math.h>
#include <stddef.h>
#include <stdio.h>

#include "acc_algorithm.h"
#include "acc_config.h"
#include "acc_detector_presence.h"
#include "acc_detector_presence_processing.h"
#include "acc_integration.h"
#include "acc_integration_log.h"
#include "acc_processing.h"
#include "acc_rss_a121.h"
#include "example_hand_motion_detection.h"

#define MODULE "example_hand_motion_detection"

struct hand_motion_detection_handle
{
	hand_motion_detection_config_t config;
	bool                           prepare_needed;

	bool     has_detected;
	uint32_t update_index;
	uint32_t update_index_at_detection;

	acc_sensor_id_t                            sensor_id;
	acc_config_t                              *hand_motion_detection_sensor_config;
	acc_processing_t                          *sensor_processing;
	acc_processing_metadata_t                  sensor_processing_metadata;
	acc_detector_presence_processing_config_t *hand_motion_detection_presence_processing_config;
	acc_detector_presence_processing_handle_t *hand_motion_detection_presence_processor;

	acc_detector_presence_handle_t  *presence_detector;
	acc_detector_presence_metadata_t presence_metadata;

	hand_motion_detection_app_mode_t app_mode;
	uint16_t                         hand_motion_detection_timeout_duration;
	uint16_t                         hand_motion_detection_timer;
	uint16_t                         detection_retention_duration;
};

static void reinit_hand_motion_detection_state(hand_motion_detection_handle_t *handle);

static void set_sensor_config(const hand_motion_detection_config_t *hand_motion_detection_config, acc_config_t *sensor_config);

static void set_processing_config(const hand_motion_detection_config_t      *hand_motion_detection_config,
                                  acc_detector_presence_processing_config_t *processing_config);

static hand_motion_detection_detection_state_t process_presence_processing_result(
    hand_motion_detection_handle_t                  *handle,
    const acc_detector_presence_processing_result_t *presence_processing_result);

static void determine_app_mode(hand_motion_detection_handle_t *handle, hand_motion_detection_result_t *result);

static void swap_app_mode(hand_motion_detection_handle_t *handle, hand_motion_detection_app_mode_t new_app_mode);

hand_motion_detection_config_t *hand_motion_detection_config_create(void)
{
	hand_motion_detection_config_t *config = acc_integration_mem_calloc(1U, sizeof(*config));

	if (config != NULL)
	{
		config->presence_config = acc_detector_presence_config_create();

		if (config->presence_config == NULL)
		{
			hand_motion_detection_config_destroy(config);
			config = NULL;
		}
	}

	return config;
}

void hand_motion_detection_config_destroy(hand_motion_detection_config_t *config)
{
	if (config != NULL)
	{
		if (config->presence_config != NULL)
		{
			acc_detector_presence_config_destroy(config->presence_config);
		}

		acc_integration_mem_free(config);
	}
}

void hand_motion_detection_set_config(hand_motion_detection_preset_t preset, hand_motion_detection_config_t *config)
{
	switch (preset)
	{
		case HAND_MOTION_DETECTION_PRESET_NONE:
			break;
		case HAND_MOTION_DETECTION_PRESET_FAUCET:
			// Hand motion algo settings
			config->algo_config.sensor_to_water_distance     = 0.12f;
			config->algo_config.water_jet_width              = 0.02f;
			config->algo_config.measurement_range_end        = 0.2f;
			config->algo_config.filter_time_const            = 0.3f;
			config->algo_config.threshold                    = 1.5f;
			config->algo_config.detection_retention_duration = 1.0f;

			// Sensor settings
			config->algo_config.hwaas            = 32U;
			config->algo_config.sweeps_per_frame = 64U;
			config->algo_config.sweep_rate       = 1000.0f;
			config->algo_config.frame_rate       = 10.0f;

			// Presence settings
			acc_detector_presence_config_reset_filters_on_prepare_set(config->presence_config, true);
			acc_detector_presence_config_frame_rate_set(config->presence_config, 2.0f);
			acc_detector_presence_config_start_set(config->presence_config, 0.7f);
			acc_detector_presence_config_end_set(config->presence_config, 0.7f);
			acc_detector_presence_config_sweeps_per_frame_set(config->presence_config, 32U);
			acc_detector_presence_config_inter_frame_deviation_time_const_set(config->presence_config, 0.01f);
			acc_detector_presence_config_inter_output_time_const_set(config->presence_config, 0.01f);
			acc_detector_presence_config_intra_frame_time_const_set(config->presence_config, 0.01f);
			acc_detector_presence_config_intra_output_time_const_set(config->presence_config, 0.01f);

			// Application settings
			config->hand_detection_timeout = 5.0f;
			config->use_presence_detection = true;
			break;
		default:
			break;
	}
}

hand_motion_detection_handle_t *hand_motion_detection_handle_create(const hand_motion_detection_config_t *hand_motion_detection_config,
                                                                    acc_sensor_id_t                       sensor_id)
{
	hand_motion_detection_handle_t *handle = acc_integration_mem_calloc(1U, sizeof(*handle));

	bool status = handle != NULL;

	if (status)
	{
		handle->config         = *hand_motion_detection_config;
		handle->prepare_needed = true;
		handle->app_mode       = HAND_MOTION_DETECTION_APP_MODE_HAND_MOTION;
		handle->sensor_id      = sensor_id;

		handle->hand_motion_detection_timeout_duration =
		    (uint16_t)(roundf(hand_motion_detection_config->hand_detection_timeout * hand_motion_detection_config->algo_config.frame_rate));
		handle->hand_motion_detection_timer  = 0U;
		handle->detection_retention_duration = (uint16_t)(roundf(hand_motion_detection_config->algo_config.detection_retention_duration *
		                                                         hand_motion_detection_config->algo_config.frame_rate));

		reinit_hand_motion_detection_state(handle);

		handle->hand_motion_detection_sensor_config              = acc_config_create();
		handle->hand_motion_detection_presence_processing_config = acc_detector_presence_processing_config_create();

		status = handle->hand_motion_detection_sensor_config != NULL && handle->hand_motion_detection_presence_processing_config != NULL;
	}

	if (status)
	{
		set_sensor_config(hand_motion_detection_config, handle->hand_motion_detection_sensor_config);
		set_processing_config(hand_motion_detection_config, handle->hand_motion_detection_presence_processing_config);

		handle->sensor_processing = acc_processing_create(handle->hand_motion_detection_sensor_config, &handle->sensor_processing_metadata);
		handle->hand_motion_detection_presence_processor = acc_detector_presence_processing_create(
		    handle->hand_motion_detection_presence_processing_config, handle->hand_motion_detection_sensor_config);

		status = handle->hand_motion_detection_presence_processor != NULL;
	}

	if (status)
	{
		if (handle->config.use_presence_detection)
		{
			handle->app_mode = HAND_MOTION_DETECTION_APP_MODE_PRESENCE;

			handle->presence_detector = acc_detector_presence_create(handle->config.presence_config, &handle->presence_metadata);

			status = handle->presence_detector != NULL;
		}
	}

	if (!status)
	{
		hand_motion_detection_handle_destroy(handle);
		handle = NULL;
	}

	return handle;
}

void hand_motion_detection_handle_destroy(hand_motion_detection_handle_t *handle)
{
	if (handle != NULL)
	{
		if (handle->presence_detector != NULL)
		{
			acc_detector_presence_destroy(handle->presence_detector);
		}

		if (handle->hand_motion_detection_presence_processor != NULL)
		{
			acc_detector_presence_processing_destroy(handle->hand_motion_detection_presence_processor);
		}

		if (handle->sensor_processing != NULL)
		{
			acc_processing_destroy(handle->sensor_processing);
		}

		if (handle->hand_motion_detection_presence_processing_config != NULL)
		{
			acc_detector_presence_processing_config_destroy(handle->hand_motion_detection_presence_processing_config);
		}

		if (handle->hand_motion_detection_sensor_config != NULL)
		{
			acc_config_destroy(handle->hand_motion_detection_sensor_config);
		}

		acc_integration_mem_free(handle);
	}
}

void hand_motion_detection_config_log(hand_motion_detection_handle_t *handle)
{
	if (handle != NULL)
	{
		printf("App config:\n");
		printf("sensor_to_water_distance:     %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(handle->config.algo_config.sensor_to_water_distance));
		printf("water_jet_width:              %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(handle->config.algo_config.water_jet_width));
		printf("measurement_range_end:        %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(handle->config.algo_config.measurement_range_end));
		printf("filter_time_const:            %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(handle->config.algo_config.filter_time_const));
		printf("threshold:                    %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(handle->config.algo_config.threshold));
		printf("detection_retention_duration: %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(handle->config.algo_config.detection_retention_duration));
		printf("hwaas:                        %" PRIu16 "\n", handle->config.algo_config.hwaas);
		printf("sweeps_per_frame:             %" PRIu16 "\n", handle->config.algo_config.sweeps_per_frame);
		printf("sweep_rate:                   %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(handle->config.algo_config.sweep_rate));
		printf("frame_rate:                   %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(handle->config.algo_config.frame_rate));
		printf("hand_detection_timeout:       %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(handle->config.hand_detection_timeout));
		printf("use_presence_detection:        %s\n", handle->config.use_presence_detection ? "True" : "False");

		printf("Sensor config:\n");
		acc_config_log(handle->hand_motion_detection_sensor_config);

		if (handle->config.use_presence_detection)
		{
			printf("Presence config:\n");
			acc_detector_presence_config_log(handle->config.presence_config);
		}
	}
}

bool hand_motion_detection_get_buffer_size(hand_motion_detection_handle_t *handle, uint32_t *buffer_size)
{
	uint32_t hand_motion_detection_buffer_size = 0U;
	uint32_t presence_detector_buffer_size     = 0U;

	bool status = acc_rss_get_buffer_size(handle->hand_motion_detection_sensor_config, &hand_motion_detection_buffer_size);

	if (status)
	{
		hand_motion_detection_buffer_size += acc_detector_presence_processing_get_buffer_size(handle->hand_motion_detection_sensor_config);

		if (handle->config.use_presence_detection)
		{
			status = acc_detector_presence_get_buffer_size(handle->presence_detector, &presence_detector_buffer_size);
		}
	}

	if (status)
	{
		*buffer_size =
		    (hand_motion_detection_buffer_size > presence_detector_buffer_size) ? hand_motion_detection_buffer_size : presence_detector_buffer_size;
	}

	return status;
}

bool hand_motion_detection_prepare(hand_motion_detection_handle_t *handle,
                                   acc_sensor_t                   *sensor,
                                   const acc_cal_result_t         *cal_result,
                                   void                           *buffer,
                                   uint32_t                        buffer_size,
                                   bool                            force_prepare)
{
	bool status = true;

	if (handle->prepare_needed || force_prepare)
	{
		if (handle->app_mode == HAND_MOTION_DETECTION_APP_MODE_PRESENCE)
		{
			status =
			    acc_detector_presence_prepare(handle->presence_detector, handle->config.presence_config, sensor, cal_result, buffer, buffer_size);
		}
		else // HAND_MOTION_DETECTION_APP_MODE_HAND_MOTION
		{
			status = acc_sensor_prepare(sensor, handle->hand_motion_detection_sensor_config, cal_result, buffer, buffer_size);
		}

		handle->prepare_needed = false;
	}

	return status;
}

void hand_motion_detection_process(hand_motion_detection_handle_t *handle, void *buffer, hand_motion_detection_result_t *hand_motion_detection_result)
{
	acc_detector_presence_result_t presence_result;

	if (handle->app_mode == HAND_MOTION_DETECTION_APP_MODE_PRESENCE)
	{
		acc_detector_presence_process(handle->presence_detector, buffer, &presence_result);

		hand_motion_detection_result->presence_result_available = true;
		hand_motion_detection_result->presence_result           = presence_result;
		hand_motion_detection_result->algo_result_available     = false;

		hand_motion_detection_result->proc_result = presence_result.processing_result;
	}
	else // HAND_MOTION_DETECTION_APP_MODE_HAND_MOTION
	{
		acc_detector_presence_processing_result_t presence_processing_result;
		acc_processing_execute(handle->sensor_processing, buffer, &hand_motion_detection_result->proc_result);

		acc_detector_presence_processing_process(
		    handle->hand_motion_detection_presence_processor, buffer, hand_motion_detection_result->proc_result.frame, &presence_processing_result);

		hand_motion_detection_result->algo_result_available       = true;
		hand_motion_detection_result->algo_result.detection_state = process_presence_processing_result(handle, &presence_processing_result);
		hand_motion_detection_result->presence_result_available   = false;
	}

	hand_motion_detection_result->app_mode = handle->app_mode;

	determine_app_mode(handle, hand_motion_detection_result);
}

static void reinit_hand_motion_detection_state(hand_motion_detection_handle_t *handle)
{
	handle->has_detected              = false;
	handle->update_index              = 0U;
	handle->update_index_at_detection = 0U;
}

static void set_sensor_config(const hand_motion_detection_config_t *hand_motion_detection_config, acc_config_t *sensor_config)
{
	const hand_motion_detection_algo_config_t *algo_config             = &hand_motion_detection_config->algo_config;
	acc_config_profile_t                       profile                 = ACC_CONFIG_PROFILE_1;
	uint16_t                                   step_length             = 6U;
	float                                      water_jet_half_width    = algo_config->water_jet_width / 2.0f;
	uint8_t                                    subsweep_idx            = 0U;
	float                                      profile_envelope_fwhm_m = acc_algorithm_get_fwhm(profile);

	// Determine if subsweep between sensor and water jet is feasible
	if (profile_envelope_fwhm_m < (algo_config->sensor_to_water_distance - water_jet_half_width))
	{
		int32_t start_point =
		    (int32_t)roundf((algo_config->sensor_to_water_distance - profile_envelope_fwhm_m - water_jet_half_width) / ACC_APPROX_BASE_STEP_LENGTH_M);
		acc_config_subsweep_start_point_set(sensor_config, start_point, subsweep_idx);
		acc_config_subsweep_num_points_set(sensor_config, 1U, subsweep_idx);
		acc_config_subsweep_step_length_set(sensor_config, step_length, subsweep_idx);
		acc_config_subsweep_profile_set(sensor_config, profile, subsweep_idx);
		acc_config_subsweep_hwaas_set(sensor_config, algo_config->hwaas, subsweep_idx);
		acc_config_subsweep_receiver_gain_set(sensor_config, 4U, subsweep_idx);

		subsweep_idx++;
	}

	// Determine if subsweep after water jet is required
	if (profile_envelope_fwhm_m + water_jet_half_width + algo_config->sensor_to_water_distance < algo_config->measurement_range_end)
	{
		float    start_m     = algo_config->sensor_to_water_distance + water_jet_half_width + profile_envelope_fwhm_m;
		int32_t  start_point = (int32_t)roundf(start_m / ACC_APPROX_BASE_STEP_LENGTH_M);
		uint16_t num_points =
		    (uint16_t)roundf(fmaxf(1.0f, (algo_config->measurement_range_end - start_m) / ((float)step_length * ACC_APPROX_BASE_STEP_LENGTH_M)));

		acc_config_subsweep_start_point_set(sensor_config, start_point, subsweep_idx);
		acc_config_subsweep_num_points_set(sensor_config, num_points, subsweep_idx);
		acc_config_subsweep_step_length_set(sensor_config, step_length, subsweep_idx);
		acc_config_subsweep_profile_set(sensor_config, profile, subsweep_idx);
		acc_config_subsweep_hwaas_set(sensor_config, algo_config->hwaas, subsweep_idx);
		acc_config_subsweep_receiver_gain_set(sensor_config, 4U, subsweep_idx);

		subsweep_idx++;
	}

	acc_config_num_subsweeps_set(sensor_config, subsweep_idx);
	acc_config_frame_rate_set(sensor_config, algo_config->frame_rate);
	acc_config_sweep_rate_set(sensor_config, algo_config->sweep_rate);
	acc_config_sweeps_per_frame_set(sensor_config, algo_config->sweeps_per_frame);
	acc_config_inter_frame_idle_state_set(sensor_config, ACC_CONFIG_IDLE_STATE_SLEEP);
	acc_config_inter_sweep_idle_state_set(sensor_config, ACC_CONFIG_IDLE_STATE_SLEEP);
}

static void set_processing_config(const hand_motion_detection_config_t      *hand_motion_detection_config,
                                  acc_detector_presence_processing_config_t *processing_config)
{
	const hand_motion_detection_algo_config_t *algo_config = &hand_motion_detection_config->algo_config;

	acc_detector_presence_processing_config_frame_rate_set(processing_config, algo_config->frame_rate);
	acc_detector_presence_processing_config_intra_detection_set(processing_config, true);
	acc_detector_presence_processing_config_inter_detection_set(processing_config, true);
	acc_detector_presence_processing_config_intra_detection_threshold_set(processing_config, algo_config->threshold);
	acc_detector_presence_processing_config_inter_detection_threshold_set(processing_config, algo_config->threshold);
	acc_detector_presence_processing_config_inter_phase_boost_set(processing_config, false);
	acc_detector_presence_processing_config_inter_frame_presence_timeout_set(processing_config, 0U);
	acc_detector_presence_processing_config_inter_frame_fast_cutoff_set(processing_config, 3.0f);
	acc_detector_presence_processing_config_inter_frame_slow_cutoff_set(processing_config, 0.8f);
	acc_detector_presence_processing_config_inter_frame_deviation_time_const_set(processing_config, 0.5f);
	acc_detector_presence_processing_config_inter_output_time_const_set(processing_config, algo_config->filter_time_const);
	acc_detector_presence_processing_config_intra_frame_time_const_set(processing_config, 0.15f);
	acc_detector_presence_processing_config_intra_output_time_const_set(processing_config, algo_config->filter_time_const);
}

static hand_motion_detection_detection_state_t process_presence_processing_result(
    hand_motion_detection_handle_t                  *handle,
    const acc_detector_presence_processing_result_t *presence_processing_result)
{
	hand_motion_detection_detection_state_t detection_state = HAND_MOTION_DETECTION_STATE_NO_DETECTION;

	if (presence_processing_result->presence_detected)
	{
		handle->has_detected              = true;
		handle->update_index_at_detection = handle->update_index;
		detection_state                   = HAND_MOTION_DETECTION_STATE_DETECTION;
	}

	if (handle->has_detected && detection_state != HAND_MOTION_DETECTION_STATE_DETECTION &&
	    ((handle->update_index - handle->update_index_at_detection) < handle->detection_retention_duration))
	{
		detection_state = HAND_MOTION_DETECTION_STATE_RETENTION;
	}

	handle->update_index++;

	return detection_state;
}

static void determine_app_mode(hand_motion_detection_handle_t *handle, hand_motion_detection_result_t *result)
{
	hand_motion_detection_app_mode_t new_app_mode = handle->app_mode;

	if (handle->app_mode == HAND_MOTION_DETECTION_APP_MODE_PRESENCE)
	{
		if (result->presence_result.presence_detected)
		{
			new_app_mode = HAND_MOTION_DETECTION_APP_MODE_HAND_MOTION;
		}
	}
	else // HAND_MOTION_DETECTION_APP_MODE_HAND_MOTION
	{
		if (handle->config.use_presence_detection)
		{
			if (result->algo_result.detection_state != HAND_MOTION_DETECTION_STATE_NO_DETECTION)
			{
				handle->hand_motion_detection_timer = 0U;
			}
			else if (handle->hand_motion_detection_timeout_duration < handle->hand_motion_detection_timer)
			{
				new_app_mode = HAND_MOTION_DETECTION_APP_MODE_PRESENCE;
			}
			else
			{
				handle->hand_motion_detection_timer++;
			}
		}
	}

	if (new_app_mode != handle->app_mode)
	{
		swap_app_mode(handle, new_app_mode);
	}
}

static void swap_app_mode(hand_motion_detection_handle_t *handle, hand_motion_detection_app_mode_t new_app_mode)
{
	handle->prepare_needed = true;
	handle->app_mode       = new_app_mode;

	if (handle->app_mode == HAND_MOTION_DETECTION_APP_MODE_HAND_MOTION)
	{
		acc_detector_presence_processing_reset(handle->hand_motion_detection_presence_processor);
		reinit_hand_motion_detection_state(handle);
		handle->hand_motion_detection_timer = 0U;
	}
	else
	{
	}
}
