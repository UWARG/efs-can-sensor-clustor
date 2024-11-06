// Copyright (c) Acconeer AB, 2024
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "acc_algorithm.h"
#include "acc_config.h"
#include "acc_definitions_a121.h"
#include "acc_definitions_common.h"
#include "acc_hal_integration_a121.h"
#include "acc_integration.h"
#include "acc_integration_log.h"
#include "acc_processing.h"
#include "acc_rss_a121.h"
#include "acc_sensor.h"
#include "ref_app_parking.h"

#define CAL_ITERATIONS (3U)
#define MAX_AMPLITUDE  (46341.0f) // sqrt((2^15)^2 + (2^15)^2)

typedef struct
{
	float energy;
	float weighted_distance;
} signature_t;

struct ref_app_parking_handle
{
	// config
	ref_app_parking_config_t parking_config;
	acc_config_t            *sensor_config;

	// sensor handling
	acc_sensor_id_t  sensor_id;
	acc_sensor_t    *sensor;
	void            *buffer;
	uint32_t         buffer_size;
	acc_cal_result_t cal_result;
	uint32_t         measure_timeout_ms;

	// noise calibration
	float  *noise_magnitudes;
	int16_t noise_cal_temp;
	float   noise_level;

	// obstruction calibration and processing
	float      *obstruction_distances;
	float      *obstruction_magnitudes;
	signature_t obstruction_center;
	float       obstruction_noise_level;
	signature_t obstruction_lp_signature;
	float       obstruction_lp_const;
	float       obstruction_energy_threshold;
	float       obstruction_weighted_distance_threshold;

	// processing and result
	float                    *distances;
	float                    *norm_distances;
	float                    *magnitudes;
	signature_t              *signature_history;
	signature_t              *signature_history_sorted;
	acc_processing_t         *proc;
	acc_processing_metadata_t proc_meta;
	acc_processing_result_t   proc_result;
};

/**
 * @brief Set sensor config using already set parking config
 *
 * @param[in] handle The parking handle
 */
static void set_sensor_config(ref_app_parking_handle_t *handle);

/**
 * @brief Log config
 *
 * @param[in] handle The parking handle
 */
static void log_config(ref_app_parking_handle_t *handle);

/**
 * @brief Allocate resources for application
 *
 * @param[in] handle The parking handle
 */
static bool allocate_application_resources(ref_app_parking_handle_t *handle);

/**
 * @brief Initialize resources for application using already set parking config
 *
 * @param[in] handle The parking handle
 */
static void initialize_application_resources(ref_app_parking_handle_t *handle);

/**
 * @brief Get timeout value, in ms, for specified update rate
 *
 * @param[in] update_rate_hz The update rate, in Hz
 * @return The timeout value, in ms
 */
static uint32_t get_measure_timeout_ms(float update_rate_hz);

/**
 * @brief Calculate noise level from noise data
 *
 * @param[in] noise_data The noise data
 * @param[in] noise_magnitudes Intermediate step in processing will be stored here
 * @param[in] noise_data_length The length of noise data and magnitudes
 * @return The noise level
 */
static float noise_process(acc_int16_complex_t *noise_data, float *noise_magnitudes, uint16_t noise_data_length);

/**
 * @brief Calculate noise mean from obstruction noise data
 *
 * @param[in] obs_noise_data The obstruction noise data
 * @param[in] obs_noise_data_length The length of obstruction noise data
 * @return The noise mean
 */
static float obstruction_noise(acc_int16_complex_t *obs_noise_data, uint16_t obs_noise_data_length);

/**
 * @brief Calculate obstruction signature from obstruction data
 *
 * @param[in] subsweep The obstruction data
 * @param[in] magnitudes Intermediate step in processing will be stored here
 * @param[in] distances Corresponding distance for each point in the obstruction data
 * @param[in] length The length of subsweep, magnitudes, and distances
 * @param[in] noise_level The current noise level to compensate for
 * @param[out] signature The calculated signature
 */
static void obstruction_signature(acc_int16_complex_t *subsweep,
                                  float               *magnitudes,
                                  float               *distances,
                                  uint16_t             length,
                                  float                noise_level,
                                  signature_t         *signature);

/**
 * @brief Calculate car signature from car data
 *
 * @param[in] magnitudes The car data
 * @param[in] distances Corresponding distance for each point in the car data
 * @param[in] length The length of magnitudes and distances
 * @param[out] signature The calculated signature
 */
static void car_signature(const float *magnitudes, const float *distances, uint16_t length, signature_t *signature);

/**
 * @brief Determine if there are any objects present using signature history and thresholds
 *
 * @param[in] handle The parking handle
 * @return true if objects present, false otherwise
 */
static bool objects_present(const ref_app_parking_handle_t *handle);

/**
 * @brief Determine if multiple signatures relates to same object
 *
 * @param[in] handle The parking handle
 * @return true if multiple signatures relates to same object, false otherwise
 */
static bool same_objects(ref_app_parking_handle_t *handle);

/**
 * @brief Function used to compare two signatures for use in qsort
 *
 * @param[in] a First signature
 * @param[in] b Second signature
 * @return -1 if a is closer to sensor than b
 *         1 if b is closer to sensor than a
 *         0 if a and b is at same distance from sensor
 */
static int compare_signature(const void *a, const void *b);

//-----------------------------
// Public definitions
//-----------------------------

void ref_app_parking_set_config(ref_app_parking_parking_preset_t preset, ref_app_parking_config_t *parking_config)
{
	parking_config->preset = preset;

	switch (parking_config->preset)
	{
		case PARKING_PRESET_NONE:
			break;
		case PARKING_PRESET_GROUND:
			parking_config->frame_rate                    = 0.1f;
			parking_config->obstruction_detection_enabled = true;

			parking_config->car_config.range_start_m                  = 0.1f;
			parking_config->car_config.range_end_m                    = 0.4f;
			parking_config->car_config.hwaas                          = 24;
			parking_config->car_config.profile                        = ACC_CONFIG_PROFILE_1;
			parking_config->car_config.subsweep_index                 = 0U;
			parking_config->car_config.queue_length_n                 = 3U;
			parking_config->car_config.amplitude_threshold            = 8.0f;
			parking_config->car_config.weighted_distance_threshold_m  = 0.1f;
			parking_config->car_config.signature_similarity_threshold = 0.6f;

			parking_config->obstruction_config.range_start_m  = 0.03f;
			parking_config->obstruction_config.range_end_m    = 0.05f;
			parking_config->obstruction_config.hwaas          = 16;
			parking_config->obstruction_config.profile        = ACC_CONFIG_PROFILE_1;
			parking_config->obstruction_config.subsweep_index = 1U;
			parking_config->obstruction_config.threshold      = 0.06f;
			parking_config->obstruction_config.time_constant  = 0.8f;
			break;
		case PARKING_PRESET_POLE:
			parking_config->frame_rate                    = 6.0f;
			parking_config->obstruction_detection_enabled = false;

			parking_config->car_config.range_start_m                  = 0.2f;
			parking_config->car_config.range_end_m                    = 3.0f;
			parking_config->car_config.hwaas                          = 24;
			parking_config->car_config.profile                        = ACC_CONFIG_PROFILE_2;
			parking_config->car_config.subsweep_index                 = 0U;
			parking_config->car_config.queue_length_n                 = 20U;
			parking_config->car_config.amplitude_threshold            = 6.0f;
			parking_config->car_config.weighted_distance_threshold_m  = 0.9f;
			parking_config->car_config.signature_similarity_threshold = 0.6f;
			break;
		default:
			break;
	}

	parking_config->frame_rate_app_driven = false;
}

ref_app_parking_handle_t *ref_app_parking_handle_create(ref_app_parking_config_t *parking_config, acc_sensor_id_t sensor_id)
{
	ref_app_parking_handle_t *handle = acc_integration_mem_alloc(sizeof(*handle));

	if (handle != NULL)
	{
		handle->parking_config = *parking_config;
		handle->sensor_id      = sensor_id;

		handle->sensor_config = acc_config_create();
		if (handle->sensor_config == NULL)
		{
			ref_app_parking_handle_destroy(handle);
			return NULL;
		}

		set_sensor_config(handle);

		log_config(handle);

		if (!allocate_application_resources(handle))
		{
			ref_app_parking_handle_destroy(handle);
			return NULL;
		}

		initialize_application_resources(handle);
	}

	return handle;
}

void ref_app_parking_handle_destroy(ref_app_parking_handle_t *handle)
{
	if (handle != NULL)
	{
		acc_hal_integration_sensor_disable(handle->sensor_id);
		acc_hal_integration_sensor_supply_off(handle->sensor_id);

		if (handle->sensor != NULL)
		{
			acc_sensor_destroy(handle->sensor);
		}

		if (handle->signature_history_sorted != NULL)
		{
			acc_integration_mem_free(handle->signature_history_sorted);
		}

		if (handle->signature_history != NULL)
		{
			acc_integration_mem_free(handle->signature_history);
		}

		if (handle->magnitudes != NULL)
		{
			acc_integration_mem_free(handle->magnitudes);
		}

		if (handle->norm_distances != NULL)
		{
			acc_integration_mem_free(handle->norm_distances);
		}

		if (handle->distances != NULL)
		{
			acc_integration_mem_free(handle->distances);
		}

		if (handle->obstruction_magnitudes != NULL)
		{
			acc_integration_mem_free(handle->obstruction_magnitudes);
		}

		if (handle->obstruction_distances != NULL)
		{
			acc_integration_mem_free(handle->obstruction_distances);
		}

		if (handle->noise_magnitudes != NULL)
		{
			acc_integration_mem_free(handle->noise_magnitudes);
		}

		if (handle->buffer != NULL)
		{
			acc_integration_mem_free(handle->buffer);
		}

		if (handle->proc != NULL)
		{
			acc_processing_destroy(handle->proc);
		}

		if (handle->sensor_config != NULL)
		{
			acc_config_destroy(handle->sensor_config);
		}

		acc_integration_mem_free(handle);
	}
}

bool ref_app_parking_sensor_calibration(ref_app_parking_handle_t *handle)
{
	bool           status                      = false;
	bool           cal_complete                = false;
	const uint16_t calibration_retries         = 1U;
	const uint32_t calibration_step_timeout_ms = 500U;

	// Random disturbances may cause the calibration to fail. At failure, retry at least once.
	for (uint16_t i = 0; !status && (i <= calibration_retries); i++)
	{
		// Reset sensor before calibration by disabling/enabling it
		acc_hal_integration_sensor_disable(handle->sensor_id);
		acc_hal_integration_sensor_enable(handle->sensor_id);

		do
		{
			status = acc_sensor_calibrate(handle->sensor, &cal_complete, &handle->cal_result, handle->buffer, handle->buffer_size);

			if (status && !cal_complete)
			{
				status = acc_hal_integration_wait_for_sensor_interrupt(handle->sensor_id, calibration_step_timeout_ms);
			}
		} while (status && !cal_complete);
	}

	if (status)
	{
		// Reset sensor after calibration by disabling/enabling it
		acc_hal_integration_sensor_disable(handle->sensor_id);
		acc_hal_integration_sensor_enable(handle->sensor_id);
	}

	return status;
}

bool ref_app_parking_noise_calibration(ref_app_parking_handle_t *handle)
{
	// The config to be used for calibration is the same as the 'car config' but with TX off
	// and a frame rate corresponding to a total measurement time of ~1s.
	uint16_t noise_subsweep_index = handle->parking_config.car_config.subsweep_index;

	acc_config_subsweep_enable_tx_set(handle->sensor_config, false, noise_subsweep_index);
	acc_config_frame_rate_set(handle->sensor_config, (float)CAL_ITERATIONS);
	handle->measure_timeout_ms = get_measure_timeout_ms((float)CAL_ITERATIONS);

	if (!acc_sensor_prepare(handle->sensor, handle->sensor_config, &handle->cal_result, handle->buffer, handle->buffer_size))
	{
		printf("acc_sensor_prepare() failed\n");
		return false;
	}

	int16_t noise_cal_temps[CAL_ITERATIONS] = {0};
	float   noise_level_sum                 = 0.0f;

	uint16_t noise_data_offset = handle->proc_meta.subsweep_data_offset[noise_subsweep_index];
	uint16_t noise_data_length = handle->proc_meta.subsweep_data_length[noise_subsweep_index];

	for (uint16_t i = 0; i < CAL_ITERATIONS; i++)
	{
		if (!ref_app_parking_measure(handle, false))
		{
			return false;
		}

		// Only a few measurements and recently calibrated - assume indications ok
		noise_cal_temps[i] = handle->proc_result.temperature;

		noise_level_sum += noise_process(&handle->proc_result.frame[noise_data_offset], handle->noise_magnitudes, noise_data_length);
	}

	handle->noise_cal_temp = acc_algorithm_median_i16(noise_cal_temps, CAL_ITERATIONS);
	handle->noise_level    = noise_level_sum / (float)CAL_ITERATIONS;
	handle->noise_level    = (handle->noise_level == 0.0f) ? 1.0f : handle->noise_level;

	// Restore the 'car config'
	acc_config_subsweep_enable_tx_set(handle->sensor_config, true, noise_subsweep_index);
	if (handle->parking_config.frame_rate_app_driven)
	{
		acc_config_frame_rate_set(handle->sensor_config, 0.0f);
	}
	else
	{
		acc_config_frame_rate_set(handle->sensor_config, handle->parking_config.frame_rate);
	}

	handle->measure_timeout_ms = get_measure_timeout_ms(handle->parking_config.frame_rate);

	return true;
}

bool ref_app_parking_obstruction_calibration(ref_app_parking_handle_t *handle)
{
	acc_config_frame_rate_set(handle->sensor_config, (float)CAL_ITERATIONS);
	handle->measure_timeout_ms = get_measure_timeout_ms((float)CAL_ITERATIONS);

	uint16_t obs_cal_subsweep_index = handle->parking_config.obstruction_config.subsweep_index;
	uint16_t obs_cal_data_offset    = handle->proc_meta.subsweep_data_offset[obs_cal_subsweep_index];
	uint16_t obs_cal_data_length    = handle->proc_meta.subsweep_data_length[obs_cal_subsweep_index];

	// Noise calibration without TX
	acc_config_subsweep_enable_tx_set(handle->sensor_config, false, handle->parking_config.obstruction_config.subsweep_index);

	if (!acc_sensor_prepare(handle->sensor, handle->sensor_config, &handle->cal_result, handle->buffer, handle->buffer_size))
	{
		printf("acc_sensor_prepare() failed\n");
		return false;
	}

	float noise_levels[CAL_ITERATIONS] = {0.0f};
	float noise_level_sum              = 0.0f;

	for (uint16_t i = 0; i < CAL_ITERATIONS; i++)
	{
		if (!ref_app_parking_measure(handle, false))
		{
			return false;
		}

		// Only a few measurements and recently calibrated - assume indications ok

		noise_levels[i]  = obstruction_noise(&handle->proc_result.frame[obs_cal_data_offset], obs_cal_data_length);
		noise_level_sum += noise_levels[i];
	}

	handle->obstruction_noise_level = noise_level_sum / CAL_ITERATIONS;

	// Signature calibration with TX - sensitive to environment.
	// Must be done with sensor free from obstruction and no car present.
	acc_config_subsweep_enable_tx_set(handle->sensor_config, true, handle->parking_config.obstruction_config.subsweep_index);

	if (!acc_sensor_prepare(handle->sensor, handle->sensor_config, &handle->cal_result, handle->buffer, handle->buffer_size))
	{
		printf("acc_sensor_prepare() failed\n");
		return false;
	}

	signature_t signature             = {0};
	float       energy_sum            = 0.0f;
	float       weighted_distance_sum = 0.0f;

	for (uint16_t i = 0; i < CAL_ITERATIONS; i++)
	{
		if (!ref_app_parking_measure(handle, false))
		{
			return false;
		}

		// Only a few measurements and recently calibrated - assume indications ok

		acc_int16_complex_t *subsweep = &handle->proc_result.frame[obs_cal_data_offset];

		obstruction_signature(
		    subsweep, handle->obstruction_magnitudes, handle->obstruction_distances, obs_cal_data_length, noise_levels[i], &signature);

		energy_sum            += signature.energy;
		weighted_distance_sum += signature.weighted_distance;
	}

	handle->obstruction_center.energy            = energy_sum / CAL_ITERATIONS;
	handle->obstruction_center.weighted_distance = weighted_distance_sum / CAL_ITERATIONS;

	handle->obstruction_lp_signature = handle->obstruction_center;

	// Restore the 'obstruction config'
	if (handle->parking_config.frame_rate_app_driven)
	{
		acc_config_frame_rate_set(handle->sensor_config, 0.0f);
	}
	else
	{
		acc_config_frame_rate_set(handle->sensor_config, handle->parking_config.frame_rate);
	}

	handle->measure_timeout_ms = get_measure_timeout_ms(handle->parking_config.frame_rate);

	return true;
}

bool ref_app_parking_sensor_prepare(ref_app_parking_handle_t *handle)
{
	if (!acc_sensor_prepare(handle->sensor, handle->sensor_config, &handle->cal_result, handle->buffer, handle->buffer_size))
	{
		printf("acc_sensor_prepare failed\n");
		return false;
	}

	return true;
}

bool ref_app_parking_measure(ref_app_parking_handle_t *handle, bool hibernate)
{
	(void)hibernate;

	if (!acc_sensor_measure(handle->sensor))
	{
		printf("acc_sensor_measure failed\n");
		return false;
	}

	if (!acc_hal_integration_wait_for_sensor_interrupt(handle->sensor_id, handle->measure_timeout_ms))
	{
		printf("Sensor interrupt timeout\n");
		return false;
	}

	if (!acc_sensor_read(handle->sensor, handle->buffer, handle->buffer_size))
	{
		printf("acc_sensor_read failed\n");
		return false;
	}

	acc_processing_execute(handle->proc, handle->buffer, &handle->proc_result);

	return true;
}

bool ref_app_parking_handle_indications(ref_app_parking_handle_t *handle, bool *data_reliable)
{
	bool status = true;

	*data_reliable = true;

	if (handle->proc_result.data_saturated)
	{
		*data_reliable = false;

		printf("Data saturated. Try to reduce the sensor gain.\n");
	}

	if (handle->proc_result.frame_delayed)
	{
		printf("Frame delayed. Could not read data fast enough.\n");
		printf("Try lowering the frame rate or call 'acc_sensor_read' more frequently.\n");
	}

	if (handle->proc_result.calibration_needed)
	{
		printf("The current calibration is not valid for the current temperature.\n");
		printf("Re-calibrating sensor...\n");

		if (status)
		{
			if (!ref_app_parking_sensor_calibration(handle))
			{
				printf("ref_app_parking_sensor_calibration() failed\n");
				acc_sensor_status(handle->sensor);
				status = false;
			}
		}

		if (status)
		{
			if (!acc_sensor_prepare(handle->sensor, handle->sensor_config, &handle->cal_result, handle->buffer, handle->buffer_size))
			{
				printf("acc_sensor_prepare() failed\n");
				status = false;
			}
		}

		*data_reliable = false;

		printf("The sensor was successfully re-calibrated.\n");
	}

	return status;
}

void ref_app_parking_obstruction_process(ref_app_parking_handle_t *handle, bool *obstruction_detected)
{
	ref_app_parking_config_t             *parking_config     = &handle->parking_config;
	acc_config_t                         *sensor_config      = handle->sensor_config;
	ref_app_parking_obstruction_config_t *obstruction_config = &parking_config->obstruction_config;

	acc_config_profile_t profile = acc_config_subsweep_profile_get(sensor_config, obstruction_config->subsweep_index);

	float signal_adjust_factor;
	float deviation_adjust_factor;

	acc_processing_get_temperature_adjustment_factors(
	    handle->noise_cal_temp, handle->proc_result.temperature, profile, &signal_adjust_factor, &deviation_adjust_factor);

	float noise_level_adjusted = handle->obstruction_noise_level * deviation_adjust_factor;

	acc_int16_complex_t *subsweep       = &handle->proc_result.frame[handle->proc_meta.subsweep_data_offset[obstruction_config->subsweep_index]];
	uint16_t             num_points     = acc_config_subsweep_num_points_get(sensor_config, obstruction_config->subsweep_index);
	signature_t          curr_signature = {0};

	// Adjust for temperature changes
	for (uint16_t i = 0; i < num_points; i++)
	{
		subsweep[i].real = ((float)subsweep[i].real / signal_adjust_factor) + 0.5f;
		subsweep[i].imag = ((float)subsweep[i].imag / signal_adjust_factor) + 0.5f;
	}

	// Extract signature
	obstruction_signature(subsweep, handle->obstruction_magnitudes, handle->obstruction_distances, num_points, noise_level_adjusted, &curr_signature);

	// Low-pass filter of signature
	signature_t *lp_signature = &handle->obstruction_lp_signature;
	float        lp_const     = handle->obstruction_lp_const;

	lp_signature->energy            = lp_signature->energy * lp_const + (1.0f - lp_const) * curr_signature.energy;
	lp_signature->weighted_distance = lp_signature->weighted_distance * lp_const + (1.0f - lp_const) * curr_signature.weighted_distance;

	// Compare against thresholds
	float energy_diff            = fabsf(lp_signature->energy - handle->obstruction_center.energy);
	float weighted_distance_diff = fabsf(lp_signature->weighted_distance - handle->obstruction_center.weighted_distance);

	*obstruction_detected =
	    (energy_diff > handle->obstruction_energy_threshold) || (weighted_distance_diff > handle->obstruction_weighted_distance_threshold);
}

void ref_app_parking_process(ref_app_parking_handle_t *handle, bool *car_detected)
{
	ref_app_parking_config_t     *parking_config = &handle->parking_config;
	acc_config_t                 *sensor_config  = handle->sensor_config;
	ref_app_parking_car_config_t *car_config     = &parking_config->car_config;

	acc_int16_complex_t *subsweep = &handle->proc_result.frame[handle->proc_meta.subsweep_data_offset[car_config->subsweep_index]];

	uint16_t             num_points = acc_config_subsweep_num_points_get(sensor_config, car_config->subsweep_index);
	acc_config_profile_t profile    = acc_config_subsweep_profile_get(sensor_config, car_config->subsweep_index);

	float signal_adjust_factor;
	float deviation_adjust_factor;

	acc_processing_get_temperature_adjustment_factors(
	    handle->noise_cal_temp, handle->proc_result.temperature, profile, &signal_adjust_factor, &deviation_adjust_factor);

	float noise_level_adjusted = handle->noise_level * deviation_adjust_factor;

	for (uint16_t i = 0; i < num_points; i++)
	{
		handle->magnitudes[i] = sqrtf((float)subsweep[i].real * (float)subsweep[i].real + (float)subsweep[i].imag * (float)subsweep[i].imag);

		// Compensate for temperature changes
		handle->magnitudes[i] /= noise_level_adjusted;

		// Remove noise
		handle->magnitudes[i] = (handle->magnitudes[i] - 1.0f < 0.0f) ? 0.0f : handle->magnitudes[i] - 1.0f;

		// Scale with normalized distance
		handle->magnitudes[i] *= handle->norm_distances[i];
	}

	for (uint16_t i = 1; i < car_config->queue_length_n; i++)
	{
		handle->signature_history[i - 1] = handle->signature_history[i];
	}

	car_signature(handle->magnitudes, handle->distances, num_points, &handle->signature_history[car_config->queue_length_n - 1]);

	bool obj_present = objects_present(handle);
	bool same_obj    = same_objects(handle);

	*car_detected = obj_present && same_obj;
}

//-----------------------------
// Private definitions
//-----------------------------

static void set_sensor_config(ref_app_parking_handle_t *handle)
{
	ref_app_parking_config_t *parking_config = &handle->parking_config;
	acc_config_t             *sensor_config  = handle->sensor_config;

	// Common settings
	acc_config_sweeps_per_frame_set(sensor_config, 1U); // This should not be changed
	if (parking_config->frame_rate_app_driven)
	{
		acc_config_frame_rate_set(sensor_config, 0.0f);
	}
	else
	{
		acc_config_frame_rate_set(sensor_config, parking_config->frame_rate);
	}

	uint8_t num_subsweeps = parking_config->obstruction_detection_enabled ? 2U : 1U;

	acc_config_num_subsweeps_set(sensor_config, num_subsweeps);

	// Car settings
	ref_app_parking_car_config_t *car_config = &parking_config->car_config;

	int32_t start_point = (int32_t)roundf(car_config->range_start_m / ACC_APPROX_BASE_STEP_LENGTH_M);

	// Handling of step_length based on profile and constraint in RSS API
	uint16_t step_length       = 0U;
	float    fwhm              = acc_algorithm_get_fwhm(car_config->profile);
	uint16_t step_length_ideal = (uint16_t)roundf(fwhm / ACC_APPROX_BASE_STEP_LENGTH_M);

	if (step_length_ideal >= 24)
	{
		step_length = (step_length_ideal / 24U) * 24U;
	}
	else
	{
		step_length = step_length_ideal;
		while (24U % step_length != 0U)
		{
			step_length--;
		}
	}

	uint16_t num_points = (uint16_t)ceilf((car_config->range_end_m - car_config->range_start_m) / (step_length * ACC_APPROX_BASE_STEP_LENGTH_M));

	acc_config_subsweep_start_point_set(sensor_config, start_point, car_config->subsweep_index);
	acc_config_subsweep_step_length_set(sensor_config, step_length, car_config->subsweep_index);
	acc_config_subsweep_num_points_set(sensor_config, num_points, car_config->subsweep_index);
	acc_config_subsweep_hwaas_set(sensor_config, car_config->hwaas, car_config->subsweep_index);
	acc_config_subsweep_profile_set(sensor_config, car_config->profile, car_config->subsweep_index);

	// Obstruction settings
	if (parking_config->obstruction_detection_enabled)
	{
		ref_app_parking_obstruction_config_t *obstruction_config = &parking_config->obstruction_config;

		start_point = (int32_t)roundf(obstruction_config->range_start_m / ACC_APPROX_BASE_STEP_LENGTH_M);
		step_length = 2U;

		num_points =
		    (uint16_t)ceilf((obstruction_config->range_end_m - obstruction_config->range_start_m) / (step_length * ACC_APPROX_BASE_STEP_LENGTH_M));

		acc_config_subsweep_start_point_set(sensor_config, start_point, obstruction_config->subsweep_index);
		acc_config_subsweep_step_length_set(sensor_config, step_length, obstruction_config->subsweep_index);
		acc_config_subsweep_num_points_set(sensor_config, num_points, obstruction_config->subsweep_index);
		acc_config_subsweep_hwaas_set(sensor_config, obstruction_config->hwaas, obstruction_config->subsweep_index);
		acc_config_subsweep_profile_set(sensor_config, obstruction_config->profile, obstruction_config->subsweep_index);
	}
}

static void log_config(ref_app_parking_handle_t *handle)
{
	ref_app_parking_config_t *parking_config = &handle->parking_config;

	printf("\nParking config:\n");

	char *preset_string = NULL;

	switch (parking_config->preset)
	{
		case PARKING_PRESET_NONE:
			preset_string = "None";
			break;
		case PARKING_PRESET_GROUND:
			preset_string = "Ground";
			break;
		case PARKING_PRESET_POLE:
			preset_string = "Pole";
			break;
		default:
			preset_string = "";
			break;
	}

	printf("preset: %s\n", preset_string);
	printf("frame_rate: %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(parking_config->frame_rate));
	printf("obstruction_detection_enabled: %s\n", parking_config->obstruction_detection_enabled ? "true" : "false");

	printf("\nCar config:\n");
	printf("range_start_m: %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(parking_config->car_config.range_start_m));
	printf("range_end_m: %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(parking_config->car_config.range_end_m));
	printf("queue_length_n: %" PRIu16 "\n", parking_config->car_config.queue_length_n);
	printf("amplitude_threshold: %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(parking_config->car_config.amplitude_threshold));
	printf("weighted_distance_threshold_m: %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(parking_config->car_config.weighted_distance_threshold_m));
	printf("signature_similarity_threshold: %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(parking_config->car_config.signature_similarity_threshold));

	if (parking_config->obstruction_detection_enabled)
	{
		printf("\nObstruction config:\n");
		printf("range_start_m: %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(parking_config->obstruction_config.range_start_m));
		printf("range_end_m: %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(parking_config->obstruction_config.range_end_m));
		printf("threshold: %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(parking_config->obstruction_config.threshold));
		printf("time_constant: %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(parking_config->obstruction_config.time_constant));
	}

	printf("\nSensor config:\n");
	acc_config_log(handle->sensor_config);
}

static bool allocate_application_resources(ref_app_parking_handle_t *handle)
{
	ref_app_parking_config_t *parking_config = &handle->parking_config;
	acc_config_t             *sensor_config  = handle->sensor_config;

	handle->proc = acc_processing_create(handle->sensor_config, &handle->proc_meta);
	if (handle->proc == NULL)
	{
		return false;
	}

	if (!acc_rss_get_buffer_size(sensor_config, &handle->buffer_size))
	{
		return false;
	}

	handle->buffer = acc_integration_mem_alloc(handle->buffer_size);
	if (handle->buffer == NULL)
	{
		return false;
	}

	uint16_t car_num_points = acc_config_subsweep_num_points_get(sensor_config, parking_config->car_config.subsweep_index);
	uint16_t obs_num_points = acc_config_subsweep_num_points_get(sensor_config, parking_config->obstruction_config.subsweep_index);

	handle->noise_magnitudes = acc_integration_mem_alloc(car_num_points * sizeof(*handle->noise_magnitudes));
	if (handle->noise_magnitudes == NULL)
	{
		return false;
	}

	handle->obstruction_magnitudes = acc_integration_mem_alloc(obs_num_points * sizeof(*handle->obstruction_magnitudes));
	if (handle->obstruction_magnitudes == NULL)
	{
		return false;
	}

	handle->obstruction_distances = acc_integration_mem_alloc(obs_num_points * sizeof(*handle->obstruction_distances));
	if (handle->obstruction_distances == NULL)
	{
		return false;
	}

	handle->distances = acc_integration_mem_alloc(car_num_points * sizeof(*handle->distances));
	if (handle->distances == NULL)
	{
		return false;
	}

	handle->norm_distances = acc_integration_mem_alloc(car_num_points * sizeof(*handle->norm_distances));
	if (handle->norm_distances == NULL)
	{
		return false;
	}

	handle->magnitudes = acc_integration_mem_alloc(car_num_points * sizeof(*handle->magnitudes));
	if (handle->magnitudes == NULL)
	{
		return false;
	}

	uint16_t queue_length_n = parking_config->car_config.queue_length_n;

	handle->signature_history = acc_integration_mem_alloc(queue_length_n * sizeof(*handle->signature_history));
	if (handle->signature_history == NULL)
	{
		return false;
	}

	handle->signature_history_sorted = acc_integration_mem_alloc(queue_length_n * sizeof(*handle->signature_history_sorted));
	if (handle->signature_history_sorted == NULL)
	{
		return false;
	}

	acc_hal_integration_sensor_supply_on(handle->sensor_id);
	acc_hal_integration_sensor_enable(handle->sensor_id);

	handle->sensor = acc_sensor_create(handle->sensor_id);
	if (handle->sensor == NULL)
	{
		return false;
	}

	return true;
}

static void initialize_application_resources(ref_app_parking_handle_t *handle)
{
	ref_app_parking_config_t             *parking_config     = &handle->parking_config;
	acc_config_t                         *sensor_config      = handle->sensor_config;
	ref_app_parking_obstruction_config_t *obstruction_config = &parking_config->obstruction_config;
	ref_app_parking_car_config_t         *car_config         = &parking_config->car_config;

	handle->measure_timeout_ms = get_measure_timeout_ms(parking_config->frame_rate);

	float base_step_length = acc_processing_points_to_meter(1);

	// Obstruction distances
	uint8_t subsweep_index = obstruction_config->subsweep_index;

	uint16_t num_points  = acc_config_subsweep_num_points_get(sensor_config, subsweep_index);
	int32_t  start_point = acc_config_subsweep_start_point_get(sensor_config, subsweep_index);
	uint16_t step_length = acc_config_subsweep_step_length_get(sensor_config, subsweep_index);

	for (uint16_t i = 0; i < num_points; i++)
	{
		handle->obstruction_distances[i] = (float)(start_point + i * step_length) * base_step_length;
	}

	float actual_distance_range = handle->obstruction_distances[num_points - 1U] - handle->obstruction_distances[0];

	handle->obstruction_lp_const = acc_algorithm_exp_smoothing_coefficient(parking_config->frame_rate, obstruction_config->time_constant);

	handle->obstruction_energy_threshold            = MAX_AMPLITUDE * obstruction_config->threshold;
	handle->obstruction_weighted_distance_threshold = actual_distance_range * obstruction_config->threshold;

	// Car distances
	subsweep_index = car_config->subsweep_index;

	num_points  = acc_config_subsweep_num_points_get(sensor_config, subsweep_index);
	start_point = acc_config_subsweep_start_point_get(sensor_config, subsweep_index);
	step_length = acc_config_subsweep_step_length_get(sensor_config, subsweep_index);

	for (uint16_t i = 0; i < num_points; i++)
	{
		handle->distances[i]      = (float)(start_point + i * step_length) * base_step_length;
		handle->norm_distances[i] = handle->distances[i] / handle->distances[0];
	}

	memset(handle->signature_history, 0, car_config->queue_length_n * sizeof(*handle->signature_history));
	memset(handle->signature_history_sorted, 0, car_config->queue_length_n * sizeof(*handle->signature_history_sorted));
}

static uint32_t get_measure_timeout_ms(float update_rate_hz)
{
	return (uint32_t)((1.0f / update_rate_hz) * 1000.0f) + 1000U;
}

static float noise_process(acc_int16_complex_t *noise_data, float *noise_magnitudes, uint16_t noise_data_length)
{
	float noise_mean = 0.0f;

	for (uint16_t i = 0; i < noise_data_length; i++)
	{
		noise_magnitudes[i]  = sqrtf((float)noise_data[i].real * (float)noise_data[i].real + (float)noise_data[i].imag * (float)noise_data[i].imag);
		noise_mean          += noise_magnitudes[i];
	}

	noise_mean /= noise_data_length;

	float pow_sum = 0.0f;

	for (uint16_t i = 0; i < noise_data_length; i++)
	{
		pow_sum += (noise_magnitudes[i] - noise_mean) * (noise_magnitudes[i] - noise_mean);
	}

	float noise_std = sqrtf(pow_sum / noise_data_length);

	float noise_level = noise_mean + 2.0f * noise_std;

	return noise_level;
}

static float obstruction_noise(acc_int16_complex_t *obs_noise_data, uint16_t obs_noise_data_length)
{
	float noise_mean = 0.0f;

	for (uint16_t i = 0; i < obs_noise_data_length; i++)
	{
		noise_mean +=
		    sqrtf((float)obs_noise_data[i].real * (float)obs_noise_data[i].real + (float)obs_noise_data[i].imag * (float)obs_noise_data[i].imag);
	}

	noise_mean /= obs_noise_data_length;

	return noise_mean;
}

static void obstruction_signature(acc_int16_complex_t *subsweep,
                                  float               *magnitudes,
                                  float               *distances,
                                  uint16_t             length,
                                  float                noise_level,
                                  signature_t         *signature)
{
	float energy_sum = 0.0f;

	for (uint16_t j = 0; j < length; j++)
	{
		magnitudes[j]  = sqrtf((float)subsweep[j].real * (float)subsweep[j].real + (float)subsweep[j].imag * (float)subsweep[j].imag);
		magnitudes[j] -= noise_level;
		energy_sum    += magnitudes[j];
	}

	signature->energy            = (energy_sum / length);
	signature->weighted_distance = acc_algorithm_weighted_mean(distances, magnitudes, length);
}

static void car_signature(const float *magnitudes, const float *distances, uint16_t length, signature_t *signature)
{
	signature->energy            = acc_algorithm_max_f32(magnitudes, length);
	signature->weighted_distance = acc_algorithm_weighted_mean(distances, magnitudes, length);
}

static bool objects_present(const ref_app_parking_handle_t *handle)
{
	uint16_t queue_length_n                 = handle->parking_config.car_config.queue_length_n;
	float    amplitude_threshold            = handle->parking_config.car_config.amplitude_threshold;
	float    signature_similarity_threshold = handle->parking_config.car_config.signature_similarity_threshold;

	uint16_t trig_count = 0;

	for (uint16_t i = 0; i < queue_length_n; i++)
	{
		if (handle->signature_history[i].energy > amplitude_threshold)
		{
			trig_count++;
		}
	}

	return (float)trig_count > (queue_length_n * signature_similarity_threshold);
}

static bool same_objects(ref_app_parking_handle_t *handle)
{
	uint16_t queue_length_n                 = handle->parking_config.car_config.queue_length_n;
	float    amplitude_threshold            = handle->parking_config.car_config.amplitude_threshold;
	float    weighted_distance_threshold_m  = handle->parking_config.car_config.weighted_distance_threshold_m;
	float    signature_similarity_threshold = handle->parking_config.car_config.signature_similarity_threshold;

	for (uint16_t i = 0; i < queue_length_n; i++)
	{
		handle->signature_history_sorted[i] = handle->signature_history[i];
	}

	qsort(handle->signature_history_sorted, queue_length_n, sizeof(*handle->signature_history_sorted), compare_signature);

	uint16_t active_cluster_count    = 0;
	float    active_cluster_ref_dist = 0.0f;
	uint16_t largest_cluster_count   = 0;

	for (uint16_t i = 0; i < queue_length_n; i++)
	{
		if (handle->signature_history_sorted[i].energy > amplitude_threshold)
		{
			// Start of active cluster
			if (active_cluster_count == 0)
			{
				active_cluster_count++;
				active_cluster_ref_dist = handle->signature_history_sorted[i].weighted_distance;
			}
			// End of active cluster
			else if (handle->signature_history_sorted[i].weighted_distance - active_cluster_ref_dist > weighted_distance_threshold_m)
			{
				if (active_cluster_count > largest_cluster_count)
				{
					largest_cluster_count = active_cluster_count;
				}

				active_cluster_count    = 0;
				active_cluster_ref_dist = 0.0f;
			}
			// Continuation of active cluster
			else
			{
				active_cluster_count++;
			}
		}
	}

	// End last active cluster
	if (active_cluster_count != 0)
	{
		if (active_cluster_count > largest_cluster_count)
		{
			largest_cluster_count = active_cluster_count;
		}
	}

	return (float)largest_cluster_count / (float)queue_length_n > signature_similarity_threshold;
}

static int compare_signature(const void *a, const void *b)
{
	const signature_t *sig_a = (const signature_t *)a;
	const signature_t *sig_b = (const signature_t *)b;

	if (sig_a->weighted_distance < sig_b->weighted_distance)
	{
		return -1;
	}
	else if (sig_a->weighted_distance > sig_b->weighted_distance)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
