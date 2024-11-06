// Copyright (c) Acconeer AB, 2024
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <complex.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "acc_algorithm.h"
#include "acc_config.h"
#include "acc_config_subsweep.h"
#include "acc_definitions_common.h"
#include "acc_integration.h"
#include "acc_integration_log.h"
#include "acc_processing.h"
#include "example_waste_level.h"

#define MODULE "example_waste_level"

struct waste_level_handle
{
	/** State. Affects output between frames */
	struct
	{
		/** Ring buffer for distance estimations. Has capacity processing_config.median_filter_len */
		float *distance_history_points;

		/** How many valid elements there are in distance_history_points (starting at 0) */
		uint16_t distance_history_len;

		/** Write index for the distance_history_points */
		uint16_t distance_history_write_idx;
	} state;

	/** Float scratch. Has length == max(sweeps_per_frame, median_filter_len) */
	float *scratch;
};

/**
 * @brief Validate an app_config (processing & sensor config).
 *
 * @param[in] app_config The app config
 * @return True if config was valid, false otherwise.
 */
static bool validate_app_config(const waste_level_app_config_t *app_config);

/**
 * @brief Helper function for validation
 *
 * @param[in] sensor_config The sensor config
 * @return True if all subsweeps' start_points are strictly increasing, false otherwise
 */
static bool subsweep_start_points_strictly_increases(const acc_config_t *sensor_config);

/**
 * @brief Helper function for validation
 *
 * @param[in] sensor_config The sensor config
 * @param[out] offending_subsweep The subsweep index of a subsweep that overlaps into
 *                                the subsweep after it
 * @return True if sensor config has overlapping (in distance) subsweeps, false otherwise
 */
static bool has_overlapping_subsweeps(const acc_config_t *sensor_config, uint8_t *offending_subsweep);

/**
 * @brief Helper function for validation
 *
 * @param[in] sensor_config The sensor config
 * @return True if start_point for subsweep 0 is invalid
 */
static bool is_invalid_start_point(const acc_config_t *sensor_config);

/**
 * @brief Helper function for validation
 *
 * @param[in] sensor_config The sensor config
 * @param[in] distance_m The distance to check
 * @return True if distance_m is outside the interval measured by sensor_config, false otherwise
 */
static bool is_outside_measuring_interval(const acc_config_t *sensor_config, float distance_m);

/**
 * @brief Helper function for validation
 *
 * @param[in] sensor_config The sensor config
 * @param[in] subsweep_idx The distance to check
 * @return The approximate distance to the first point measured in subsweep with index subsweep_idx
 */
static float subsweep_approx_start_m(const acc_config_t *sensor_config, uint8_t subsweep_idx);

/**
 * @brief Helper function for validation
 *
 * @param[in] sensor_config The sensor config
 * @param[in] subsweep_idx The distance to check
 * @return The approximate distance to the last point measured in subsweep with index subsweep_idx
 */
static float subsweep_approx_end_m(const acc_config_t *sensor_config, uint8_t subsweep_idx);

/**
 * @brief Calculate a point's corresponding distance in meters
 *
 * @param[in] sensor_config The sensor config
 * @param[in] metadata The processing metadata
 * @param[in] point_idx The index in a sweep of the point to get distance for
 * @return The distance of point at point_idx
 */
static float point_in_sweep_to_distance_m(const acc_config_t *sensor_config, const acc_processing_metadata_t *metadata, uint16_t point_idx);

/**
 * @brief Write next value into a ring buffer
 *
 * @param[in, out] buffer The circular buffer
 * @param[in] capacity The allocated number of elements of buffer
 * @param[in, out] length The buffer length (number of valid elements)
 * @param[in, out] write_idx The next index in buffer to write to
 * @param[in] elem The new element to write
 */
static void write_to_ring_buffer(float *buffer, uint16_t capacity, uint16_t *length, uint16_t *write_idx, float elem);

/**
 * @brief Copy non-NaN floats between buffers
 *
 * @param[in] in Source buffer of floats which can contain NaNs
 * @param[out] out The destination buffer. Will not contain NaNs
 * @param[in] length The allocated size of in & out
 * @return The number of elements written to out
 */
static uint16_t copy_non_nan_floats(const float *in, float *out, uint16_t length);

/**
 * @brief Populate a waste level result with its human-readable entries
 *
 * @param[in] filtered_distance The median filtered distance
 * @param[in] processing_config The waste processing config
 * @param[out] result The waste result
 */
static void set_level_numbers(float filtered_distance, const waste_level_processing_config_t *processing_config, waste_level_result_t *result);

waste_level_app_config_t *waste_level_app_config_create(void)
{
	waste_level_app_config_t *app_config = acc_integration_mem_alloc(sizeof(*app_config));
	bool                      status     = app_config != NULL;

	if (status)
	{
		app_config->sensor_config = acc_config_create();
		status                    = app_config->sensor_config != NULL;
	}

	if (status)
	{
		waste_level_app_config_set_preset(WASTE_LEVEL_PRESET_PLASTIC_WASTE_BIN, app_config);
	}

	return app_config;
}

void waste_level_app_config_destroy(waste_level_app_config_t *app_config)
{
	if (app_config != NULL)
	{
		if (app_config->sensor_config != NULL)
		{
			acc_config_destroy(app_config->sensor_config);
		}

		acc_integration_mem_free(app_config);
	}
}

void waste_level_app_config_set_preset(waste_level_preset_t preset, waste_level_app_config_t *app_config)
{
	bool status = app_config != NULL;

	if (status)
	{
		switch (preset)
		{
			case WASTE_LEVEL_PRESET_PLASTIC_WASTE_BIN:
				app_config->processing_config.bin_start_m           = 0.15f;
				app_config->processing_config.bin_end_m             = 1.00f;
				app_config->processing_config.threshold             = 0.30f;
				app_config->processing_config.distance_sequence_len = 4U;
				app_config->processing_config.median_filter_len     = 5U;

				acc_config_frame_rate_set(app_config->sensor_config, 5.0f);
				acc_config_sweeps_per_frame_set(app_config->sensor_config, 32U);
				acc_config_num_subsweeps_set(app_config->sensor_config, 2U);

				acc_config_subsweep_start_point_set(app_config->sensor_config, 40U, 0U);
				acc_config_subsweep_step_length_set(app_config->sensor_config, 8U, 0U);
				acc_config_subsweep_num_points_set(app_config->sensor_config, 14U, 0U);
				acc_config_subsweep_profile_set(app_config->sensor_config, ACC_CONFIG_PROFILE_1, 0U);
				acc_config_subsweep_hwaas_set(app_config->sensor_config, 4U, 0U);

				acc_config_subsweep_start_point_set(app_config->sensor_config, 150U, 1U);
				acc_config_subsweep_step_length_set(app_config->sensor_config, 12U, 1U);
				acc_config_subsweep_num_points_set(app_config->sensor_config, 23U, 1U);
				acc_config_subsweep_profile_set(app_config->sensor_config, ACC_CONFIG_PROFILE_3, 1U);
				acc_config_subsweep_hwaas_set(app_config->sensor_config, 8U, 1U);
				break;
			default:
			case WASTE_LEVEL_PRESET_NONE:
				break;
		}
	}
}

waste_level_handle_t *waste_level_handle_create(const waste_level_app_config_t *app_config)
{
	uint16_t              sweeps_per_frame  = 0U;
	uint16_t              median_filter_len = 0U;
	waste_level_handle_t *handle            = NULL;

	bool status = validate_app_config(app_config);

	if (status)
	{
		handle = acc_integration_mem_alloc(sizeof(*handle));
		status = handle != NULL;
	}

	if (status)
	{
		sweeps_per_frame  = acc_config_sweeps_per_frame_get(app_config->sensor_config);
		median_filter_len = app_config->processing_config.median_filter_len;

		handle->state.distance_history_len       = 0U;
		handle->state.distance_history_write_idx = 0U;
	}

	if (status)
	{
		uint16_t num_elem = sweeps_per_frame > median_filter_len ? sweeps_per_frame : median_filter_len;
		handle->scratch   = (float *)acc_integration_mem_alloc(num_elem * sizeof(*handle->scratch));
		status            = handle->scratch != NULL;
	}

	if (status)
	{
		handle->state.distance_history_points =
		    (float *)acc_integration_mem_alloc(median_filter_len * sizeof(*handle->state.distance_history_points));
		status = handle->state.distance_history_points != NULL;
	}

	return handle;
}

void waste_level_handle_destroy(waste_level_handle_t *handle)
{
	if (handle != NULL)
	{
		if (handle->state.distance_history_points != NULL)
		{
			acc_integration_mem_free(handle->state.distance_history_points);
		}

		if (handle->scratch != NULL)
		{
			acc_integration_mem_free(handle->scratch);
		}

		acc_integration_mem_free(handle);
	}
}

void waste_level_process(waste_level_handle_t            *handle,
                         const waste_level_app_config_t  *app_config,
                         const acc_processing_metadata_t *metadata,
                         const acc_int16_complex_t       *frame,
                         waste_level_result_t            *waste_level_result)
{
	if (handle != NULL && app_config != NULL && metadata != NULL && frame != NULL && waste_level_result != NULL)
	{
		uint16_t sweep_length      = metadata->sweep_data_length;
		uint16_t sweeps_per_frame  = acc_config_sweeps_per_frame_get(app_config->sensor_config);
		uint16_t distance_seq_len  = app_config->processing_config.distance_sequence_len;
		uint16_t median_filter_len = app_config->processing_config.median_filter_len;
		float    threshold_squared = app_config->processing_config.threshold * app_config->processing_config.threshold;

		uint16_t phase_vars_under_threshold = 0U;

		bool     point_of_waste_found = false;
		uint16_t point_of_waste       = 0U;

		for (uint16_t point_idx = 0U; point_idx < sweep_length; point_idx++)
		{
			float complex column_sum;

			acc_algorithm_sum_sweep(frame, sweep_length, sweeps_per_frame, point_idx, point_idx + 1U, &column_sum);

			acc_algorithm_conj_f32(&column_sum, 1U);

			for (uint16_t sweep_idx = 0U; sweep_idx < sweeps_per_frame; sweep_idx++)
			{
				uint16_t            frame_idx   = sweep_idx * sweep_length + point_idx;
				acc_int16_complex_t point       = frame[frame_idx];
				float complex       float_point = (float)point.real + (float)point.imag * I;

				handle->scratch[sweep_idx] = cargf(float_point * column_sum);
			}

			float point_phase_variance = acc_algorithm_variance_f32(handle->scratch, sweeps_per_frame);

			if (point_phase_variance < threshold_squared)
			{
				phase_vars_under_threshold++;
			}
			else
			{
				phase_vars_under_threshold = 0U;
			}

			if (phase_vars_under_threshold >= distance_seq_len)
			{
				point_of_waste       = point_idx - (phase_vars_under_threshold - 1U);
				point_of_waste_found = true;
				break;
			}
		}

		float new_distance = !point_of_waste_found ? NAN : point_in_sweep_to_distance_m(app_config->sensor_config, metadata, point_of_waste);

		write_to_ring_buffer(handle->state.distance_history_points,
		                     median_filter_len,
		                     &handle->state.distance_history_len,
		                     &handle->state.distance_history_write_idx,
		                     new_distance);

		// copy non-NaN distance history points as median sorts inplace
		uint16_t num_normal = copy_non_nan_floats(handle->state.distance_history_points, handle->scratch, handle->state.distance_history_len);
		if (num_normal > 0U)
		{
			waste_level_result->level_found = true;

			float median_distance = acc_algorithm_median_f32(handle->scratch, num_normal);
			set_level_numbers(median_distance, &app_config->processing_config, waste_level_result);
		}
		else
		{
			waste_level_result->level_found = false;
		}
	}
}

void waste_level_processing_config_log(const waste_level_processing_config_t *config)
{
	printf("Waste level config:\n");
	printf("bin_start_m:           %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(config->bin_start_m));
	printf("bin_end_m:             %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(config->bin_end_m));
	printf("threshold:             %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(config->threshold));
	printf("distance_sequence_len: %" PRIu16 "\n", config->distance_sequence_len);
	printf("median_filter_len:     %" PRIu16 "\n", config->median_filter_len);
}

static bool validate_app_config(const waste_level_app_config_t *app_config)
{
	const waste_level_processing_config_t *processing_config;
	const acc_config_t                    *sensor_config;

	bool status = app_config != NULL;

	if (status)
	{
		sensor_config     = app_config->sensor_config;
		processing_config = &app_config->processing_config;
		status            = sensor_config != NULL && processing_config != NULL;
	}

	if (status && processing_config->bin_start_m < 0.08f)
	{
		fprintf(stderr, "Bin start needs to be greater or equal than 0.08m\n");
		status = false;
	}

	if (status && processing_config->bin_end_m < processing_config->bin_start_m)
	{
		fprintf(stderr, "Bin end needs to be greater than bin start\n");
		status = false;
	}

	if (status && (processing_config->threshold < 0.0f || processing_config->threshold > 4.0f))
	{
		fprintf(stderr, "Threshold shoud be in the range [0.0, 4.0], was %" PRIfloat "\n", ACC_LOG_FLOAT_TO_INTEGER(processing_config->threshold));
		status = false;
	}

	if (status && (processing_config->distance_sequence_len > 10U || processing_config->distance_sequence_len < 1U))
	{
		fprintf(stderr, "Distance sequence length shoud be in the range [1, 10], was %" PRIu16 "\n", processing_config->distance_sequence_len);
		status = false;
	}

	if (status && (processing_config->median_filter_len > 10U || processing_config->median_filter_len < 1U))
	{
		fprintf(stderr, "Median filter length shoud be in the range [1, 10], was %" PRIu16 "\n", processing_config->median_filter_len);
		status = false;
	}

	if (status && !subsweep_start_points_strictly_increases(sensor_config))
	{
		fprintf(stderr, "Subsweeps with higher indexes should measure farther away than those with lower indexes\n");
		status = false;
	}

	if (status)
	{
		uint8_t subsweep_idx;

		if (has_overlapping_subsweeps(sensor_config, &subsweep_idx))
		{
			fprintf(stderr, "Disallowed range overlap between susweeps %" PRIu8 " and %" PRIu8 "\n", subsweep_idx, subsweep_idx + 1U);
			status = false;
		}
	}

	if (status && is_invalid_start_point(sensor_config))
	{
		fprintf(stderr, "Start point must be greater or equal than 32\n");
		status = false;
	}

	if (status && is_outside_measuring_interval(sensor_config, processing_config->bin_start_m))
	{
		fprintf(stderr, "Bin start is outside measuring interval\n");
		status = false;
	}

	if (status && is_outside_measuring_interval(sensor_config, processing_config->bin_end_m))
	{
		fprintf(stderr, "Bin end is outside measuring interval\n");
		status = false;
	}

	if (status && acc_config_sweeps_per_frame_get(sensor_config) <= 3)
	{
		fprintf(stderr, "Sweeps per frame needs to be greater than 3\n");
		status = false;
	}

	return status;
}

static bool subsweep_start_points_strictly_increases(const acc_config_t *sensor_config)
{
	bool increasing = true;

	uint8_t num_subsweeps = acc_config_num_subsweeps_get(sensor_config);

	for (uint8_t subsweep_idx = 1U; subsweep_idx < num_subsweeps; subsweep_idx++)
	{
		int32_t prev_start = acc_config_subsweep_start_point_get(sensor_config, subsweep_idx - 1U);
		int32_t curr_start = acc_config_subsweep_start_point_get(sensor_config, subsweep_idx);

		if (curr_start <= prev_start)
		{
			increasing = false;
			break;
		}
	}

	return increasing;
}

static bool has_overlapping_subsweeps(const acc_config_t *sensor_config, uint8_t *offending_subsweep)
{
	bool    overlap_found = false;
	uint8_t num_subsweeps = acc_config_num_subsweeps_get(sensor_config);

	for (uint8_t subsweep_idx = 0U; subsweep_idx < num_subsweeps - 1U; subsweep_idx++)
	{
		float curr_end   = subsweep_approx_end_m(sensor_config, subsweep_idx);
		float next_start = subsweep_approx_start_m(sensor_config, subsweep_idx + 1U);

		if (curr_end >= next_start)
		{
			*offending_subsweep = subsweep_idx;
			overlap_found       = true;
			break;
		}
	}

	return overlap_found;
}

static bool is_invalid_start_point(const acc_config_t *sensor_config)
{
	const int32_t minimum_allowed_start_point = 32;

	int32_t min_start_point = acc_config_subsweep_start_point_get(sensor_config, 0U);

	return min_start_point < minimum_allowed_start_point;
}

static bool is_outside_measuring_interval(const acc_config_t *sensor_config, float distance_m)
{
	float min_measured_point_m = 1000.0f;
	float max_measured_point_m = -1000.0f;

	uint8_t num_subsweeps = acc_config_num_subsweeps_get(sensor_config);

	for (uint8_t subsweep_idx = 0U; subsweep_idx < num_subsweeps; subsweep_idx++)
	{
		float current_begin_m = subsweep_approx_start_m(sensor_config, subsweep_idx);
		float current_end_m   = subsweep_approx_end_m(sensor_config, subsweep_idx);

		if (current_begin_m < min_measured_point_m)
		{
			min_measured_point_m = current_begin_m;
		}

		if (current_end_m > max_measured_point_m)
		{
			max_measured_point_m = current_end_m;
		}
	}

	return distance_m < min_measured_point_m || distance_m > max_measured_point_m;
}

static float subsweep_approx_start_m(const acc_config_t *sensor_config, uint8_t subsweep_idx)
{
	return acc_algorithm_get_distance_m(acc_config_subsweep_step_length_get(sensor_config, subsweep_idx),
	                                    acc_config_subsweep_start_point_get(sensor_config, subsweep_idx),
	                                    ACC_APPROX_BASE_STEP_LENGTH_M,
	                                    0U);
}

static float subsweep_approx_end_m(const acc_config_t *sensor_config, uint8_t subsweep_idx)
{
	return acc_algorithm_get_distance_m(acc_config_subsweep_step_length_get(sensor_config, subsweep_idx),
	                                    acc_config_subsweep_start_point_get(sensor_config, subsweep_idx),
	                                    ACC_APPROX_BASE_STEP_LENGTH_M,
	                                    acc_config_subsweep_num_points_get(sensor_config, subsweep_idx) - 1U);
}

static float point_in_sweep_to_distance_m(const acc_config_t *sensor_config, const acc_processing_metadata_t *metadata, uint16_t point_idx)
{
	uint8_t num_subsweeps = acc_config_num_subsweeps_get(sensor_config);
	float   distance_m    = 0.0f;

	for (uint8_t subsweep_idx = 0U; subsweep_idx < num_subsweeps; subsweep_idx++)
	{
		uint16_t start = metadata->subsweep_data_offset[subsweep_idx];
		uint16_t end   = start + metadata->subsweep_data_length[subsweep_idx];

		if (point_idx >= start && point_idx < end)
		{
			distance_m = acc_algorithm_get_distance_m(acc_config_subsweep_step_length_get(sensor_config, subsweep_idx),
			                                          acc_config_subsweep_start_point_get(sensor_config, subsweep_idx),
			                                          acc_processing_points_to_meter(1),
			                                          point_idx - start);
			break;
		}
	}

	return distance_m;
}

static void write_to_ring_buffer(float *buffer, uint16_t capacity, uint16_t *length, uint16_t *write_idx, float elem)
{
	buffer[*write_idx] = elem;

	*write_idx = (*write_idx + 1U) % capacity;

	if (*length < capacity)
	{
		*length = *length + 1U;
	}
}

static uint16_t copy_non_nan_floats(const float *in, float *out, uint16_t length)
{
	uint16_t num_normal = 0U;

	for (uint16_t i = 0U; i < length; i++)
	{
		if (fpclassify(in[i]) != FP_NAN)
		{
			out[num_normal] = in[i];
			num_normal++;
		}
	}

	return num_normal;
}

static void set_level_numbers(float filtered_distance, const waste_level_processing_config_t *processing_config, waste_level_result_t *result)
{
	float bin_end_m   = processing_config->bin_end_m;
	float bin_start_m = processing_config->bin_start_m;

	float fill_level_m         = bin_end_m - filtered_distance;
	float fill_level_percent_f = acc_algorithm_clip_f32(100.0f * fill_level_m / (bin_end_m - bin_start_m), 0.0f, 100.0f);

	fill_level_percent_f       += 0.5f; // for rounding
	uint8_t fill_level_percent  = (uint8_t)fill_level_percent_f;

	result->level_m       = fill_level_m;
	result->level_percent = fill_level_percent;
}
