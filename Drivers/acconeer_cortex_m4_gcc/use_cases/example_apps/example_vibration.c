// Copyright (c) Acconeer AB, 2024
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <complex.h>
#include <float.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "acc_alg_basic_utils.h"
#include "acc_algorithm.h"
#include "acc_config.h"
#include "acc_config_subsweep.h"
#include "acc_integration.h"
#include "acc_integration_log.h"
#include "acc_processing.h"
#include "example_vibration.h"

#define MODULE "example_vibration"

/**    /  c/f  \
 *    | ------- |  * 1e6      with c = 299792458 m/s,
 *     \   2   /                   f = 60.5 GHz
 * ------------------------
 *         2 pi
 */
#define RADIANS_TO_DISPLACEMENT_FACTOR (394.32604621792916f)

#define NUM_POINTS             (1U)
#define CFAR_WINDOW_LENGTH     (10U)
#define CFAR_HALF_GUARD_LENGTH (5U)
#define CFAR_MARGIN            (CFAR_WINDOW_LENGTH + CFAR_HALF_GUARD_LENGTH)

#define IS_POWER_OF_TWO(x) (((x) & ((x) - 1)) == 0)
#define BOOL_TO_STR(b)     (b ? "true" : "false")

/**
 * Circular buffer.
 * Usually contains a "length" to keep track of the number of valid elements,
 * But that was not needed here.
 */
typedef struct
{
	float *buffer;

	/** The next index to write to. Starts at 0 and grows up to 'capacity' before wrapping */
	uint16_t write_idx;

	/** The maximum number of elements the buffer can hold */
	uint16_t capacity;
} circular_float_buffer_t;

struct acc_vibration_handle
{
	acc_config_t *sensor_config;

	bool  low_frequency_enhancement_enabled;
	bool  continuous_data_acquisition;
	float psd_to_radians_conversion_factor;
	float displacement_conversion_factor;

	bool has_init;

	circular_float_buffer_t time_series;
	float                  *frequencies;
	float                  *lp_displacements;

	int32_t *double_buffer_filter_buffer;

	uint16_t subframe_length;
	uint16_t padded_time_series_length;
	uint16_t rfft_length_shift;
	uint16_t rfft_output_length;
	uint16_t rfft_read_offset;
	uint16_t data_length;

	float         *zero_mean_time_series;
	complex float *rfft_output;
	float         *threshold;
};

//-----------------------------
// Private declarations
//-----------------------------

/** @brief Write an angle (in rads) to the circular buffer. The angle will be "unwrapped" before write */
static void circular_float_buffer_write_angle(circular_float_buffer_t *cb, float new_element);

/** @brief Get an element of the circular buffer given its "age". A chronological index of 0 returns the oldest element in the buffer */
static float circular_float_buffer_get(const circular_float_buffer_t *cb, uint16_t chronological_idx);

static bool validate_config(const acc_vibration_config_t *config);

static bool translate_config(const acc_vibration_config_t *config, acc_config_t *sensor_config);

static void setup_rfft_bounds(acc_vibration_handle_t *handle, const acc_vibration_config_t *config);

static void setup_sample_frequencies(acc_vibration_handle_t *handle, float sweep_rate);

static void update_vibration_result(acc_vibration_handle_t *handle, acc_vibration_config_t *config, acc_vibration_result_t *result);

static float *get_zero_mean_time_series(acc_vibration_handle_t *handle);

static void calculate_threshold(acc_vibration_handle_t *handle, acc_vibration_config_t *config);

//-----------------------------
// Public definitions
//-----------------------------

void acc_vibration_preset_set(acc_vibration_config_t *config, acc_vibration_preset_t preset)
{
	bool status = config != NULL;

	if (!status)
	{
		ACC_LOG_ERROR("Invalid config");
	}
	else
	{
		switch (preset)
		{
			case ACC_VIBRATION_PRESET_LOW_FREQUENCY:
				config->measured_point             = 80U;
				config->time_series_length         = 1024U;
				config->lp_coeff                   = 0.8f;
				config->threshold_sensitivity      = 1.5f;
				config->amplitude_threshold        = 100.0f;
				config->reported_displacement_mode = ACC_VIBRATION_REPORT_DISPLACEMENT_AS_AMPLITUDE;
				config->low_frequency_enhancement  = true;
				config->profile                    = ACC_CONFIG_PROFILE_3;
				config->frame_rate                 = 0.0f;
				config->sweep_rate                 = 200.0f;
				config->sweeps_per_frame           = 20U;
				config->hwaas                      = 16U;
				config->double_buffering           = true;
				config->continuous_sweep_mode      = true;
				config->inter_frame_idle_state     = ACC_CONFIG_IDLE_STATE_READY;
				config->inter_sweep_idle_state     = ACC_CONFIG_IDLE_STATE_READY;
				break;
			case ACC_VIBRATION_PRESET_HIGH_FREQUENCY:
				config->measured_point             = 80U;
				config->time_series_length         = 1024U;
				config->lp_coeff                   = 0.5f;
				config->threshold_sensitivity      = 1.5f;
				config->amplitude_threshold        = 100.0f;
				config->reported_displacement_mode = ACC_VIBRATION_REPORT_DISPLACEMENT_AS_AMPLITUDE;
				config->low_frequency_enhancement  = false;
				config->profile                    = ACC_CONFIG_PROFILE_3;
				config->frame_rate                 = 0.0f;
				config->sweep_rate                 = 10000.0f;
				config->sweeps_per_frame           = 2048;
				config->hwaas                      = 16U;
				config->double_buffering           = false;
				config->continuous_sweep_mode      = false;
				config->inter_frame_idle_state     = ACC_CONFIG_IDLE_STATE_READY;
				config->inter_sweep_idle_state     = ACC_CONFIG_IDLE_STATE_READY;
				break;
			default:
				ACC_LOG_ERROR("Unknown preset");
				break;
		}
	}
}

void acc_vibration_config_log(const acc_vibration_config_t *config)
{
	ACC_LOG_INFO("measured_point:             %" PRIi32, config->measured_point);
	ACC_LOG_INFO("time_series_length:         %" PRIu16, config->time_series_length);
	ACC_LOG_INFO("lp_coeff:                   %" PRIfloat, ACC_LOG_FLOAT_TO_INTEGER(config->lp_coeff));
	ACC_LOG_INFO("threshold_sensitivity:      %" PRIfloat, ACC_LOG_FLOAT_TO_INTEGER(config->threshold_sensitivity));
	ACC_LOG_INFO("amplitude_threshold:        %" PRIfloat, ACC_LOG_FLOAT_TO_INTEGER(config->amplitude_threshold));

	switch (config->reported_displacement_mode)
	{
		case ACC_VIBRATION_REPORT_DISPLACEMENT_AS_AMPLITUDE:
			ACC_LOG_INFO("reported_displacement_mode: AMPLITUDE");
			break;
		case ACC_VIBRATION_REPORT_DISPLACEMENT_AS_PEAK2PEAK:
			ACC_LOG_INFO("reported_displacement_mode: PEAK2PEAK");
			break;
		default:
			ACC_LOG_ERROR("reported_displacement_mode: UNKNOWN");
			break;
	}

	ACC_LOG_INFO("low_frequency_enhancement:  %s", BOOL_TO_STR(config->low_frequency_enhancement));
	ACC_LOG_INFO("profile:                    %d", config->profile);
	ACC_LOG_INFO("frame_rate:                 %" PRIfloat, ACC_LOG_FLOAT_TO_INTEGER(config->frame_rate));
	ACC_LOG_INFO("sweep_rate:                 %" PRIfloat, ACC_LOG_FLOAT_TO_INTEGER(config->sweep_rate));
	ACC_LOG_INFO("sweeps_per_frame:           %" PRIu16, config->sweeps_per_frame);
	ACC_LOG_INFO("hwaas:                      %" PRIu16, config->hwaas);
	ACC_LOG_INFO("double_buffering:           %s", BOOL_TO_STR(config->double_buffering));
	ACC_LOG_INFO("continuous_sweep_mode:      %s", BOOL_TO_STR(config->continuous_sweep_mode));

	switch (config->inter_frame_idle_state)
	{
		case ACC_CONFIG_IDLE_STATE_DEEP_SLEEP:
			ACC_LOG_INFO("inter_frame_idle_state:     DEEP_SLEEP");
			break;
		case ACC_CONFIG_IDLE_STATE_SLEEP:
			ACC_LOG_INFO("inter_frame_idle_state:     SLEEP");
			break;
		case ACC_CONFIG_IDLE_STATE_READY:
			ACC_LOG_INFO("inter_frame_idle_state:     READY");
			break;
		default:
			ACC_LOG_ERROR("inter_frame_idle_state:     UNKNOWN");
			break;
	}

	switch (config->inter_sweep_idle_state)
	{
		case ACC_CONFIG_IDLE_STATE_DEEP_SLEEP:
			ACC_LOG_INFO("inter_sweep_idle_state:     DEEP_SLEEP");
			break;
		case ACC_CONFIG_IDLE_STATE_SLEEP:
			ACC_LOG_INFO("inter_sweep_idle_state:     SLEEP");
			break;
		case ACC_CONFIG_IDLE_STATE_READY:
			ACC_LOG_INFO("inter_sweep_idle_state:     READY");
			break;
		default:
			ACC_LOG_ERROR("inter_sweep_idle_state:     UNKNOWN");
			break;
	}
}

acc_vibration_handle_t *acc_vibration_handle_create(const acc_vibration_config_t *config)
{
	uint16_t sweeps_per_frame = 0U;
	float    sweep_rate       = 0.0f;

	acc_vibration_handle_t *handle = NULL;

	bool status = config != NULL;

	if (status)
	{
		status = validate_config(config);
	}

	if (status)
	{
		handle = acc_integration_mem_alloc(sizeof(*handle));
		status = handle != NULL;
	}

	if (status)
	{
		handle->low_frequency_enhancement_enabled = config->low_frequency_enhancement;
		sweeps_per_frame                          = config->sweeps_per_frame;
		sweep_rate                                = config->sweep_rate;

		handle->has_init = false;

		setup_rfft_bounds(handle, config);

		handle->subframe_length    = sweeps_per_frame;
		handle->rfft_output_length = (handle->padded_time_series_length / 2U) + 1U;
		handle->rfft_read_offset   = 1U;
		handle->data_length        = handle->rfft_output_length - handle->rfft_read_offset;

		handle->continuous_data_acquisition = config->continuous_sweep_mode;

		handle->psd_to_radians_conversion_factor =
		    2.0f / (handle->continuous_data_acquisition ? (float)config->time_series_length : (float)sweeps_per_frame);

		handle->displacement_conversion_factor = handle->psd_to_radians_conversion_factor * RADIANS_TO_DISPLACEMENT_FACTOR;

		if (config->reported_displacement_mode == ACC_VIBRATION_REPORT_DISPLACEMENT_AS_PEAK2PEAK)
		{
			handle->displacement_conversion_factor *= 2.0f;
		}
	}

	if (status)
	{
		handle->sensor_config = acc_config_create();
		status                = handle->sensor_config != NULL;
	}

	if (status)
	{
		status = translate_config(config, handle->sensor_config);
	}

	if (status)
	{
		handle->time_series.buffer    = acc_integration_mem_calloc(config->time_series_length, sizeof(*handle->time_series.buffer));
		handle->time_series.write_idx = 0U;
		handle->time_series.capacity  = config->time_series_length;

		status = handle->time_series.buffer != NULL;
	}

	if (status)
	{
		handle->frequencies = acc_integration_mem_calloc(handle->rfft_output_length, sizeof(*handle->frequencies));
		status              = handle->frequencies != NULL;
	}

	if (status)
	{
		setup_sample_frequencies(handle, sweep_rate);
	}

	if (status)
	{
		handle->double_buffer_filter_buffer = acc_integration_mem_calloc(sweeps_per_frame - 2U, sizeof(*handle->double_buffer_filter_buffer));
		status                              = handle->double_buffer_filter_buffer != NULL;
	}

	if (status)
	{
		handle->zero_mean_time_series = acc_integration_mem_calloc(config->time_series_length, sizeof(*handle->zero_mean_time_series));
		status                        = handle->zero_mean_time_series != NULL;
	}

	if (status)
	{
		handle->rfft_output = acc_integration_mem_calloc(handle->rfft_output_length, sizeof(*handle->rfft_output));
		status              = handle->rfft_output != NULL;
	}

	if (status)
	{
		handle->lp_displacements = acc_integration_mem_calloc(handle->data_length, sizeof(*handle->lp_displacements));
		status                   = handle->lp_displacements != NULL;
	}

	if (status)
	{
		handle->threshold = acc_integration_mem_calloc(handle->data_length, sizeof(*handle->threshold));
		status            = handle->threshold != NULL;
	}

	if (!status)
	{
		acc_vibration_handle_destroy(handle);
	}

	return status ? handle : NULL;
}

const acc_config_t *acc_vibration_handle_sensor_config_get(acc_vibration_handle_t *handle)
{
	return handle->sensor_config;
}

const float *acc_vibration_handle_displacement_history_get(acc_vibration_handle_t *handle, uint16_t *num_elem)
{
	float *history_p = NULL;
	bool   status    = handle != NULL;

	if (status)
	{
		history_p = handle->lp_displacements;
		*num_elem = handle->data_length;
	}

	return status ? history_p : NULL;
}

bool acc_vibration_handle_continuous_data_acquisition_get(acc_vibration_handle_t *handle, bool *continuous_data_acquisition)
{
	bool status = handle != NULL;

	if (status)
	{
		*continuous_data_acquisition = handle->continuous_data_acquisition;
	}

	return status;
}

void acc_vibration_handle_destroy(acc_vibration_handle_t *handle)
{
	if (handle != NULL)
	{
		if (handle->sensor_config != NULL)
		{
			acc_config_destroy(handle->sensor_config);
			handle->sensor_config = NULL;
		}

		if (handle->time_series.buffer != NULL)
		{
			acc_integration_mem_free(handle->time_series.buffer);
			handle->time_series.buffer = NULL;
		}

		if (handle->frequencies != NULL)
		{
			acc_integration_mem_free(handle->frequencies);
			handle->frequencies = NULL;
		}

		if (handle->lp_displacements != NULL)
		{
			acc_integration_mem_free(handle->lp_displacements);
			handle->lp_displacements = NULL;
		}

		if (handle->double_buffer_filter_buffer != NULL)
		{
			acc_integration_mem_free(handle->double_buffer_filter_buffer);
			handle->double_buffer_filter_buffer = NULL;
		}

		if (handle->zero_mean_time_series != NULL)
		{
			acc_integration_mem_free(handle->zero_mean_time_series);
			handle->zero_mean_time_series = NULL;
		}

		if (handle->rfft_output != NULL)
		{
			acc_integration_mem_free(handle->rfft_output);
			handle->rfft_output = NULL;
		}

		if (handle->threshold != NULL)
		{
			acc_integration_mem_free(handle->threshold);
			handle->threshold = NULL;
		}

		acc_integration_mem_free(handle);
	}
}

void acc_vibration_process(acc_processing_result_t *proc_result,
                           acc_vibration_handle_t  *handle,
                           acc_vibration_config_t  *config,
                           acc_vibration_result_t  *result)
{
	const uint16_t sweeps_per_frame = config->sweeps_per_frame;

	result->max_sweep_amplitude   = 0.0f;
	result->max_displacement      = FLT_MAX;
	result->max_displacement_freq = FLT_MAX;
	result->time_series_std       = FLT_MAX;

	/*
	 * Check if an object is close enough in front of the sensor to
	 * reach the configured threshold.
	 */

	for (uint16_t i = 0; i < handle->subframe_length; i++)
	{
		uint16_t frame_idx;

		if (handle->low_frequency_enhancement_enabled)
		{
			frame_idx = 2U * i;
		}
		else
		{
			frame_idx = i;
		}

		acc_int16_complex_t point   = proc_result->frame[frame_idx];
		result->max_sweep_amplitude = fmaxf(result->max_sweep_amplitude, cabsf(point.real + (point.imag * I)));
	}

	if (result->max_sweep_amplitude < config->amplitude_threshold)
	{
		// Return without a (complete) result.
		handle->has_init = false;
		return;
	}

	if (handle->low_frequency_enhancement_enabled)
	{
		/* Adjust measure frame with loopback frame */
		for (uint16_t frame_idx = 0U; frame_idx < handle->subframe_length; frame_idx++)
		{
			uint16_t measure_src_idx  = 2U * frame_idx;
			uint16_t loopback_src_idx = (2U * frame_idx) + 1U;
			uint16_t dst_idx          = frame_idx;

			acc_int16_complex_t measure  = proc_result->frame[measure_src_idx];
			acc_int16_complex_t loopback = proc_result->frame[loopback_src_idx];

			complex float measure_cf  = (float)measure.real + ((float)measure.imag * I);
			complex float loopback_cf = (float)loopback.real + ((float)loopback.imag * I);

			complex float lb_norm     = loopback_cf / cabsf(loopback_cf);
			complex float compensated = measure_cf * conjf(lb_norm);

			proc_result->frame[dst_idx].real = (int16_t)(crealf(compensated) + 0.5f);
			proc_result->frame[dst_idx].imag = (int16_t)(cimagf(compensated) + 0.5f);
		}
	}

	/*
	 * Process sensor data.
	 */

	// Handle frame based on whether or not continuous sweep mode is used.
	if (handle->continuous_data_acquisition)
	{
		acc_algorithm_double_buffering_frame_filter(proc_result->frame, sweeps_per_frame, NUM_POINTS, handle->double_buffer_filter_buffer);
	}

	for (uint16_t i = 0; i < handle->subframe_length; i++)
	{
		acc_int16_complex_t point       = proc_result->frame[i];
		float               new_element = cargf(point.real + (point.imag * I));

		circular_float_buffer_write_angle(&handle->time_series, new_element);
	}

	/*
	 * Estimate displacement per frequency.
	 */

	float *zero_mean_time_series = get_zero_mean_time_series(handle);

	acc_algorithm_rfft(zero_mean_time_series, config->time_series_length, handle->rfft_length_shift, handle->rfft_output);

	for (uint16_t i = 0; i < handle->data_length; i++)
	{
		float z_abs        = cabsf(handle->rfft_output[i + handle->rfft_read_offset]);
		float displacement = z_abs * handle->displacement_conversion_factor;

		if (handle->has_init)
		{
			handle->lp_displacements[i] = handle->lp_displacements[i] * config->lp_coeff + displacement * (1.0f - config->lp_coeff);
		}
		else
		{
			handle->lp_displacements[i] = displacement;
		}
	}

	handle->has_init = true;

	for (uint16_t i = 0U; i < config->time_series_length; i++)
	{
		zero_mean_time_series[i] *= RADIANS_TO_DISPLACEMENT_FACTOR;
	}

	result->time_series_std = acc_algorithm_stddev_f32(zero_mean_time_series, config->time_series_length);

	update_vibration_result(handle, config, result);
}

//-----------------------------
// Private definitions
//-----------------------------

static void circular_float_buffer_write_angle(circular_float_buffer_t *cb, float new_element)
{
	// "unwrap" the angle to make fall in the range [-pi, pi] instead of being e.g. 42pi
	float unwrapped = new_element;
	acc_algorithm_unwrap(&unwrapped, 1U);

	cb->buffer[cb->write_idx] = unwrapped;
	cb->write_idx             = (cb->write_idx + 1U) % cb->capacity;
}

static float circular_float_buffer_get(const circular_float_buffer_t *cb, uint16_t chronological_idx)
{
	// Say a, b & c has just been written, then 'write_index' will point to the next index to
	// write over (i.e. the oldest). We can use this to index the circular buffer in ascending
	// chronological order (oldest to newest).
	// | a | b | c | x | y | z |
	//               ^
	uint16_t buffer_idx = (cb->write_idx + chronological_idx) % cb->capacity;

	return cb->buffer[buffer_idx];
}

static bool validate_config(const acc_vibration_config_t *config)
{
	bool success = true;

	if (!IS_POWER_OF_TWO(config->time_series_length))
	{
		ACC_LOG_WARNING("**************************** [Warning] *********************************");
		ACC_LOG_WARNING("** time_series_length should be power of 2 for efficient usage of FFT **");
		ACC_LOG_WARNING("************************************************************************");
	}

	if (config->time_series_length < 2U)
	{
		ACC_LOG_ERROR("time_series_length too small");
		success = false;
	}

	if (config->sweep_rate == 0.0f)
	{
		ACC_LOG_ERROR("sweep_rate must be set");
		success = false;
	}

	if (config->continuous_sweep_mode && !config->double_buffering)
	{
		ACC_LOG_ERROR("double buffering needs to be activated to achieve sufficient sweep rate when using continuous sweep mode");
		success = false;
	}

	return success;
}

static bool translate_config(const acc_vibration_config_t *config, acc_config_t *sensor_config)
{
	bool status = config != NULL;

	if (status)
	{
		acc_config_prf_t measure_prf = acc_algorithm_select_prf(config->measured_point, config->profile, ACC_APPROX_BASE_STEP_LENGTH_M);

		// clang-format off
		acc_config_sweep_rate_set(sensor_config,             config->sweep_rate);
		acc_config_frame_rate_set(sensor_config,             config->frame_rate);
		acc_config_sweeps_per_frame_set(sensor_config,       config->sweeps_per_frame);
		acc_config_continuous_sweep_mode_set(sensor_config,  config->continuous_sweep_mode);
		acc_config_double_buffering_set(sensor_config,       config->double_buffering);
		acc_config_inter_frame_idle_state_set(sensor_config, config->inter_frame_idle_state);
		acc_config_inter_sweep_idle_state_set(sensor_config, config->inter_sweep_idle_state);
		// clang-format on

		acc_config_num_subsweeps_set(sensor_config, 1U);

		// clang-format off
		acc_config_subsweep_start_point_set(sensor_config,       config->measured_point, 0U);
		acc_config_subsweep_num_points_set(sensor_config,        1U,                     0U);
		acc_config_subsweep_step_length_set(sensor_config,       1U,                     0U);
		acc_config_subsweep_hwaas_set(sensor_config,             config->hwaas,          0U);
		acc_config_subsweep_receiver_gain_set(sensor_config,     10U,                    0U);
		acc_config_subsweep_enable_tx_set(sensor_config,         true,                   0U);
		acc_config_subsweep_phase_enhancement_set(sensor_config, false,                  0U);
		acc_config_subsweep_enable_loopback_set(sensor_config,   false,                  0U);
		acc_config_subsweep_profile_set(sensor_config,           config->profile,        0U);
		acc_config_subsweep_prf_set(sensor_config,               measure_prf,            0U);
		// clang-format on

		if (config->low_frequency_enhancement)
		{
			acc_config_prf_t loopback_prf = acc_algorithm_select_prf(0, config->profile, ACC_APPROX_BASE_STEP_LENGTH_M);

			acc_config_num_subsweeps_set(sensor_config, 2U);

			// clang-format off
			acc_config_subsweep_start_point_set(sensor_config,       0,                    1U);
			acc_config_subsweep_num_points_set(sensor_config,        1U,                   1U);
			acc_config_subsweep_step_length_set(sensor_config,       1U,                   1U);
			acc_config_subsweep_hwaas_set(sensor_config,             8U,                   1U);
			acc_config_subsweep_receiver_gain_set(sensor_config,     10U,                  1U);
			acc_config_subsweep_enable_tx_set(sensor_config,         true,                 1U);
			acc_config_subsweep_phase_enhancement_set(sensor_config, false,                1U);
			acc_config_subsweep_enable_loopback_set(sensor_config,   true,                 1U);
			acc_config_subsweep_profile_set(sensor_config,           ACC_CONFIG_PROFILE_5, 1U);
			acc_config_subsweep_prf_set(sensor_config,               loopback_prf,         1U);
			// clang-format on
		}
	}

	return status;
}

static void setup_sample_frequencies(acc_vibration_handle_t *handle, float sweep_rate)
{
	float sample_spacing = 1.0f / sweep_rate;

	acc_algorithm_rfftfreq(handle->padded_time_series_length, sample_spacing, handle->frequencies);
}

static void update_vibration_result(acc_vibration_handle_t *handle, acc_vibration_config_t *config, acc_vibration_result_t *result)
{
	/*
	 * Compare displacements to threshold (exclude first point as it
	 * does not form a peak).
	 */

	calculate_threshold(handle, config);

	float    max_displacement     = 0.0f;
	uint16_t max_displacement_idx = 0;

	result->max_displacement = 0.0f;

	for (uint16_t i = 1U; i < handle->data_length; i++)
	{
		if (handle->lp_displacements[i] > handle->threshold[i])
		{
			max_displacement = result->max_displacement;

			result->max_displacement = fmaxf(result->max_displacement, handle->lp_displacements[i]);

			if (result->max_displacement > max_displacement)
			{
				max_displacement_idx = i;
			}
		}
	}

	if (max_displacement_idx > 0)
	{
		result->max_displacement_freq = handle->frequencies[max_displacement_idx + handle->rfft_read_offset];
	}
	else
	{
		result->max_displacement = FLT_MAX;
	}
}

static float *get_zero_mean_time_series(acc_vibration_handle_t *handle)
{
	float mean = 0.0f;

	for (uint16_t i = 0; i < handle->time_series.capacity; i++)
	{
		mean += handle->time_series.buffer[i];
	}

	mean /= (float)handle->time_series.capacity;

	for (uint16_t i = 0; i < handle->time_series.capacity; i++)
	{
		handle->zero_mean_time_series[i] = circular_float_buffer_get(&handle->time_series, i) - mean;
	}

	return handle->zero_mean_time_series;
}

static void calculate_threshold(acc_vibration_handle_t *handle, acc_vibration_config_t *config)
{
	for (uint16_t i = 0; i < handle->data_length; i++)
	{
		handle->threshold[i] = acc_algorithm_calculate_cfar(
		    handle->lp_displacements, handle->data_length, CFAR_WINDOW_LENGTH, CFAR_HALF_GUARD_LENGTH, config->threshold_sensitivity, i);
	}

	/*
	 * Extend the CFAR threshold using extrapolation.
	 *
	 * The head is extended using linear extrapolation based on the first
	 * points of the original threshold.
	 *
	 * The tail is extended using the average of the last points of the
	 * original threshold.
	 */

	const float    head_slope_multiplier        = 2.0f;
	const uint16_t head_slope_calculation_width = 2U;
	const uint16_t tail_mean_calculation_width  = 10U;
	const uint16_t tail_mean_calculation_start  = handle->data_length - CFAR_MARGIN - tail_mean_calculation_width;

	const float    head_base_offset = handle->threshold[CFAR_MARGIN];
	const uint16_t tail_offset      = handle->data_length - CFAR_MARGIN;

	/*
	 * Calculate head.
	 */

	float head_mean_slope = 0.0f;

	for (uint16_t i = 0; i < head_slope_calculation_width; i++)
	{
		uint16_t pos     = CFAR_MARGIN + i;
		head_mean_slope += handle->threshold[pos + 1U] - handle->threshold[pos];
	}

	head_mean_slope /= (float)head_slope_calculation_width;
	head_mean_slope *= head_slope_multiplier;

	/*
	 * Calculate tail.
	 */

	float tail_mean = 0.0f;

	for (int16_t i = 0; i < tail_mean_calculation_width; i++)
	{
		tail_mean += handle->threshold[tail_mean_calculation_start + i];
	}

	tail_mean /= (float)tail_mean_calculation_width;

	/*
	 * Extend the threshold at both ends.
	 */

	for (uint16_t i = 0; i < CFAR_MARGIN; i++)
	{
		handle->threshold[i]               = ((float)i - CFAR_MARGIN) * head_mean_slope + head_base_offset;
		handle->threshold[tail_offset + i] = tail_mean;
	}
}

static void setup_rfft_bounds(acc_vibration_handle_t *handle, const acc_vibration_config_t *config)
{
	const uint16_t N = config->time_series_length;

	uint16_t shift  = 0;
	uint16_t length = 1U;

	if (N > 1U)
	{
		while (length < N)
		{
			length <<= 1;
			shift++;
		}
	}

	handle->padded_time_series_length = length;
	handle->rfft_length_shift         = shift;
}
