// Copyright (c) Acconeer AB, 2023-2024
// All rights reserved

#ifndef EXAMPLE_VIBRATION_H_
#define EXAMPLE_VIBRATION_H_

#include <complex.h>
#include <stdbool.h>
#include <stdint.h>

#include "acc_config.h"
#include "acc_processing.h"

/**
 * @brief Vibration presets
 */
typedef enum
{
	ACC_VIBRATION_PRESET_HIGH_FREQUENCY,
	ACC_VIBRATION_PRESET_LOW_FREQUENCY,
} acc_vibration_preset_t;

/**
 * @brief Specifies how displacement is reported
 */
typedef enum
{
	/** Reports displacement as amplitude */
	ACC_VIBRATION_REPORT_DISPLACEMENT_AS_AMPLITUDE,
	/** Reports displacement as peak-to-peak */
	ACC_VIBRATION_REPORT_DISPLACEMENT_AS_PEAK2PEAK,
} acc_vibration_reported_displacement_mode_t;

/**
 * @brief Vibration config container
 */
typedef struct
{
	/** The measured point */
	int32_t measured_point;

	/** Length of time series */
	uint16_t time_series_length;

	/** Filter coefficient of exponential filter */
	float lp_coeff;

	/** Threshold sensitivity */
	float threshold_sensitivity;

	/** Minimum amplitude for calculating vibration */
	float amplitude_threshold;

	/** Reported displacement mode: amplitude or peak2peak */
	acc_vibration_reported_displacement_mode_t reported_displacement_mode;

	/** Adds a loopback subsweep for phase correction to enhance low frequency detection */
	bool low_frequency_enhancement;

	/** The profile to use */
	acc_config_profile_t profile;

	/** The frame rate in Hz */
	float frame_rate;

	/** The sweep rate in Hz */
	float sweep_rate;

	/** The number of sweeps per frame */
	uint16_t sweeps_per_frame;

	/** The number of HWAAS */
	uint16_t hwaas;

	/** Wether to enable double buffering */
	bool double_buffering;

	/** Wether to enable continuous sweep mode */
	bool continuous_sweep_mode;

	/** The inter frame idle state */
	acc_config_idle_state_t inter_frame_idle_state;

	/** The inter sweep idle state */
	acc_config_idle_state_t inter_sweep_idle_state;
} acc_vibration_config_t;

/**
 * @brief Vibration processing result
 */
typedef struct
{
	float max_sweep_amplitude;
	float max_displacement;
	float max_displacement_freq;
	float time_series_std;
} acc_vibration_result_t;

/**
 * @brief Vibration processing handle
 */
typedef struct acc_vibration_handle acc_vibration_handle_t;

/**
 * @param[out] config The vibration config to set a preset for
 * @param[in] preset The preset
 */
void acc_vibration_preset_set(acc_vibration_config_t *config, acc_vibration_preset_t preset);

/**
 * @param[in] config The config to log
 */
void acc_vibration_config_log(const acc_vibration_config_t *config);

/**
 * @param[in] config A vibration app config to set default settings to.
 * @return A vibration processing handle
 */
acc_vibration_handle_t *acc_vibration_handle_create(const acc_vibration_config_t *config);

/**
 * @param[in] handle The handle
 * @return pointer to a sensor config corresponding to the vibration config
 *                 'handle' was created with.
 */
const acc_config_t *acc_vibration_handle_sensor_config_get(acc_vibration_handle_t *handle);

/**
 * @param[in] handle A handle to get the internal displacement history from
 * @param[out] num_elem The number of elements in the returned array
 * @return An array with displacements if arguments are non-NULL
 */
const float *acc_vibration_handle_displacement_history_get(acc_vibration_handle_t *handle, uint16_t *num_elem);

/**
 * @param[in] handle A handle to get the internal setting from
 * @param[out] continuous_data_acquisition Wether continuous data acquisition is enabled
 * @return True is arguments are non-NULL, false otherwise
 */
bool acc_vibration_handle_continuous_data_acquisition_get(acc_vibration_handle_t *handle, bool *continuous_data_acquisition);

/**
 * @param[in] handle The vibration handle to destroy
 */
void acc_vibration_handle_destroy(acc_vibration_handle_t *handle);

/**
 * @param[in] proc_result Input for vibration processing
 * @param[in] handle The vibration processing handle
 * @param[in] config The vibration app config
 * @param[out] result The vibration processing result
 */
void acc_vibration_process(acc_processing_result_t *proc_result,
                           acc_vibration_handle_t  *handle,
                           acc_vibration_config_t  *config,
                           acc_vibration_result_t  *result);

#endif
