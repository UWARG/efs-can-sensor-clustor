// Copyright (c) Acconeer AB, 2024
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#ifndef EXAMPLE_WASTE_LEVEL_H_
#define EXAMPLE_WASTE_LEVEL_H_

#include <stdbool.h>
#include <stdint.h>

#include "acc_config.h"
#include "acc_processing.h"

/**
 * @brief Waste level handle
 */
typedef struct waste_level_handle waste_level_handle_t;

/**
 * @brief Configuration for waste_level
 */
typedef struct
{
	/** Minimum detection distance from sensor */
	float bin_start_m;

	/** Maximum detection distance from sensor */
	float bin_end_m;

	/** Threshold for which the standard deviation of the phase should be below */
	float threshold;

	/** Number of distance needed in sequence below threshold to be presumed as waste level */
	uint16_t distance_sequence_len;

	/** Length of the median filter used to stabilize the level result */
	uint16_t median_filter_len;
} waste_level_processing_config_t;

typedef struct
{
	/** Processing configuration */
	waste_level_processing_config_t processing_config;

	/** Sensor configuration */
	acc_config_t *sensor_config;
} waste_level_app_config_t;

/**
 * @brief Waste level preset
 */
typedef enum
{
	WASTE_LEVEL_PRESET_NONE = 0,
	WASTE_LEVEL_PRESET_PLASTIC_WASTE_BIN,
} waste_level_preset_t;

/**
 * @brief Result type
 */
typedef struct
{
	/** True if a level was found, false otherwise.
	 * Note: The other fields are valid if and only if level_found == true.
	 */
	bool level_found;

	/** The fill level in meters, measured from the bottom. */
	float level_m;

	/** The fill level in percent. */
	uint8_t level_percent;
} waste_level_result_t;

/**
 * @brief Create a waste level app config (includes an acc_config_t)
 */
waste_level_app_config_t *waste_level_app_config_create(void);

/**
 * @brief Destroy a waste level app config
 *
 * @param[in] app_config The config to destroy
 */
void waste_level_app_config_destroy(waste_level_app_config_t *app_config);

/**
 * @brief Apply a preset to an app config
 *
 * @param[in] preset The preset
 * @param[in] app_config The waste level app config
 */
void waste_level_app_config_set_preset(waste_level_preset_t preset, waste_level_app_config_t *app_config);

/**
 * @brief Create a waste level handle
 *
 * The handle is used only for processing
 *
 * @param[in] app_config Waste level app configuration
 * @return A waste_level handle, or NULL if creation failed
 */
waste_level_handle_t *waste_level_handle_create(const waste_level_app_config_t *app_config);

/**
 * @brief Destroy a waste level handle
 *
 * @param[in] handle The waste_level handle to destroy
 */
void waste_level_handle_destroy(waste_level_handle_t *handle);

/**
 * @brief Process Sparse IQ data
 *
 * @param[in] handle The waste level handle
 * @param[in] app_config The waste level app configuration handle was created with
 * @param[in] metadata The processing metadata
 * @param[in] frame The Sparse IQ frame
 * @param[out] waste_level_result Result processed by waste_level
 */
void waste_level_process(waste_level_handle_t            *handle,
                         const waste_level_app_config_t  *app_config,
                         const acc_processing_metadata_t *metadata,
                         const acc_int16_complex_t       *frame,
                         waste_level_result_t            *waste_level_result);

/**
 * @brief Log a waste level config
 *
 * @param[in] config The waste level config
 */
void waste_level_processing_config_log(const waste_level_processing_config_t *config);

#endif
