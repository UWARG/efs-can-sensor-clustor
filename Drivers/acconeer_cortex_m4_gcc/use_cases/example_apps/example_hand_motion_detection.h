// Copyright (c) Acconeer AB, 2024
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#ifndef EXAMPLE_HAND_MOTION_DETECTION_H_
#define EXAMPLE_HAND_MOTION_DETECTION_H_

#include <stdint.h>

#include "acc_config.h"
#include "acc_detector_presence.h"
#include "acc_processing.h"

/**
 * @brief hand motion handle
 */
typedef struct hand_motion_detection_handle hand_motion_detection_handle_t;

typedef enum
{
	/*! No preset, user needs to set each specific parameter themselves */
	HAND_MOTION_DETECTION_PRESET_NONE,
	/*! Sensor placed on a faucet to detect hands and turn on water */
	HAND_MOTION_DETECTION_PRESET_FAUCET,
} hand_motion_detection_preset_t;

typedef enum
{
	HAND_MOTION_DETECTION_APP_MODE_PRESENCE,
	HAND_MOTION_DETECTION_APP_MODE_HAND_MOTION,
} hand_motion_detection_app_mode_t;

typedef enum
{
	HAND_MOTION_DETECTION_STATE_NO_DETECTION,
	HAND_MOTION_DETECTION_STATE_DETECTION,
	HAND_MOTION_DETECTION_STATE_RETENTION,
} hand_motion_detection_detection_state_t;

/**
 * @brief Hand motion algo configuration
 */
typedef struct
{
	float    sensor_to_water_distance;
	float    water_jet_width;
	float    measurement_range_end;
	float    filter_time_const;
	float    threshold;
	float    detection_retention_duration;
	uint16_t hwaas;
	uint16_t sweeps_per_frame;
	float    sweep_rate;
	float    frame_rate;
} hand_motion_detection_algo_config_t;

/**
 * @brief Configuration for hand_motion
 */
typedef struct
{
	hand_motion_detection_algo_config_t algo_config;
	acc_detector_presence_config_t     *presence_config;
	float                               hand_detection_timeout;
	bool                                use_presence_detection;
} hand_motion_detection_config_t;

/**
 * @brief Hand motion algo result
 */
typedef struct
{
	hand_motion_detection_detection_state_t detection_state;
} hand_motion_detection_algo_result_t;

/**
 * @brief Result type
 */
typedef struct
{
	hand_motion_detection_app_mode_t    app_mode;
	bool                                presence_result_available;
	acc_detector_presence_result_t      presence_result;
	bool                                algo_result_available;
	hand_motion_detection_algo_result_t algo_result;
	acc_processing_result_t             proc_result;
} hand_motion_detection_result_t;

/**
 * @brief Create a hand motion configuration
 *
 * @return Hand motion configuration if successful, otherwise false
 */
hand_motion_detection_config_t *hand_motion_detection_config_create(void);

/**
 * @brief Destroy a hand motion configuration
 *
 * @param[in] config Configuration to destroy
 */
void hand_motion_detection_config_destroy(hand_motion_detection_config_t *config);

/**
 * @brief Set config specified by the preset
 *
 * @param[in] preset The preset used to set hand motion config
 * @param[out] config The hand motion config to be set
 */
void hand_motion_detection_set_config(hand_motion_detection_preset_t preset, hand_motion_detection_config_t *config);

/**
 * @brief Create a hand_motion handle
 *
 * The handle is used only for processing
 *
 * @param[in] hand_motion_detection_config hand motion configuration
 * @param[in] sensor_id Sensor id
 * @return A hand_motion handle, or NULL if creation failed
 */
hand_motion_detection_handle_t *hand_motion_detection_handle_create(const hand_motion_detection_config_t *hand_motion_detection_config,
                                                                    acc_sensor_id_t                       sensor_id);

/**
 * @brief Destroy a hand_motion handle
 *
 * @param[in] handle The hand_motion handle to destroy
 */
void hand_motion_detection_handle_destroy(hand_motion_detection_handle_t *handle);

/**
 * @brief Print configuration
 *
 * @param[in] handle Handle to print config for
 */
void hand_motion_detection_config_log(hand_motion_detection_handle_t *handle);

/**
 * @brief Get buffer size needed
 *
 * @param[in] handle The hand motion handle
 * @param[out] buffer_size Buffer size needed
 * @return True if successful, otherwise false
 */
bool hand_motion_detection_get_buffer_size(hand_motion_detection_handle_t *handle, uint32_t *buffer_size);

/**
 * @brief Prepare hand motion
 *
 * @param[in] handle The hand motion handle
 * @param[in] sensor Sensor to prepare
 * @param[in] cal_result Sensor calibration result
 * @param[in] buffer Data buffer
 * @param[in] buffer_size Size of data buffer
 * @param[in] force_prepare Force executing prepare
 * @return True if successful, otherwise false
 */
bool hand_motion_detection_prepare(hand_motion_detection_handle_t *handle,
                                   acc_sensor_t                   *sensor,
                                   const acc_cal_result_t         *cal_result,
                                   void                           *buffer,
                                   uint32_t                        buffer_size,
                                   bool                            force_prepare);

/**
 * @brief Process Sparse IQ data
 *
 * @param[in] handle The hand_motion handle
 * @param[in] buffer Data buffer
 * @param[out] hand_motion_detection_result Result processed by hand_motion
 */
void hand_motion_detection_process(hand_motion_detection_handle_t *handle,
                                   void                           *buffer,
                                   hand_motion_detection_result_t *hand_motion_detection_result);

#endif
