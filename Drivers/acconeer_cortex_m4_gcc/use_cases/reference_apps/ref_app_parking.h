// Copyright (c) Acconeer AB, 2024
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#ifndef REF_APP_PARKING_H_
#define REF_APP_PARKING_H_


#include <stdbool.h>
#include <stdint.h>

#include "acc_definitions_a121.h"
#include "acc_definitions_common.h"


/**
 * @brief Parking application handle
 */
struct ref_app_parking_handle;

typedef struct ref_app_parking_handle ref_app_parking_handle_t;


/**
 * @brief Configuration presets - a set of configurations for a specific sensor placement
 */
typedef enum
{
	/*! No preset, user needs to set each specific parameter themselves */
	PARKING_PRESET_NONE,
	/*! Sensor placed on ground beneath car, obstruction detection enabled */
	PARKING_PRESET_GROUND,
	/*! Sensor placed on pole beside car, obstruction detection disabled */
	PARKING_PRESET_POLE,
} ref_app_parking_parking_preset_t;


/**
 * @brief Configuration for car detection
 */
typedef struct
{
	float                range_start_m;
	float                range_end_m;
	uint16_t             hwaas;
	acc_config_profile_t profile;
	uint8_t              subsweep_index;

	uint16_t queue_length_n;
	float    amplitude_threshold;
	float    weighted_distance_threshold_m;
	float    signature_similarity_threshold;
} ref_app_parking_car_config_t;


/**
 * @brief Configuration for obstruction detection
 */
typedef struct
{
	float                range_start_m;
	float                range_end_m;
	uint16_t             hwaas;
	acc_config_profile_t profile;
	uint8_t              subsweep_index;

	float threshold;
	float time_constant;
} ref_app_parking_obstruction_config_t;


/**
 * @brief Overall configuration for parking application
 */
typedef struct
{
	ref_app_parking_parking_preset_t preset;
	float                            frame_rate;
	bool                             frame_rate_app_driven;
	bool                             obstruction_detection_enabled;

	ref_app_parking_car_config_t         car_config;
	ref_app_parking_obstruction_config_t obstruction_config;
} ref_app_parking_config_t;


/**
 * @brief Set parking config according to specified preset
 *
 * @param[in] preset The preset used to set parking config
 * @param[out] parking_config The parking config to be set
 */
void ref_app_parking_set_config(ref_app_parking_parking_preset_t preset, ref_app_parking_config_t *parking_config);


/**
 * @brief Create parking handle
 *
 * The handle is used for both control and processing
 *
 * This function enables the sensor
 *
 * @param[in] parking_config The parking config used to create the parking handle
 * @param[in] sensor_id The sensor_id for the sensor to be used
 * @return A parking handle, or NULL if creation failed
 */
ref_app_parking_handle_t *ref_app_parking_handle_create(ref_app_parking_config_t *parking_config, acc_sensor_id_t sensor_id);


/**
 * @brief Destroy parking handle
 *
 * This function disables the sensor
 *
 * @param[in] handle The handle to destroy
 */
void ref_app_parking_handle_destroy(ref_app_parking_handle_t *handle);


/**
 * @brief Sensor calibration
 *
 * @param[in] handle The handle to calibrate sensor for
 * @return true if successful, false otherwise
 */
bool ref_app_parking_sensor_calibration(ref_app_parking_handle_t *handle);


/**
 * @brief Noise calibration
 *
 * @param[in] handle The handle to calibrate noise for
 * @return true if successful, false otherwise
 */
bool ref_app_parking_noise_calibration(ref_app_parking_handle_t *handle);


/**
 * @brief Obstruction calibration
 *
 * Note that the sensor must be free from obstruction when
 * calling this function.
 *
 * @param[in] handle The handle to calibrate obstruction for
 * @return true if successful, false otherwise
 */
bool ref_app_parking_obstruction_calibration(ref_app_parking_handle_t *handle);


/**
 * @brief Prepare sensor for measurement
 *
 * @param[in] handle The handle to prepare sensor for
 * @return true if successful, false otherwise
 */
bool ref_app_parking_sensor_prepare(ref_app_parking_handle_t *handle);


/**
 * @brief Perform a sensor measurement
 *
 * @param[in] handle The handle to do a sensor measurement for
 * @param[in] hibernate Hibernate sensor between measurements
 * @return true if successful, false otherwise
 */
bool ref_app_parking_measure(ref_app_parking_handle_t *handle, bool hibernate);


/**
 * @brief Handle indications from a measurement
 *
 * @param[in] handle The handle to do a sensor measurement for
 * @param[out] data_reliable Flag to determine if data from measurement is reliable.
 *                           Se log for more details if false
 * @return true if successful, false otherwise
 */
bool ref_app_parking_handle_indications(ref_app_parking_handle_t *handle, bool *data_reliable);


/**
 * @brief Do obstruction processing
 *
 * @param[in] handle The handle to do obstruction processing for
 * @param[out] obstruction_detected true if obstruction detected, false otherwise
 */
void ref_app_parking_obstruction_process(ref_app_parking_handle_t *handle, bool *obstruction_detected);


/**
 * @brief Do parking processing
 *
 * @param[in] handle The handle to do parking processing for
 * @param[out] car_detected true if car detected, false otherwise
 */
void ref_app_parking_process(ref_app_parking_handle_t *handle, bool *car_detected);


#endif
