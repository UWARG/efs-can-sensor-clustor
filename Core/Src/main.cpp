/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "canard_stm32_driver.h"
#include <time.h>
#include <stdio.h>
#include "dronecan_msgs.h"
#include "mlx90393_i2c.hpp"
#include "node_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CLOCK_MONOTONIC 1  // Define CLOCK_MONOTONIC for compatibility
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static CanardInstance canard;
static uint8_t memory_pool[1024];
static struct uavcan_protocol_NodeStatus node_status;
static struct uavcan_equipment_ahrs_MagneticFieldStrength2 mlx90393_node;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// The actual ISR, modify this to your needs
// Run HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) once to set up the ISR
MLX90393 mlx90393(&hi2c3);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	// Receiving
	CanardCANFrame rx_frame;

	const uint64_t timestamp = HAL_GetTick() * 1000ULL;
	const int16_t rx_res = canardSTM32Recieve(hcan, CAN_RX_FIFO0, &rx_frame);

	if (rx_res < 0) {
		printf("Receive error %d\n", rx_res);
	}
	else if (rx_res > 0)        // Success - process the frame
	{
		canardHandleRxFrame(&canard, &rx_frame, timestamp);
	}
}

// CAN Filter setup, modify this to your needs
// Run this once before calling HAL_CAN_Start()
void setupCANFilter(CAN_HandleTypeDef *hcan) {
    CAN_FilterTypeDef filter;
    filter.FilterBank = 0;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	filter.FilterIdHigh = 0;
	filter.FilterIdLow = 0;
	filter.FilterMaskIdHigh = 0;
	filter.FilterMaskIdLow = 0;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.FilterActivation = ENABLE;
	filter.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(hcan, &filter);
}

// Custom clock_gettime function for STM32
int clock_gettime(clockid_t clk_id, struct timespec *tp) {
    if (clk_id == CLOCK_MONOTONIC) {
        uint32_t millis = HAL_GetTick();  // Get the current tick in milliseconds
        tp->tv_sec = millis / 1000;       // Convert milliseconds to seconds
        tp->tv_nsec = (millis % 1000) * 1000000;  // Convert remaining milliseconds to nanoseconds
        return 0;  // Success
    }
    return -1;  // Unsupported clock ID
}

// NOTE: All canard handlers and senders are based on this reference: https://dronecan.github.io/Specification/7._List_of_standard_data_types/
// Alternatively, you can look at the corresponding generated header file in the dsdlc_generated folder

// Canard Handlers ( Many have code copied from libcanard esc_node example: https://github.com/dronecan/libcanard/blob/master/examples/ESCNode/esc_node.c )

void handle_NodeStatus(CanardInstance *ins, CanardRxTransfer *transfer) {
	struct uavcan_protocol_NodeStatus nodeStatus;

	if (uavcan_protocol_NodeStatus_decode(transfer, &nodeStatus)) {
		return;
	}

	printf("Node health: %ud Node Mode: %ud\n", nodeStatus.health, nodeStatus.mode);

	printf("Node Health ");

	switch (nodeStatus.health) {
	case UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK:
		printf("OK\n");
		break;
	case UAVCAN_PROTOCOL_NODESTATUS_HEALTH_WARNING:
		printf("WARNING\n");
		break;
	case UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR:
		printf("ERROR\n");
		break;
	case UAVCAN_PROTOCOL_NODESTATUS_HEALTH_CRITICAL:
		printf("CRITICAL\n");
		break;
	default:
		printf("UNKNOWN?\n");
		break;
	}

	printf("Node Mode ");

	switch(nodeStatus.mode) {
	case UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL:
		printf("OPERATIONAL\n");
		break;
	case UAVCAN_PROTOCOL_NODESTATUS_MODE_INITIALIZATION:
		printf("INITIALIZATION\n");
		break;
	case UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE:
		printf("MAINTENANCE\n");
		break;
	case UAVCAN_PROTOCOL_NODESTATUS_MODE_SOFTWARE_UPDATE:
		printf("SOFTWARE UPDATE\n");
		break;
	case UAVCAN_PROTOCOL_NODESTATUS_MODE_OFFLINE:
		printf("OFFLINE\n");
		break;
	default:
		printf("UNKNOWN?\n");
		break;
	}
}

void handle_NotifyState(CanardInstance *ins, CanardRxTransfer *transfer) {
	struct ardupilot_indication_NotifyState notifyState;

	if (ardupilot_indication_NotifyState_decode(transfer, &notifyState)) {
		return;
	}

	uint32_t nl = notifyState.vehicle_state & 0xFFFFFFFF;  // ignoring the last 32 bits for printing since the highest vehicle_state value right now is 23 even though they're allowed to be up to 64bit unsigned integer

	printf("Vehicle State: %lu ", nl);

	if (notifyState.aux_data.len > 0) {
		printf("Aux Data: 0x");

		for (int i = 0; i < notifyState.aux_data.len; i++) {
			printf("%02x", notifyState.aux_data.data[i]);
		}
	}

	printf("\n");

}

/*
  handle a GetNodeInfo request
*/
// TODO: All the data in here is temporary for testing. If actually need to send valid data, edit accordingly.
void handle_GetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer) {
	printf("GetNodeInfo request from %d\n", transfer->source_node_id);

	uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
	struct uavcan_protocol_GetNodeInfoResponse pkt;

	memset(&pkt, 0, sizeof(pkt));

	node_status.uptime_sec = HAL_GetTick() / 1000ULL;
	pkt.status = node_status;

	// fill in your major and minor firmware version
	pkt.software_version.major = 1;
	pkt.software_version.minor = 0;
	pkt.software_version.optional_field_flags = 0;
	pkt.software_version.vcs_commit = 0; // should put git hash in here

	// should fill in hardware version
	pkt.hardware_version.major = 1;
	pkt.hardware_version.minor = 0;

	// just setting all 16 bytes to 1 for testing
	getUniqueID(pkt.hardware_version.unique_id);

	strncpy((char*)pkt.name.data, "SERVONode", sizeof(pkt.name.data));
	pkt.name.len = strnlen((char*)pkt.name.data, sizeof(pkt.name.data));

	uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);

	canardRequestOrRespond(ins,
						   transfer->source_node_id,
						   UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
						   UAVCAN_PROTOCOL_GETNODEINFO_ID,
						   &transfer->transfer_id,
						   transfer->priority,
						   CanardResponse,
						   &buffer[0],
						   total_size);
}

// Canard Senders

/*
  send the 1Hz NodeStatus message. This is what allows a node to show
  up in the DroneCAN GUI tool and in the flight controller logs
 */
/////////////////////////////////Example/////////////////////////////////////
void send_NodeStatus(void) {
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];

    node_status.uptime_sec = HAL_GetTick() / 1000UL;
    node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status.sub_mode = 0;

    // put whatever you like in here for display in GUI
    node_status.vendor_specific_status_code = 1234;

    uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status, buffer);

    // we need a static variable for the transfer ID. This is
    // incremeneted on each transfer, allowing for detection of packet
    // loss
    static uint8_t transfer_id;

    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}
///////////////////////////////////////////////////////////////////////////////

//Magnetometer sender
void send_MLX90393(void) {
    uint8_t buffer[UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_MAX_SIZE];

    // put whatever you like in here for display in GUI
    mlx90393_node.sensor_id = 1;
    //1 uT = 0.01 Gauss, transfer in Gauss
    mlx90393_node.magnetic_field_ga[0] = (float)mlx90393.get_x_data() / 100.0;
    mlx90393_node.magnetic_field_ga[1] = (float)mlx90393.get_y_data() / 100.0;
    mlx90393_node.magnetic_field_ga[2] = (float)mlx90393.get_z_data() / 100.0;
    mlx90393_node.magnetic_field_covariance.len = 9;
    //Covariance matrix with variance terms only, refer to 13.1 for noise standard deviation
    //Settings: OSR = 3, digital filter = 5, conversion time = 52.92ms
    //Standard deviation for XY-axis error = 5 mGauss, Z-axis = 7 mGauss
    //Covariance is 0?
    mlx90393_node.magnetic_field_covariance.data[0] = 2.5E-5;
    mlx90393_node.magnetic_field_covariance.data[1] = 0.0;
    mlx90393_node.magnetic_field_covariance.data[2] = 0.0;
    mlx90393_node.magnetic_field_covariance.data[3] = 0.0;
    mlx90393_node.magnetic_field_covariance.data[4] = 2.5E-5;
    mlx90393_node.magnetic_field_covariance.data[5] = 0.0;
    mlx90393_node.magnetic_field_covariance.data[6] = 0.0;
    mlx90393_node.magnetic_field_covariance.data[7] = 0.0;
    mlx90393_node.magnetic_field_covariance.data[8] = 4.9E-5;
    uint32_t len = uavcan_equipment_ahrs_MagneticFieldStrength2_encode(&mlx90393_node, buffer);

    // we need a static variable for the transfer ID. This is
    // incremeneted on each transfer, allowing for detection of packet
    // loss
    static uint8_t transfer_id;

    canardBroadcast(&canard,
    				UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_SIGNATURE,
					UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

// Canard Util

bool shouldAcceptTransfer(const CanardInstance *ins,
                                 uint64_t *out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
	if (transfer_type == CanardTransferTypeRequest) {
	// check if we want to handle a specific service request
		switch (data_type_id) {
		case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
			*out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
			return true;
		}
		}
	}
	if (transfer_type == CanardTransferTypeResponse) {
		// check if we want to handle a specific service request
		switch (data_type_id) {
		}
	}
	if (transfer_type == CanardTransferTypeBroadcast) {
		// see if we want to handle a specific broadcast packet
		switch (data_type_id) {
		case UAVCAN_PROTOCOL_NODESTATUS_ID: {
			*out_data_type_signature = UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE;
			return true;
		}
		case ARDUPILOT_INDICATION_NOTIFYSTATE_ID: {
			*out_data_type_signature = ARDUPILOT_INDICATION_NOTIFYSTATE_SIGNATURE;
			return true;
		}
		}
	}
	// we don't want any other messages
	return false;
}

void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer) {
	// switch on data type ID to pass to the right handler function
//	printf("Transfer type: %du, Transfer ID: %du \n", transfer->transfer_type, transfer->data_type_id);
//	printf("0x");
//		for (int i = 0; i < transfer->payload_len; i++) {
//			printf("%02x", transfer->payload_head[i]);
//		}
//
//		printf("\n");
	if (transfer->transfer_type == CanardTransferTypeRequest) {
		// check if we want to handle a specific service request
		switch (transfer->data_type_id) {
		case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
			handle_GetNodeInfo(ins, transfer);
			break;
		}
		}
	}
	if (transfer->transfer_type == CanardTransferTypeResponse) {
		switch (transfer->data_type_id) {
		}
	}
	if (transfer->transfer_type == CanardTransferTypeBroadcast) {
		// check if we want to handle a specific broadcast message
		switch (transfer->data_type_id) {
		case UAVCAN_PROTOCOL_NODESTATUS_ID: {
			handle_NodeStatus(ins, transfer);
			break;
		}
		case ARDUPILOT_INDICATION_NOTIFYSTATE_ID: {
			handle_NotifyState(ins, transfer);
			break;
		}
		}
	}
}


// Processes the canard Tx queue and attempts to transmit the messages
// Call this function very often to check if there are any Tx to process
// Calling it once every cycle of the while(1) loop is not a bad idea
void processCanardTxQueue(CAN_HandleTypeDef *hcan) {
	// Transmitting

	for (const CanardCANFrame *tx_frame ; (tx_frame = canardPeekTxQueue(&canard)) != NULL;) {
		const int16_t tx_res = canardSTM32Transmit(hcan, tx_frame);

		if (tx_res <= 0) {
			printf("Transmit error %d\n", tx_res);
		} else if (tx_res > 0) {
			printf("Successfully transmitted message\n");
		}

		// Pop canardTxQueue either way
		canardPopTxQueue(&canard);
	}
}

/*
  This function is called at 1 Hz rate from the main loop.
*/
void process1HzTasks(uint64_t timestamp_usec) {
    /*
      Purge transfers that are no longer transmitted. This can free up some memory
    */
    canardCleanupStaleTransfers(&canard, timestamp_usec);

    /*
      Transmit the node status message
    */
    send_NodeStatus(); //For test
    send_MLX90393();
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C3_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  setupCANFilter(&hcan1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  canardInit(&canard,
			  memory_pool,
			  sizeof(memory_pool),
			  onTransferReceived,
			  shouldAcceptTransfer,
			  NULL);

  // Could use DNA (Dynamic Node Allocation) by following example in esc_node.c but that requires a lot of setup and I'm not too sure of what advantage it brings
  // Instead, set a different NODE_ID for each device on the CAN bus by configuring node_settings
  if (NODE_ID > 0) {
	  canardSetLocalNodeID(&canard, NODE_ID);
  } else {
	  printf("Node ID is 0, this node is anonymous and can't transmit most messaged. Please update this in node_settings.h\n");
  }

  uint64_t next_1hz_service_at = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(mlx90393.read_wait_flag()){
		  mlx90393.i2c_SM();
		  mlx90393.i2c_RM();
	  }
	  if(mlx90393.read_update_flag()){
		  mlx90393.set_wait_flag(false);
		  mlx90393.decode();
		  mlx90393.convert();
	  }else{
		  continue;
	  }
	const uint64_t ts = HAL_GetTick();

	if (ts >= next_1hz_service_at){
	  next_1hz_service_at += 1000ULL;
	  process1HzTasks(ts);
	}

	processCanardTxQueue(&hcan1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(hi2c->Instance == I2C3){
		if(mlx90393.get_rm_flag()){
			mlx90393.set_update_flag(true);
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
