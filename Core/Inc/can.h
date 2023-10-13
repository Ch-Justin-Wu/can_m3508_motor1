/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can.h
 * @brief   This file contains all the function prototypes for
 *          the can.c file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */
  //extern CAN_HandleTypeDef CAN_Handler;

extern  CAN_TxHeaderTypeDef can_tx_message;
extern CAN_RxHeaderTypeDef can_rx_message;

extern uint8_t can_send_data[8];    //发�?�的数据�?
extern uint8_t can_receive_data[8]; //接受的数据帧
/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */
//  typedef struct
//  {
//    uint16_t ecd;
//    int16_t speed_rpm;
//    int16_t given_current;
//    uint8_t temperate;
//    int16_t last_ecd;
//  } motor_measure_t;

  void can_filter_init(void);
  void can_cmd_send(int motor1);
  // void can_cmd_receive();
  void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

