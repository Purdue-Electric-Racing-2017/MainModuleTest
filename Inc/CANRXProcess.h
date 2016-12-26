/*
 * CANRXProcess.h
 *
 *  Created on: Dec 22, 2016
 *      Author: ben
 */

#ifndef CANRXPROCESS_H_
#define CANRXPROCESS_H_

//includes
#include "FreeRTOS.h"
#include "stm32f7xx_hal_can.h"
#include "main_module_tasks.h"
#include "motor_controller_functions.h"

//defines for reading data from RxCanMsgTypeDef
#define ID_PEDALBOX1							0x500




QueueHandle_t	q_rxcan;


#endif /* CANRXPROCESS_H_ */
