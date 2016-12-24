/*
 * Motor_Controller.h
 *
 *  Created on: Dec 21, 2016
 *      Author: Josh Shao
 */

#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#include "stm32f7xx_hal.h"


CanTxMsgTypeDef torque_ref (int data);
CanTxMsgTypeDef inhibit (int data);
CanTxMsgTypeDef station_address (int data);

#define BAMOCAR_STATION_ID			0x201;


#endif /* MOTOR_CONTROLLER_H_ */
