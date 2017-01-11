/*
 * CANRXProcess.h
 *
 *  Created on: Dec 22, 2016
 *      Author: ben
 */

#ifndef CANPROCESS_H_
#define CANPROCESS_H_

//includes
#include "main_module_tasks.h"
#include "motor_controller_functions.h"
#include "car.h"

//defines for reading data from RxCanMsgTypeDef
#define ID_PEDALBOX1							0x100
#define ID_BAMOCAR_STATION_TX					0x201	//message recieved by MC
#define ID_BAMOCAR_STATION_RX					0x181	//message sent by MC
#define ID_BMS_PACK_VOLTAGE						0x300

//pedalbox defines //todo not sure if better to send whole frame or just pbmsg.
#define PEDALBOX1_FILTER 						0	//filter number corresponding to the PEDALBOX1 message
#define PEDALBOX1_THROT_7_0_BYTE				0
#define PEDALBOX1_THROT_7_0_OFFSET				0
#define PEDALBOX1_THROT_7_0_MASK				0b11111111
#define PEDALBOX1_THROT_11_8_BYTE				1
#define PEDALBOX1_THROT_11_8_OFFSET				4
#define PEDALBOX1_THROT_11_8_MASK				0b11110000
#define PEDALBOX1_BRAKE_7_0_BYTE				2
#define PEDALBOX1_BRAKE_7_0_OFFSET				0
#define PEDALBOX1_BRAKE_7_0_MASK				0b11111111
#define PEDALBOX1_BRAKE_11_8_BYTE				1
#define PEDALBOX1_BRAKE_11_8_OFFSET				0
#define PEDALBOX1_BRAKE_11_8_MASK				0b00001111
#define PEDALBOX1_EOR_BYTE						3
#define PEDALBOX1_EOR_OFFSET					0
#define PEDALBOX1_EOR_MASK						0b00000001
#define PEDALBOX1_IMP_BYTE						3
#define PEDALBOX1_IMP_OFFSET					1
#define PEDALBOX1_IMP_MASK						0b00000010

void ISR_RXCAN();
void CANFilterConfig();
void taskRXCAN();
void taskTXCAN();

void processPedalboxFrame(CanRxMsgTypeDef * rx);

#endif /* CANPROCESS_H_ */
