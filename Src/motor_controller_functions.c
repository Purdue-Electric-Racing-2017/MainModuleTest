/***************************************************************************
*
*     File Information
*
*     Name of File: main_module_tasks.c
*
*     Authors (Include Email):
*       1. Ben Ng				xbenng@gmail.com
*
*     File dependents: (header files, flow charts, referenced documentation)
*       1. motor_controller_functions.h
*       http://www.unitek-online.de/pdf/download/Antriebe-Drive/Servo-Digital/E-DS-CAN.pdf
*     	http://www.unitek-online.de/pdf/download/Antriebe-Drive/Servo-Digital/E-DS-NDrive.pdf
*
*
*     File Description:
*     	Functions to control the motor controller.

***************************************************************************/
#include "motor_controller_functions.h"


void cmdTorque(uint16_t torqueVal) {
	//example 5ay tod, BAMOCAR CAN MANUAL
	CanTxMsgTypeDef tx;
	tx.IDE = 		CAN_ID_STD;
	tx.StdId = 		ID_BAMOCAR_STATION_TX;
	tx.DLC = 		DLC_CMD_TORQUE;
	tx.Data[0] = 	REGID_CMD_TORQUE;
	tx.Data[1] =	(uint8_t) torqueVal;	//bytes 7-0
	tx.Data[2] =	(uint8_t) (torqueVal >> 8);		//bytes 11-8
}

void cmdTransmissionRequestPermenant (uint8_t regid, uint8_t retransmitTimeMS) {
	//example 10, BAMOCAR CAN MANUAL
	CanTxMsgTypeDef tx;
	tx.IDE = 		CAN_ID_STD;
	tx.StdId = 		ID_BAMOCAR_STATION_TX;
	tx.DLC = 		DLC_CMD_REQUEST_DATA;
	tx.Data[0] = 	REGID_CMD_REQUEST_DATA;
	tx.Data[1] =	regid;
	tx.Data[2] =	(uint8_t) retransmitTimeMS;
}

void cmdTransmissionRequestSingle(uint8_t regid) {
	//example 10, BAMOCAR CAN MANUAL
	CanTxMsgTypeDef tx;
	tx.IDE = 		CAN_ID_STD;
	tx.StdId = 		ID_BAMOCAR_STATION_TX;
	tx.DLC = 		DLC_CMD_REQUEST_DATA;
	tx.Data[0] = 	REGID_CMD_REQUEST_DATA;
	tx.Data[1] =	regid;
	tx.Data[2] =	RETRANSMISSION_SINGLE;
}

void cmdTransmissionAbortPermenant(uint8_t regid) {
	//example 10, BAMOCAR CAN MANUAL
	CanTxMsgTypeDef tx;
	tx.IDE = 		CAN_ID_STD;
	tx.StdId = 		ID_BAMOCAR_STATION_TX;
	tx.DLC = 		DLC_CMD_REQUEST_DATA;
	tx.Data[0] = 	REGID_CMD_REQUEST_DATA;
	tx.Data[1] =	regid;
	tx.Data[2] =	RETRANSMISSION_ABORT;
}

