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
#include "CANProcess.h"

int taskProcessMotorControllerFrame() {
	CanRxMsgTypeDef rx;

	while (1) {
		if(xQueueReceive(car.q_mc_frame, &rx, 100))
		{
			//received a message from motor controller.
			//todo interpret frame
		}
	}
	return 0;
}

void mcCmdTorque(uint16_t torqueVal) {
	//example 5ay tod, BAMOCAR CAN MANUAL
	CanTxMsgTypeDef tx;
	tx.IDE = 		CAN_ID_STD;
	tx.StdId = 		ID_BAMOCAR_STATION_TX;
	tx.DLC = 		DLC_CMD_TORQUE;
	tx.Data[0] = 	REGID_CMD_TORQUE;
	tx.Data[1] =	(uint8_t) torqueVal;	//bytes 7-0
	tx.Data[2] =	(uint8_t) (torqueVal >> 8);		//bytes 11-8
	xQueueSendToBack(car.q_txcan, &tx, 100);
}

void mcCmdTransmissionRequestPermenant (uint8_t regid, uint8_t retransmitTimeMS) {
	//example 10, BAMOCAR CAN MANUAL
	CanTxMsgTypeDef tx;
	tx.IDE = 		CAN_ID_STD;
	tx.StdId = 		ID_BAMOCAR_STATION_TX;
	tx.DLC = 		DLC_CMD_REQUEST_DATA;
	tx.Data[0] = 	REGID_CMD_REQUEST_DATA;
	tx.Data[1] =	regid;
	tx.Data[2] =	(uint8_t) retransmitTimeMS;
	xQueueSendToBack(car.q_txcan, &tx, 100);

}

void mcCmdTransmissionRequestSingle(uint8_t regid) {
	//example 10, BAMOCAR CAN MANUAL
	CanTxMsgTypeDef tx;
	tx.IDE = 		CAN_ID_STD;
	tx.StdId = 		ID_BAMOCAR_STATION_TX;
	tx.DLC = 		DLC_CMD_REQUEST_DATA;
	tx.Data[0] = 	REGID_CMD_REQUEST_DATA;
	tx.Data[1] =	regid;
	tx.Data[2] =	RETRANSMISSION_SINGLE;
	xQueueSendToBack(car.q_txcan, &tx, 100);

}

void mcCmdTransmissionAbortPermenant(uint8_t regid) {
	//example 10, BAMOCAR CAN MANUAL
	CanTxMsgTypeDef tx;
	tx.IDE = 		CAN_ID_STD;
	tx.StdId = 		ID_BAMOCAR_STATION_TX;
	tx.DLC = 		DLC_CMD_REQUEST_DATA;
	tx.Data[0] = 	REGID_CMD_REQUEST_DATA;
	tx.Data[1] =	regid;
	tx.Data[2] =	RETRANSMISSION_ABORT;
	xQueueSendToBack(car.q_txcan, &tx, 100);

}

void disableMotor()
/***************************************************************************
*
*     Function Information
*
*     Name of Function: disableMotor
*
*     Programmer's Name: Ben Ng
*
*     Function Return Type: void
*
*     Parameters (list data type, name, and comment one per line):
*       1.Pedalbox_msg_t msg
			brake_level from pedalbox potentiometer
*			throttle_level from pedalbox potentiometer
*			APPS_Implausible flag
*			EOR flag
*		2.
*
*      Global Dependents:
*
*     Function Description:
*			sends 0 torque, then disables RUN, and REF
***************************************************************************/
{
	mcCmdTorque(0);
	HAL_GPIO_WritePin(FRG_RUN_PORT, FRG_RUN_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RFE_PORT, RFE_PIN, GPIO_PIN_RESET);

}

void enableMotorController() {
/***************************************************************************
*
*     Function Information
*
*     Name of Function: initMotorController
*
*     Programmer's Name: Ben Ng
*
*     Function Return Type: void
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*      Global Dependents:
*
*     Function Description:
*		Initializes the motor controller
*		chapter 10 bamocar ndrive manual
*
***************************************************************************/

	HAL_GPIO_WritePin(RFE_PORT, RFE_PIN, GPIO_PIN_SET);
	vTaskDelay(500 / portTICK_RATE_MS);  //wait 500ms, see BAMOCAR manual
	HAL_GPIO_WritePin(FRG_RUN_PORT, FRG_RUN_PIN, GPIO_PIN_SET);


}

