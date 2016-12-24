/*
 * Motor_Controller.c
 *
 *  Created on: Dec 21, 2016
 *      Author: Josh Shao
 */
#include "Motor_Controller.h"

CanTxMsgTypeDef torque_ref (int data)
/***************************************************************************
*
*     Function Information
*
*     Name of Function: torque_ref
*
*     Programmer's Name: Josh Shao
*
*     Function Return Type: CanTxMsgTypeDef
*
*     Parameters (list data type, name, and comment one per line):
*		1. int, data, data to be filled in can frame
*
*     Global Dependents:
*		1.
*
*     Function Description: create frame for motor controller torque reference
*
***************************************************************************/
{
	CanTxMsgTypeDef Msg;
	Msg.IDE = CAN_ID_STD;  //Standard CAN frame, NOT EXTENDED
	Msg.RTR = CAN_RTR_DATA;
	Msg.DLC = 3;

	//Station address for receiving
	Msg.StdId = BAMOCAR_STATION_ID;

	//Fill in data
	Msg.Data[0] = 0x90;

	//TODO Need to resize data into char
	Msg.Data[1] = data;
	Msg.Data[2] = data;

	return Msg;
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: torque_ref
*
*     Programmer's Name: Josh Shao
*
*     Function Return Type: CanTxMsgTypeDef
*
*     Parameters (list data type, name, and comment one per line):
*		1. int, data, data to be filled in can frame
*
*     Global Dependents:
*		1.
*
*     Function Description: create frame for motor controller inhibit control
*
***************************************************************************/
CanTxMsgTypeDef inhibit (int data)
{
	CanTxMsgTypeDef Msg;
	Msg.IDE = CAN_ID_STD;  //Standard CAN frame, NOT EXTENDED
	Msg.RTR = CAN_RTR_DATA;
	Msg.DLC = 3;

	//Station address for receiving
	Msg.StdId = 0x201;

	//Fill in data
	Msg.Data[0] = 0x51;

	//TODO Need to resize data into char
	Msg.Data[1] = data;
	Msg.Data[2] = data;

	return Msg;
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: torque_ref
*
*     Programmer's Name: Josh Shao
*
*     Function Return Type: CanTxMsgTypeDef
*
*     Parameters (list data type, name, and comment one per line):
*		1. int, data, data to be filled in can frame
*
*     Global Dependents:
*		1.
*
*     Function Description: create frame for motor controller station address change
*
***************************************************************************/
CanTxMsgTypeDef station_address (int data)
{
	CanTxMsgTypeDef Msg;
	Msg.IDE = CAN_ID_STD;  //Standard CAN frame, NOT EXTENDED
	Msg.RTR = CAN_RTR_DATA;
	Msg.DLC = 3;

	//Station address for receiving
	Msg.StdId = 0x201;

	//Fill in data
	Msg.Data[0] = 0x68;

	//TODO Need to resize data into char
	Msg.Data[1] = data;
	Msg.Data[2] = data;

	return Msg;
}



