/***************************************************************************
*
*     File Information
*
*     Name of File: car.c
*
*     Authors (Include Email):
*       1. Ben Ng				xbenng@gmail.com
*
*     File dependents: (header files, flow charts, referenced documentation)
*       1. car.h
*
*     File Description:
*     	Functions to control the physical car
*
***************************************************************************/
#include "car.h"


void carSetBrakeLight(Brake_light_status_t status)
/***************************************************************************
*
*     Function Information
*
*     Name of Function: setBrakeLight
*
*     Programmer's Name: Ben Ng xbenng@gmail.com
*
*     Function Return Type: void
*
*     Parameters (list data type, name, and comment one per line):
*       1. Brake_light_status_t status, value to write to GPIO pin
*
*      Global Dependents:
*
*     Function Description:
*			turns brakelight on or off
***************************************************************************/
{
	HAL_GPIO_WritePin(BRAKE_LIGHT_PORT, BRAKE_LIGHT_PIN, status);
}


void carInit() {
	car.state = CAR_STATE_INIT;
	car.throttle = 0;
	car.brake = 0;
	car.phcan = &hcan1;
}

void ISR_StartButtonPressed() {
	if (1) {
		if (car.state == CAR_STATE_INIT)
		{
			if (car.brake >= BRAKE_PRESSED_THRESHOLD)
			car.state = CAR_STATE_READY2DRIVE;
		}
	}
}
