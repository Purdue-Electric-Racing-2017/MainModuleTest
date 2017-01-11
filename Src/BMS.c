/***************************************************************************
*
*     File Information
*
*     Name of File: car.c
*
*     Authors (Include Email):
*     	1. Hussain Khambata
*       2. Ben Ng				xbenng@gmail.com
*
*     File dependents: (header files, flow charts, referenced documentation)
*       1. BMS.h
*
*     File Description:
*     	Functions to control the physical car
*
***************************************************************************/
#include "BMS.h"

void processBMSFrame(CanRxMsgTypeDef rx)
{
	if (rx.StdId == 0x300)
	{

	}
}
