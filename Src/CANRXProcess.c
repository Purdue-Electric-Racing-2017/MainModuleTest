/***************************************************************************
*
*     File Information
*
*     Name of File: CANRXProcess.c
*
*     Authors (Include Email):
*       1. Ben Ng,       xbenng@gmail.com
*
*     File dependents: (header files, flow charts, referenced documentation)
*       1. FreeRTOS.h
* 		2. stm32f7xx_hal_can.h
* 		3. CANRXProcess.h
*
*     File Description: Used for interpreting incoming CAN messages on
*						main module
*
***************************************************************************/
#include "CANRXProcess.h"

/***************************************************************************
*
*     Function Information
*
*     Name of Function: RXCAN_ISR
*
*     Programmer's Name: Ben Ng, xbenng@gmail.com
*
*     Function Return Type:
*
*     Parameters (list data type, name, and comment one per line):
*
*     Global Dependents:
*	  1. QueueHandle_t	q_pedalbox_msg;
*
*     Function Description:
*			To be called by CAN1_RX0_IRQHandler in order to queue
*			received CAN messages to be processed by RXCANProcessTask
*
***************************************************************************/
void RXCAN_ISR(CAN_HandleTypeDef *hcan) {
	xQueueSendFromISR(q_rxcan, hcan->pRxMsg, NULL);
}


/***************************************************************************
*
*     Function Information
*
*     Name of Function: RXCANProcessTask
*
*     Programmer's Name: Ben Ng, xbenng@gmail.com
*
*     Function Return Type:
*
*     Parameters (list data type, name, and comment one per line):
*       1. CAN_HandleTypeDef *hcan, hcan structure address to add filter to
*
*     Global Dependents:
*	    1.
*
*     Function Description: Filter Configuration.
*
***************************************************************************/
void CANFilterConfig(CAN_HandleTypeDef *hcan)
{
	  CAN_FilterConfTypeDef filter_conf;  //filter config object

	  //see filter configuration section of manifesto for filter numbering
	  //also refer to "CAN Messages" in team documentation for addresses
	  filter_conf.FilterIdHigh = 		0 << 5; // 2
	  filter_conf.FilterIdLow = 		ID_PEDALBOX1 << 5; // 0, "pedalbox1" throttle values and pedalbox errors
	  filter_conf.FilterMaskIdHigh = 	0 << 5; //3
	  filter_conf.FilterMaskIdLow = 	0 << 5;	//1, pedal errors
	  filter_conf.FilterFIFOAssignment = CAN_FilterFIFO0;  //use interrupt RX0
	  filter_conf.FilterNumber = 0;
	  filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;  //four different filters, no masks
	  filter_conf.FilterScale = CAN_FILTERSCALE_16BIT; //16 bit filters
	  filter_conf.FilterActivation = ENABLE;
	  HAL_CAN_ConfigFilter(hcan, &filter_conf); //add filter
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: RXCANProcessTask
*
*     Programmer's Name: Ben Ng, xbenng@gmail.com
*
*     Function Return Type: none
*
*     Parameters (list data type, name, and comment one per line):
*       1. void *q_rxcan_process, pointer to the queue which receives the CanRxMsgTypeDef types
*
*      Global Dependents:
*	   1.
*
*     Function Description:
*     	Task function to process received CAN Messages.
*     	CanRxMsgTypeDef are sent from the CAN RX interrupt handler
*     	to this the q_rxcan_process queue to be processed.
*
***************************************************************************/
void RXCANProcessTask()
{
	CanRxMsgTypeDef rx;  //CanRxMsgTypeDef to be received on the queue
	for (;;)
	{

		//if there is a CanRxMsgTypeDef in the queue, pop it, and store in rx

		if (xQueueReceive(q_rxcan, &rx, portMAX_DELAY) == pdTRUE)
		{
			//A CAN message has been recieved
			//check what kind of message we received

			if (rx.StdId == ID_PEDALBOX1) //if pedalbox1 message
			{
				//send to pedalbox frame queue
				xQueueSend(q_pedalbox_frame, rx, 100);

			} else if (rx.StdId == ID_BAMOCAR_STATION_RX) {
				//more messages....
				xQueueSend(q_mc_frame, rx, 100);
			}

		}
	}

}
