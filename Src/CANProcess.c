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
#include <CANProcess.h>
#include <main_module_tasks.h>

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
void ISR_RXCAN()
{
	xQueueSendFromISR(car.q_rxcan, (car.phcan->pRxMsg), NULL);
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
//void CANFilterConfig()
//{
//	  CAN_FilterConfTypeDef filter_conf;  //filter config object
//
//	  //see filter configuration section of manifesto for filter numbering
//	  //also refer to "CAN Messages" in team documentation for addresses
//	  filter_conf.FilterIdHigh = 		0x7ff << 5; // 2
//	  filter_conf.FilterIdLow = 		ID_PEDALBOX1 << 5; // 0, "pedalbox1" throttle values and pedalbox errors
//	  filter_conf.FilterMaskIdHigh = 	0 << 5; //3
//	  filter_conf.FilterMaskIdLow = 	0 << 5;	//1, pedal errors
//	  filter_conf.FilterFIFOAssignment = CAN_FilterFIFO0;  //use interrupt RX0
//	  filter_conf.FilterNumber = 0;
//	  filter_conf.FilterMode = CAN_FILTERMODE_IDMASK;  //four different filters, no masks
//	  filter_conf.FilterScale = CAN_FILTERSCALE_16BIT; //16 bit filters
//	  filter_conf.FilterActivation = ENABLE;
//	  HAL_CAN_ConfigFilter(car.phcan, &filter_conf); //add filter
//}
void CANFilterConfig()
{
	  CAN_FilterConfTypeDef FilterConf;
	  FilterConf.FilterIdHigh = 	0b0000000001000000; // 2
	  FilterConf.FilterIdLow = 		0b0000000000100000; // 0
	  FilterConf.FilterMaskIdHigh = 0x7ff << 5; //3
	  FilterConf.FilterMaskIdLow = 	0x113 << 5;	//1
	  FilterConf.FilterFIFOAssignment = CAN_FilterFIFO0;
	  FilterConf.FilterNumber = 0;
	  FilterConf.FilterMode = CAN_FILTERMODE_IDLIST;
	  FilterConf.FilterScale = CAN_FILTERSCALE_16BIT;
	  FilterConf.FilterActivation = ENABLE;
	  HAL_CAN_ConfigFilter(&hcan1, &FilterConf);
	  FilterConf.FilterIdHigh = 	0b0000000010000000;
	  FilterConf.FilterIdLow = 		0b0000000001100000;
	  FilterConf.FilterMaskIdHigh = 0b1111111111111111;
	  FilterConf.FilterMaskIdLow = 	0b1111111111111111;
	  FilterConf.FilterFIFOAssignment = CAN_FilterFIFO0;
	  FilterConf.FilterNumber = 1;
	  FilterConf.FilterMode = CAN_FILTERMODE_IDMASK;
	  FilterConf.FilterScale = CAN_FILTERSCALE_16BIT;
	  FilterConf.FilterActivation = ENABLE;
	  HAL_CAN_ConfigFilter(&hcan1, &FilterConf);
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: taskTXCAN
*
*     Programmer's Name: Ben Ng, xbenng@gmail.com
*
*     Function Return Type: none
*
*     Parameters (list data type, name, and comment one per line):
*
*      Global Dependents:
*	   1.
*
*     Function Description:
*     	Task function to send CAN messages using the CAN peripheral
*
***************************************************************************/
void taskTXCAN()
{
	for (;;)
	{
		CanTxMsgTypeDef tx;
		//check if this task is triggered
		if (xQueuePeek(car.q_txcan, &tx, portMAX_DELAY) == pdTRUE)
		{
			//check if CAN mutex is available
			if (xSemaphoreTake(car.m_CAN, 50) == pdTRUE)
			{
				//HAL_CAN_StateTypeDef state = HAL_CAN_GetState(car.phcan);
				//if (state != HAL_CAN_STATE_ERROR)
				{
					xQueueReceive(car.q_txcan, &tx, portMAX_DELAY);  //actually take item out of queue
					car.phcan->pTxMsg = &tx;
					HAL_CAN_Transmit_IT(car.phcan);
				}
				xSemaphoreGive(car.m_CAN);  //release CAN mutex
			}

		}
	}
}

void taskRXCAN()
{
	for (;;)
	{

		//check if CAN mutex is available
		if (xSemaphoreTake(car.m_CAN, 10) == pdTRUE )
		{
			HAL_CAN_Receive_IT(&hcan1, 0);
			xSemaphoreGive(car.m_CAN);  //release CAN mutex
		}
		vTaskDelay(10);
	}
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: taskRXCAN
*
*     Programmer's Name: Ben Ng, xbenng@gmail.com
*
*     Function Return Type: none
*
*     Parameters (list data type, name, and comment one per line):
*
*      Global Dependents:
*	   1.
*
*     Function Description:
*     	Task function to process received CAN Messages.
*     	CanRxMsgTypeDef are sent here to the q_rxcan queue to be processed
*     	from the CAN RX interrupt handler.
*     	The data is process and handled according to what kind of message is received
*
***************************************************************************/
void taskRXCANProcess()
{
	CanRxMsgTypeDef rx;  //CanRxMsgTypeDef to be received on the queue
	for (;;)
	{

		//if there is a CanRxMsgTypeDef in the queue, pop it, and store in rx
		if (xQueueReceive(car.q_rxcan, &rx, portMAX_DELAY) == pdTRUE)
		{
			//A CAN message has been recieved

			//check what kind of message we received
			if (rx.StdId == ID_PEDALBOX1) //if pedalbox1 message
			{
				processPedalboxFrame(&rx); //todo check if copies properly

			} else if (rx.StdId == ID_BAMOCAR_STATION_RX) { //if bamocar message
				//forward frame to mc frame queue
				xQueueSend(car.q_mc_frame, &rx, 100);
			}

		}
	}
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: processPedalboxFrame
*
*     Programmer's Name: Ben Ng, xbenng@gmail.com
*
*     Function Return Type: none
*
*     Parameters (list data type, name, and comment one per line):
*		1. CanRxMsgTypeDef* rx, CAN frame to be converted into a pedalbox message
*      Global Dependents:
*	    1.
*
*     Function Description:
*     	Converts CAN frame into a pedalbox message and sends it to pedalboxmsg handler
*
***************************************************************************/
void processPedalboxFrame(CanRxMsgTypeDef* rx)
{
	if (car.pb_mode == PEDALBOX_MODE_DIGITAL)	//
	{
		Pedalbox_msg_t pedalboxmsg;

		///////////SCRUB DATA the from the CAN frame//////////////
		//mask then shift the throttle value data
		uint8_t throttle_7_0 	=
				rx->Data[PEDALBOX1_THROT_7_0_BYTE]  >> PEDALBOX1_THROT_7_0_OFFSET;  //Throttle Value (7:0) [7:0]
		uint8_t throttle_11_8	=
				(rx->Data[PEDALBOX1_THROT_11_8_BYTE] & PEDALBOX1_THROT_11_8_MASK) >> PEDALBOX1_THROT_11_8_OFFSET;  //Throttle Value (11:8) [3:0]
		//mask then shift the brake value data
		uint8_t brake_7_0 	=
				rx->Data[PEDALBOX1_BRAKE_7_0_BYTE]  >> PEDALBOX1_THROT_7_0_OFFSET;  //Throttle Value (7:0) [7:0]
		uint8_t brake_11_8	=
				(rx->Data[PEDALBOX1_BRAKE_11_8_BYTE] & PEDALBOX1_BRAKE_11_8_MASK) >> PEDALBOX1_BRAKE_11_8_OFFSET;  //Throttle Value (11:8) [3:0]

		//mask then shift the error flags
		pedalboxmsg.APPS_Implausible =
				(rx->Data[PEDALBOX1_IMP_BYTE] & PEDALBOX1_IMP_MASK) >> PEDALBOX1_IMP_OFFSET;  //Throttle Value (7:0) [7:0] and mask 1 bit
		pedalboxmsg.EOR =
				(rx->Data[PEDALBOX1_EOR_BYTE] & PEDALBOX1_EOR_MASK) >> PEDALBOX1_EOR_OFFSET;  //Throttle Value (7:0) [7:0] and mask 1 bit


		//build the data
		pedalboxmsg.throttle_level_raw = 0;
		pedalboxmsg.throttle_level_raw |= throttle_7_0 << 0;
		pedalboxmsg.throttle_level_raw |= throttle_11_8 << 8;
		pedalboxmsg.brake_level_raw = 0;
		pedalboxmsg.brake_level_raw |= brake_7_0 << 0;
		pedalboxmsg.brake_level_raw |= brake_11_8 << 8;


		//send to pedalboxmsg to queue
		xQueueSendToBack(car.q_pedalboxmsg, &pedalboxmsg, 100);
	}
}


