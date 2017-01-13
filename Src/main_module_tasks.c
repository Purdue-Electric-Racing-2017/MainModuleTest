/***************************************************************************
*
*     File Information
*
*     Name of File: main_module_tasks.c
*
*     Authors (Include Email):
*       1. Kai Strubel			kstrubel@purdue.edu
*       2. Ben Ng				xbenng@gmail.com
*
*     File dependents: (header files, flow charts, referenced documentation)
*       1. main_module_tasks.h
*
*     File Description: Takes inputs from the pedal box,
*       handles safety checks, and sets throttle level
*
***************************************************************************/
#include "main_module_tasks.h"


void taskPedalBoxMsgHandler() {
/***************************************************************************
*
*     Function Information
*
*     Name of Function: pedalBoxMessageHandler
*
*     Programmer's Name: 	Kai Strubel
*     						Ben Ng			xbenng@gmail.com
*
*     Function Return Type: int
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
*			Takes input from pedal box, runs safetly check, sets throttle
***************************************************************************/

	Pedalbox_msg_t pedalboxmsg;		//struct to store pedalbox msg

	while (1) {

		if(xQueueReceive(car.q_pedalboxmsg, &pedalboxmsg, 1000)){

			//get current time in ms
			uint32_t current_time_ms = xTaskGetTickCount() / portTICK_PERIOD_MS;;

			// update time stamp, indicates when a pedalbox message was last received
			car.pb_msg_rx_time = current_time_ms;


			/////////////PROCESS DATA///////////////
			Pedalbox_status_t  pedalbox_status = PEDALBOX_STATUS_NO_ERROR;  //local flag to be set if there is an error

			// EV 2.4.6: Encoder out of range
			if (pedalboxmsg.EOR == PEDALBOX_STATUS_ERROR) {
				pedalbox_status = PEDALBOX_STATUS_ERROR;
			} 
			
			//APPS Implausibility error handling, EV 2.3.5
			if (pedalboxmsg.APPS_Implausible == PEDALBOX_STATUS_ERROR) {
				//if error is persistent
				if (car.apps_imp_last_state == PEDALBOX_STATUS_ERROR)
				{
					//if time between first error and this error >= 100ms
					if (car.apps_imp_first_time_ms - current_time_ms >= 100)
					{
						pedalbox_status = PEDALBOX_STATUS_ERROR;
					}
				} else {  //else this is the first message to have an imp error
					//record the time
					car.apps_imp_first_time_ms = current_time_ms;
				}
			}
			//update last state variable so we know this state, next time we get an error
			car.apps_imp_last_state = pedalboxmsg.APPS_Implausible;


			//Brake
			//check if brake level is greater than the threshold level
			if (pedalboxmsg.brake_level_raw >= BRAKE_PRESSED_THRESHOLD * MAX_BRAKE_LEVEL) {
				//brake is presssed
				carSetBrakeLight(BRAKE_LIGHT_ON);  //turn on brake light


				//EV 2.5, check if the throttle level is greater than 25% while brakes are on
				if (pedalboxmsg.throttle_level_raw > APPS_BP_PLAUS_THRESHOLD * MAX_THROTTLE_LEVEL) {
					//set apps brake pedal plausibility error
					car.apps_bp_plaus = PEDALBOX_STATUS_ERROR;
				}
			} else {
				//brake is not pressed
				carSetBrakeLight(BRAKE_LIGHT_OFF);  //turn off brake light
			}


			if (car.apps_bp_plaus == PEDALBOX_STATUS_ERROR) {
				//EV 2.5.1, reset apps brake pedal plausibility error only if throttle level is less than the .05
				if(pedalboxmsg.throttle_level_raw <= APPS_BP_PLAUS_RESET_THRESHOLD * MAX_THROTTLE_LEVEL){
					car.apps_bp_plaus = PEDALBOX_STATUS_NO_ERROR;
				}
			}

			//set the car's throttle to the throttle just received
			if (pedalbox_status == PEDALBOX_STATUS_NO_ERROR)
			{
				//no errors, set throttle to value received from pedalbox
				car.throttle = pedalboxmsg.throttle_level_raw;
			} else {
				//if there were errors, set throttle = 0
				car.throttle = 0;
			}
		}
	}

	//if this task breaks from the loop kill it
	vTaskDelete(NULL);
}

//TODO Potential MC ping function
//TODO BMS functions

int mainModuleWatchdogTask() {
/***************************************************************************
*
*     Function Information
*
*     Name of Function: mainModuleTimeCheckIdle
*
*     Programmer's Name: Kai Strubel
*     					 Ben Ng			xbenng@gmail.com
*
*     Function Return Type: int
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*       
*      Global Dependents:
*	    1.bool launchControl
*		2.float MMPB_TIME time pedal box message handler function was last run 
*		3.float MMWM_TIME time wheel module handler function was last run
*		4.float torque
*		5.float currentTime
*
*     Function Description:
*		Checks if wheel module and pedal box are still communicating
*		
***************************************************************************/
	while (1) {
		/*
		//check how old the wheel module data is, if its too old, then turn off LC
		if (current_time_ms - MMWM_TIME > LC_THRESHOLD) {
			launchControl = 0;
			//error
		}*/
		vTaskDelay(500);
	}
}

int taskHeartbeat() {
/***************************************************************************
*.
*     Function Information
*
*     Name of Function: heartbeatIdle
*
*     Programmer's Name: Kai Strubel
*
*     Function Return Type: int
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*       
*      Global Dependents:
*
*     Function Description:
*		Heart beat to communicate that main module is alive
*		
***************************************************************************/
	// write to GPIO
	while (1) {
		HAL_GPIO_TogglePin(HEARTBEAT_PORT, HEARTBEAT_PIN);
		vTaskDelay(HEARTBEAT_PERIOD);
	}
}

void initRTOSObjects() {
/***************************************************************************
*
*     Function Information
*
*     Name of Function: startTasks
*
*     Programmer's Name: Ben Ng
*
*     Function Return Type: int
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*     Global Dependents:
*
*     Function Description:
*		all xTaskCreate calls
*		all xQueueCreate calls
*
***************************************************************************/

	/* Create Queues */
	car.q_rxcan = 			xQueueCreate(QUEUE_SIZE_RXCAN, sizeof(CanRxMsgTypeDef));
	car.q_txcan = 			xQueueCreate(QUEUE_SIZE_TXCAN, sizeof(CanTxMsgTypeDef));
	car.q_pedalboxmsg = 	xQueueCreate(QUEUE_SIZE_PEDALBOXMSG, sizeof(Pedalbox_msg_t));
	car.q_mc_frame = 		xQueueCreate(QUEUE_SIZE_MCFRAME, sizeof(CanRxMsgTypeDef));

	car.m_CAN =				xSemaphoreCreateMutex();

	/* Create Tasks */

	xTaskCreate(taskPedalBoxMsgHandler, "PedalBoxMsgHandler", 512, NULL, 1, NULL);
	xTaskCreate(taskCarMainRoutine, "CarMainRoutine", 512, NULL, 1, NULL);
	xTaskCreate(taskTXCAN, "TX CAN", 512, NULL, 1, NULL);
	xTaskCreate(taskRXCANProcess, "RX CAN Process", 512, NULL, 1, NULL);
	xTaskCreate(taskRXCAN, "RX CAN", 512, NULL, 1, NULL);




 }


void taskCarMainRoutine() {
	while (1)
	{
		//get current time in ms
		uint32_t current_time_ms = xTaskGetTickCount() / portTICK_PERIOD_MS;

		if (car.state == CAR_STATE_INIT)
		{

		}
		if (car.state == CAR_STATE_READY2DRIVE)
		{
			uint16_t torque_to_send = 0;

			//check if the age of the pedalbox message is greater than the timeout
			if (current_time_ms - car.pb_msg_rx_time > PEDALBOX_TIMEOUT) {
				torque_to_send = 0;
				//todo send a CAN message to dash?
			} else {
				//launch control
				/*if (lc_status == LC_ACTIVATED) {
					if(throttle > apps_max) {
						// apps - accelerator pedal position system
						// bse - brake system encoder
						apps_max = throttle;
						throttle *= scaleFactor;
					}
					else {
						if (apps_max - throttle > LC_THRESHOLD) {
							launchControl = false;
						}
						else {
							throttle *= scaleFactor;

						}
					}

				}*/

				torque_to_send = car.throttle;
			}
			mcCmdTorque(torque_to_send);  //command the MC to move the motor
		}
		if (car.state == CAR_STATE_ERROR)
		{
			mcCmdTorque(0);
		}

		//wait until
		vTaskDelay(TORQUE_SEND_PERIOD);

	}

}

