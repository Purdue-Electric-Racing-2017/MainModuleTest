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
#include "stm32f7xx_hal.h"
#include "cmsis_os.h"
#include "car.h"
#include "CANProcess.h"
#include "main_module_tasks.h"
#include "BMS.h"
#include "WheelModule.h"
#include <math.h>

void taskLaunchControl() {
	while (1) {
		vTaskDelay(LAUNCH_CONTROL_INTERVAL_MS);
	}
}


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

			//check if calibration values should be updated
			if (car.calibrate_flag == CALIBRATE_THROTTLE_MIN) {
				car.throttle1_min = pedalboxmsg.throttle1_raw;
				car.throttle2_min = pedalboxmsg.throttle2_raw;
			} else if (car.calibrate_flag == CALIBRATE_THROTTLE_MAX) {
				car.throttle1_max = pedalboxmsg.throttle1_raw;
				car.throttle2_max = pedalboxmsg.throttle2_raw;
			} else 	if (car.calibrate_flag == CALIBRATE_BRAKE_MIN) {
				car.brake1_min = pedalboxmsg.brake1_raw;
				car.brake2_min = pedalboxmsg.brake2_raw;
			} else if (car.calibrate_flag == CALIBRATE_BRAKE_MAX) {
				car.brake1_max = pedalboxmsg.brake1_raw;
				car.brake2_max = pedalboxmsg.brake2_raw;
			}

			/////////////PROCESS DATA///////////////
			Pedalbox_status_t  	pedalbox_status = PEDALBOX_STATUS_NO_ERROR;  //local flag to be set if there is an error
			float				throttle1_cal = (pedalboxmsg.throttle1_raw - car.throttle1_min) / (car.throttle1_max - car.throttle1_min);  //value 0-1, throttle 1 calibrated between min and max
			float				throttle2_cal = (pedalboxmsg.throttle2_raw - car.throttle2_min) / (car.throttle2_max - car.throttle2_min);;  //value 0-1, throttle 2 calibrated between min and max
			float				brake1_cal	  = (pedalboxmsg.brake1_raw - car.brake1_min) / (car.brake1_max - car.brake1_min);  //value 0-1, brake 1 calibrated between min and max
			float				brake2_cal	  = (pedalboxmsg.brake2_raw - car.brake2_min) / (car.brake2_max - car.brake2_min);  //value 0-1, brake 2 calibrated between min and max
			float				throttle_avg  = (throttle1_cal + throttle2_cal) / 2;
			float				brake_avg     = (brake1_cal + brake2_cal) / 2;



			// EV 2.4.6: Encoder out of range
			if (pedalboxmsg.throttle1_raw >	car.throttle1_max ||					//legacy: pedalboxmsg.EOR == PEDALBOX_STATUS_ERROR
				pedalboxmsg.throttle2_raw >	car.throttle2_max)
			{
				pedalbox_status = PEDALBOX_STATUS_ERROR;
			} 
			
			//APPS Implausibility error handling, EV 2.3.5,
			//	error if throttle sensors disagree more than 10% travel
			//	for more than 100ms
			if (fabs(throttle1_cal - throttle2_cal) > .1 ) //legacy: (pedalboxmsg.APPS_Implausible == PEDALBOX_STATUS_ERROR)
			{
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
			if (brake_avg >= BRAKE_PRESSED_THRESHOLD) {
				//brake is presssed
				carSetBrakeLight(BRAKE_LIGHT_ON);  //turn on brake light


				//EV 2.5, check if the throttle level is greater than 25% while brakes are on
				if (throttle_avg > APPS_BP_PLAUS_THRESHOLD) {
					//set apps-brake pedal plausibility error
					car.apps_bp_plaus = PEDALBOX_STATUS_ERROR;
				}
			} else {
				//brake is not pressed
				carSetBrakeLight(BRAKE_LIGHT_OFF);  //turn off brake light
			}


			if (car.apps_bp_plaus == PEDALBOX_STATUS_ERROR) {
				//EV 2.5.1, reset apps-brake pedal plausibility error only if throttle level is less than the .05
				if(throttle_avg <= APPS_BP_PLAUS_RESET_THRESHOLD){
					car.apps_bp_plaus = PEDALBOX_STATUS_NO_ERROR;
				}
			}

			//set the car's throttle to the throttle just received
			if (pedalbox_status == PEDALBOX_STATUS_NO_ERROR)
			{
				//no errors, set throttle to value received from pedalbox
				car.throttle = throttle_avg * MAX_THROTTLE_LEVEL;
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
	//todo optimize stack depths http://www.freertos.org/FAQMem.html#StackSize
	xTaskCreate(taskPedalBoxMsgHandler, "PedalBoxMsgHandler", 512, NULL, 1, NULL);
	xTaskCreate(taskCarMainRoutine, "CarMainRoutine", 512, NULL, 1, NULL);
	xTaskCreate(taskTXCAN, "TX CAN", 512, NULL, 1, NULL);
	xTaskCreate(taskRXCANProcess, "RX CAN Process", 1024, NULL, 1, NULL);
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

