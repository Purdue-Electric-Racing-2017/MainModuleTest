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
	sendTorque(0);
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

	//
	HAL_GPIO_WritePin(RFE_PORT, RFE_PIN, GPIO_PIN_SET);
	vTaskDelay(500 / portTICK_RATE_MS);
	HAL_GPIO_WritePin(FRG_RUN_PORT, FRG_RUN_PIN, GPIO_PIN_SET);


}

void setBrakeLight(Brake_light_status_t status)
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

void pedalBoxMsgHandlerTask(Car_handle_t *car) {
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
	CanRxMsgTypeDef rx;

	while (1) {
		if(xQueueReceive(q_pedalbox_frame, &rx, 1000)){
			Pedalbox_msg_t pedalboxmsg;		//struct to store the scrubbed data

			//get current time in ms
			uint32_t current_time_ms;

			// set time stamp, indicates when a pedalbox message was last received
			car->pb_msg_rx_time = xTaskGetTickCount() / portTICK_PERIOD_MS;

			///////////SCRUB DATA the from the CAN frame//////////////
			//mask then shift the throttle value data
			uint8_t throttle_7_0 	=
					rx.Data[PEDALBOX1_THROT_7_0_BYTE]  >> PEDALBOX1_THROT_7_0_OFFSET;  //Throttle Value (7:0) [7:0]
			uint8_t throttle_11_8	=
					(rx.Data[PEDALBOX1_THROT_11_8_BYTE] & PEDALBOX1_THROT_11_8_MASK) >> PEDALBOX1_THROT_11_8_OFFSET;  //Throttle Value (11:8) [3:0]
			//mask then shift the brake value data
			uint8_t brake_7_0 	=
					rx.Data[PEDALBOX1_BRAKE_7_0_BYTE]  >> PEDALBOX1_THROT_7_0_OFFSET;  //Throttle Value (7:0) [7:0]
			uint8_t brake_11_8	=
					(rx.Data[PEDALBOX1_BRAKE_11_8_BYTE] & PEDALBOX1_BRAKE_11_8_MASK) >> PEDALBOX1_BRAKE_11_8_OFFSET;  //Throttle Value (11:8) [3:0]

			//build the data
			pedalboxmsg.throttle_level_raw = 0;
			pedalboxmsg.throttle_level_raw |= throttle_7_0 << 0;
			pedalboxmsg.throttle_level_raw |= throttle_11_8 << 8;
			pedalboxmsg.brake_level = 0;
			pedalboxmsg.brake_level |= brake_7_0 << 0;
			pedalboxmsg.brake_level |= brake_11_8 << 8;

			//mask then shift the error flags
			pedalboxmsg.APPS_Implausible =
					(rx.Data[PEDALBOX1_IMP_BYTE] & PEDALBOX1_IMP_MASK) >> PEDALBOX1_IMP_OFFSET;  //Throttle Value (7:0) [7:0] and mask 1 bit
			pedalboxmsg.EOR =
					(rx.Data[PEDALBOX1_EOR_BYTE] & PEDALBOX1_EOR_MASK) >> PEDALBOX1_EOR_OFFSET;  //Throttle Value (7:0) [7:0] and mask 1 bit

			/////////////PROCESS DATA///////////////
			Pedalbox_status_t  pedalbox_status = PEDALBOX_NO_ERROR;  //flag to be set if there is an error

			// EV 2.4.6: Encoder out of range
			if (pedalboxmsg.EOR == PEDALBOX_ERROR) {
				pedalbox_status = PEDALBOX_ERROR;
			} 
			
			//APPS Implausibility error handling, EV 2.3.5
			if (pedalboxmsg.APPS_Implausible == PEDALBOX_ERROR) {
				//if error is persistent
				if (car->apps_imp_last_state == PEDALBOX_ERROR)
				{
					//if time between first error and this error >= 100ms
					if (car->apps_imp_first_time_ms - current_time_ms >= 100)
					{
						pedalbox_status = PEDALBOX_ERROR;
					}
				} else {  //else this is the first message to have an imp error
					//record the time
					car->apps_imp_first_time_ms = current_time_ms;
				}
			}
			//update last state variable so we know this state, next time we get an error
			car->apps_imp_last_state = pedalboxmsg.APPS_Implausible;


			//Brake
			//check if brake level is greater than the threshold level
			if (pedalboxmsg.brake_level >= BRAKE_PRESSED_THRESHOLD * MAX_BRAKE_LEVEL) {
				//brake is presssed
				setBrakeLight(BRAKE_LIGHT_ON);  //turn on brake light


				//EV 2.5, check if the throttle level is greater than 25% while brakes are on
				if (pedalboxmsg.throttle_level_raw > APPS_BP_PLAUS_THRESHOLD * MAX_THROTTLE_LEVEL) {
					//set apps brake pedal plausibility error
					apps_bp_plaus = PEDALBOX_ERROR;
				}
			} else {
				//brake is not pressed
				setBrakeLight(BRAKE_LIGHT_OFF);  //turn off brake light
			}


			if (apps_bp_plaus == PEDALBOX_ERROR) {
				//EV 2.5.1, reset apps brake pedal plausibility error only if throttle level is less than the .05
				if(pedalboxmsg.throttle_level_raw <= APPS_BP_PLAUS_RESET_THRESHOLD * MAX_THROTTLE_LEVEL){
					apps_bp_plaus = PEDALBOX_NO_ERROR;
				}
			}

			//set the car's throttle to the throttle just received
			if (pedalbox_status == PEDALBOX_NO_ERROR)
			{
				//no errors, set throttle to value received from pedalbox
				car->throttle = pedalboxmsg.throttle_level_raw;
			} else {
				//if there were errors, set throttle = 0
				car->throttle = 0;
			}
		}
	}

	//if this task breaks from the loop kill it
	vTaskDelete(NULL);
}

//TODO Potential MC ping function

int SendTorqueTask(Car_handle_t *car) {
/***************************************************************************
*
*     Function Information
*
*     Name of Function: SendTorqueTask
*
*     Programmer's Name: Kai Strubel
*
*     Function Return Type: int
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*       
*      Global Dependents:
*	    1.bool launchControl
*		2.float apps_max
*		3.float torque
*		4.float scaleFactor
*
*     Function Description:
*		Calculates the torque based of slipping factor and throttle position
*		
***************************************************************************/
	while(1) {
		uint32_t current_time_ms = xTaskGetTickCount() / portTICK_PERIOD_MS;
		uint16_t torque_to_send = 0;

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

		//check if the age of the pedalbox message is greater than the timeout
		if (current_time_ms - car->pb_msg_rx_time > PEDALBOX_TIMEOUT) {
			torque_to_send = 0;
			//todo send a CAN message to dash?
		} else {
			torque_to_send = car->throttle;
		}

		//send torque to motor controller
		sendMotorMessage(torque_to_send);
		vTaskDelay(TORQUE_SEND_PERIOD);
	}
}

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

int heartbeatIdle() {
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

int initRTOSObjects(Car_handle_t *car) {
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


	//init queues
	q_rxcan = xQueueCreate(3, sizeof(CanRxMsgTypeDef));
	q_pedalbox_frame = xQueueCreate(3, sizeof(CanRxMsgTypeDef));
	q_mc_frame = xQueueCreate(3, sizeof(CanRxMsgTypeDef));

	/* Create Tasks */
	xTaskCreate(pedalBoxMsgHandlerTask, (signed char*) "pedalBoxMsgHandler", 1024, car, 1, NULL);
	xTaskCreate(mainModuleWatchdogTask, (signed char*) "mainModuleTimeCheckIdle", 1024, car, 1, NULL);
	xTaskCreate(SendTorqueTask, (signed char*) "mainModuleTorque", 1024, car, 1, NULL);
	xTaskCreate(heartbeatIdle, (signed char*) "heartbeatIdle", 1024, car, 1, NULL);


	return 0;

 }
