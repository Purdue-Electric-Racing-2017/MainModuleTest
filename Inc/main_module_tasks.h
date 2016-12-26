/*
 * main_module_tasks.h
 *
 *  Created on: Dec 22, 2016
 *      Author: ben
 */

#ifndef MAIN_MODULE_TASKS_H_
#define MAIN_MODULE_TASKS_H_

//includes
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_can.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "main.h"


//defines
//gpio aliases
//see main.h for user names
#define BUZZER_PORT  //todo
#define BUZZER_PIN
#define FRG_RUN_PORT		GPIOE
#define FRG_RUN_PIN			GPIO_PIN_11
#define RFE_PORT			GPIOE
#define RFE_PIN				GPIO_PIN_12
#define BRAKE_LIGHT_PORT	GPIOE
#define BRAKE_LIGHT_PIN		GPIO_PIN_7
#define HEARTBEAT_PORT		GPIOE
#define HEARTBEAT_PIN		GPIO_PIN_1

typedef enum
{
  BRAKE_LIGHT_OFF = GPIO_PIN_RESET,
  BRAKE_LIGHT_ON = GPIO_PIN_SET
} Brake_light_status_t;


#define BRAKE_PRESSED_THRESHOLD	.05
#define APPS_BP_PLAUS_RESET_THRESHOLD .05  //EV 2.5
#define APPS_BP_PLAUS_THRESHOLD .25  //EV 2.5

#define TORQUE_SEND_PERIOD		100 / portTICK_RATE_MS
#define HEARTBEAT_PULSEWIDTH	10 / portTICK_RATE_MS
#define HEARTBEAT_PERIOD		100 / portTICK_RATE_MS
#define PEDALBOX_TIMEOUT		500 / portTICK_RATE_MS
#define MAX_BRAKE_LEVEL 		0xFFF
#define MAX_THROTTLE_LEVEL		0xFFF
#define LC_THRESHOLD			10 			// BOGUS VALUE

typedef enum {
	PEDALBOX_ERROR = 1,
	PEDALBOX_NO_ERROR = 0
} Pedalbox_status_t;

//launch control
typedef enum {
	LC_ACTIVATED,
	LC_DISABLED
} LC_status_t;


//pedalbox defines
#define PEDALBOX1_FILTER 						0	//filter number corresponding to the PEDALBOX1 message
#define PEDALBOX1_THROT_7_0_BYTE				0
#define PEDALBOX1_THROT_7_0_OFFSET				0
#define PEDALBOX1_THROT_7_0_MASK				0b11111111
#define PEDALBOX1_THROT_11_8_BYTE				1
#define PEDALBOX1_THROT_11_8_OFFSET				4
#define PEDALBOX1_THROT_11_8_MASK				0b11110000
#define PEDALBOX1_BRAKE_7_0_BYTE				2
#define PEDALBOX1_BRAKE_7_0_OFFSET				0
#define PEDALBOX1_BRAKE_7_0_MASK				0b11111111
#define PEDALBOX1_BRAKE_11_8_BYTE				1
#define PEDALBOX1_BRAKE_11_8_OFFSET				0
#define PEDALBOX1_BRAKE_11_8_MASK				0b00001111
#define PEDALBOX1_EOR_BYTE						3
#define PEDALBOX1_EOR_OFFSET					0
#define PEDALBOX1_EOR_MASK						0b00000001
#define PEDALBOX1_IMP_BYTE						3
#define PEDALBOX1_IMP_OFFSET					1
#define PEDALBOX1_IMP_MASK						0b00000010

// Structure to hold data passed through the queue to pedalBoxMsgHandler
typedef struct _pedalbox_msg {
	Pedalbox_status_t EOR; 				// EV 2.4.6: Encoder out of range
	Pedalbox_status_t APPS_Implausible; 	// EV 2.3.5
	uint16_t throttle_level_raw;		// raw throttle data from pedalbox, (average of the two sensors)
	uint16_t brake_level;
} Pedalbox_msg_t;

//function prototypes
void pedalBoxMsgHandlerTask();
int SendTorqueTask();
int mainModuleWatchdogTask();
int heartbeatIdle();
void disableMotor();

typedef struct _car_status {
	uint16_t 			throttle;					//car's intended throttle position
	uint16_t 			brake;						//car's intended brake position
	uint32_t			pb_msg_rx_time;				//indicates when a pedalbox message was last received
	uint32_t			apps_imp_first_time_ms;		//indicates when the first imp error was received
	Pedalbox_status_t	apps_imp_last_state;		//the last pedalbox message imp sate

} Car_handle_t;

//variable delcarations
QueueHandle_t	 	q_pedalbox_frame;
QueueHandle_t		q_mc_frame;
LC_status_t			lc_status;
Pedalbox_status_t	apps_bp_plaus;



#endif /* MAIN_MODULE_TASKS_H_ */
