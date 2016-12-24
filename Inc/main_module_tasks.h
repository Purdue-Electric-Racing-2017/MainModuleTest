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
#include "FreeRTOS.h"
#include "queue.h"
#include "main.h"


//defines
//gpio aliases
//see main.h for user names
#define FRG_RUN_PORT		GPIOE
#define FRG_RUN_PIN			GPIO_PIN_11
#define REF_PORT			GPIOE
#define REF_PIN				GPIO_PIN_12
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

#define
#define HEARTBEAT_PULSEWIDTH	10 / portTICK_RATE_MS
#define HEARTBEAT_PERIOD		100 / portTICK_RATE_MS
#define PEDALBOX_TIMEOUT		100 / portTICK_RATE_MS
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

// Structure to hold data passed through the queue to pedalBoxMsgHandler
typedef struct _pedalbox_msg {
	Pedalbox_status_t EOR; 				// EV 2.4.6: Encoder out of range
	Pedalbox_status_t APPS_Implausible; 	// EV 2.3.5
	uint16_t throttle_level;		// raw throttle data from pedalbox, (average of the two sensors)
	uint16_t brake_level;
} Pedalbox_msg_t;



//function prototypes
int pedalBoxMsgHandlerTask();
int SendTorqueTask();
int mainModuleWatchdogTask();
int heartbeatIdle();
void disableMotor();


//variable delcarations
QueueHandle_t	 	q_pedalbox_msg;

uint32_t			pb_msg_rx_time;
uint32_t			apps_imp_last_time_ms;
Pedalbox_status_t	apps_imp_last;
uint16_t			throttle;	//the throttle position, after error handling
LC_status_t			lc_status;
Pedalbox_status_t	apps_bp_plaus;



#endif /* MAIN_MODULE_TASKS_H_ */
