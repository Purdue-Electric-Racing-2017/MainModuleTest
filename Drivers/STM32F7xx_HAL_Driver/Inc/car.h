/*
 * car.h
 *
 *  Created on: Jan 3, 2017
 *      Author: ben
 */

#ifndef CAR_H_
#define CAR_H_

//includes
#include "stm32f7xx_hal.h"
#include "cmsis_os.h"

//gpio aliases
#define BUZZER_PORT				//todo
#define BUZZER_PIN
#define FRG_RUN_PORT		GPIOE
#define FRG_RUN_PIN			GPIO_PIN_11
#define RFE_PORT			GPIOE
#define RFE_PIN				GPIO_PIN_12
#define BRAKE_LIGHT_PORT	GPIOE
#define BRAKE_LIGHT_PIN		GPIO_PIN_7
#define HEARTBEAT_PORT		GPIOE
#define HEARTBEAT_PIN		GPIO_PIN_1


#define BRAKE_PRESSED_THRESHOLD	.05
#define APPS_BP_PLAUS_RESET_THRESHOLD .05  //EV 2.5
#define APPS_BP_PLAUS_THRESHOLD .25  //EV 2.5

#define TORQUE_SEND_PERIOD			100 / portTICK_RATE_MS
#define HEARTBEAT_PULSEWIDTH		10 / portTICK_RATE_MS
#define HEARTBEAT_PERIOD			100 / portTICK_RATE_MS
#define PEDALBOX_TIMEOUT			500 / portTICK_RATE_MS
#define MAX_BRAKE_LEVEL 			0xFFF
#define MAX_THROTTLE_LEVEL			0xFFF
#define LC_THRESHOLD				10			// todo lc threshold BOGUS VALUE

//rtos parameter defines
#define QUEUE_SIZE_RXCAN			3
#define QUEUE_SIZE_TXCAN			3
#define QUEUE_SIZE_PEDALBOXMSG	3
#define QUEUE_SIZE_MCFRAME			3


typedef enum
{
	BRAKE_LIGHT_OFF = GPIO_PIN_RESET,
  	BRAKE_LIGHT_ON = GPIO_PIN_SET
} Brake_light_status_t;

typedef enum {
	PEDALBOX_STATUS_ERROR = 1,
	PEDALBOX_STATUS_NO_ERROR = 0
} Pedalbox_status_t;

//launch control
typedef enum {
	LC_ACTIVATED,
	LC_DISABLED
} LC_status_t;

typedef enum {
	CAR_STATE_INIT,
	CAR_STATE_READY2DRIVE,
	CAR_STATE_ERROR
} Car_state_t;

typedef enum {
	PEDALBOX_MODE_ANALOG,
	PEDALBOX_MODE_DIGITAL
} Pedalbox_mode_t;

// Structure to hold data passed through the queue to pedalBoxMsgHandler
typedef struct _pedalbox_msg {
	Pedalbox_status_t 		EOR; 				// EV 2.4.6: Encoder out of range
	Pedalbox_status_t 		APPS_Implausible; 	// EV 2.3.5
	uint16_t 				throttle_level_raw;		// raw throttle data from pedalbox, (average of the two sensors)
	uint16_t 				brake_level_raw;
} Pedalbox_msg_t;


typedef struct {

	Car_state_t 			state;
	uint8_t					errorFlags;

	uint16_t 				throttle;					//car's intended throttle position
	uint16_t 				brake;						//car's intended brake position
	uint32_t				pb_msg_rx_time;				//indicates when a pedalbox message was last received
	uint32_t				apps_imp_first_time_ms;		//indicates when the first imp error was received
	Pedalbox_status_t		apps_imp_last_state;		//the last pedalbox message imp sate
	Pedalbox_status_t		apps_bp_plaus;				//apps-brake plausibility status
	Pedalbox_mode_t			pb_mode;					//determines whether pb will be analog or CAN

	LC_status_t				lc_status;
	//Pedalbox_msg_t 			pb_current_msg;

	//RTOS objects, initialized in initRTOSObjects
	QueueHandle_t			q_rxcan;
	QueueHandle_t			q_txcan;
	QueueHandle_t	 		q_pedalboxmsg;
	QueueHandle_t			q_mc_frame;

	SemaphoreHandle_t		m_CAN;						//mutex for CAN peripheral

	CAN_HandleTypeDef *		phcan;						//pointer to car's CAN peripheral handle

} Car_t;

extern Car_t car;
extern CAN_HandleTypeDef hcan1;

//function prototypes
void carSetBrakeLight(Brake_light_status_t status);
void ISR_StartButtonPressed();
void carInit();

#endif /* CAR_H_ */
