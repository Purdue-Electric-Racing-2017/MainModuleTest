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
#include "cmsis_os.h"
#include "car.h"
#include "CANProcess.h"
#include "motor_controller_functions.h"
#include "BMS.h"



//function prototypes
void taskPedalBoxMsgHandler();
void taskCarMainRoutine();
int SendTorqueTask();
int mainModuleWatchdogTask();
int taskHeartbeat();
void initRTOSObjects();


//variable delcarations





#endif /* MAIN_MODULE_TASKS_H_ */
