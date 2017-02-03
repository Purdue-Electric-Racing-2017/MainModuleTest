/*
 * main_module_tasks.h
 *
 *  Created on: Dec 22, 2016
 *      Author: ben
 */

#ifndef MAIN_MODULE_TASKS_H_
#define MAIN_MODULE_TASKS_H_


//includes
#include "motor_controller_functions.h"

#define LAUNCH_CONTROL_INTERVAL_MS	100



//function prototypes
void taskPedalBoxMsgHandler();
void taskCarMainRoutine();
int SendTorqueTask();
int mainModuleWatchdogTask();
int taskHeartbeat();
void initRTOSObjects();


//variable delcarations





#endif /* MAIN_MODULE_TASKS_H_ */
