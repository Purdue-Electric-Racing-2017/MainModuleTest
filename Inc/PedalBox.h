/*
 * PedalBox.h
 *
 *  Created on: Jan 12, 2017
 *      Author: ben
 */

#ifndef PEDALBOX_H_
#define PEDALBOX_H_

#include "stm32f7xx_hal.h"
#include "cmsis_os.h"

typedef struct {
	int a;
} PedalBox_t;

extern PedalBox_t PedalBox;

#endif /* PEDALBOX_H_ */
