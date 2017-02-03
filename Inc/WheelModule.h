/*
 * WheelModule.h
 *
 *  Created on: Jan 13, 2017
 *      Author: ben
 */

#ifndef WHEELMODULE_H_
#define WHEELMODULE_H_

typedef struct {
	uint16_t speedFR;
	uint16_t speedFL;
	uint16_t speedRR;
	uint16_t speedRL;
} WheelModule_t;

extern WheelModule_t wheelModule;



#endif /* WHEELMODULE_H_ */
