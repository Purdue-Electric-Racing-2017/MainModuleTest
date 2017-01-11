
#ifndef BMS_H_
#define BMS_H_

#include "stm32f7xx_hal.h"
#include "cmsis_os.h"

typedef struct {
	uint16_t packVoltage;
	uint32_t packVoltageLastReceivedTime;
} BMS_t;

#define PACK_VOLTAGE_BYTE1		1;
#define PACK_VOLTAGE_BYTE1		0;


extern BMS_t BMS;

#endif /* BMS_H_ */
