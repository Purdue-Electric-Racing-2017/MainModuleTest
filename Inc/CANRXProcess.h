/*
 * CANRXProcess.h
 *
 *  Created on: Dec 22, 2016
 *      Author: ben
 */

#ifndef CANRXPROCESS_H_
#define CANRXPROCESS_H_

//defines for reading data from RxCanMsgTypeDef
#define PEDALBOX1_ID							0x500
#define PEDALBOX1_FILTER 						0	//filter number corresponding to the PEDALBOX1 message
#define PEDALBOX1_THROT_7_0_BYTE				0
#define PEDALBOX1_THROT_7_0_OFFSET				0
#define PEDALBOX1_THROT_11_8_BYTE				1
#define PEDALBOX1_THROT_11_8_OFFSET				4
#define PEDALBOX1_THROT_11_8_MASK				0b11110000
#define PEDALBOX1_BRAKE_7_0_BYTE				2
#define PEDALBOX1_BRAKE_7_0_OFFSET				0
#define PEDALBOX1_BRAKE_11_8_BYTE				1
#define PEDALBOX1_BRAKE_11_8_OFFSET				0
#define PEDALBOX1_BRAKE_11_8_MASK				0b00001111
#define PEDALBOX1_EOR_BYTE						3
#define PEDALBOX1_EOR_OFFSET					0
#define PEDALBOX1_EOR_MASK						0b00000001
#define PEDALBOX1_IMP_BYTE						3
#define PEDALBOX1_IMP_OFFSET					1
#define PEDALBOX1_IMP_MASK						0b00000010



QueueHandle_t	q_rxcan_process;


#endif /* CANRXPROCESS_H_ */
