///***************************************************************************
//*
//*     File Information
//*
//*     Name of File: car.c
//*
//*     Authors (Include Email):
//*     	1. Hussain Khambata
//*       2. Ben Ng				xbenng@gmail.com
//*
//*     File dependents: (header files, flow charts, referenced documentation)
//*       1. BMS.h
//*
//*     File Description:
//*     	Functions to control the physical car
//*
//***************************************************************************/
//#include "BMS.h"
//
//
//void processBMSFrame(CanRxMsgTypeDef rx)
//{
//	switch (rx.StdId)
//	{
//	uint16 value = 0;
//	case ID_BMS_AVGCELLVOLTAGE:
//		bms.avgCellVoltage = 0;
//		bms.avgCellvoltageLRTime = xTaskGetTickCount();
//		value |= rx.Data[AVG_CELLVOLTAGE_BITS_11_8] << 8;
//	    value |=  rx.Data[AVG_CELLVOLTAGE_BITS_0_7];
//	    bms.avgCellVoltage = value * 0.0001 //scale value to volts 0.0001V
//
//	case ID_BMS_HIGHCELLVOLTAGE:
//		bms.highCellVoltage = 0;
//		bms.highCellvoltageLRTime = xTaskGetTickCount();
//		value |= rx.Data[HIGH_CELLVOLTAGE_BITS_11_8] << 8;
//		value |= rx.Data[HIGH_CELLVOLTAGE_BITS_0_7];
//		bms.highCellVoltage = value * 0.0001 //scale value to volts 0.0001V
//
//	case ID_BMS_LOWCELLVOLTAGE:
//		bms.lowCellVoltage = 0;
//		bms.lowCellvoltageLRTime = xTaskGetTickCount();
//		value |= rx.Data[LOW_CELLVOLTAGE_BITS_11_8] << 8;
//		value |= rx.Data[LOW_CELLVOLTAGE_BITS_0_7];
//		bms.lowCellVoltage = value * 0.0001 //scale value to volts 0.0001V
//
//	case ID_BMS_AVGTemperature:
//		bms.avgTemperature = 0;
//		bms.avgTemperatureLRTime = xTaskGetTickCount();
//		bms.avgTemperature |= rx.Data[AVG_Temperature_BITS_11_8] << 8;
//		bms.avgTemperature |= rx.Data[AVG_Temperature_BITS_0_7];
//
//
//	case ID_BMS_HIGHTemperature:
//		bms.highTemperature = 0;
//		bms.highTemperatureLRTime = xTaskGetTickCount();
//		bms.highTemperature |= rx.Data[HIGH_Temperature_BITS_11_8] << 8;
//		bms.highTemperature |= rx.Data[HIGH_Temperature_BITS_0_7];
//
//
//	case ID_BMS_LOWTemperature:
//		bms.lowTemperature = 0;
//		bms.lowTempertureLRTime = xTaskGetTickCount();
//		bms.lowTemperature |= rx.Data[LOW_Temperature_BITS_11_8] << 8;
//		bms.lowTemperature |= rx.Data[LOW_Temperature_BITS_0_7];
//
//
//	case ID_BMS_PACKCURRENT:
//		bms.packCurrent = 0;
//		bms.packCurrentLRTime = xTaskGetTickCount();
//		value |= rx.Data[PACK_CURRENT_BITS_11_8] << 8;
//		value |= rx.Data[PACK_CURRENT_BITS_0_7];
//		bms.packCurrent = value * 0.1; //Scale value to Amperes 0.1A
//
//	case ID_BMS_PACKINSTVOLTAGE:
//		bms.packInstVoltage = 0;
//		bms.packInstVoltageLRTime = xTaskGetTickCount();
//		value |= rx.Data[PACK_INST_VOTLAGE_BITS_11_8] << 8;
//		value |= rx.Data[PACK_INST_VOLTAGE_BITS_0_7];
//		bms.packInstVoltage = value * 0.1 //Scale value to volts 0.1V
//	 }
//}
