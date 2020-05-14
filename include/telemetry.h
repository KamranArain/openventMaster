#ifndef __TELEMETRY_H
#define __TELEMETRY_H

#include "Arduino.h"

#define TEL_PACKET_LENGTH 48
struct TEL_TYPE {

byte FDCB;

uint8_t txUpdateRate;

unsigned long Time;
float mTV;
float mTVinsp;
float mTVexp;
float mPressure;
float mFlowRate;
float mPEEP;
float mPltPress;
float mFiO2;
float minuteVentilation;
float staticCompliance;
float mRR;
float mPeakPressure;

uint16_t spTV;
int16_t spInsPressure;
int16_t spExpPressure;
uint8_t spFiO2_UpperBound;
uint8_t spFiO2_LowerBound;
uint8_t spBPM;
uint8_t spIE_Inhale;
uint8_t spIE_Exhale;
uint8_t patientWeight;
float spTrigger;

uint8_t ErrorStatusByte1 = 0;
uint8_t ErrorStatusByte2 = 0;
uint8_t ErrorStatusByte3 = 0;
uint8_t ErrorStatusByte4 = 0;
uint8_t statusByte1;

}; 

void Prepare_Tx_Telemetry();

#endif
