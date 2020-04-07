#ifndef __TELEMETRY_H
#define __TELEMETRY_H

#define TEL_PACKET_LENGTH 35
struct TEL_TYPE {

unsigned int txPktCtr;
unsigned int txUpdateRate;

unsigned long Time;
float mTV;
float mPressure;
float mFlowRate;
float mPEEP;
float mPltPress;
float mFiO2;

int spTV;
int spPressure;
int spFiO2;
int spBPM;
int spIE_Inhale;
int spIE_Exhale;
int spPEEP;
int patientWeight;

unsigned char statusByteError;
unsigned char statusByte1;
}; 

void Prepare_Tx_Telemetry();

#endif
