#ifndef __FLOWSENSOR_H
#define __FLOWSENSOR_H

#if defined(SFM3200AW)
#include "sfm3x00.h"
#define I2C_ADDR_SFM 64 // 0x40 //Default for SFM3000, SFM3200, SFM3200AW and SFM3300
#endif

#if defined(FLOW_SENSOR_CN)
#define I2C_ADDR_FS 0x50
#endif

#ifdef FS6122
#define FS6122_SENSOR_ADDR 0x01

#define FS6122_READ_SENSOR_SN           0x82 // ASCII
#define FS6122_READ_FLOWRATE            0x83 // Int32/1000 SLPM
#define FS6122_READ_FLOWRATE_PRESSURE   0x84 // Int32/1000 SLPM, int32/1000 cmH20

#define FS6122_READ_ADDRESS             0x85 // Bit7 ~ Bit1
#define FS6122_READ_FILTERDEPTH         0x8B // Int8, 0~254
#define FS6122_READ_PRESSURE            0xA3 // Int32/1000 cmH20
#define FS6122_READ_TEMPERATURE         0xB2  // Int16/100 deg celcius
#define FS6122_READ_HUMIDITY            0xB3  // Int16/100 %RH

#endif

//#define APPLY_Q_CORRECTION_FACTOR

void initFlowSensor();

float getFlowValue();

#if defined(SFM3200AW)
float getCorrectedFlowValue(int phase);
#endif

float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh);

struct Flow_Sensor
{
  byte connectionStatus = 127;
  unsigned long offsetFlow = 32768; // Offset for the sensor
  float scaleFactorFlow_Air = 120.0; // Scale factor for Air
//  float scaleFactorFlow_O2 = 142.8; // Scale factor for O2 is SFM3000 Only
//  unsigned int offsetTemperature = 20000; // Offset for the sensor for SFM3200AW Only
//  float scaleFactorTemperature = 100.0; // Scale factor for Temperature for SFM3200AW Only
  float Q_SLM = 0.0;
  float Q_SFM = 0.0;
  uint8_t sensorHealth;
};


#ifdef FS6122
struct FS6122_TYPE
{
  float flow_rate_slpm_filt = 0.0;
  float flow_rate_slpm = 0.0;
  float pressure_cmh2o = 0.0;
  float temperature_c = 0.0;
  float humidity_prh = 0.0;
  int filter = 0.0;
};

void read_fs6122_flow_pressure();
void read_fs6122_temperature();
void read_fs6122_humidity();
void read_fs6122_filter();
void write_fs6122_filter();
#endif

#endif