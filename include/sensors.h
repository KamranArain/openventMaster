#ifndef __SENSORS_H
#define __SENSORS_H
#include "Arduino.h"
#include "header.h"

#define I2C_ADDRESS_ADS1015 0x48 // For Oxygen Sensor

/**********Pressure sensors parameters*************/
#ifdef MS4525DO
#define I2C_ADDRESS_MS4525DO 0x28 /**< 7-bit address. Depends on the order code (this is for code "I") */
#endif

#define K 520 //constant cofficent for flow rate q
#define P_min -1.0f
#define P_max 1.0f
#define PSI_to_Pa 6894.757f

struct P_Sensor
{
  float bmp_pressure = 0,
        bmp_temperature = 0;
  float diff_press_PSI = 0,    //differential pressure from sensor in psi
      diff_press_pa = 0,       //differential pressure form sensor in pa
      q = 0;                   //flow rate in lit/min
  float pressure_gauge = 0;    // Pressure from the sensor in Pa
  float pressure_gauge_CM = 0; // Pressure from the sensor in cmH2O
  uint8_t connectionStatus = 255;
  uint8_t sensorHealth = HEALTH_BAD;
};

struct O2_Sensor
{
  float Pbar = 0.0; // Pressure from the sensor in Bar
  float FIO2_conc = 21.0; // Oxygen Concentration
  unsigned long settlingTime = 4UL*60UL*1000UL;
  unsigned long timestamp = 0;
  uint8_t sensorHealth = HEALTH_BAD;
};

void readSensors();
void checkSensorHealth();
void initO2Sensor();
float getO2Concentration();

#endif