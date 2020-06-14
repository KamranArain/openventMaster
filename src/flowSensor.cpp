#include "header.h"
#include "flowSensor.h"
#include "Wire.h"

#if defined(SFM3200AW)
SFM3x00 measflow(I2C_ADDR_SFM);
#endif

struct Flow_Sensor FS;
#ifdef FS6122
struct FS6122_TYPE fs6122;
#endif
#if defined(FLOW_SENSOR_CN)
void get_sensor_data(byte *a, byte *b);
#endif

extern struct Slave slave;

void initFlowSensor()
{
    #if defined(SFM3200AW)
    FS.offsetFlow = 32768; // Offset for the sensor
    FS.scaleFactorFlow_Air = 120.0; // Scale factor for Air and N2 is 140.0, O2 is 142.8
    //float scaleFactorFlow_O2 = Not Applicable; // Scale factor for Air and N2 is 140.0, O2 is 142.8
    // For compensation for O2 and humidity see application note: 
    // “GF_AN_SFM3200_SFM3300_Effects_Humidity_Gas_mixtures”
    // https://www.repcomsrl.com/wp-content/uploads/2016/05/GF_AN_SFM3200_SFM3300_Effects_Humidity_Gas_mixtures_V1_D2.pdf

//    FS.offsetTemperature = 20000; // Offset for the sensor
//    FS.scaleFactorTemperature = 100.0; // Scale factor for Temperature
    FS.sensorHealth = HEALTH_BAD;
    FS.connectionStatus = measflow.init();
    #endif

    #if defined(FLOW_SENSOR_CN)
    #endif

    #ifdef FS6122
    write_fs6122_filter();
    delay(1000);
    read_fs6122_filter();
    #endif
}

float getFlowValue()
{
    #ifdef SFM3200AW
    float volFlowRate = 0.0;
    unsigned int rawVal = 0.0;
    rawVal = measflow.getvalue();
    volFlowRate = ((float)rawVal - FS.offsetFlow) / (FS.scaleFactorFlow_Air);
    volFlowRate = (-1.0) * volFlowRate;

    uint8_t dataValid = measflow.checkDataValidity();
    if (dataValid == DATA_VALID)
        FS.sensorHealth = HEALTH_GOOD;
    else if (dataValid == DATA_INVALID)
        FS.sensorHealth = HEALTH_BAD;

    return volFlowRate;
    #endif

    #ifdef FLOW_SENSOR_CN
    byte a0,a1;
    get_sensor_data(&a0,&a1);
    //long unsigned raw = (a0 & 0x3F) << 8; //0011 1111 0x3F
    long unsigned raw = (a0<<8) | a1;
    float v = (float)raw;
    v = v-300.0
    v = c*2.0;
    // if (v>0)
    //   v = v*1.102
    return v;
    #endif

}

#if defined(SFM3200AW)
float getCorrectedFlowValue(int phase)
{
    float volFlowRate = 0.0;
    float correctedFlowRate = 0.0;
    volFlowRate = getFlowValue();

    #ifndef APPLY_Q_CORRECTION_FACTOR
    correctedFlowRate = volFlowRate;
    #else
    float qMeas, qMeasAbs, qMin, qMax, factorMin, factorMax, corrFactor;
    qMeas = volFlowRate;
    qMeasAbs = abs(qMeas);
    switch (phase)
    {
    case INSPIRATION_PHASE:
        if (qMeasAbs <= 30.0)
        {
            qMin = 0.0;
            qMax = 30.0;
            factorMin = 0.0;
            factorMax = 2.5;
            corrFactor = (mapFloat(qMeasAbs, qMin, qMax, factorMin, factorMax)) / 100.0f;
        }
        else    corrFactor = 2.5 / 100.0f;                
        break;
    case EXPIRATION_PHASE:
        if (qMeasAbs <= 30.0)
        {
            qMin = 0.0;
            qMax = 30.0;
            factorMin = 1.80;
            factorMax = 0.64;
            corrFactor = (mapFloat(qMeasAbs, qMin, qMax, factorMin, factorMax)) / 100.0f;
        }
        else
        {
            qMin = 30.0;
            qMax = 250.0;
            factorMin = 0.64;
            factorMax = 0.20;
            corrFactor = (mapFloat(qMeasAbs, qMin, qMax, factorMin, factorMax)) / 100.0f;
        }    
        break;
    default:
        corrFactor = 0.0;
        break;
    }
//    if (qMeas >= 0.0)
        correctedFlowRate = qMeas * (1 - corrFactor);
    //else
    // Seperate handling for negative values reqd or not
    #endif

    return correctedFlowRate;
}
#endif

float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow; 
}


#if defined(FLOW_SENSOR_CN)
void get_sensor_data(byte *a, byte *b) {
  
    Wire.beginTransmission(I2C_ADDR_FS);
    Wire.write(1);
    Wire.endTransmission();
    FS.connectionStatus = Wire.endTransmission();
    //delay(1);
    Wire.requestFrom(I2C_ADDR_FS, 2);
    *a = Wire.read();
    *b = Wire.read();
    
    if (FS.connectionStatus != 0)
        FS.sensorHealth = HEALTH_BAD;
    else FS.sensorHealth = HEALTH_GOOD;

//   if (status.homeAtBadFlowSensor)
//   {
//     Serial.println("Waiting For Homing Complete: Flow Sensor");
//   }
//   else {
//     Wire.beginTransmission(I2C_ADDR_FS);
//     Wire.write(1);
//     Wire.endTransmission();
//     FS.connectionStatus = Wire.endTransmission();
//     //delay(1);
//     Wire.requestFrom(I2C_ADDR_FS, 2);
//     *a = Wire.read();
//     *b = Wire.read();
//   }

//   if (FS.connectionStatus != 0)
//     FS.sensorHealth = HEALTH_BAD;
//   else FS.sensorHealth = HEALTH_GOOD;
}

#endif

#if defined(FS6122)
void read_fs6122_flow_pressure() {

  byte a,b,c,d;
  int32_t flow_rate, pressure;

  Wire.beginTransmission(FS6122_SENSOR_ADDR);
  Wire.write(0x84);
  Wire.endTransmission();
  delay(1);
  Wire.requestFrom(FS6122_SENSOR_ADDR, 8);

  a = Wire.read();
  b = Wire.read();
  c = Wire.read();
  d = Wire.read();


/*
  flow_rate = a  << 24;
  flow_rate |= b << 16;
  flow_rate |= c << 8;
  flow_rate |= d;
*/
  
  flow_rate =  a; flow_rate <<= 8; // shift by byte each read
  flow_rate |= b; flow_rate <<= 8;
  flow_rate |= c; flow_rate <<= 8;
  flow_rate |= d; 
  
  pressure = Wire.read(); pressure <<= 8;
  pressure |= Wire.read(); pressure <<= 8;
  pressure |= Wire.read(); pressure <<= 8;
  pressure |= Wire.read(); 

  uint32_t divider = 1000; // divide both values by 1000 (TODO: CHECK: is this a problem, using 1024 instead?!)

  fs6122.flow_rate_slpm = -1.0 * (float)flow_rate / divider;
  fs6122.pressure_cmh2o = (float)pressure / divider;


}


void read_fs6122_temperature(void) {
  int16_t temperature;
  
  Wire.beginTransmission(FS6122_SENSOR_ADDR);
  Wire.write(FS6122_READ_TEMPERATURE);
  Wire.endTransmission();
  delay(1);
  Wire.requestFrom(FS6122_SENSOR_ADDR, 2);

  temperature = Wire.read(); temperature <<= 8; // shift by byte each read
  temperature |= Wire.read();

  uint16_t divider = 100; // divide both values by 1000

  fs6122.temperature_c = (float)temperature / divider;

}

void read_fs6122_humidity(void) {
  int16_t humidity;
  
  Wire.beginTransmission(FS6122_SENSOR_ADDR);
  Wire.write(FS6122_READ_HUMIDITY);
  Wire.endTransmission();
  delay(1);
  Wire.requestFrom(FS6122_SENSOR_ADDR, 2);

  
  humidity = Wire.read(); humidity <<= 8; // shift by byte each read
  humidity |= Wire.read();

  uint16_t divider = 100; // divide both values by 1000

  fs6122.humidity_prh = (float)humidity / divider;

}


void read_fs6122_filter(void) {
  
  Wire.beginTransmission(FS6122_SENSOR_ADDR);
  Wire.write(0x8B);
  Wire.endTransmission();
  delay(1);
  Wire.requestFrom(FS6122_SENSOR_ADDR, 1);

  uint8_t val = Wire.read(); 
  fs6122.filter = val;
  Serial.print(F("Filter Depth: "));Serial.println(val);
  // Wire.endTransmission();
  // delay(20000);
}


void write_fs6122_filter(void) {
  
  // Wire.beginTransmission(FS6122_SENSOR_ADDR);
  // Wire.write(0x0B);
  // int status = Wire.endTransmission(false);
  // Serial.println(status);
  // delay(1);
  // Wire.write(0x02);
  // status = Wire.endTransmission();
  // Serial.println(status);
  // delay(1);

  Wire.beginTransmission(FS6122_SENSOR_ADDR);
  Wire.write(0x0B);
  Wire.write(0xFE);
  int status = Wire.endTransmission(false);
  Serial.println(status);
  delay(1);
}
#endif