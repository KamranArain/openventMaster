
#include "header.h"
#include "flowSensor.h"
#include "sensors.h"
#include <Adafruit_ADS1015.h>

extern struct Flow_Sensor FS;
extern struct STATUS_FLAGS status;
extern struct Slave slave;
extern struct setpointStatus setpoint;
#ifdef FS6122
extern struct FS6122_TYPE fs6122;
#endif


struct P_Sensor p_sensor;
struct O2_Sensor O2;

Adafruit_ADS1115 ads(I2C_ADDRESS_ADS1015);  /* Use this for the 16-bit version */
const float o2_mv_air = 15.3;

//float alpha=0.5;
float avgO2=0;

#ifdef MS4525DO
/*
   used to get values from MS4525 sensor
   and should be called after 10ms
   due to sensor update or refresh rate
*/

inline static void get_sensor_data(uint16_t *raw) {
//  if ((slave.lastCMD_ID == HOME) && (status.homeAtBadPressSensor))
  if (status.homeAtBadPressSensor)
  {
    // Serial.println("Waiting For Homing Complete: Press Sensor");
  }
  else {
    Wire.beginTransmission(I2C_ADDRESS_MS4525DO);
    Wire.write(1);
    p_sensor.connectionStatus = Wire.endTransmission();
    Wire.requestFrom(I2C_ADDRESS_MS4525DO, 2); //request for two pressure bytes from sensor
    *raw = (Wire.read() & 0x3F) << 8; // read the msb from the I2C device
    *raw |= Wire.read();//read the lsb from the device
  }

  if (p_sensor.connectionStatus != 0)
    p_sensor.sensorHealth = HEALTH_BAD;
  else p_sensor.sensorHealth = HEALTH_GOOD;
}
#endif

float readVcc() {
  long result; // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert while (bit_is_set(ADCSRA,ADSC));
  result = ADCL; result |= ADCH << 8;
  result = 1126400L / result; // Ba,,ck-calculate AVcc in mV
  return result;
}


void voltage_correction(float &diff_press_pa)
{
  const float slope = 65.0f;
  /*
    apply a piecewise linear correction, flattening at 0.5V from 5V
  */
  float voltage_diff = readVcc() - 5.0f;
  if (voltage_diff > 0.5f) {
    voltage_diff = 0.5f;
  }

  if (voltage_diff < -0.5f) {
    voltage_diff = -0.5f;
  }
  diff_press_pa -= voltage_diff * slope;
}

void readSensors() // Read Values from Installed Sensor
{
#if defined (MS4525DO)
  // Calculate differential pressure. As its centered around 8000
  // and can go positive or negative
  uint16_t raw_pressure = 0;
  get_sensor_data(&raw_pressure);

if (p_sensor.sensorHealth == HEALTH_GOOD) {
  /*this equation is an inversion of the equation in the
    pressure transfer function figure on page 4 of the datasheet
    We negate the result so that positive differential pressures
    are generated when the bottom port is used as the static
    port on the pitot and top port is used as the dynamic port*/
  p_sensor.diff_press_PSI = -((raw_pressure - 0.1f * 16383) * (P_max - P_min) / (0.8f * 16383) + P_min);
  p_sensor.diff_press_pa = p_sensor.diff_press_PSI * PSI_to_Pa;

  voltage_correction(p_sensor.diff_press_pa); //Recommended by Hamza
#ifdef MS4525_AS_Gauge_Pressure
  p_sensor.pressure_gauge = p_sensor.diff_press_pa;

  static float oldY = 0.0;
  static float avgP = 0.8;
  float y = p_sensor.pressure_gauge * Pa2cmH2O;
  y = (y * avgP) + ( oldY * (1.0 - avgP));
  oldY = y;
  p_sensor.pressure_gauge_CM = oldY;

  //  p_sensor.pressure_gauge_CM = p_sensor.pressure_gauge * Pa2cmH2O; //Unfiltered

#else
  float diff_press_bar = (p_sensor.diff_press_PSI * 0.0689476);
  //liter/min flow rate 60000 where is K is 520 constant
  p_sensor.q = ( K * diff_press_Bar ) * 60000;
#endif
}
#endif
 
  //FLOW SENSORS
  #if defined(FLOW_SENSOR_INSTALLED)
  #ifdef FLOW_SENSOR_CN
  static float flowValue;
  static unsigned long pre_millis_FS = 0;
  if ((millis() - pre_millis_FS) >= 100)
    flowValue =  getFlowValue();
  if (FS.sensorHealth == HEALTH_GOOD)
    FS.Q_SLM =flowValue;
  #elif defined(SFM3200AW)    
  float flowValue =  getFlowValue();
  if (FS.sensorHealth == HEALTH_GOOD) {
    
    FS.Q_SFM = (1.2405*flowValue) - 0.2334;
    
    
    FS.Q_SLM = FS.Q_SFM;

    

  }
  #endif
  #endif

  #ifdef FS6122
  read_fs6122_flow_pressure();
  // FS.sensorHealth = HEALTH_GOOD;
  // FS.connectionStatus = 0;

  static float oldQ = 0.0;
  static float avgQ = 0.5;
  float q = fs6122.flow_rate_slpm;
  q = (q * avgQ) + ( oldQ * (1.0 - avgQ));
  oldQ = q;
  fs6122.flow_rate_slpm_filt = oldQ;
  // FS.Q_SLM = oldQ;
  // FS.Q_SLM = fs6122.flow_rate_slpm;


  p_sensor.pressure_gauge_CM = (0.9503 *fs6122.pressure_cmh2o) + 0.5025;;


  p_sensor.connectionStatus = 0;
  p_sensor.sensorHealth = HEALTH_GOOD;
  // read_fs6122_humidity();
  // read_fs6122_temperature();
  #endif

 O2.FIO2_conc = getO2Concentration();

  static uint8_t curFiO2_sp = setpoint.reqFiO2;

  if (curFiO2_sp != setpoint.reqFiO2)
  {
    curFiO2_sp = setpoint.reqFiO2;
    status.FIO2Valid = false;
    O2.timestamp = millis();
  }  
  if (((millis() - O2.timestamp) >= O2.settlingTime) || status.FIO2Valid)
  {
    status.FIO2Valid = true;
  }

}


void checkSensorHealth()
{
  #define decisionTime  2 //10 * 10 = 100 ms 
  #define decisionTimeStatusDeclaration 3 //10 * 10 = 100 ms 
  static uint8_t HealthCtr = 0;
  static int16_t FS_HealthCtr = 0;
  static int16_t PS_HealthCtr = 0;
  
  if ((FS.sensorHealth == HEALTH_GOOD) && (p_sensor.sensorHealth == HEALTH_GOOD))
    HealthCtr++;
  else HealthCtr--;

  HealthCtr = constrain(HealthCtr, 0, decisionTime);
  if (HealthCtr >= decisionTime)  status.sensorsHealth = HEALTH_GOOD;
  else if (HealthCtr <= 0)        status.sensorsHealth = HEALTH_BAD;

//  Serial.print(F("Sensor Health = ")); Serial.println(status.sensorsHealth);
//  Serial.print(F("Flow Sensor Health = ")); Serial.println(FS.sensorHealth);
//  Serial.print(F("Press Sensor Health = ")); Serial.println(p_sensor.sensorHealth);

////////////////////////////////////////////////////////////////////////////////
  if (FS.sensorHealth == HEALTH_GOOD)
    FS_HealthCtr++;
  else {
    FS_HealthCtr--;
    #if defined(SFM3200AW)
    // if ((slave.lastCMD_ID == HOME) && (slave.homeAck == 2))
    if ((slave.lastCMD_ID == HOME) && (!status.homeAtBadFlowSensor))
      initFlowSensor();
    #endif
  #ifndef TEL_AT_UART0
      // Serial.println(F("Flow Sensor Health BAD"));
      // Serial.print(F("Flow Sensor Error Code: ")); Serial.println(FS.connectionStatus);
  #endif  
  }

  FS_HealthCtr = constrain(FS_HealthCtr, -1*decisionTimeStatusDeclaration, decisionTimeStatusDeclaration);
  if (FS_HealthCtr >= decisionTimeStatusDeclaration)  status.flowSensorFailure = false;
  else if (FS_HealthCtr <= (-1*decisionTimeStatusDeclaration))        status.flowSensorFailure = true;

////////////////////////////////////////////////////////////////////////////////////
  if (p_sensor.sensorHealth == HEALTH_GOOD)
    PS_HealthCtr++;
  else {
    PS_HealthCtr--;
  #ifndef TEL_AT_UART0
      // Serial.println(F("Pressure Sensor Health BAD"));
      // Serial.print(F("Pressure Sensor Error Code: ")); Serial.println(p_sensor.connectionStatus);
  #endif
  }

  PS_HealthCtr = constrain(PS_HealthCtr, -1*decisionTimeStatusDeclaration, decisionTimeStatusDeclaration);
  if (PS_HealthCtr >= decisionTimeStatusDeclaration)  status.presSensorFailure = false;
  else if (PS_HealthCtr <= (-1*decisionTimeStatusDeclaration))        status.presSensorFailure = true;
}

void initO2Sensor() {
  ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV  
  ads.begin();
  O2.timestamp = millis();
}

float getO2Concentration() {
  int16_t adc0, adc1;
  float   o2_volt;
  float   o2_comp_volt;
  float   o2_conc;
  float   p_volt, p_bar;
  float retval;
  float newO2;

  //O2
  adc0 = ads.readADC_SingleEnded(0); 
  //P
  adc1 = ads.readADC_SingleEnded(1);

  p_volt = adc1 * 0.0078125;
  p_bar = p_volt / 10.0;
  p_bar = (p_bar - 4.0) / 16.0;
  p_bar = p_bar * 22.0;
  
  o2_volt = adc0 * 0.0078125; 
  o2_comp_volt = o2_volt / (1.0 + p_bar);
  o2_conc = o2_comp_volt / o2_mv_air * 20.9;


  newO2 = (1.81*o2_conc) - 15.844;


  //o2_conc = (o2_conc / 1.25); 

  //avgO2 = (alpha * o2_conc) + (1-alpha)*avgO2;

  retval = newO2;

  if (retval > 100.0) retval= 100.0;

  /* 

  Serial.print("O2 Volts: "); Serial.println(o2_volt);
  Serial.print("O2 Conc: "); Serial.println(o2_conc);
  Serial.print("P Volts: "); Serial.println(p_volt);
  Serial.print("P bar: "); Serial.println(p_bar);
*/

//  Serial.print("$");
//  Serial.print(o2_volt);
//  Serial.print(" ");
//  Serial.print(o2_comp_volt);
//  Serial.print(" ");
//  Serial.print(o2_conc);
//  Serial.print(" ");
//  Serial.print(newO2);
//  Serial.print(" ");
//   Serial.print(p_bar);
//  Serial.print(";");

  O2.Pbar = p_bar;
//  Serial.print("O2: ");Serial.println(o2_conc);
//  Serial.print("pbar: ");Serial.println(p_bar);
  return retval;
}