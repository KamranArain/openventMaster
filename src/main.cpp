/*
 * All the #define are in Plateformio.ini file with -D name 
   OpenVentPK Prototype 1 OVPD-1 - Source Code
    //TODO:
        ADD Version History Here
        In Future;
        Implement Watchdog
        Address Wire.h long reported bug
        Interface O2 Sensor

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

    Author: OpenVentPK.
    Created March 28, 2020
    EMAIL : <sohaib_ashraf@hotamail.com>
*/
#include "header.h"
#include "flowSensor.h"
#include "control.h"
#include "userInterface.h"
#include "command.h"
#include "alarms.h"
#ifdef EN_WATCHDOG
  #include <avr/wdt.h>
#endif


//Other Address could be 0x20, 0x27 , 0x3F
// Run I2C_Scanner Script to discover device

#ifdef TX_SERIAL_TELEMETRY
#include "telemetry.h"
#endif

extern double VolCoeffs[ORDER+1];
extern double PressCoeffs[ORDER_PRESS_EQ+1];

extern uint8_t IE_R_Value[3][2];
extern uint8_t FiO2_Value[4][2];

extern byte calibStatus;
extern byte estimateVolume;
extern byte calibrationParam;

bool activateVentilatorOperation = false; 
bool activateVentilatorControlSerial = false;
bool plotterData = true;

bool PEEPMesaureFlag = false;
bool scanBreathingAttempt = false;
bool patientTriggeredBreath = false;
bool holdManeuver = true;
uint16_t holdDur_ms = 150;
uint32_t actualBreathCycleTime = 0;
//int triggerVariable = FLOW_VAR;
//int cyclingVariable = TIME_VAR;


#ifdef TX_SERIAL_TELEMETRY
extern struct TEL_TYPE TEL;
#endif
extern struct Flow_Sensor FS;
extern struct ALARMS Alarms;

struct setpointStatus setpoint;
struct P_Sensor p_sensor;
struct TidalVolume TV;
struct Slave slave;
struct STATUS_FLAGS status;
struct MONITORING_PARAMS monitoring;
#ifdef Beeper
struct Buzzer buzzer;
#endif
Control *control;



boolean checkValues()
{
  boolean isOk = true;
  if (setpoint.reqBPM < minBPM)  isOk = false;                  // BPM in allowed range ?
  if (setpoint.reqBPM > maxBPM) isOk = false;
  if (setpoint.reqVolume < minVolume) isOk = false;            // Volume in allowed range ?
  if (setpoint.reqVolume > maxVolume) isOk = false;
  if (setpoint.reqPressure < minPressure) isOk = false;  // Compression in allowed range ?
  if (setpoint.reqPressure > maxPressure) isOk = false;
  if (setpoint.reqFiO2 < 0) isOk = false;  // 
  if (setpoint.reqFiO2 > 3) isOk = false;
  if (setpoint.reqI_E_Section < 0) isOk = false;  // 
  if (setpoint.reqI_E_Section > 2) isOk = false;
  if (setpoint.flowTriggerSenstivity < (-0.5f)) isOk = false;  // 
  if (setpoint.flowTriggerSenstivity > 5.0f) isOk = false;
  if (setpoint.reqVentMode < 0) isOk = false;  // 
  if (setpoint.reqVentMode > 4) isOk = false;
  if (isnan(setpoint.reqBPM)) isOk = false;                    // Check for malformed floating point values (NaN)
  if (isnan(setpoint.reqVolume)) isOk = false;
  if (isnan(setpoint.reqPressure)) isOk = false;
  if (isnan(setpoint.reqFiO2)) isOk = false;
  if (isnan(setpoint.reqI_E_Section)) isOk = false;
  if (isnan(setpoint.flowTriggerSenstivity)) isOk = false;
  if (isnan(setpoint.reqVentMode)) isOk = false;


  return isOk;
}

#ifdef Beeper

void beep() // Launch a beep
{
  static unsigned int currentToneFreq = SNOOZE_ALARM;
  static unsigned long t_millis = 0;

  if (buzzer.action == SNOOZE_ALARM)
  {
    noTone(pin_BUZZER);
    currentToneFreq = SNOOZE_ALARM;
    buzzer.toneFreq = SNOOZE_ALARM;
    buzzer.timePeriod = SNOOZE_ALARM;
  }
  else
  {
    if (buzzer.toneFreq > currentToneFreq)
    {
      currentToneFreq = buzzer.toneFreq; //High Severity Alarm Has priority
      t_millis = 0;
    }

    if ((millis() - t_millis) >= buzzer.timePeriod)
    {
      t_millis = millis();
      tone(pin_BUZZER, currentToneFreq, (int)(buzzer.timePeriod / 2)); //Duration in milliseconds
    }    
  }
}
#endif

void eeput() // records to EEPROM (only if values are validated)
{
#ifdef E2PROM
  int eeAddress = eeStart;
  boolean isOk = checkValues();

 // if (n == 1) isOk = true; // override (for debug testing)
  if (isOk)
  {
    // EEPROM.put(eeAddress, setpoint.reqControlVariable);
    // eeAddress += sizeof(uint8_t);
    // EEPROM.put(eeAddress, setpoint.reqAssistMode_F);
    // eeAddress += sizeof(uint8_t);
    // EEPROM.put(eeAddress, setpoint.reqEnableCPAP_F);
    // eeAddress += sizeof(uint8_t);
    EEPROM.put(eeAddress, setpoint.reqVentMode);
    eeAddress += sizeof(uint8_t);
    EEPROM.put(eeAddress, setpoint.reqBPM);
    eeAddress += sizeof(uint8_t);
    EEPROM.put(eeAddress, setpoint.reqVolume);
    eeAddress += sizeof(uint16_t);
    EEPROM.put(eeAddress, setpoint.reqPressure);
    eeAddress += sizeof(uint8_t);
    EEPROM.put(eeAddress, setpoint.reqI_E_Section);
    eeAddress += sizeof(uint8_t);
    EEPROM.put(eeAddress, setpoint.reqFiO2);
    eeAddress += sizeof(uint8_t);
    EEPROM.put(eeAddress, setpoint.flowTriggerSenstivity);
    Serial.println(F("User Settings Saved in EEPROM."));
    
  }
  else {
        Serial.println(F("Unsuccessul saving in EEPROM"));

  }
#endif
}

void eeget()
{
#ifdef E2PROM
  int eeAddress = eeStart;
  EEPROM.get(eeAddress, setpoint.reqVentMode);
  eeAddress += sizeof(uint8_t);
  EEPROM.get(eeAddress, setpoint.reqBPM);
  eeAddress += sizeof(uint8_t);
  EEPROM.get(eeAddress, setpoint.reqVolume);
  eeAddress += sizeof(uint16_t);
  EEPROM.get(eeAddress, setpoint.reqPressure);
  eeAddress += sizeof(uint8_t);
  EEPROM.get(eeAddress, setpoint.reqI_E_Section);
  eeAddress += sizeof(uint8_t);
  EEPROM.get(eeAddress, setpoint.reqFiO2);
  eeAddress += sizeof(uint8_t);
  EEPROM.get(eeAddress, setpoint.flowTriggerSenstivity);
  
  // eeAddress += sizeof(float);
  // EEPROM.get(eeAddress, VolCoeffs);
  // eeAddress += sizeof(VolCoeffs);
  // EEPROM.get(eeAddress, PressCoeffs);
  // eeAddress += sizeof(PressCoeffs);
  
  // Serial.println("Saved Coefficients:");
  // Serial.println("Highest to lowest for 3rd order y=ax^3+bx^2+cx+d where x is volume and y is step in mm. for 3rd order equation");
  // for (int i = 0; i <= ORDER; i++)
  // {
  //       Serial.print(VolCoeffs[i], 5);
  //       Serial.print('\t');
  // }
  // Serial.println();
  boolean isOK = checkValues();
  if (isOK) {
    V_Mode_Breakdown();
    Serial.println(F("Settings Loaded from EEPROM."));
  }
  else {
    setpoint.reqBPM = defaultBPM;
    setpoint.reqVolume = defaultVolume;
    setpoint.reqPressure = defaultPressure;
    setpoint.reqI_E_Section = defaultExpirationRatioIndex;
    setpoint.reqFiO2        = 1;
    setpoint.flowTriggerSenstivity = 0.5;
    setpoint.reqVentMode = VENT_MODE_VCV;
    V_Mode_Breakdown();
    Serial.print(F("Read Default Settings\n")); 
  }
  Serial.print(F("Vent Mode: ")); Serial.println(setpoint.reqVentMode);
  Serial.print(F("CPAPmode: ")); Serial.println(setpoint.reqEnableCPAP_F);
  Serial.print(F("Control Variable: ")); Serial.println(setpoint.reqControlVariable);
  Serial.print(F("Assist Mode Enabled: ")); Serial.println(setpoint.reqAssistMode_F);
  Serial.print(F("req BPM: ")); Serial.println(setpoint.reqBPM);
  Serial.print(F("req Volume: ")); Serial.println(setpoint.reqVolume);
  Serial.print(F("req Pressure: ")); Serial.println(setpoint.reqPressure);
  Serial.print(F("req FiO2: ")); Serial.print(FiO2_Value[setpoint.reqFiO2][0]);Serial.print(F("-")); Serial.println(FiO2_Value[setpoint.reqFiO2][1]);
  Serial.print(F("req flow Trigger: ")); Serial.println(setpoint.flowTriggerSenstivity);
  Serial.print(F("req IE Ratio: ")); Serial.print(IE_R_Value[setpoint.reqI_E_Section][0]);Serial.print(F(":")); Serial.println(IE_R_Value[setpoint.reqI_E_Section][1]);
#else
  setpoint.reqBPM = defaultBPM;
  setpoint.reqVolume = defaultVolume;
  setpoint.reqPressure = defaultPressure;
  setpoint.reqI_E_Section = defaultExpirationRatioIndex;
  setpoint.reqFiO2        = 1;
  setpoint.flowTriggerSenstivity = 0.5;
  setpoint.reqVentMode = VENT_MODE_VCV;
  V_Mode_Breakdown();
  Serial.print(F("Read Default Settings\n")); 
#endif
}

void Timer1ISR()
{
}

//Self Test and Auto Calibrate Routines
void selfTest()
{
  static uint8_t ctr = 0;
  static unsigned long timeElapsedCtr = 0; //multiple of loop time
  status.selfTestStatus = ST_PASS;
  status.selfTestProg   = ST_IN_PROG;

  timeElapsedCtr++;
  if (timeElapsedCtr > 400) //4 seconds
  {
    if (status.Homing_Done_F == 3)
      status.mechIntergrityFaiure = true;
    else
      status.mechIntergrityFaiure = false;

    if (status.Homing_Done_F != 2)
      status.homingFailure = true;
    else
      status.homingFailure = false; 
  }
  
  if (calibStatus != ST_IN_PROG)
  {
      // slave.stopAck = 2;
      // if (slave.stopAck == 0)
      // {
      //   txSlaveCMD(STOP);
      //   slave.lastCMD_ID = STOP;
      //   return;
      // }
      // else if (slave.stopAck == 2) {

      status.Homing_Done_F = slave.homeAck;

      if (status.Homing_Done_F != 2)
      {    
        if (ctr == 0) { 
          //Serial2.print("#HOME 2000"); //2000us
          txSlaveCMD(HOME, 2000);
          slave.lastCMD_ID = HOME; }
        else {ctr++; if (ctr == (1000/samplePeriod1)) ctr = 0;}        

        status.selfTestStatus  = ST_FAIL;
        return;
      }
      else if (status.Homing_Done_F == 2)
      {
        status.homingFailure = false;//HOMING COMPLETE and SUCCESSFUL
      }
    // }
    // else
    // {
    //   Serial.println("Waiting for Motor Stop.");
    //   return;
    // }  
  }

  if (FS.connectionStatus != 0)
  {
    #if defined(SFM3200AW)
    initFlowSensor();
    #endif
    #ifndef TEL_AT_UART0
    Serial.print(F("Flow Sensor Error Code: ")); Serial.println(FS.connectionStatus);  
    #endif

    status.flowSensorFailure = true;
    return;
  }
  else
  {
    status.flowSensorFailure = false;
  }
  

  if (p_sensor.sensorHealth == HEALTH_BAD)
  {
    #ifndef TEL_AT_UART0
    Serial.print(F("Pressure Sensor Error Code: ")); Serial.println(p_sensor.connectionStatus);
    #endif
    status.presSensorFailure = true;
    return;
  }
  else
  {
    status.presSensorFailure = false;
  }


  if (calibStatus != ST_COMPLETE)
  {
    calibStatus = ST_IN_PROG;
    calibrate(calibrationParam);
    return;
  }
  
  status.selfTestProg    = ST_COMPLETE;
  return;
}

void Monitoring()
{
  static bool initInsp = true;
  static bool initHold = true;
  static bool initExp = true;
  static bool initMeasurePEEP = true;
  static float minuteVentilationSum = 0.0;

  static unsigned long T_old_us = millis();

  static unsigned long pre_millis_1min = 0;

  if ((millis() - pre_millis_1min) >= 60000)
  {
    pre_millis_1min = millis();
    TV.minuteVentilation = minuteVentilationSum / 1000.0f;
    minuteVentilationSum = 0.0;
  }


  float delta_t = ((float)(millis() - T_old_us)); //ms

  if (calibStatus == ST_IN_PROG)
  {
    if (estimateVolume)
    {
      TV.measured += (((FS.Q_SLM * 1000.0f) / 60000.0f) * delta_t);
    }
  }
  else
  {
  // Plateau Pressure & PEEP and Set Breathing Flags
    switch (status.breathPhase)
    {
      case INSPIRATION_PHASE:
        if (initInsp) {
          minuteVentilationSum += TV.inspiration;
          TV.inspiration = 0.0; TV.measured = 0.0;
          TV.maxInhale = 0.0;
          monitoring.peakInspPressure = p_sensor.pressure_gauge_CM;
          initInsp = false;
        }
        
        TV.inspiration += (((FS.Q_SLM * 1000.0f) / 60000.0f) * delta_t);
        TV.measured = TV.inspiration;
        if (TV.measured > TV.maxInhale)  
        TV.maxInhale = TV.measured; 


        if (monitoring.peakInspPressure < p_sensor.pressure_gauge_CM)
          monitoring.peakInspPressure = p_sensor.pressure_gauge_CM;

        // reset init Flags
        initHold = true;
        initExp = true;
        initMeasurePEEP = true;
        scanBreathingAttempt = false;
        patientTriggeredBreath = false;
      break;

      case HOLD_PHASE:
        TV.measured += (((FS.Q_SLM * 1000.0f) / 60000.0f) * delta_t);
        if (TV.measured > TV.maxInhale)  
        TV.maxInhale = TV.measured; 


        status.PltPrsValid = true;
        if (initHold)
        {
          monitoring.plateauPressure = (p_sensor.pressure_gauge_CM);
          initHold = false;
        }
        else
        {
          monitoring.plateauPressure = (monitoring.plateauPressure + (p_sensor.pressure_gauge_CM)) * 0.5;
        }
      
        TV.staticCompliance = (TV.inspiration / (monitoring.plateauPressure - monitoring.PEEPressure));

        // reset init Flags
        initInsp = true;
        initExp = true;
        initMeasurePEEP = true;
        scanBreathingAttempt = false;
        patientTriggeredBreath = false;
        break;
      case EXPIRATION_PHASE:
        if (initExp) {TV.expiration = 0.0; initExp = false;}
        TV.expiration += (-1.0) * (((FS.Q_SLM * 1000.0f) / 60000.0f) * delta_t);      
        TV.measured += (((FS.Q_SLM * 1000.0f) / 60000.0f) * delta_t);
        if (TV.measured > TV.maxInhale)  
        TV.maxInhale = TV.measured; 

        if (setpoint.curAssistMode_F == 1) {
          if (scanBreathingAttempt)
          {
            if (FS.Q_SLM >= setpoint.flowTriggerSenstivity) {
            patientTriggeredBreath = true; }          
          }
        }
        if (PEEPMesaureFlag)
        {
          status.PeepValid = true;
          if (initMeasurePEEP)
          {
            monitoring.PEEPressure = (p_sensor.pressure_gauge_CM);
            initMeasurePEEP = false;
          }
          else
          {
            monitoring.PEEPressure = (monitoring.PEEPressure + (p_sensor.pressure_gauge_CM)) * 0.5;
          }
  //        TV.staticCompliance = (TV.inspiration / (monitoring.plateauPressure - monitoring.PEEPressure));
        }

        // reset init Flags
        initInsp = true;
        initHold = true;
        break;
      default: //WAIT PHASE
        // reset init Flags
        initInsp = true;
        initHold = true;
        initExp = true;
        initMeasurePEEP = true;
        scanBreathingAttempt = false;
        patientTriggeredBreath = false;
        break;
    }
  }

  T_old_us = millis();

#ifdef QT_PLOTTER
if (plotterData) {
  Serial.print("$");
  Serial.print(FS.Q_SLM, 5);
  Serial.print(" ");
  Serial.print(TV.measured, 5);
  Serial.print(" ");
//  Serial.print(TV.inspiration, 5);
//  Serial.print(" ");
//  Serial.print(TV.expiration, 5);
//  Serial.print(" ");
  Serial.print(p_sensor.pressure_gauge_CM, 5);

  Serial.print(" ");
  Serial.print(control->valuePredicted,5);
  Serial.print(" ");
  Serial.print(control->error, 5);
  Serial.print(" ");
  Serial.print((float)actualBreathCycleTime/1000.0f, 5);
  // Serial.print(" ");
  // Serial.print(status.breathPhase);
  // Serial.print(" ");
  // Serial.print(delta_t, 5);
  Serial.print(";\n");
}
#endif


}


#ifdef Beeper

/*
     This is the main Alarm Control.
        Sensor Monitoring.
        Alarm Triggering.
*/
void alarmControlOld() // Read Values from Installed Sensor
{

  #define COUNT_10ms  10/samplePeriod1
  #define COUNT_50ms  50/samplePeriod1
  #define COUNT_100ms 100/samplePeriod1

  //1 count = 10ms
  static uint8_t HighPeakPressAlarmCnt = 0;
  static uint8_t HighPltPressAlarmCnt = 0;
  static uint8_t LowPEEPAlarmCnt = 0;

  //ADD Power Related Alarms here
  // Low Battery
  // Battery Power In Use  

  if ((status.VentilatorOperationON == 1) && (status.breathPhase != WAIT_PHASE)) //Ventilation Related Alarms
  {
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////FAULT DETECTION////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////


    if (p_sensor.pressure_gauge_CM > (setpoint.curPressure)) //Pressure Setpoint in Control Loop (User Input Based)
    {
      HighPeakPressAlarmCnt++;
    } else HighPeakPressAlarmCnt--;

    if (status.PltPrsValid == true) //To Prevent False Alarms
    {
      if (monitoring.plateauPressure > (35.0)) //FOR COVID-19 : NHS Requirement
      {
        HighPltPressAlarmCnt++;
      } else HighPltPressAlarmCnt--;
    } else HighPltPressAlarmCnt = 0;


    if (status.PeepValid == true) //To Prevent False Alarms
    {
      if (monitoring.PEEPressure < (5.0)) //FOR COVID-19 : NHS Requirement
      {
        LowPEEPAlarmCnt++;
      } else LowPEEPAlarmCnt--;
    } else LowPEEPAlarmCnt = 0;

    
    ////////////////////////////////////////////////////////////////////////////////
    /////////////                   SET ALARMS                      ////////////////
    /////////////Order: LOW SEVERITY (TOP) -> HIGH SEVERITY (BOTTOM)///////////////
    ///////////////////////////////////////////////////////////////////////////////

  //HIGH SEVERITY ALARMS HAVE PREFEREENCE
    if (buzzer.toneFreq <= SEVERITY_LOW_FREQ) //ONLY RING IF PREVIOUSLY MUTE
    {
      if (HighPltPressAlarmCnt >= ((int)(COUNT_100ms))) //10 loop counts = 100ms;
      {
        buzzer.action = RING_ALARM;
    //    buzzer.toneFreq = SEVERITY_HIGH_FREQ;
    //    buzzer.timePeriod = SEVERITY_HIGH_TP;
        buzzer.toneFreq = SEVERITY_LOW_FREQ; //TESTING ONLY
        buzzer.timePeriod = SEVERITY_LOW_TP; //TESTING ONLY
      }
    }

    if (buzzer.toneFreq <= SEVERITY_MED_FREQ) //ONLY RING IF PREVIOUSLY MUTE OR PREVIOUS ALARM IS LOWER SEVERITY
    {
      if (HighPltPressAlarmCnt >= ((int)(COUNT_100ms))) //10 loop counts = 100ms;
      {
        buzzer.action = RING_ALARM;
        buzzer.toneFreq = SEVERITY_MED_FREQ;
        buzzer.timePeriod = SEVERITY_MED_TP;
      }
    }
    
    if (buzzer.toneFreq <= SEVERITY_HIGH_FREQ)  //ONLY RING IF PREVIOUSLY MUTE OR PREVIOUS ALARM IS LOWER SEVERITY
    {
      if (LowPEEPAlarmCnt >= ((int)(COUNT_100ms))) //10 loop counts = 100ms;
      {
        buzzer.action = RING_ALARM;
        buzzer.toneFreq = SEVERITY_HIGH_FREQ;
        buzzer.timePeriod = SEVERITY_HIGH_TP;
      }
    }
  }
  else
  {
    HighPeakPressAlarmCnt = 0;
    HighPltPressAlarmCnt = 0;
    LowPEEPAlarmCnt = 0;

    status.PeepValid = false; //To Prevent False Alarms
    status.PltPrsValid = false; //To Prevent False Alarms
  }
}
#endif
void setup()
{
  // put your setup code here, to run once:
#ifdef EN_WATCHDOG
  wdt_disable();
#endif

  unsigned long pre_millis = 0;
  bool initWarmUp = true;

  // pinMode(pin_Button_OK, INPUT);
  // pinMode(pin_Button_SNZ, INPUT);
  pinMode(pin_Switch_START, INPUT);
  // pinMode(pin_Switch_MODE, INPUT);

  //    pinMode(pin_LmtSWT_CL1, INPUT);
  //    pinMode(pin_LmtSWT_CL2, INPUT);

  pinMode(pin_Knob_1, INPUT);
  pinMode(pin_Knob_2, INPUT);
  pinMode(pin_Knob_3, INPUT);
  pinMode(pin_Knob_4, INPUT);
#ifdef Beeper
  pinMode(pin_BUZZER, OUTPUT);
#endif
  pinMode(pin_SLAVE_RESET, OUTPUT);

  calibStatus = ST_COMPLETE;//ST_NOT_INIT;//ST_COMPLETE;
  calibrationParam = PRESS_CONT_MODE;

//  Timer1.initialize(200);
//  Timer1.attachInterrupt(Timer1ISR);

// Timer3: used for Monitoring & Alarms Management

  //    noInterrupts();
  pre_millis = millis();
  while ((millis() - pre_millis) < WARM_UP_TIME)
  {
    if (initWarmUp) {
      initWarmUp = false;
      status.WarmUpFlag = true;
      Serial.begin(SERIAL_BAUD);
      Serial2.begin(SERIAL_BAUD);
      #ifndef TEL_AT_UART0
      #ifdef TX_SERIAL_TELEMETRY
    //     Serial1.begin(SERIAL_BAUD);
      #endif
      #endif
      Serial1.begin(SERIAL_BAUD);
      
      Wire.begin();
      
      LCD_setup();
      Display();
      
      initFlowSensor();
      digitalWrite(pin_SLAVE_RESET, LOW);
      delay(5000);
      digitalWrite(pin_SLAVE_RESET, HIGH);
      delay(5000);
    }
  }
  status.WarmUpFlag = false;

  //    interrupts();

  noInterrupts();
  monitoring.plateauPressure = 0.0;
  monitoring.PEEPressure = 0.0;
  monitoring.peakInspPressure = 0.0;

  // reserve 50 bytes for the inputString:
  slave.AckStr.reserve(50);

  eeget();    // read startup parameters (either from EEPROM or default value)
//  eeput();
//  eeget();    // read startup parameters (either from EEPROM or default value)
  
  setpoint.curBPM = setpoint.reqBPM;                 // Start from these values without sweep
  setpoint.curVolume = setpoint.reqVolume;
  setpoint.curPressure = setpoint.reqPressure;
  setpoint.curI_E_Section = setpoint.reqI_E_Section; 
  setpoint.curI_E = IE_R_Value[setpoint.curI_E_Section][1]; //Exhale Factor
  setpoint.curFiO2 = setpoint.reqFiO2; //Exhale Factor
  setpoint.curVentMode = setpoint.reqVentMode;
  setpoint.curAssistMode_F = setpoint.reqAssistMode_F;
  setpoint.curEnableCPAP_F = setpoint.reqEnableCPAP_F;
  setpoint.curControlVariable = setpoint.reqControlVariable;

  InitializeParams();

#ifdef CLOSED_LOOP
  control =new Control();
  float deadBand = 20; //deadBand to stop adjustment.
// Without Lung (Zero Resistance)
  // VolCoeffs[0] = 0.00000003;
  // VolCoeffs[1] = -0.00005837;
  // VolCoeffs[2] = 0.07054022;
  // VolCoeffs[3] = 0.65576958;

//0.00000004      -0.00009575     0.11151478      -0.31410825  
//0.00000007      -0.00012079     0.10933197      1.79025626 //200430 -With test lung -Damn good
//0.00000003	-0.00005732	0.07018520	0.76970458	
// With Nominal Resistance Test Lung
  VolCoeffs[0] = 0.00000003;
  VolCoeffs[1] = -0.00005732;
  VolCoeffs[2] = 0.07018520;
  VolCoeffs[3] = 0.76970458;

  if (setpoint.reqControlVariable == VOL_CONT_MODE)
    control->setConstants(0.8,0.1,deadBand,1200); //values send in this function are needed to be tested.
  else //PRESS_CON_MODE
    control->setConstants(0.8,0.1,1,70);
    //Set_constant initialization for pressure control goes here
  // Distance: 40.00mm
  //Pressure: 23.85cmH2O
  //Pressure Equation 200504 //Nominal Resistance Test Lung
  // -0.00221921	0.10285785	0.33219816	3.53803133	////Nominal Resistance Test Lung //PEEP = 5cmH2O

 // -0.00258854	0.11181313	0.41835732	1.68697428    ///Nominal Resistance Test Lung //PEEP = 5cmH2O	
 
// -0.00025210     0.01868789      0.43968305      -0.32840135 //With Standard Test Lung PEEP= 20cmH2O
  PressCoeffs[0] = -0.00025210;
  PressCoeffs[1] = 0.01868789;
  PressCoeffs[2] = 0.43968305;
  PressCoeffs[3] = -0.32840135;
#endif
  interrupts();

  alarmsSetup();

#ifdef EN_WATCHDOG
  wdt_enable(WDTO_8S);
#endif

}

void Ventilator_Control()
{
  static boolean skip = true;
  static boolean initIns = true;
  static boolean initHld = true;
  static boolean initExp = true;
  static boolean initWait = true;
  static boolean runMotor = true;
  static boolean init = true;
  static boolean skipStep=1;

  static uint16_t breathLength = 0;      // duration of the current breathing cycle in milliseconds. This is 60000/BPM.
  static unsigned int Tin = 0;
  static unsigned int Tex = 0;
  static unsigned int Th = 0; //ms
  static unsigned int Ttrigger = 500; //ms
  static uint16_t Tcur = 0;
  static unsigned long BreathStartTimestamp = 0;
  static unsigned long cycleStartTimestamp = millis();

  static float reqMotorPos = 0.0; //mm
  /*static */float Vin = 0.0; //mm/s
  /*static */float Vex = 0.0;  //mm/s
  /*static */float RPMin   = 0.0;
  /*static */float RPMex   = 0.0;

  static uint16_t stepIn = 0;
  static uint16_t stepEx = 0;
  static uint16_t periodIn = 0; //us
  static uint16_t periodEx = 0; //us

  static float stepsPredicted = 0.0;


  //    noInterrupts();

  if (init)
  {
    setpoint.curBPM = setpoint.reqBPM;                 // Start from these values without sweep
    setpoint.curVolume = setpoint.reqVolume;
    setpoint.curPressure = setpoint.reqPressure;
    setpoint.curI_E_Section = setpoint.reqI_E_Section; 
    setpoint.curI_E = IE_R_Value[setpoint.curI_E_Section][1]; //Exhale Factor
    setpoint.curFiO2 = setpoint.reqFiO2; //Exhale Factor
    setpoint.curAssistMode_F = setpoint.reqAssistMode_F;
    setpoint.curEnableCPAP_F = setpoint.reqEnableCPAP_F;
    setpoint.curControlVariable = setpoint.reqControlVariable;

    breathLength = (int)(60000 / setpoint.curBPM);
    // Take the hold time out of the exhale cycle. Do this to ensure respitory rate is correct.
//    Tex = (int)((breathLength - Th) / (1 + setpoint.curI_E)); // if I/E ratio = 0.5 ; it means expiration is twice as long as inspiration
//    Tin = (int)(Tex * setpoint.curI_E);
    if (holdManeuver) Th = holdDur_ms; else Th = 0;

    slave.runAck = 0;
    init = false;
    Tcur = breathLength;
  }

  if ((activateVentilatorOperation || activateVentilatorControlSerial) && (status.sensorsHealth == HEALTH_GOOD)) 
  {
    status.VentilatorOperationON = 1;
    initWait = true;
  }
  else
    status.VentilatorOperationON = 0;

  if ((((slave.lastCMD_ID == HOME) && (slave.homeAck != 2)) || (status.homeAtBadSensor)) && (!skip))
  {
    status.VentilatorOperationON = 0;
    Serial.println(slave.lastCMD_ID);
    Serial.println(slave.homeAck);
    Serial.println(F("Homing Not Complete"));
  }
  

  if (status.VentilatorOperationON == 1)
  {
    skip = false;
//    status.VentilatorOperationON = 1;
    initWait = true;

    if (setpoint.curAssistMode_F == 1) {
      if (patientTriggeredBreath)
      {
        Tcur = breathLength; //Start a new inhale Cycle 
      }
    }
    

    if (Tcur >= breathLength)
    {
      Tcur = 0;
      setpoint.curAssistMode_F = setpoint.reqAssistMode_F;
      setpoint.curEnableCPAP_F = setpoint.reqEnableCPAP_F;
      if (setpoint.reqControlVariable != setpoint.curControlVariable)
      {
        setpoint.curControlVariable = setpoint.reqControlVariable;
        if (setpoint.curControlVariable == VOL_CONT_MODE){
          control->setConstants(0.8,0.1,20,1200);
          control->resetController((float)setpoint.curVolume);
        }
        else if(setpoint.curControlVariable == PRESS_CONT_MODE){
          control->setConstants(0.8,0.1,1,70);
          control->resetController((float)setpoint.curPressure);
        }
        skipStep=1;
      }
      if (abs(setpoint.curVolume - setpoint.reqVolume) >= 1) {
        setpoint.curVolume = setpoint.reqVolume;
        if (setpoint.curControlVariable == VOL_CONT_MODE) {
          control->resetController((float)setpoint.curVolume);
          skipStep=1;}
      }
      if (abs(setpoint.curPressure - setpoint.reqPressure) >= 1) {
        setpoint.curPressure = setpoint.reqPressure;
        if(setpoint.curControlVariable == PRESS_CONT_MODE) {
          control->resetController((float)setpoint.curPressure);
          skipStep=1; }
      }
      setpoint.curBPM = setpoint.reqBPM;                 // Load Fresh User Settings
      setpoint.curI_E_Section = setpoint.reqI_E_Section; 
      setpoint.curI_E = IE_R_Value[setpoint.curI_E_Section][1]; //Exhale Factor


      breathLength = (int)(60000 / setpoint.curBPM);
      if (holdManeuver) Th = holdDur_ms; else Th = 0;
      // Take the hold time out of the exhale cycle. Do this to ensure respitory rate is correct.
      Tin = (int)((breathLength - Th) / (1 + setpoint.curI_E)); // if I/E ratio = 0.5 ; it means expiration is twice as long as inspiration
      Tex = (int)(breathLength - Th - Tin);
      if (setpoint.curControlVariable == VOL_CONT_MODE) {

     if(!skipStep){
//      stepsPredicted = control->compensateError((float)setpoint.curVolume,TV.inspiration);    
      stepsPredicted = control->compensateError((float)setpoint.curVolume,TV.maxInhale);    
     }
      else{
      stepsPredicted= setpoint.curVolume;  
   skipStep=0;
     }
//        reqMotorPos = setpoint.curVolume / LINEAR_FACTOR_VOLUME; //mm
        reqMotorPos = (VolCoeffs[0] * pow(stepsPredicted, 3)) + (VolCoeffs[1] * pow(stepsPredicted, 2)) + (VolCoeffs[2] * stepsPredicted) + VolCoeffs[3];
       // reqMotorPos = stepsPredicted;
       reqMotorPos = 30.0;
        reqMotorPos = constrain(reqMotorPos, 0.0, 40.0);
      }
      else //PRESS_CONT_MODE 
      {
        
      if(!skipStep){
        stepsPredicted = control->compensateError((float)setpoint.curPressure, monitoring.plateauPressure);
      }
      else{
        stepsPredicted= setpoint.curPressure;  
        skipStep=0;
      }
        reqMotorPos = (PressCoeffs[0] * pow(stepsPredicted, 3)) + (PressCoeffs[1] * pow(stepsPredicted, 2)) + (PressCoeffs[2] * stepsPredicted) + PressCoeffs[3];
       // reqMotorPos = stepsPredicted;
        reqMotorPos = constrain(reqMotorPos, 0.0, 40.0);
      }
        Vin = reqMotorPos / ((float)Tin / 1000.0f); // mm/s
        Vex = reqMotorPos / ((float)Tex / 1000.0f); // mm/s
        RPMin = (Vin / LIN_MECH_mm_per_rev) * 60.0;
        RPMex = (Vex / LIN_MECH_mm_per_rev) * 60.0;
        stepIn = (long)((reqMotorPos / LIN_MECH_mm_per_rev) * STEPPER_MICROSTEP * STEPPER_PULSES_PER_REV);
        stepEx = (long)(stepIn + ((2.0 / LIN_MECH_mm_per_rev) * STEPPER_MICROSTEP * STEPPER_PULSES_PER_REV));
        periodIn = (long)((((float)Tin / 1000.0) / stepIn) * 1000000); //us
        periodEx = (long)((((float)Tex / 1000.0) / stepIn) * 1000000); //us
        BreathStartTimestamp = millis();

//#ifdef __DEBUG
#ifndef TEL_AT_UART0
          static int i = 0;
           Serial.print(F("In Ventilator Control: ")); Serial.println(i++);
           Serial.print(F("Control Mode:      ")); Serial.println(setpoint.curControlVariable); //0 = VOL; 1 = PRESS          
           Serial.print(F("Breathing Length:      ")); Serial.println(breathLength);
           Serial.print(F("Inspiration Time:      ")); Serial.print(Tin); Serial.println(F(" ms"));
           Serial.print(F("Expiration Time:       ")); Serial.print(Tex); Serial.println(F(" ms"));
           Serial.print(F("targetPosition:        ")); Serial.print(reqMotorPos); Serial.println(F(" mm"));
           Serial.print(F("Motor Speed Insp:      ")); Serial.print(Vin); Serial.println(F(" mm/s"));
           Serial.print(F("Motor Speed Exp:       ")); Serial.print(Vex); Serial.println(F(" mm/s"));
           Serial.print(F("RPM Insp:              ")); Serial.println(RPMin);
           Serial.print(F("RPM Exp:               ")); Serial.println(RPMex);
           Serial.print(F("Steps Insp:            ")); Serial.println(stepIn);
           Serial.print(F("Steps Exp:             ")); Serial.println(stepEx);
           Serial.print(F("Period Insp:           ")); Serial.print(periodIn); Serial.println(F(" us"));
           Serial.print(F("Period Exp:            ")); Serial.print(periodEx); Serial.println(F(" us"));
#endif           
           
//#endif
    }
    Tcur = millis() - BreathStartTimestamp;

      if (Tcur <= Tin)
      {
        if (initIns)
        {
//          slave.runAck = 0;
          runMotor = true;
          initHld = true;
          initIns = false;
          initExp = true;
        }        
//        status.breathPhase = INSPIRATION_PHASE;
        if (runMotor && (slave.runAck == 0 || slave.runAck == 2)) //!setpointAchieved && //CMD NOT RECEIVED
        {
          actualBreathCycleTime = millis() - cycleStartTimestamp;
          status.breathPhase = INSPIRATION_PHASE;
          txSlaveCMD(RUN, periodIn, stepIn, "1");
          runMotor = false;
          slave.lastCMD_ID = RUN;
          cycleStartTimestamp = millis();
        }
        PEEPMesaureFlag = false;
      }
      else if ((Tcur > Tin) && (Tcur <= (Tin + Th)))
      {
        if (initHld)
        {
         slave.runAck = 2;
         txSlaveCMD(STOP);
          slave.lastCMD_ID = STOP;
//          slave.stopAck = 0;
          initHld = false;
          initIns = true;
          initExp = true;
        }        
        status.breathPhase = HOLD_PHASE;
      }
      else if ((Tcur > (Tin + Th)) && (Tcur < (Tin + Th + Tex)))
      {
        if (initExp)
        {
         // slave.runAck = 0;
          runMotor = true;
          initHld = true;
          initIns = true;
          initExp = false;
        }                
//        status.breathPhase = EXPIRATION_PHASE;

        if (runMotor && (slave.runAck == 0 || slave.runAck == 2)) //CMD NOT RECEIVED
        {
          status.breathPhase = EXPIRATION_PHASE;
          txSlaveCMD(RUN, periodEx, stepEx, "0");
          runMotor = false;
          slave.lastCMD_ID = RUN;
        }

        if ((Tcur >= (Tin + Tex)) && (Tcur < (Tin + Th + Tex)))
        {
          PEEPMesaureFlag = true;
        }

        if (setpoint.curAssistMode_F == 1) {
          if ((Tin + Th + Tex - Tcur) < Ttrigger) 
          {
            scanBreathingAttempt = true;
          }        
        }
      }
  }
  else
  {
    static uint8_t ctr = 0;
    #ifndef TEL_AT_UART0
          Serial.println(F("Ventilator Operation Halt"));
    #endif
    if (initWait) {
      ctr = 0;
      slave.homeAck = 0; slave.stopAck = 0; 
      txSlaveCMD(STOP);
      slave.lastCMD_ID = STOP;

      if (status.sensorsHealth == HEALTH_BAD)
        status.homeAtBadSensor = true;
      if (p_sensor.sensorHealth == HEALTH_BAD)
        status.homeAtBadPressSensor = true;
      if (FS.sensorHealth == HEALTH_BAD)
        status.homeAtBadFlowSensor = true;
      initWait = false;}
      ctr++;
    initHld = true;
    initIns = true;
    initExp = true;
    //status.VentilatorOperationON = 0;
    status.breathPhase = WAIT_PHASE;
    Tcur = breathLength; // This will always start inspiration breath cycle on setting switch to start position
    status.PeepValid = false; //To Prevent False Alarms
    status.PltPrsValid = false; //To Prevent False Alarms

    // if (slave.stopAck == 0)
    // {
    //   txSlaveCMD(STOP);
    //   slave.lastCMD_ID = STOP;
    // }
    if ((slave.homeAck == 0) && (ctr >= 5))
    {
        txSlaveCMD(HOME, 2000);
        slave.lastCMD_ID = HOME;

	    slave.runAck = 0;
    }
    else if (slave.homeAck == 2) {
      status.homeAtBadSensor = false;
      status.homeAtBadPressSensor = false;
      status.homeAtBadFlowSensor = false;
    }
    PEEPMesaureFlag = false;
  }
  //    interrupts();
}

void loop()
{
  static float timeSincePowerUp = 0.0;
  static unsigned long tick1 = 0;            // counter used to trigger sensor measurements //loop 1
  static unsigned long tick2 = 0;             // counter used to trigger Control Loop // loop 2
  static unsigned long tick3 = 0;

  unsigned long start_Ts = 0;

  #ifdef EN_WATCHDOG
    wdt_reset();
  #endif

  if (millis() > (tick1 + samplePeriod1))
  {
    timeSincePowerUp = ((float)(millis() - tick1) / 1000.0f);
    tick1 = millis();

    if (timeSincePowerUp >= 345600.0) // 96 * 60 * 60
      status.hours96_complete = true;
    else if (timeSincePowerUp >= 10.0)
      status.systemReset = false;

    //start_Ts = micros();
    readSensors();
    checkSensorHealth();
    Monitoring();

    if (status.selfTestProg != ST_COMPLETE) {
      selfTest();
    }

    start_Ts = micros();
    alarmControl();
    Serial.print(F("Busy Time alarmsControl: ")); Serial.println(micros()-start_Ts);

#ifdef Beeper
    beep(); //alarmAction = RING_ALARM, SNOOZE_ALARM; alarmSeverity = SEVERITY_HIGH, SEVERITY_MED, SEVERITY_LOW, SEVERITY_MUTE
#endif
#ifdef TX_SERIAL_TELEMETRY
    GetTelData(); //Called at 100Hz
    Prepare_Tx_Telemetry(); //Called at 100Hz
#endif
//Serial.print(F("Busy Time 1: ")); Serial.println(micros()-start_Ts);
  }

  if ((status.selfTestProg == ST_COMPLETE) && (status.selfTestStatus == ST_PASS)) // I am not writing control loop inside 100Hz loop to keep both loop rates independant
  {
    if (millis() > (tick2 + samplePeriod2))
    {
      //start_Ts = micros();
      tick2 = millis();
      Ventilator_Control();
//Serial.print(F("Busy Time 2: ")); Serial.println(micros()-start_Ts);
    }
  }
  if (millis() > (tick3 + 500))
  {
    tick3 = millis();
    readSwitches();
     start_Ts = micros();
     Display();
     Serial.print(F("Busy Time Display: ")); Serial.println(micros()-start_Ts);
  }
}

#ifndef TX_SERIAL_TELEMETRY
void serialEvent1()
{
  processCommands();
}
#endif

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent2() {
  receiveSlaveTel();
}
