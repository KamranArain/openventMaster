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
#include "drivers/flowSensor.h"
#include "control.h"
#include "userInterface.h"
#include "command.h"
#include "alarms.h"
#include "sensors.h"

#ifdef EN_WATCHDOG
#ifndef __AVR__
#include <avr/wdt.h> // for avr
#else
// #include "drivers/stm32_wwdg.h" //for stm32/
#endif
#endif

#ifdef STM32F4xx
HardwareSerial Serial1(USART3_RX, USART3_TX);
HardwareSerial Serial3(USART6_RX, USART6_TX);
HardwareSerial Serial4(UART4_RX, UART4_TX);
#endif

#ifdef TX_SERIAL_TELEMETRY
#include "telemetry.h"
#endif

extern double VolCoeffs[ORDER + 1];
extern double PressCoeffs[ORDER_PRESS_EQ + 1];

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
bool initAssistedBreath = false;
bool holdManeuver = true;
uint16_t holdDur_ms = 150;
uint16_t breathLength = (int)(60000 / defaultBPM); // duration of the current breathing cycle in milliseconds. This is 60000/BPM.
uint32_t actualBreathCycleTime = 0;
//int triggerVariable = FLOW_VAR;
//int cyclingVariable = TIME_VAR;

uint16_t measuredRR = 0;
uint16_t breathCtr = 0;

float maxNegativeFlow = 0.0;
float maxInspFlow = 0.0;

#ifdef TX_SERIAL_TELEMETRY
extern struct TEL_TYPE TEL;
#endif
#ifdef FS6122
extern struct FS6122_TYPE fs6122;
#endif
extern struct Flow_Sensor FS;
extern struct P_Sensor p_sensor;
extern struct O2_Sensor O2;
extern struct ALARMS Alarms;

struct setpointStatus setpoint;
struct TidalVolume TV;
struct Slave slave;
struct STATUS_FLAGS status;
struct MONITORING_PARAMS monitoring;
Control *control;

bool checkValues()
{
  bool isOk = true;
  if (setpoint.reqBPM < minBPM)
    isOk = false; // BPM in allowed range ?
  if (setpoint.reqBPM > maxBPM)
    isOk = false;
  if (setpoint.reqVolume < minVolume)
    isOk = false; // Volume in allowed range ?
  if (setpoint.reqVolume > maxVolume)
    isOk = false;
  if (setpoint.reqPressure < minPressure)
    isOk = false; // Compression in allowed range ?
  if (setpoint.reqPressure > maxPressure)
    isOk = false;
  if (setpoint.reqFiO2 < 0)
    isOk = false; //
  if (setpoint.reqFiO2 > 3)
    isOk = false;
  if (setpoint.reqI_E_Section < 0)
    isOk = false; //
  if (setpoint.reqI_E_Section > 2)
    isOk = false;
  if (setpoint.flowTriggerSenstivity < (0.5f))
    isOk = false; //
  if (setpoint.flowTriggerSenstivity > 5.0f)
    isOk = false;
  if (setpoint.reqVentMode < 0)
    isOk = false; //
  if (setpoint.reqVentMode > 4)
    isOk = false;
  if (isnan(setpoint.reqBPM))
    isOk = false; // Check for malformed floating point values (NaN)
  if (isnan(setpoint.reqVolume))
    isOk = false;
  if (isnan(setpoint.reqPressure))
    isOk = false;
  if (isnan(setpoint.reqFiO2))
    isOk = false;
  if (isnan(setpoint.reqI_E_Section))
    isOk = false;
  if (isnan(setpoint.flowTriggerSenstivity))
    isOk = false;
  if (isnan(setpoint.reqVentMode))
    isOk = false;

  return isOk;
}

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
    // Serial.println(F("User Settings Saved in EEPROM."));
  }
  else
  {
    // Serial.println(F("Unsuccessul saving in EEPROM"));
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
  if (isOK)
  {
    V_Mode_Breakdown();
    // Serial.println(F("Settings Loaded from EEPROM."));
  }
  else
  {
    setpoint.reqBPM = defaultBPM;
    setpoint.reqVolume = defaultVolume;
    setpoint.reqPressure = defaultPressure;
    setpoint.reqI_E_Section = defaultExpirationRatioIndex;
    setpoint.reqFiO2 = defaultFIO2Index;
    setpoint.flowTriggerSenstivity = 0.5;
    setpoint.reqVentMode = VENT_MODE_VCV;
    V_Mode_Breakdown();
    // Serial.print(F("Read Default Settings\n"));
  }
  // Serial.print(F("Vent Mode: ")); Serial.println(setpoint.reqVentMode);
  // Serial.print(F("CPAPmode: ")); Serial.println(setpoint.reqEnableCPAP_F);
  // Serial.print(F("Control Variable: ")); Serial.println(setpoint.reqControlVariable);
  // Serial.print(F("Assist Mode Enabled: ")); Serial.println(setpoint.reqAssistMode_F);
  // Serial.print(F("req BPM: ")); Serial.println(setpoint.reqBPM);
  // Serial.print(F("req Volume: ")); Serial.println(setpoint.reqVolume);
  // Serial.print(F("req Pressure: ")); Serial.println(setpoint.reqPressure);
  // Serial.print(F("req FiO2: ")); Serial.print(FiO2_Value[setpoint.reqFiO2][0]);Serial.print(F("-")); Serial.println(FiO2_Value[setpoint.reqFiO2][1]);
  // Serial.print(F("req flow Trigger: ")); Serial.println(setpoint.flowTriggerSenstivity);
  // Serial.print(F("req IE Ratio: ")); Serial.print(IE_R_Value[setpoint.reqI_E_Section][0]);Serial.print(F(":")); Serial.println(IE_R_Value[setpoint.reqI_E_Section][1]);
#else
  setpoint.reqBPM = defaultBPM;
  setpoint.reqVolume = defaultVolume;
  setpoint.reqPressure = defaultPressure;
  setpoint.reqI_E_Section = defaultExpirationRatioIndex;
  setpoint.reqFiO2 = defaultFIO2Index;
  setpoint.flowTriggerSenstivity = 0.5;
  setpoint.reqVentMode = VENT_MODE_VCV;
  V_Mode_Breakdown();
  // Serial.print(F("Read Default Settings\n"));
#endif
}

// void Timer1ISR()
// {
// }

//Self Test and Auto Calibrate Routines
void selfTest()
{
  static uint8_t ctr = 0;
  static unsigned long timeElapsedCtr = 0; //multiple of loop time
  status.selfTestStatus = ST_PASS;
  status.selfTestProg = ST_IN_PROG;

  static int tempInit = 1;
  if (tempInit == 1)
  {
    tempInit = 0;
    uint32_t steps = (long)((1.0 / LIN_MECH_mm_per_rev) * STEPPER_MICROSTEP * STEPPER_PULSES_PER_REV);
    txSlaveCMD(RUN, HOMING_PERIOD, steps, "1");
    slave.lastCMD_ID = RUN;
  }
  if (slave.runAck != 2)
  {
    status.selfTestStatus = ST_FAIL;
    return;
  }

  timeElapsedCtr++;

  if (timeElapsedCtr > 1000) //4 seconds
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

    // Serial.println("HA");Serial.println(status.Homing_Done_F);
    // Serial.println("RA");Serial.println(slave.runAck);
    if (status.Homing_Done_F == 0)
    {
      if (ctr == 0)
      {
        //Serial2.print("#HOME 2000"); //2000us
        txSlaveCMD(HOME, HOMING_PERIOD);
        slave.lastCMD_ID = HOME;
      }
      else
      {
        if (ctr == (1000 / samplePeriod1))
          ctr = 0;
      }
      ctr++;

      status.selfTestStatus = ST_FAIL;
      return;
    }
    else if (status.Homing_Done_F == 1)
    {
      status.selfTestStatus = ST_FAIL;
      return;
    }
    else if (status.Homing_Done_F == 2)
    {
      status.homingFailure = false; //HOMING COMPLETE and SUCCESSFUL
    }
    // else if (status.Homing_Done_F == 2)
    // {
    //   status.selfTestStatus  = ST_FAIL;
    //   if (slave.runAck == 0 && slave.lastCMD_ID == HOME) {
    //     // move forward 3.0mm
    //     uint32_t steps = (long) ((3.0 / LIN_MECH_mm_per_rev) * STEPPER_MICROSTEP * STEPPER_PULSES_PER_REV);
    //     txSlaveCMD(RUN, HOMING_PERIOD, steps, "1");
    //     slave.lastCMD_ID = RUN;
    //     return;
    //   }
    //   else if (slave.lastCMD_ID == RUN & slave.runAck == 1) {
    //     return;
    //   }
    //   else if (slave.lastCMD_ID == RUN & slave.runAck == 2) {
    //     status.Homing_Done_F = 0;
    //     slave.homeAck = 0;
    //     ctr = 0;
    //     return;
    //   }
    //   else {
    //     status.selfTestStatus  = ST_PASS;
    //     status.homingFailure = false;//HOMING COMPLETE and SUCCESSFUL
    //   }
    // }

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
// Serial.print(F("Flow Sensor Error Code: ")); Serial.println(FS.connectionStatus);
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
// Serial.print(F("Pressure Sensor Error Code: ")); Serial.println(p_sensor.connectionStatus);
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

  status.selfTestProg = ST_COMPLETE;
  return;
}

void Monitoring()
{
  static bool initInsp = true;
  static bool initHold = true;
  static bool initExp = true;
  static bool initOperations = true;
  static bool initMeasurePEEP = true;
  static float minuteVentilationSum = 0.0;
  // static float Q_old = 0.0;

  static float peakPrs = 0.0;
  static float pltPrs = 0.0;
  static float peep = 0.0;
  static float TVin = 0.0;
  static float TVex = 0.0;
  static float TVmax = 0.0;
  static float maxInspFlow_6122 = 0.0;
  static float maxExpFlow_6122 = 0.0;
  static bool negativeFlowDetected = true;
  static int16_t ctrNegFlow = 0;

  static float TVmeas_FS6122 = 0.0;

  static uint8_t lastBreathingPhase = WAIT_PHASE;

  static unsigned long T_old_us = millis();

  static unsigned long pre_millis_1min = 0;

  if ((millis() - pre_millis_1min) >= 60000)
  {
    pre_millis_1min = millis();
    TV.minuteVentilation = minuteVentilationSum / 1000.0f;
    measuredRR = breathCtr;
    breathCtr = 0;
    status.MinVolValid = true;
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
      if (initInsp)
      {
        breathCtr++;
        // status.TidalVolValid = false; //Tidal Vol Update in Progress
        // status.MinVolValid = false; //Min Vol Update in Progress
        maxInspFlow = FS.Q_SLM;
        maxInspFlow_6122 = fs6122.flow_rate_slpm;

        minuteVentilationSum += TV.inspiration;
        // TV.inspiration = 0.0;
        // TV.maxInhale = 0.0;
        TV.measured = 0.0;
        TVin = 0.0;
        TVmax = 0.0;
        TVmeas_FS6122 = 0.0;

        // status.PeakPrsValid = false; //Peak Pressure Update in Progress
        // monitoring.peakInspPressure = p_sensor.pressure_gauge_CM;
        peakPrs = p_sensor.pressure_gauge_CM;

        if (lastBreathingPhase == EXPIRATION_PHASE)
        {
          monitoring.PEEPressure = peep;
          status.PeepValid = true; // Fresh Value of PEEP Available
          TV.expiration = TVex;
          if (maxNegativeFlow < -1.0)
            ctrNegFlow++;
          else
            ctrNegFlow--;
          ctrNegFlow = constrain(ctrNegFlow, -3, 3);
          if (ctrNegFlow >= 3)
            negativeFlowDetected = true;
          else if (ctrNegFlow <= -3)
            negativeFlowDetected = false;

          // Serial.println("VENT CKT");
          // Serial.println(negativeFlowDetected);
          // Serial.println(maxNegativeFlow);
          // Serial.println(ctrNegFlow);
          // Serial.println(status.ventCktDisconnected);
        }
        initInsp = false;
      }

      if (FS.Q_SLM > maxInspFlow)
        maxInspFlow = FS.Q_SLM;
      if (maxInspFlow > 0.0)
      {
        TVin += (((FS.Q_SLM * 1000.0f) / 60000.0f) * delta_t);
        TV.measured = TVin;
        if (TV.measured > TVmax)
          TVmax = TV.measured;
      }

#ifdef FS6122
      if (fs6122.flow_rate_slpm_filt > maxInspFlow_6122)
        maxInspFlow_6122 = fs6122.flow_rate_slpm_filt;
      if (maxInspFlow_6122 > 0.0)
        TVmeas_FS6122 += (((fs6122.flow_rate_slpm_filt * 1000.0f) / 60000.0f) * delta_t);
// TVmeas_FS6122 += (((FS.Q_SFM * 1000.0f) / 60000.0f) * delta_t);
#endif

      if (peakPrs < p_sensor.pressure_gauge_CM)
        peakPrs = p_sensor.pressure_gauge_CM;

      // reset init Flags
      initHold = true;
      initExp = true;
      initMeasurePEEP = true;
      scanBreathingAttempt = false;
      patientTriggeredBreath = false;
      initAssistedBreath = true;
      break;

    case HOLD_PHASE:
      if (initHold)
      {
        if (lastBreathingPhase == INSPIRATION_PHASE)
        {
          monitoring.peakInspPressure = peakPrs;
          TV.inspiration = TVin;
          status.PeakPrsValid = true; // Fresh Value of Peak Prs Available
        }
        // status.PltPrsValid = false; //PLT Pressure Update in Progress
        pltPrs = (p_sensor.pressure_gauge_CM);
        initHold = false;
      }
      else
      {
        pltPrs = (pltPrs + (p_sensor.pressure_gauge_CM)) * 0.5;
      }

      TV.measured += (((FS.Q_SLM * 1000.0f) / 60000.0f) * delta_t);
      if (TV.measured > TVmax)
        TVmax = TV.measured;
#ifdef FS6122
      TVmeas_FS6122 += (((fs6122.flow_rate_slpm_filt * 1000.0f) / 60000.0f) * delta_t);
// TVmeas_FS6122 += (((FS.Q_SFM * 1000.0f) / 60000.0f) * delta_t);
#endif

      TV.staticCompliance = (TV.maxInhale / (monitoring.plateauPressure - monitoring.PEEPressure));

      // reset init Flags
      initInsp = true;
      initExp = true;
      initMeasurePEEP = true;
      scanBreathingAttempt = false;
      patientTriggeredBreath = false;
      initAssistedBreath = true;
      break;
    case EXPIRATION_PHASE:
      if (initExp)
      {
        // TV.expiration = 0.0;
        TVex = 0.0;
        if (holdManeuver)
        {
          if (lastBreathingPhase == HOLD_PHASE)
          {
            monitoring.plateauPressure = pltPrs;
            status.PltPrsValid = true; // Fresh Value of PltPrs Available
            TV.maxInhale = TVmax;
            status.TidalVolValid = true; //Fresh Value of Tidal Vol Available
          }
        }
        else
        {
          if (lastBreathingPhase == INSPIRATION_PHASE)
          {
            TV.maxInhale = TVmax;
            status.TidalVolValid = true; //Fresh Value of Tidal Vol Available
          }
        }
        maxNegativeFlow = FS.Q_SLM;
        maxExpFlow_6122 = fs6122.flow_rate_slpm;
        initExp = false;
      }

      TVex += (-1.0) * (((FS.Q_SLM * 1000.0f) / 60000.0f) * delta_t);
      TV.measured += (((FS.Q_SLM * 1000.0f) / 60000.0f) * delta_t);
      if (TV.measured > TVmax)
        TVmax = TV.measured;
#ifdef FS6122
      TVmeas_FS6122 += (((fs6122.flow_rate_slpm_filt * 1000.0f) / 60000.0f) * delta_t);
      // TVmeas_FS6122 += (((FS.Q_SFM * 1000.0f) / 60000.0f) * delta_t);
      if (maxExpFlow_6122 > fs6122.flow_rate_slpm_filt)
        maxExpFlow_6122 = fs6122.flow_rate_slpm_filt;
#endif

      if (maxNegativeFlow > FS.Q_SLM)
      {
        maxNegativeFlow = FS.Q_SLM;
      }

      if (setpoint.curAssistMode_F == 1)
      {
        //static int ctr = 0;
        if (maxNegativeFlow < -5.0 && slave.runAck == 2 && slave.lastCMD_ID == RUN)
        {
          // if (abs(FS.Q_SLM) < 0.5)
          //   ctr++;
          // else
          //   ctr--;
          // ctr = constrain(ctr, -3, 3);
          // if (ctr >= 3)
          scanBreathingAttempt = true;
          // else if (ctr <= -3)
          //   scanBreathingAttempt = false;
        }
        if (scanBreathingAttempt)
        {
          Serial.print("Set Trigger: ");
          Serial.println(setpoint.flowTriggerSenstivity);
          if (FS.Q_SLM >= setpoint.flowTriggerSenstivity)
          {
            patientTriggeredBreath = true;
          }
        }
      }
      if (PEEPMesaureFlag)
      {
        // status.PeepValid = false; //PEEP Update in Progress
        if (initMeasurePEEP)
        {
          peep = (p_sensor.pressure_gauge_CM);
          initMeasurePEEP = false;
        }
        else
        {
          peep = (peep + (p_sensor.pressure_gauge_CM)) * 0.5;
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
      initAssistedBreath = true;
      patientTriggeredBreath = false;
      break;
    }
    lastBreathingPhase = status.breathPhase;
  }
  // Q_old = FS.Q_SLM;

  if (status.VentilatorOperationON)
  {
    // if ((TV.expiration > 200.0) && (InspPressurmonitoring.peake >= 3.0))
    //   status.ventCktDisconnected = false;
    // if ((status.breathPhase == INSPIRATION_PHASE) && TV.expiration < 200.0)
    //   status.ventCktDisconnected = true;

    if (!negativeFlowDetected)
      status.ventCktDisconnected = true;
    else
      status.ventCktDisconnected = false;

    status.ventCktDisconnectedValid = true;
    // else if ((status.breathPhase == EXPIRATION_PHASE) && (TV.maxInhale < 200.0))
    //   status.ventCktDisconnected = true;
    // else if ((TV.maxInhale > 200.0) && (monitoring.peakInspPressure < 3.0))
    //   status.ventCktDisconnected = true;
  }

  // Value Settling Times Handling on Setpoint change
  if (status.VentilatorOperationON)
  {

    static uint8_t curRR_sp = setpoint.curBPM;
    static uint16_t curVol_sp = setpoint.curVolume;
    static uint8_t curPrs_sp = setpoint.curPressure;
    static uint8_t curContVar = setpoint.curControlVariable;

    if (initOperations)
    {
      monitoring.MV_timestamp = millis();
      monitoring.RR_timestamp = millis();
      monitoring.TV_timestamp = millis();
      monitoring.PR_timestamp = millis();
      monitoring.CV_timestamp = millis();
      initOperations = false;
    }

    if (curContVar != setpoint.curControlVariable)
    {
      curContVar = setpoint.curControlVariable;
      status.MinVolValid = false;
      status.TidalVolValid = false;
      status.PltPrsValid = false;
      status.PeakPrsValid = false;
      status.ventCktDisconnectedValid = false;
      status.RRValid = false;
      monitoring.CV_timestamp = millis();
    }

    if (curVol_sp != setpoint.curVolume)
    {
      curVol_sp = setpoint.curVolume;
      status.MinVolValid = false;
      monitoring.MV_timestamp = millis();

      status.ventCktDisconnectedValid = false;
      status.TidalVolValid = false;
      monitoring.TV_timestamp = millis();
    }

    if (curRR_sp != setpoint.curBPM)
    {
      curRR_sp = setpoint.curBPM;
      status.MinVolValid = false;
      status.RRValid = false;
      monitoring.MV_timestamp = millis();
      monitoring.RR_timestamp = millis();
    }

    if (curPrs_sp != setpoint.curPressure)
    {
      curPrs_sp = setpoint.curPressure;
      status.PltPrsValid = false;
      status.PeakPrsValid = false;
      status.ventCktDisconnectedValid = false;
      monitoring.PR_timestamp = millis();
    }

    if (((millis() - monitoring.MV_timestamp) <= 60000))
    {
      status.MinVolValid = false;
    }

    if (((millis() - monitoring.RR_timestamp) <= 60000))
    {
      status.RRValid = false;
    }

    if ((millis() - monitoring.TV_timestamp) < (3 * breathLength))
    {
      ctrNegFlow = 0;
      status.ventCktDisconnectedValid = false;
      status.TidalVolValid = false;
    }

    if ((millis() - monitoring.PR_timestamp) < (1 * breathLength))
    {
      status.PeakPrsValid = false;
      status.PltPrsValid = false;
      ctrNegFlow = 0;
      status.ventCktDisconnectedValid = false;
    }

    if ((millis() - monitoring.CV_timestamp) < (3 * breathLength))
    {
      status.MinVolValid = false;
      //      status.PeakPrsValid = false;
      //      status.PltPrsValid = false;
      status.TidalVolValid = false;
      status.RRValid = false;
      ctrNegFlow = 0;
      status.ventCktDisconnectedValid = false;
    }
  }
  else
  {
    initOperations = true;

    status.RRValid = false;
    status.TidalVolValid = false;
    status.MinVolValid = false;
    status.PeakPrsValid = false;
    status.PeepValid = false;
    status.PltPrsValid = false;
    status.ventCktDisconnectedValid = false;
    ctrNegFlow = 0;
  }

  if (O2.Pbar < 1.0)
    status.oxygenFailure = true;
  else
    status.oxygenFailure = false;

  T_old_us = millis();

#ifdef QT_PLOTTER
  if (plotterData)
  {

    Serial.print("$");
    Serial.print(FS.Q_SFM, 5);
    Serial.print(" ");
    Serial.print(TV.measured, 5);
    Serial.print(" ");
    Serial.print(p_sensor.pressure_gauge_CM, 5);
    Serial.print(" ");
#ifdef FS6122
    Serial.print(fs6122.flow_rate_slpm_filt, 5);
    Serial.print(" ");
    Serial.print(fs6122.flow_rate_slpm, 5);
    Serial.print(" ");
    Serial.print(maxInspFlow, 5);
    Serial.print(" ");
    Serial.print(maxNegativeFlow, 5);
    Serial.print(" ");
    Serial.print(maxInspFlow_6122, 5);
    Serial.print(" ");
    Serial.print(maxExpFlow_6122, 5);
    Serial.print(" ");
    Serial.print(TVmeas_FS6122, 5);
    Serial.print(" ");
    // Serial.print(fs6122.pressure_cmh2o, 5);
    // Serial.print(" ");
#endif
    Serial.print(setpoint.curBPM);
    Serial.print(" ");
    Serial.print(setpoint.curVolume);
    Serial.print(" ");
    Serial.print(setpoint.curPressure);
    Serial.print(" ");
    Serial.print(setpoint.curI_E_Section);
    Serial.print(" ");
    Serial.print(setpoint.curVentMode);
    Serial.print(" ");
    Serial.print(status.breathPhase);
    Serial.print(" ");
    Serial.print(50 * scanBreathingAttempt);
    Serial.print(" ");
    Serial.print(100 * patientTriggeredBreath);
    Serial.print(" ");

    // //  Serial.print(O2.FIO2_conc, 5);
    // //  Serial.print(" ");
    // //  Serial.print(O2.Pbar, 5);
    // //  Serial.print(" ");
    Serial.print(status.compressionMechFailure * 100);
    Serial.print(" ");
    Serial.print(control->motorIsWorking * 50);
    Serial.print(" ");
    Serial.print(control->valuePredicted, 5);
    Serial.print(" ");
    Serial.print(control->error, 5);
    //   Serial.print(" ");
    //   Serial.print((float)actualBreathCycleTime/1000.0f, 5);
    Serial.print(" ");
    Serial.print(delta_t, 5);
    Serial.print(" ");
    Serial.print(millis());
    Serial.print(";\n");
  }
#endif
}

void setup()
{
  // put your setup code here, to run once:
#ifdef EN_WATCHDOG
#ifdef __AVR__
  wdt_disable();
#endif
#endif

#ifdef STM32F4xx
  //************** DATA_PINS ****************//
  pinMode(KEY_DATA0, INPUT); //DATA0
  pinMode(KEY_DATA1, INPUT); //DATA1
  pinMode(KEY_DATA2, INPUT); //DATA2
  pinMode(KEY_DATA3, INPUT); //DATA3
#endif
  unsigned long pre_millis = 0;
  bool initWarmUp = true;

  // pinMode(pin_Button_OK, INPUT);
  // pinMode(pin_Button_SNZ, INPUT);
  // pinMode(pin_Switch_MODE, INPUT);

  pinMode(pin_Switch_START, INPUT);
  pinMode(pin_LED1, OUTPUT);

  //    pinMode(pin_LmtSWT_CL1, INPUT);
  //    pinMode(pin_LmtSWT_CL2, INPUT);

  // pinMode(pin_Knob_1, INPUT);
  // pinMode(pin_Knob_2, INPUT);
  // pinMode(pin_Knob_3, INPUT);
  // pinMode(pin_Knob_4, INPUT);
#ifdef Beeper
  pinMode(pin_BUZZER, OUTPUT);
#endif

  pinMode(pin_SLAVE_RESET, OUTPUT);

  calibStatus = ST_COMPLETE; //ST_NOT_INIT;//ST_COMPLETE;
  calibrationParam = PRESS_CONT_MODE;

  //  Timer1.initialize(200);
  //  Timer1.attachInterrupt(Timer1ISR);

  // Timer3: used for Monitoring & Alarms Management

  //    noInterrupts();
  pre_millis = millis();
  while ((millis() - pre_millis) < WARM_UP_TIME)
  {
    if (initWarmUp)
    {
      initWarmUp = false;
      status.WarmUpFlag = true;
#if defined(__AVR__)
      Serial.begin(SERIAL_BAUD);
      Serial2.begin(SERIAL_BAUD);
#ifndef TEL_AT_UART0
#ifdef TX_SERIAL_TELEMETRY
      //     Serial1.begin(SERIAL_BAUD);
#endif
#endif
      Serial1.begin(SERIAL_BAUD);
      Serial3.begin(SERIAL_BAUD);
#elif defined(STM32F4xx)
      Serial.begin(SERIAL_BAUD);
      Serial1.begin(SERIAL_BAUD);
#ifndef TEL_AT_UART0
#ifdef TX_SERIAL_TELEMETRY
      //     Serial1.begin(SERIAL_BAUD);
#endif
#endif
      Serial3.begin(SERIAL_BAUD);
      Serial4.begin(SERIAL_BAUD);
#endif
      Wire.begin();

      LCD_setup();
      Display();

      initFlowSensor();
      initO2Sensor();
      digitalWrite(pin_SLAVE_RESET, LOW);
      delay(5000);
      digitalWrite(pin_SLAVE_RESET, HIGH);
      delay(5000);

      //   Serial3.print(F("Breath Number,"));
      //  Serial3.print(F("Timestamp (ms),"));
      //   Serial3.print(F("Vent Mode (0-VCV|1-PCV|2-AC-VCV|3-AC-PCV|4-CPAP),"));
      //   Serial3.print(F("Breathing Length (ms),"));
      //   Serial3.print(F("Inspiration Time (ms),"));
      //   Serial3.print(F("Expiration Time (ms),"));
      //   Serial3.print(F("targetPosition (mm),"));
      //   Serial3.print(F("Motor Speed Insp (mm/s),"));
      //   Serial3.print(F("Motor Speed Exp (mm/s),"));
      //   Serial3.print(F("RPM Insp,"));
      //   Serial3.print(F("RPM Exp,"));
      //   Serial3.print(F("Steps Insp,"));
      //   Serial3.print(F("Steps Exp,"));
      //   Serial3.print(F("Period Insp (us),"));
      //   Serial3.print(F("Period Exp (us),"));
      //   Serial3.print(F("Comp Mech Failure,"));
      //   Serial3.print(F("Trigger Window,"));
      //   Serial3.print(F("Patient Trigger Detected,"));
      //   Serial3.print(F("Control (Motor Working),"));
      //   Serial3.print(F("Control (valuePredicted),"));
      //    Serial3.print(F("Control (error),"));
      //    Serial3.print(F("Flow Rate (SLPM),"));
      //    Serial3.print(F("Tidal Volume (ml),"));
      //    Serial3.print(F("Pressure (cmH2O)"));
      //    Serial3.print(F("\n"));
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

  eeget(); // read startup parameters (either from EEPROM or default value)
           //  eeput();
           //  eeget();    // read startup parameters (either from EEPROM or default value)

  setpoint.curBPM = setpoint.reqBPM; // Start from these values without sweep
  setpoint.curVolume = setpoint.reqVolume;
  setpoint.curPressure = setpoint.reqPressure;
  setpoint.curI_E_Section = setpoint.reqI_E_Section;
  setpoint.curI_E = IE_R_Value[setpoint.curI_E_Section][1]; //Exhale Factor
  setpoint.curFiO2 = setpoint.reqFiO2;                      //Exhale Factor
  setpoint.curVentMode = setpoint.reqVentMode;
  setpoint.curAssistMode_F = setpoint.reqAssistMode_F;
  setpoint.curEnableCPAP_F = setpoint.reqEnableCPAP_F;
  setpoint.curControlVariable = setpoint.reqControlVariable;

  breathLength = (int)(60000 / setpoint.curBPM);
  SetAlarmsWaitTimes();
  InitializeParams();

#ifdef CLOSED_LOOP
  control = new Control();
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
    control->setConstants(0.8, 0.1, deadBand, 1200); //values send in this function are needed to be tested.
  else                                               //PRESS_CON_MODE
    control->setConstants(0.8, 0.1, 1, 80);

  control->resetController(defaultVolume);
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
#ifdef EN_ALARMS
  alarmsSetup();
#endif
#ifdef EN_WATCHDOG
  wdt_enable(WDTO_8S);
#endif
}

void Ventilator_Control()
{
  static bool skip = true;
  static bool initIns = true;
  static bool initHld = true;
  static bool initExp = true;
  static bool initWait = true;
  static bool initOps = true;
  static bool runMotor = true;
  static bool init = true;
  static bool skipStep = 1;

  static unsigned int Tin = 0;
  static unsigned int Tex = 0;
  static unsigned int Th = 0;          //ms
  static unsigned int Ttrigger = 1000; //ms
  static uint16_t Tcur = 0;
  static unsigned long BreathStartTimestamp = 0;
  static unsigned long cycleStartTimestamp = millis();

  static float reqMotorPos = 0.0; //mm
  /*static */ float Vin = 0.0;    //mm/s
  /*static */ float Vex = 0.0;    //mm/s
  /*static */ float RPMin = 0.0;
  /*static */ float RPMex = 0.0;

  static uint16_t stepIn = 0;
  static uint16_t stepEx = 0;
  static uint16_t periodIn = 0; //us
  static uint16_t periodEx = 0; //us

  static unsigned long breathNumber = 1;
  static unsigned long retry_timestamp = 0;

  static unsigned long estimatedHomingTime = 0.0;

  static float stepsPredicted = 0.0;

  static unsigned long homeCmd_Ts = 0;
  static unsigned long RunAck_Ts = 0;

  //    noInterrupts();

  if (init)
  {
    setpoint.curBPM = setpoint.reqBPM; // Start from these values without sweep
    setpoint.curVolume = setpoint.reqVolume;
    setpoint.curPressure = setpoint.reqPressure;
    setpoint.curI_E_Section = setpoint.reqI_E_Section;
    setpoint.curI_E = IE_R_Value[setpoint.curI_E_Section][1]; //Exhale Factor
    setpoint.curFiO2 = setpoint.reqFiO2;                      //Exhale Factor
    setpoint.curVentMode = setpoint.reqVentMode;
    setpoint.curAssistMode_F = setpoint.reqAssistMode_F;
    setpoint.curEnableCPAP_F = setpoint.reqEnableCPAP_F;
    setpoint.curControlVariable = setpoint.reqControlVariable;

    breathLength = (int)(60000 / setpoint.curBPM);
    // Take the hold time out of the exhale cycle. Do this to ensure respitory rate is correct.
    //    Tex = (int)((breathLength - Th) / (1 + setpoint.curI_E)); // if I/E ratio = 0.5 ; it means expiration is twice as long as inspiration
    //    Tin = (int)(Tex * setpoint.curI_E);
    if (holdManeuver)
      Th = holdDur_ms;
    else
      Th = 0;

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
#ifdef __DEBUG
    Serial.println(slave.lastCMD_ID);
    Serial.println(slave.homeAck);
    Serial.println(F("Homing Not Complete"));
#endif
  }

  if (status.VentilatorOperationON == 1)
  {

    if (initOps)
    {
      initOps = false;
      RunAck_Ts = millis();
      initWait = true;
    }
    // if (slave.runAck == 2)
    // {
    //   RunAck_Ts = millis();
    //   status.compressionMechFailure = false;
    // }
    if (status.VentilatorOperationON == 1 && TV.maxInhale > 10.0)
    {
      RunAck_Ts = millis();
      status.compressionMechFailure = false;
    }
    if ((millis() - RunAck_Ts) >= (3 * breathLength))
    {
      status.compressionMechFailure = true;
#ifdef __DEBUG
      Serial.println(F("COMPRESSION MECH FAILURE.!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
#endif
    }
    skip = false;
    //    status.VentilatorOperationON = 1;

    if (setpoint.curAssistMode_F == 1)
    {
      if (patientTriggeredBreath)
      {
        {
          Tcur = breathLength; //Start a new inhale Cycle
        }
        if (initAssistedBreath)
        {
          txSlaveCMD(STOP);
          slave.lastCMD_ID = STOP;
          slave.runAck = 2;
          initAssistedBreath = false;
          return;
        }
      }
    }

    if (Tcur >= breathLength)
    {
      if (slave.runAck != 2 && slave.lastCMD_ID == RUN && (micros() - retry_timestamp) >= Tex)
      {
        retry_timestamp = micros();
        Serial3.println(F("Retry Attempt Initiated."));
        txSlaveCMD(STOP);
        slave.lastCMD_ID = STOP;
        Tcur = breathLength;
        return;
      }

      if (slave.runAck != 2 && slave.lastCMD_ID == STOP)
      {
        Serial3.println(F("Re-Run Command Sent."));
        txSlaveCMD(RUN, HOMING_PERIOD, stepEx, "0");
        slave.lastCMD_ID = RUN;
        Tcur = breathLength;
        return;
      }

      if (slave.runAck != 2 && status.breathPhase == EXPIRATION_PHASE)
      {
        Tcur = breathLength;
        return;
      }

      Tcur = 0;
      setpoint.curAssistMode_F = setpoint.reqAssistMode_F;
      setpoint.curEnableCPAP_F = setpoint.reqEnableCPAP_F;
      setpoint.curVentMode = setpoint.reqVentMode;

      if (setpoint.reqControlVariable != setpoint.curControlVariable)
      {
        setpoint.curControlVariable = setpoint.reqControlVariable;
        if (setpoint.curControlVariable == VOL_CONT_MODE)
        {
          control->setConstants(0.8, 0.1, 20, 1200);
          control->resetController((float)setpoint.curVolume);
        }
        else if (setpoint.curControlVariable == PRESS_CONT_MODE)
        {
          control->setConstants(0.8, 0.1, 1, 80);
          control->resetController((float)setpoint.curPressure);
        }
        skipStep = 1;
      }
      if (abs(setpoint.curVolume - setpoint.reqVolume) >= 1)
      {
        setpoint.curVolume = setpoint.reqVolume;
        if (setpoint.curControlVariable == VOL_CONT_MODE)
        {
          control->resetController((float)setpoint.curVolume);
          skipStep = 1;
        }
      }
      if (abs(setpoint.curPressure - setpoint.reqPressure) >= 1)
      {
        setpoint.curPressure = setpoint.reqPressure;
        if (setpoint.curControlVariable == PRESS_CONT_MODE)
        {
          control->resetController((float)setpoint.curPressure);
          skipStep = 1;
        }
      }
      if (setpoint.curBPM != setpoint.reqBPM)
      {
        setpoint.curBPM = setpoint.reqBPM; // Load Fresh User Settings
        breathLength = (int)(60000 / setpoint.curBPM);
        SetAlarmsWaitTimes();
      }
      setpoint.curI_E_Section = setpoint.reqI_E_Section;
      setpoint.curI_E = IE_R_Value[setpoint.curI_E_Section][1]; //Exhale Factor

      breathLength = (int)(60000 / setpoint.curBPM);
      if (holdManeuver)
        Th = holdDur_ms;
      else
        Th = 0;
      // Take the hold time out of the exhale cycle. Do this to ensure respitory rate is correct.
      Tin = (int)((breathLength - Th) / (1 + setpoint.curI_E)); // if I/E ratio = 0.5 ; it means expiration is twice as long as inspiration
      Tex = (int)(breathLength - Th - Tin);
      if (setpoint.curControlVariable == VOL_CONT_MODE)
      {

        if (!skipStep)
        {
          //      stepsPredicted = control->compensateError((float)setpoint.curVolume,TV.inspiration);
          stepsPredicted = control->compensateError((float)setpoint.curVolume, TV.maxInhale);
        }
        else
        {
          stepsPredicted = setpoint.curVolume;
          skipStep = 0;
        }
        //        reqMotorPos = setpoint.curVolume / LINEAR_FACTOR_VOLUME; //mm
        reqMotorPos = (VolCoeffs[0] * pow(stepsPredicted, 3)) + (VolCoeffs[1] * pow(stepsPredicted, 2)) + (VolCoeffs[2] * stepsPredicted) + VolCoeffs[3];
        // reqMotorPos = stepsPredicted;
        //reqMotorPos = 30.0;
        reqMotorPos = constrain(reqMotorPos, 0.0, 40.0);
      }
      else //PRESS_CONT_MODE
      {
        if (!skipStep)
        {

          stepsPredicted = control->compensateError((float)setpoint.curPressure, monitoring.peakInspPressure);
          //stepsPredicted = control->compensateError((float)setpoint.curPressure, monitoring.plateauPressure);
        }
        else
        {
          stepsPredicted = setpoint.curPressure;
          skipStep = 0;
        }
        reqMotorPos = (PressCoeffs[0] * pow(stepsPredicted, 3)) + (PressCoeffs[1] * pow(stepsPredicted, 2)) + (PressCoeffs[2] * stepsPredicted) + PressCoeffs[3];
        // reqMotorPos = stepsPredicted;
        reqMotorPos = constrain(reqMotorPos, 0.0, 40.0);
      }

      if (reqMotorPos == 40.0)
        control->motorAtLimit = true;
      else
        control->motorAtLimit = false;

      control->motorIsWorking = !status.compressionMechFailure;

      // TODO RPMIN is not using any where...
      Vin = reqMotorPos / ((float)Tin / 1000.0f); // mm/s
      Vex = reqMotorPos / ((float)Tex / 1000.0f); // mm/s
      RPMin = (Vin / LIN_MECH_mm_per_rev) * 60.0;
      RPMex = (Vex / LIN_MECH_mm_per_rev) * 60.0;
      stepIn = (long)((reqMotorPos / LIN_MECH_mm_per_rev) * STEPPER_MICROSTEP * STEPPER_PULSES_PER_REV);
      stepEx = (long)(stepIn + ((2.0 / LIN_MECH_mm_per_rev) * STEPPER_MICROSTEP * STEPPER_PULSES_PER_REV));
      periodIn = (long)((((float)Tin / 1000.0) / stepIn) * 1000000); //us
      //ToDO
      // periodEx = (long)((((float)Tex / 1000.0) / stepIn) * 1000000); //us
      BreathStartTimestamp = millis();

#ifdef __DEBUG
#ifndef TEL_AT_UART0
      Serial.print(F("Breath Number: "));
      Serial.println(breathNumber);
      Serial.print(F("Control Mode:      "));
      Serial.println(setpoint.curControlVariable); //0 = VOL; 1 = PRESS
      Serial.print(F("Breathing Length:      "));
      Serial.println(breathLength);
      Serial.print(F("Inspiration Time:      "));
      Serial.print(Tin);
      Serial.println(F(" ms"));
      Serial.print(F("Expiration Time:       "));
      Serial.print(Tex);
      Serial.println(F(" ms"));
      Serial.print(F("targetPosition:        "));
      Serial.print(reqMotorPos);
      Serial.println(F(" mm"));
      Serial.print(F("Motor Speed Insp:      "));
      Serial.print(Vin);
      Serial.println(F(" mm/s"));
      Serial.print(F("Motor Speed Exp:       "));
      Serial.print(Vex);
      Serial.println(F(" mm/s"));
      Serial.print(F("RPM Insp:              "));
      Serial.println(RPMin);
      Serial.print(F("RPM Exp:               "));
      Serial.println(RPMex);
      Serial.print(F("Steps Insp:            "));
      Serial.println(stepIn);
      Serial.print(F("Steps Exp:             "));
      Serial.println(stepEx);
      Serial.print(F("Period Insp:           "));
      Serial.print(periodIn);
      Serial.println(F(" us"));
      Serial.print(F("Period Exp:            "));
      Serial.print(periodEx);
      Serial.println(F(" us"));
      Serial.print(F("Comp Mech Fail         "));
      Serial.println(status.compressionMechFailure);
#endif
#endif
      breathNumber++;
#ifdef __DEBUG
#ifndef TEL_AT_UART0

      Serial3.println(breathNumber++);
      Serial3.print(F(","));
      Serial3.print(TEL.Time); //0 = VOL; 1 = PRESS
      Serial3.print(F(","));
      Serial3.print(setpoint.curVentMode); //0 = VOL; 1 = PRESS
      Serial3.print(F(","));
      Serial3.print(breathLength);
      Serial3.print(F(","));
      Serial3.print(Tin);
      Serial3.print(F(","));
      Serial3.print(Tex);
      Serial3.print(F(","));
      Serial3.print(reqMotorPos);
      Serial3.print(F(","));
      Serial3.print(Vin);
      Serial3.print(F(","));
      Serial3.print(Vex);
      Serial3.print(F(","));
      Serial3.print(RPMin);
      Serial3.print(F(","));
      Serial3.print(RPMex);
      Serial3.print(F(","));
      Serial3.print(stepIn);
      Serial3.print(F(","));
      Serial3.print(stepEx);
      Serial3.print(F(","));
      Serial3.print(periodIn);
      Serial3.print(F(","));
      Serial3.print(periodEx);
      Serial3.print(F(","));
      Serial3.print(status.compressionMechFailure);
      Serial3.print(F(","));
      Serial3.print(scanBreathingAttempt);
      Serial3.print(F(","));
      Serial3.print(patientTriggeredBreath);
      Serial3.print(F(","));
      Serial3.print(control->motorIsWorking * 50);
      Serial3.print(F(","));
      Serial3.print(control->valuePredicted, 5);
      Serial3.print(F(","));
      Serial3.print(control->error, 5);
      Serial3.print(F(","));
      Serial3.print(FS.Q_SLM, 5);
      Serial3.print(F(","));
      Serial3.print(TV.measured, 5);
      Serial3.print(F(","));
      Serial3.print(p_sensor.pressure_gauge_CM, 5);
      Serial3.print(F("\n"));
#endif

#endif
    }
    Tcur = millis() - BreathStartTimestamp;

    if (Tcur <= Tin)
    {
      if (initIns)
      {
        Serial3.print(F("Breath Number : "));
        Serial3.println(breathNumber - 1);
        Serial3.print(F("Timestamp : "));
        Serial3.println(TEL.Time); //0 = VOL; 1 = PRESS

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
        // if(setpoint.curAssistMode_F==1)
        // {
        txSlaveCMD(RUN, HOMING_PERIOD, stepEx, "0");
        estimatedHomingTime = HOMING_PERIOD * stepEx;
        // }
        // else
        // {
        //  txSlaveCMD(RUN, periodEx, stepEx, "0");
        // }

        runMotor = false;
        slave.lastCMD_ID = RUN;
        // retry_timestamp = micros();
      }

      //////////////////////////////////////////////////////
      // if (slave.runAck != 2 && slave.lastCMD_ID == RUN && (micros() - retry_timestamp) > estimatedHomingTime)
      // {
      //   Serial3.println(F("Retry Attempt Initiated."));
      //   txSlaveCMD(STOP);
      //   slave.lastCMD_ID = STOP;
      //   return;
      // }
      // if (slave.runAck != 2 && slave.lastCMD_ID == STOP)
      // {
      //   Serial3.println(F("Re-Run Command Sent."));
      //   txSlaveCMD(RUN, HOMING_PERIOD, stepEx, "0");
      //   slave.lastCMD_ID = RUN;
      //   estimatedHomingTime = HOMING_PERIOD*stepEx;
      //   retry_timestamp = micros();
      //   return;
      // }
      //////////////////////////////////////////////////////

      if ((Tcur >= (Tin + Tex)) && (Tcur < (Tin + Th + Tex)))
      {
        PEEPMesaureFlag = true;
      }

      // if (setpoint.curAssistMode_F == 1) {
      //   if ((Tin + Th + Tex - Tcur) < Ttrigger)
      //   {
      //     scanBreathingAttempt = true;
      //   }
      // }
    }
  }
  else
  {
    static uint8_t ctr = 0;
#ifndef TEL_AT_UART0
    //  Serial.println(F("Ventilator Operation Halt"));
    //  Serial.print(F("Homeack: ")); Serial.println(slave.homeAck);
#endif
    if (initWait)
    {
      ctr = 0;
      Serial3.println(F("WAIT PHASE."));
      slave.homeAck = 0;
      slave.stopAck = 0;
      txSlaveCMD(STOP);
      slave.lastCMD_ID = STOP;
      homeCmd_Ts = millis();
      if (status.sensorsHealth == HEALTH_BAD)
        status.homeAtBadSensor = true;
      if (p_sensor.sensorHealth == HEALTH_BAD)
        status.homeAtBadPressSensor = true;
      if (FS.sensorHealth == HEALTH_BAD)
        status.homeAtBadFlowSensor = true;
      initWait = false;
    }

    ctr++;
    initOps = true;
    initHld = true;
    initIns = true;
    initExp = true;
    //status.VentilatorOperationON = 0;
    status.breathPhase = WAIT_PHASE;
    Tcur = breathLength; // This will always start inspiration breath cycle on setting switch to start position
    // status.PeepValid = false; //To Prevent False Alarms
    // status.PltPrsValid = false; //To Prevent False Alarms

    if ((slave.homeAck == 0) && (ctr >= 5))
    {
      if ((((millis() - homeCmd_Ts) >= 8000) && slave.lastCMD_ID == HOME) || slave.lastCMD_ID == STOP)
      {
        txSlaveCMD(HOME, HOMING_PERIOD);
        slave.lastCMD_ID = HOME;
        homeCmd_Ts = millis();
        slave.runAck = 0;
      }
    }
    else if (slave.homeAck == 2)
    {
      status.homeAtBadSensor = false;
      status.homeAtBadPressSensor = false;
      status.homeAtBadFlowSensor = false;
      status.homingFailure = false;
    }

    static bool skipHome = true;
    if (((millis() - homeCmd_Ts) >= 10000) && slave.homeAck != 2)
    {
      if (!skipHome)
      {
        status.homingFailure = true;
        initWait = true;
      }
      skipHome = false;
    }

    PEEPMesaureFlag = false;
  }
  //    interrupts();
}

void loop()
{
  static float timeSincePowerUp = 0.0;
  static unsigned long tick1 = 0; // counter used to trigger sensor measurements //loop 1
  static unsigned long tick2 = 0; // counter used to trigger Control Loop // loop 2
  static unsigned long tick3 = 0;

  unsigned long start_Ts = 0;

#ifdef EN_WATCHDOG
  wdt_reset();
#endif

  if (millis() > (tick1 + samplePeriod1))
  {
#ifdef __DEBUG

    Serial.print("Timestamp: ");
    Serial.println(millis() - start_Ts);
#endif
    timeSincePowerUp += ((float)(millis() - tick1) / 1000.0f);
    tick1 = millis();

    if (timeSincePowerUp >= 345600.0) // 96 * 60 * 60
      status.hours96_complete = true;
    else if (timeSincePowerUp >= 10.0)
      status.systemReset = false;

    start_Ts = millis();
    readSensors();
#ifdef __DEBUG

// Serial.print(F("Busy Time 1: ")); Serial.println(millis()-start_Ts);
#endif
    checkSensorHealth();
    Monitoring();

    if (status.selfTestProg != ST_COMPLETE)
    {
      selfTest();
    }
#ifdef EN_ALARMS
    // start_Ts = micros();
    alarmControl();
#ifdef __DEBUG

// Serial.print(F("Busy Time alarmControl: ")); Serial.println(micros()-start_Ts);
#endif
#endif
    // #ifdef TX_SERIAL_TELEMETRY
    //     GetTelData(); //Called at 100Hz
    //     Prepare_Tx_Telemetry(); //Called at 100Hz
    // #endif
#ifdef __DEBUG
    Serial.print(F("Busy Time 1: "));
    Serial.println(millis() - start_Ts);
#endif
  }

  if ((status.selfTestProg == ST_COMPLETE) && (status.selfTestStatus == ST_PASS)) // I am not writing control loop inside 100Hz loop to keep both loop rates independant
  {
    if (millis() > (tick2 + samplePeriod2))
    {
      //start_Ts = micros();
      tick2 = millis();
      Ventilator_Control();
#ifdef __DEBUG
      Serial.print(F("Busy Time 2: "));
      Serial.println(micros() - start_Ts);
#endif
    }
  }
  if (millis() > (tick3 + 500))
  {
    tick3 = millis();
    readSwitches();
    //  start_Ts = micros();
    Display();
#ifdef __DEBUG
    Serial.print(F("Busy Time Display: "));
    Serial.println(micros() - start_Ts);
#endif
  }

#ifdef TX_SERIAL_TELEMETRY
  GetTelData();           //Called at 100Hz
  Prepare_Tx_Telemetry(); //Called at 100Hz
#endif
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
void serialEvent2()
{
  receiveSlaveTel();
}
