#include "header.h"
#include "userInterface.h"
#include "command.h"

extern struct setpointStatus setpoint;
extern bool key_int_flag;
extern uint8_t IE_R_Value[3][2];
extern uint8_t FiO2_Value[4][2];
extern bool activateVentilatorControlSerial;
extern bool plotterData;

int slaveResetPinState = HIGH;

void (*resetFunction)(void) = 0; // Self reset (to be used with watchdog)





void processCommands()
{
#ifndef TX_SERIAL_TELEMETRY
  while (Serial1.available()) 
  {
    switch(Serial1.read())
    {
      case 1 : setpoint.reqBPM++;   // = Sensor_val_RR + 1.0;
                limitValues();
                Serial1.print(F("RR Setpoint = ")); Serial1.println(setpoint.reqBPM);
                key_int_flag = true;
              break;
      case 2 : setpoint.reqBPM--;   // = Sensor_val_RR - 1.0;
                limitValues();
                Serial1.print(F("RR Setpoint = ")); Serial1.println(setpoint.reqBPM);
                key_int_flag = true;
              break;
      case 3 : setpoint.reqPressure++;
                limitValues();
                Serial1.print(F("Pressure Setpoint = ")); Serial1.println(setpoint.reqPressure);
                key_int_flag = true;
              break;
      case 4 : setpoint.reqPressure--;
                limitValues();
                Serial1.print(F("Pressure Setpoint = ")); Serial1.println(setpoint.reqPressure);
                key_int_flag = true;
              break;
      case 5 : setpoint.reqFiO2++;
                limitValues();
                Serial1.print(F("FiO2 Setpoint = ")); Serial1.print(FiO2_Value[setpoint.reqFiO2][0]);Serial1.print(F("-")); Serial1.println(FiO2_Value[setpoint.reqFiO2][1]);
                key_int_flag = true;
              break;
      case 6 : setpoint.reqFiO2--;
                limitValues();
                Serial1.print(F("FiO2 Setpoint = ")); Serial1.print(FiO2_Value[setpoint.reqFiO2][0]);Serial1.print(F("-")); Serial1.println(FiO2_Value[setpoint.reqFiO2][1]);
                key_int_flag = true;
              break;
      case 7 : setpoint.reqVolume += 50;
                limitValues();
                Serial1.print(F("Volume Setpoint = ")); Serial1.println(setpoint.reqVolume);
                key_int_flag = true;
              break;
      case 8 : setpoint.reqVolume -= 50;
                limitValues();
                Serial1.print(F("Volume Setpoint = ")); Serial1.println(setpoint.reqVolume);
                key_int_flag = true;
              break;
        case 9 : setpoint.reqVentMode++; key_int_flag = true;
                limitValues();
            Serial1.print(F("Vent Mode = "));
            switch (setpoint.reqVentMode)
            {
            case VENT_MODE_VCV:
                Serial1.println(F("VCV"));
                break;
            case VENT_MODE_PCV:
                Serial1.println(F("PCV"));
                break;
            case VENT_MODE_AC_VCV:
                Serial1.println(F("AC-VCV"));
                break;
            case VENT_MODE_AC_PCV:
                Serial1.println(F("AC_PCV"));
                break;          
            case VENT_MODE_CPAP:
                Serial1.println(F("CPAP"));
                break;
            default:
                break;
            }

              break;
      case 10 : setpoint.reqVentMode--; key_int_flag = true;
                limitValues();
            Serial1.print(F("Vent Mode = "));
            switch (setpoint.reqVentMode)
            {
            case VENT_MODE_VCV:
                Serial1.println(F("VCV"));
                break;
            case VENT_MODE_PCV:
                Serial1.println(F("PCV"));
                break;
            case VENT_MODE_AC_VCV:
                Serial1.println(F("AC-VCV"));
                break;
            case VENT_MODE_AC_PCV:
                Serial1.println(F("AC_PCV"));
                break;          
            case VENT_MODE_CPAP:
                Serial1.println(F("CPAP"));
                break;
            default:
                break;
            }              
            break;
      case 11 : setpoint.reqI_E_Section++; key_int_flag = true;
                limitValues();
            Serial1.print(F("IE Ratio = ")); Serial1.print(IE_R_Value[setpoint.reqI_E_Section][0]);Serial1.print(F(":")); Serial1.println(IE_R_Value[setpoint.reqI_E_Section][1]);
              break;
      case 12 : setpoint.reqI_E_Section--; key_int_flag = true;
                limitValues();
            Serial1.print(F("IE Ratio = ")); Serial1.print(IE_R_Value[setpoint.reqI_E_Section][0]);Serial1.print(F(":")); Serial1.println(IE_R_Value[setpoint.reqI_E_Section][1]);
              break;   
      case 13: activateVentilatorControlSerial  = !activateVentilatorControlSerial;
                Serial1.print(F("START = ")); Serial1.println(activateVentilatorControlSerial);
        break;
      case 14 : setpoint.flowTriggerSenstivity += 0.5; key_int_flag = true;
                limitValues();
                Serial1.print(F("Flow Trigger = ")); Serial1.println(setpoint.flowTriggerSenstivity);
              break;
      case 15 : setpoint.flowTriggerSenstivity -= 0.5; key_int_flag = true;
                limitValues();
                Serial1.print(F("Flow Trigger = ")); Serial1.println(setpoint.flowTriggerSenstivity);
              break;
              case 16:
              plotterData = !plotterData;
              Serial1.print(F("QT PLOTTER = ")); Serial1.println(plotterData);
        break;
        case 17:
        Serial1.println(F("Resetting Master"));
        resetFunction();
        break;
        case 18:
        slaveResetPinState = !slaveResetPinState;
        Serial1.print(F("slaveResetPinState = ")); Serial1.println(slaveResetPinState);
        digitalWrite(pin_SLAVE_RESET, slaveResetPinState);
        break;
        default:
                break;
    }
    limitValues();
    V_Mode_Breakdown();
    InitializeParams();
  }
  #endif
}

void limitValues()
{
    setpoint.reqVentMode            = constrain(setpoint.reqVentMode, 0, 4);
    setpoint.reqBPM                 = constrain(setpoint.reqBPM, minBPM, maxBPM);
    setpoint.reqFiO2                = constrain(setpoint.reqFiO2, 0, 3);
    if (setpoint.reqVentMode == VENT_MODE_CPAP)
        setpoint.reqPressure            = constrain(setpoint.reqPressure, minPressureCPAP, maxPressureCPAP);
    else
        setpoint.reqPressure            = constrain(setpoint.reqPressure, minPressure, maxPressure);
    setpoint.reqVolume              = constrain(setpoint.reqVolume, minVolume, maxVolume);
    setpoint.reqI_E_Section         = constrain(setpoint.reqI_E_Section, 0, 2);
    setpoint.flowTriggerSenstivity  = constrain(setpoint.flowTriggerSenstivity, -0.5, 5.0);    
}



