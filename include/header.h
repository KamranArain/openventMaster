#ifndef __HEADER_H
#define __HEADER_H

#include <Arduino.h> //For PlatformIO
#ifndef __AVR__
#include <HardwareSerial.h>
// #include "uTimerLib.h" TODO

#define USART6_TX PC_6  //| USART6_TX             |
#define USART6_RX PC_7  //| USART6_RX             |

#define USART2_TX PB_10 //| USART3_TX             |
#define USART2_RX PB_11 //| USART3_RX             |

#define USART1_TX PA_9  //| USART1_TX             |
#define USART1_RX PA_10 //| USART1_RX             |

#define UART4_TX PC_10  //| UART4_TX              |
#define UART4_RX PC_11  //| UART4_RX              |

extern HardwareSerial Serial1;
extern HardwareSerial Serial3;
extern HardwareSerial Serial4;
#endif

#define CODE_VER_MAJOR 2
#define CODE_VER_MINOR 7 //(Took hold time out of expiration, Homing Done moved to interrupt, Plateau Pressure and PEEP measurements moved to readsensors function)
// #define QT_PLOTTER
//************************   DEVICE OPERATIONAL PARAMETERS   ************************/
/*
       WARNING : When changing min and max value, manually change the text in the SerialCommand procedure accordingly

       The values are taken from various documents including :
         https://www.gov.uk/government/publications/coronavirus-covid-19-ventilator-supply-specification/rapidly-manufactured-ventilator-system-specification
         https://docs.google.com/document/d/1h77FkKXqPOwVqfOj-PIJSjYX9QiEx69Av2gYuzqZolw/edit#

       some changes have been made to havee a lower default volume in case the machine is used
       with an infant to prevent damaging their lungs with an adult setting.*/

#define Pa2cmH2O 0.0101972
#define cmH2O_to_Pa 98.0665

#define minBPM 8                      // minimum respiratory speed
#define defaultBPM 20                 // default respiratory speed
#define maxBPM 35                     // maximum respiratory speed
#define minVolume 200                 // minimum respiratory volume in milliliters
#define defaultVolume 500             // default respiratory volume in milliliters
#define maxVolume 500                 // maximum respiratory volume in milliliters
#define minPressure 0                 // minimum compression for the ambu-bag in cmH2O
#define minPressureCPAP 5             // minimum compression for the ambu-bag in cmH2O
#define defaultPressure 30            // default compression for the ambu-bag in cmH2O
#define maxPressureCPAP 20            // maximum compression for the ambu-bag in cmH2O //approx 40cH20
#define maxPressure 40                // maximum compression for the ambu-bag in cmH2O //approx 40cH20
#define minWeight 2                   // minimum compression for the ambu-bag in Pa
#define maxWeight 150                 // minimum compression for the ambu-bag in Pa
#define defaultExpirationRatioIndex 0 //Corresponds to 1:2 see definition: IE_R_Value
#define defaultFIO2Index 0
#define ADC_TO_VOLTS 0.004887585532746823 //0.004887585532746823 is from 5v/1023

/*******************************   MOTOR PARAMETERS FOR STEPPER MOTOR   *******************************

       These values will be highly dependant on mechanical design.
       When modifying the values, always check with an oscilloscope that the movements completes
       without overlaps and without too much idle time.
       Also check that the motor can properly follow the speed acceleration required under load.
       Wrong values can cause unpredictables moves and motor stalls.
       The worst case scenario is at max BPM, with max volume, max sync period and max ratio
       With default parameters, the whole compression can become as short as 250 ms
*/

#define FULL_SCALE_VOLUME 800.0f //ml
#define FULL_SCALE_LENGTH 35.0f  //mm
#define LINEAR_FACTOR_VOLUME 22.86f
#define LIN_MECH_mm_per_rev 5.0f
#define STEPPER_MICROSTEP 4.0f
#define STEPPER_PULSES_PER_REV 200.0f
/*******************************   HARDWARE OPTIONS   *******************************
   It's normal for the program to not compile if some of these are undefined as they need an alternative*/

//******************************   IMPIED DEFINITIONS  ********************************
#ifdef ActiveBeeper
#define Beeper
#endif

#define RING_ALARM 1
#define SNOOZE_ALARM 0

//Frequency in HERTZ
#define SEVERITY_HIGH_FREQ 3000
#define SEVERITY_MED_FREQ 2000
#define SEVERITY_LOW_FREQ 1000

//Durations in milliseconds
#define SEVERITY_HIGH_TP 250
#define SEVERITY_MED_TP 750
#define SEVERITY_LOW_TP 1500

//******************************   MACROS  ********************************
#define HOMING_PERIOD 100

#define WAIT_PHASE 0
#define INSPIRATION_PHASE 1
#define HOLD_PHASE 2
#define EXPIRATION_PHASE 3

#define ST_NOT_INIT 0
#define ST_IN_PROG 1
#define ST_COMPLETE 2
#define ST_FAIL 0
#define ST_PASS 1
#define WARM_UP_TIME 5 * 1000

#define HEALTH_BAD 0
#define HEALTH_GOOD 1

#define DC_MOTOR_IN_USE 1
#define STEPPER_IN_USE 0

#define VOL_CONT_MODE 0
#define PRESS_CONT_MODE 1

#define VENT_MODE_VCV 0
#define VENT_MODE_PCV 1
#define VENT_MODE_AC_VCV 2
#define VENT_MODE_AC_PCV 3
#define VENT_MODE_CPAP 4

/***************end***********************/

//********************************   CONNECTION PINS   ********************************
// ATmega2560-Arduino Pin Mapping: https://www.arduino.cc/en/Hacking/PinMapping2560
#ifdef I2C
#define pin_SDA 20
#define pin_SCL 21
#endif

//#define MPX_IN A4

#ifdef Led
#define pin_LED1 48
#endif

// #define pin_Button_OK 42
#define pin_Switch_START 28
// #define pin_Switch_MODE 45
#define pin_SNOOZBTN 3
#define pin_BUZZER 37
// #define pins_KEYPAD ((PINA & 0xF0) >> 4)
#if defined(__AVR__)
#define pins_KEYPAD (PINA & 0x0F)
#endif
#define pin_KEYPAD_INTERRUPT 2

// #define pin_Knob_1 A12 //VR1 // I/E Ratio
// #define pin_Knob_2 A13 //VR2 // BPM
// #define pin_Knob_3 A14 //VR3 // Tidal Volume
// #define pin_Knob_4 A15 //VR4 // Max Pressure

//#define pin_LmtSWT_OP1 46 //HOME POSITION
//#define pin_LmtSWT_OP2 47
// #define pin_LmtSWT_CL1 48
// #define pin_LmtSWT_CL2 49

#define pin_SLAVE_RESET 31

// #define pin_M1_POT A11 //VR5

#define SERIAL_BAUD 115200 // Serial port communication speed

#ifdef E2PROM
#define eeStart 48 // EEPROM Offset for data
#define ee_reqBPM eeStart
#define ee_reqVol ee_reqBPM + (sizeof(float));
#define ee_reqPres ee_reqVol + (sizeof(float));
#define ee_reqI_E ee_reqPres + (sizeof(float));
#define ee_reqFiO2 ee_reqI_E + (sizeof(float));
#define ee_Trigger ee_reqFiO2 + (sizeof(float));
#define ee_Vol_Coef_a ee_Trigger + (sizeof(float));
#define ee_Vol_Coef_b ee_MVol_Coef_a + (sizeof(float));
#define ee_Vol_Coef_c ee_MVol_Coef_b + (sizeof(float));
#define ee_Vol_Coef_d ee_MVol_Coef_c + (sizeof(float));
#endif

#define samplePeriod1 10 // 5 ms sampling period
#define samplePeriod2 10 // 10 ms Control Loop

#define highPressureAlarmDetect 10 // delay before an overpressure alarm is triggered (in samplePeriod increments)

//Calibration Parameters
#define STEPPERRANGE 40  //mm
#define stepSize 1       //mm
#define ORDER 3          //DO not exceed 20.
#define ORDER_PRESS_EQ 3 //DO not exceed 20.

//*******************************   REQUIRED LIBRARIES   *******************************
#ifdef I2C
#include <Wire.h>              // I2C Library 2 wire Protocol
#include <LiquidCrystal_I2C.h> // Library for LCD //Liquid Crystal I2C by Frank de Brabander
#endif

#ifdef E2PROM
#include <EEPROM.h> // read / write to the processor's internal EEPROM
#endif

#if defined(__AVR__)
#include "TimerOne.h" // Timer component
//  By Jesse Tane, Jérôme Despatis, Michael Polli, Dan Clemens, Paul Stroffregen
//  https://playground.arduino.cc/Code/Timer1/
#include "TimerThree.h" // Timer3 component
#endif
//***************************************   FUNCTION PROTOTYPES   ***************************************
//void Timer1ISR();

void selfTest();
void calibrate(int calibParam);
void Monitoring();
void Ventilator_Control();

#ifdef TX_SERIAL_TELEMETRY
void GetTelData();
#endif
//#ifdef E2PROM
//#endif
void eeput(); // records to EEPROM (only if values are validated)
void eeget();

void txSlaveCMD(int CMD_ID, unsigned int period = 0, unsigned int pulses = 0, String dir = "0");
void decodeSlaveTel();
void receiveSlaveTel();
//***************************************   END   ***************************************

struct setpointStatus
{
  uint8_t curI_E;         //I/E Ratio
  uint8_t curI_E_Section; //I/E Ratio
  uint8_t curBPM;         // BPM
  uint16_t curVolume;     //Tidal Volume Setpoint
  uint8_t curPressure;    //Insp pressure limit
  uint8_t curFiO2;        //Oxygen Concentration
  uint8_t curVentMode = VENT_MODE_VCV;
  uint8_t curControlVariable = VOL_CONT_MODE;
  bool curAssistMode_F = 0;
  bool curEnableCPAP_F = 0;

  uint8_t reqI_E_Section;      //I/E Ratio
  uint8_t reqBPM;              // BPM
  uint16_t reqVolume;          //Tidal Volume Setpoint
  uint8_t reqPressure;         //Insp pressure limit
  uint8_t reqFiO2;             //Oxygen Concentration
  float flowTriggerSenstivity; //Lpm trigger for Assist mode
  uint8_t reqVentMode = VENT_MODE_VCV;
  uint8_t reqControlVariable = VOL_CONT_MODE;
  bool reqAssistMode_F = 0;
  bool reqEnableCPAP_F = 0;

  uint8_t patientWeight = 70; //kg
};

struct TidalVolume
{
  float measured = 0.0;
  float inspiration = 0.0;
  float expiration = 0.0;
  float minuteVentilation = 0.0;
  float staticCompliance = 0.0; // (ml / cmH2O)
  float maxInhale = 0.0;
};

struct STATUS_FLAGS
{
  bool hours96_complete = false;
  bool battInUse = false;
  bool homingFailure = false;
  bool circuitIntegrityFailure = false;
  bool mechIntergrityFaiure = false;
  bool oxygenFailure = false;
  bool ventCktDisconnected = false;
  bool flowSensorFailure = false;
  bool presSensorFailure = false;
  bool o2SensorFailure = false;
  bool compressionMechFailure = false;
  bool systemReset = true;

  bool PeepValid = false;
  bool PltPrsValid = false;
  bool PeakPrsValid = false;
  bool TidalVolValid = false;
  bool MinVolValid = false;
  bool RRValid = false;
  bool FIO2Valid = false;
  bool ventCktDisconnectedValid = false;

  bool homeAtBadSensor = false;
  bool homeAtBadFlowSensor = false;
  bool homeAtBadPressSensor = false;
  bool WarmUpFlag = true;

  uint8_t breathPhase = WAIT_PHASE;
  uint8_t VentilatorOperationON = 0;
  uint8_t selfTestProg = ST_NOT_INIT; // Selft Test is Implemented
  uint8_t selfTestStatus = ST_PASS;   // Selft Test is Implemented
  uint8_t sensorsHealth = HEALTH_BAD;
  uint8_t Homing_Done_F = 0;
};

struct MONITORING_PARAMS
{
  float plateauPressure, /* Plateau pressure is the pressure that is applied by the ventilator to the small airways and alveoli.
                           It is measured at end-inspiration with an inspiratory hold maneuver.*/
      PEEPressure,       // Positive end-expiratory pressure (PEEP)
      peakInspPressure;  // high pass filtered value of the pressure. Used to detect patient initiated breathing cycles
  uint8_t measuredRR = 0;
  unsigned long RR_timestamp = 0;
  unsigned long MV_timestamp = 0;
  unsigned long TV_timestamp = 0;
  unsigned long PR_timestamp = 0;
  unsigned long CV_timestamp = 0;
};

struct Slave
{
  int lastCMD_ID = 0;
  int runAck = 0;
  int stopAck = 0;
  int homeAck = 0;
  bool strComplete = false;
  String AckStr = "";
};

#define TIME_VAR 0
#define FLOW_VAR 1
#define PRESS_VAR 2
#define VOL_VAR 3

#define NO_CMD 0
#define RUN 1
#define STOP 2
#define HOME 3

#define HOMING_CMD_NOT_SENT 0
#define CMD_RECEIVED 1
#define CMD_COMPLETE 2
#define CMD_ERROR 3

//***************************************   END   ***************************************
#endif
/* NOTE*****************************************************************************************************************8
    Minor version change required in Future.
         This version allows the replacement of the buggy Wire.h arduino library  (can hang
         the controller) with acorrect version known as jm_Wire
         https://github.com/ermtl/Open-Source-Ventilator/blob/master/OpenSourceVentilator/README.md
         'Wire.h' is still the default library to prevent compiler errors.
         The processor's hardware watchdog can now be enabled (off by default, use with care, you
         risk bricking your processor.
         Modularisation is getting better (work in progress)

  #include <jm_Wire.h>              // I2C protocol, contains a workaround for a longstanding issue with the Wire library
                                  // by Jean-Marc Paratte - https://github.com/jmparatte/jm_Wire
                                  // The library is optional during developpement (unless you encounter lockups)
                                  // but must be used for any production software. To use it, you need to change
                                  // the #include "Wire.h" line in all libraries that use the I2C bus.


******************************************************************************************************************************/