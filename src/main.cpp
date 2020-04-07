/*
 * All the #define are in Plateformio.ini file with -D name 
  OpenVentPK Prototype 1 OVPD-1 - Source Code
    //TODO:
        ADD Version History Here
        In Future;
        Implement Watchdog
        Address Wire.h long reported bug
        Implement Assist Control
        Implement Pressure Controlled Mode
        Interface O2 Sensor
        Interface Flow Rate Sensor

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
#include "gui.h"

LiquidCrystal_I2C lcd(0x27, 20, 4); // Address of LCD will be finalized from LCD Datasheet
//Other Address could be 0x20, 0x27 , 0x3F
// Run I2C_Scanner Script to discover device

#ifdef TX_SERIAL_TELEMETRY
#include "telemetry.h"
extern struct TEL_TYPE TEL;
#endif

#ifdef StepGen
// Define the stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, pin_Stepper_STEP, pin_Stepper_DIR);
#endif

#ifdef DC_MOTOR_VNH2SP30
// Define the MegaServo and the pins it will use
MegaServo megaServo;
#endif

#ifdef StepGen
boolean stopMove = true; // Immediately stop motor movements (E-Stop)
boolean runAtConstantSpeed = true;
#endif

int CVmode,          // CV or CP mode indicator;
    highPressureAlarmCnt, // delay when the pressure is too high (>1.2x preset value) before an alarm is triggered
    timCnt,               // Timer interrupt cycle counter used to spread the load over time
    timerLvl;             // Count timer interrupt nesting and skips interface update when more than one
 
boolean debug;             // True if a debugging mode is used, false otherwise
float plateauPressure, /* Plateau pressure is the pressure that is applied by the ventilator to the small airways and alveoli.
                           It is measured at end-inspiration with an inspiratory hold maneuver.*/
      PEEPressure,       // Positive end-expiratory pressure (PEEP)
      ambientPressure,   // Calculated ambiant air pressure (averaged)
      peakPressure,      // high pass filtered value of the pressure. Used to detect patient initiated breathing cycles
      avgPressure,       // Averaged pressure (used to limit the motor speed with short spikes filtered out
      relPressure;       // relative pressure (measured pressure - ambiant pressure)
unsigned long tick1,            // counter used to trigger sensor measurements //loop 1
         tick2,             // counter used to trigger sensor measurements // loop 2
         breathLength;      // duration of the current breathing cycle in milliseconds. This is 60000/BPM.

char  tempChar[20];


//double CodeVer;
// Parameters saved to / recovered from EEPROM

float reqBPM;                // respiratory frequency
float reqVolume;             // respiratory volume in milliliters
float reqPressure;           // compression for the ambu-bag in Pa
int reqExpirationRatioIndex; // The proportion of each breathing cycle that is spent breathing in compared to breathing out

float bpmSetpoint;             // respiratory frequency
float volumeSetpoint;          // respiratory volume in milliliters
float pressureSetpoint;        // compression for the ambu-bag in Pa
float expirationRatioSetpoint; // The proportion of each breathing cycle that is spent breathing in compared to breathing out

int   MotorSpeedSF, // Speed for 1 liter/second
      // Ratio of distance in steps to air volume in step per milliliter.
      MotorAccelerationSF, // Acceleration for 1 liter / second
      MotorMaxAcceleration;
float MotorVolumeSF;

int breathPhase = WAIT_PHASE;
int selfTestProg = ST_NOT_INIT; // Selft Test is Implemented
int selfTestStatus = ST_PASS;   // Selft Test is Implemented

#ifdef AUTO_HOME
int HomePos_Hit_F = 0, Homing_Done_F = 0;
#endif

int ErrorNumber = 0;
int devMode = 0;
int activateVentilatorOperation = 0;

int WarmUpFlag = 1;
int DevModeDetectionInProg = 0;
int PEEPMesaureFlag = 0;
int VentilatorOperationON = 0;
char PressedKey = 0;
char keyRead = 1;

#define debounceDelay 600 //ms
int OkButton = 0;
int SnoozeButton = 0;

int pressSnsrInUse = MS4525_IN_USE;

#ifdef DC_MOTOR_VNH2SP30
int motorInUse = DC_MOTOR_IN_USE;
#endif

#ifdef StepGen
int motorInUse = STEPPER_IN_USE;
#endif

int patientWeight = 50; //kg

int spStatusAllowChange = 0;
int I_E_InpsFactor[5] = {2, 1, 1, 1, 1};
int I_E_ExpFactor[5] = {1, 1, 2, 3, 4};
float I_E_SampleSet[5] = {2.0, 1.0, 0.5, 0.33, 0.25}; // 2:1, 1:1, 1:2, 1:3, 1:4


struct setpointStatus spStatus;
struct P_Sensor p_sensor;

#ifdef Beeper
int alarmAction = SNOOZE_ALARM;
unsigned int alarmSeverity = MUTE_ALARM;
unsigned long alarmDuration = 0; //milliseconds
#endif

void (*resetFunction)(void) = 0; // Self reset (to be used with watchdog)

boolean checkValues()
{
  boolean isOk = (reqBPM >= minBPM);                  // BPM in allowed range ?
  if (reqBPM > maxBPM) isOk = false;
  if (reqVolume < minVolume) isOk = false;            // Volume in allowed range ?
  if (reqVolume > maxVolume) isOk = false;
  if (reqPressure < minPressure) isOk = false;  // Compression in allowed range ?
  if (reqPressure > maxPressure) isOk = false;
  if (isnan(reqBPM)) isOk = false;                    // Check for malformed floating point values (NaN)
  if (isnan(reqVolume)) isOk = false;
  if (isnan(reqPressure)) isOk = false;

#ifdef DC_MOTOR_VNH2SP30
  if (MotorSpeedSF < minPWM) isOk = false;
  if (MotorSpeedSF > maxPWM) isOk = false;
  if (isnan(MotorSpeedSF)) isOk = false;                    // Check for malformed floating point values (NaN)
  if (isnan(MotorVolumeSF)) isOk = false;                    // Check for malformed floating point values (NaN)
#endif

  return isOk;
}

#ifdef Beeper

void beep() // Launch a beep
{
  static unsigned int currentToneFreq = MUTE_ALARM;

#ifdef ActiveBeeper
  if (alarmAction == SNOOZE_ALARM)
  {
    noTone(pin_Beep);
    currentToneFreq = MUTE_ALARM;
  }
  else
  {
    if (alarmSeverity > currentToneFreq)
    {
      tone(pin_Beep, alarmSeverity); //alarmDuration in milliseconds
      currentToneFreq = alarmSeverity;
    }
  }
#endif

#ifdef PassiveBeeper //Call Inside Interrupt if want to use Passive Beeper
  static unsigned long old_millis = 0;
  static unsigned long buzzerToggleTime = 0; //ms
  static int buzzer = HIGH;
  if (alarmAction == SNOOZE_ALARM)
  {
    digitalWrite(pin_Beep, LOW);
    currentToneFreq = MUTE_ALARM;
  }
  else
  {
    if (alarmSeverity > currentToneFreq)
    {
      currentToneFreq = alarmSeverity;
      buzzerToggleTime = (unsigned long)(500 / currentToneFreq); //ms
    }
    if ((millis() - old_millis) >= buzzerToggleTime)
    {
      old_millis = millis();
      digitalWrite(pin_Beep, buzzer);
      buzzer = !buzzer;
    }
  }
#endif
}
#endif

void eeput(int n) // records to EEPROM (only if values are validated)
{
#ifdef E2PROM
  int eeAddress = eeStart;
  boolean isOk = checkValues();

  if (n == 1) isOk = true; // override (for debug testing)
  if (isOk)
  {
    EEPROM.put(eeAddress, reqBPM);
    eeAddress += sizeof(float);
    EEPROM.put(eeAddress, reqVolume);
    eeAddress += sizeof(float);
    EEPROM.put(eeAddress, reqPressure);
    eeAddress += sizeof(float);
    EEPROM.put(eeAddress, MotorSpeedSF);
    eeAddress += sizeof(float);
    EEPROM.put(eeAddress, MotorVolumeSF);
    eeAddress += sizeof(float);
  }
#endif
}

void eeget()
{
  reqBPM = defaultBPM;
  reqVolume = defaultVolume;
  reqPressure = defaultPressure;
  reqExpirationRatioIndex = defaultExpirationRatioIndex;
#ifdef DC_MOTOR_VNH2SP30
  MotorSpeedSF = 127;
  MotorVolumeSF = 2.0;
#endif
#ifdef StepGen
  MotorSpeedSF = defMotorSpeed;
  MotorVolumeSF = defMotorVolumeRatio;
  MotorAccelerationSF = defMotorAcceleration;
  MotorMaxAcceleration = defMotorMaxAcceleration;
#endif
  Serial.print("Read Default Settings\n");  //Arduino gets stuck if comment this line
}

int homePosHitMotorPos = 0;
int OP2HitMotorPos = 0;

#ifdef StepGen
void Timer()
{
  if (digitalRead(pin_LmtSWT_OP2) == LOW)
  {
    if ((breathPhase == INSPIRATION_PHASE) || (breathPhase == HOLD_PHASE))
    {
      OP2HitMotorPos = stepper.currentPosition();
      stepper.moveTo(stepper.currentPosition());
      stopMove = true;
      runAtConstantSpeed = false;
    }
  }
  //  else if ((digitalRead(pin_LmtSWT_OP1) == LOW) && (breathPhase == EXPIRATION_PHASE))
  else if (HomePos_Hit_F == 1)
  {
    if (Homing_Done_F == 0)
    {
      Homing_Done_F = 1;
      stepper.setCurrentPosition(0);
      stopMove = true;
      runAtConstantSpeed = false;
    }
    else if ((breathPhase == EXPIRATION_PHASE) || (breathPhase == WAIT_PHASE))
    {
      homePosHitMotorPos = stepper.currentPosition();
      stepper.moveTo(stepper.currentPosition());
      stopMove = true;
      runAtConstantSpeed = false;
    }
  }
  if (!stopMove)
  {
    if (runAtConstantSpeed)
    {
      stepper.runSpeedToPosition(); //Non Blocking
    }
  }
}
#endif
#ifdef AUTO_HOME
#ifdef StepGen
void SetHomePosition(void)
{
  if (HomePos_Hit_F == 0)
  {
    //    Serial.print("Motor Current Pos: "); Serial.println(stepper.currentPosition());
    digitalWrite(pin_Stepper_MS1, HIGH); //Eighth step
    digitalWrite(pin_Stepper_MS2, HIGH);
    digitalWrite(pin_Stepper_MS3, LOW);
    stepper.move(InvertDir * -40);
    stepper.setSpeed(500);
    stopMove = false;
    runAtConstantSpeed = true;
  }
}
#endif
#endif

//Self Test and Auto Calibrate Routines
void selfTest()
{
  ErrorNumber = 0;
  selfTestStatus = ST_PASS;
  selfTestProg   = ST_IN_PROG;

#ifdef StepGen
#ifdef AUTO_HOME
  //  Serial.print("HomePos_Hit_F = ");Serial.println(HomePos_Hit_F);
  if (Homing_Done_F == 0)
  {
    //    Serial.println("Auto Homing in Progress");
    SetHomePosition();
    ErrorNumber     = 6;
    selfTestStatus  = ST_FAIL;
    return;
  }
  //  Serial.println("Auto Homing Complete");
  delay(5000);  //For Testing Only
#endif
#endif
  if (activateVentilatorOperation == 1)
  {
    //    Serial.println("Self Test FAIL");

    ErrorNumber     = 7;
    selfTestStatus  = ST_FAIL;
    selfTestProg    = ST_COMPLETE;
    return;
  }
  else
  {
    //    Serial.println("Self Test PASS");
    selfTestProg    = ST_COMPLETE;
    return;
  }

}
#ifdef MS4525DO
/*
   used to get values from MS4525 sensor
   and should be called after 10ms
   due to sensor update or refresh rate
*/
inline static void get_sensor_data(uint16_t *raw) {
  Wire.beginTransmission(I2C_ADDRESS_MS4525DO);
  Wire.write(1);
  Wire.endTransmission();
  Wire.requestFrom(I2C_ADDRESS_MS4525DO, 2); //request for two pressure bytes from sensor
  *raw = (Wire.read() & 0x3F) << 8; // read the msb from the I2C device
  *raw |= Wire.read();//read the lsb from the device
}
#endif

#ifdef BMP_180
void Bmp180Read()
{
  char status;
  double T, P, p0, a;
#ifdef __DEBUG
  Serial.println();
  Serial.print("provided altitude: ");
  Serial.print(ALTITUDE, 0);
  Serial.print(" meters, ");
  Serial.print(ALTITUDE * 3.28084, 0);
  Serial.println(" feet");
#endif
  // If you want to measure altitude, and not pressure, you will instead need
  // to provide a known baseline pressure. This is shown at the end of the sketch.

  // You must first get a temperature measurement to perform a pressure reading.

  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = bmp180.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = bmp180.getTemperature(T);
    if (status != 0)
    {
      p_sensor.bmp_temperature = T;
#ifdef __DEBUG
      // Print out the measurement:
      Serial.print("temperature: ");
      Serial.print(T, 2);
      Serial.print(" deg C, ");
      Serial.print((9.0 / 5.0)*T + 32.0, 2);
      Serial.println(" deg F");
#endif
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = bmp180.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = bmp180.getPressure(P, T);
        if (status != 0)
        {
          p_sensor.bmp_pressure = P;
#ifdef __DEBUG
          // Print out the measurement:
          Serial.print("absolute pressure: ");
          Serial.print(P, 2);
          Serial.print(" mb, ");
          Serial.print(P * 0.0295333727, 2);
          Serial.println(" inHg");

          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb

          p0 = bmp180.sealevel(P, ALTITUDE); // we're at 1655 meters (Boulder, CO)
          Serial.print("relative (sea-level) pressure: ");
          Serial.print(p0, 2);
          Serial.print(" mb, ");
          Serial.print(p0 * 0.0295333727, 2);
          Serial.println(" inHg");

          // On the other hand, if you want to determine your altitude from the pressure reading,
          // use the altitude function along with a baseline pressure (sea-level or other).
          // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
          // Result: a = altitude in m.

          a = bmp180.altitude(P, p0);
          Serial.print("computed altitude: ");
          Serial.print(a, 0);
          Serial.print(" meters, ");
          Serial.print(a * 3.28084, 0);
          Serial.println(" feet");
#endif
        }
#ifdef __DEBUG
        else Serial.println("error retrieving pressure measurement\n");
#endif
      }
#ifdef __DEBUG
      else Serial.println("error starting pressure measurement\n");
#endif
    }
#ifdef __DEBUG
    else Serial.println("error retrieving temperature measurement\n");
#endif
  }
#ifdef __DEBUG
  else Serial.println("error starting temperature measurement\n");
#endif
  //delay(5000);  // Pause for 5 seconds.
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
  bool updated = false;
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
  static int initHold = 1;
  static int initMeasurePEEP = 1;
#ifdef MPXV7002DP
  float voltage_read = analogRead(MPX_IN) * ADC_TO_VOLTS;
  /* pressure = 1000 * (((5 * voltageRead) / 5) - (5 / 2));
     https://makersportal.com/blog/2019/02/06/arduino-pitot-tube-wind-speed-theory-and-experiment */
  p_sensor.diff_press_pa = (1000 * voltage_read) - 2500;
#elif defined (MPX4250)
  float voltage_read = analogRead(MPX_IN) * ADC_TO_VOLTS;
  // Typical MPV is 0.204v@20KPa
  // Maximum pressure is 250KPa
  // pressure = voltage * (250 / (MPV + 4.692))
  p_sensor.diff_press_pa = voltage_read * 51.06209150326797f;
#elif defined (MPX2010DP)
  float voltage_read = analogRead(MPX_IN) * ADC_TO_VOLTS;
  //changing voltage to pressure 100kPa/3.5v
  p_sensor.diff_press_pa = voltage_read * 2857.14285714286f;
#elif defined (MPX10DP)
  float voltage_read = analogRead(MPX_IN) * ADC_TO_VOLTS;
  //changing voltage to pressure 100kPa/3.5v
  p_sensor.diff_press_pa = voltage_read * 2857.14285714286f;
#elif defined (MS4525DO)
  // Calculate differential pressure. As its centered around 8000
  // and can go positive or negative
  uint16_t raw_pressure = 0;
  get_sensor_data(&raw_pressure);

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
#endif

#ifdef BMP_180
  Bmp180Read();
#endif

  //FLOW SENSORS

  // Plateau Pressure & PEEP
  switch (breathPhase)
  {
    case HOLD_PHASE:
      if (initHold == 1)
      {
        plateauPressure = (p_sensor.pressure_gauge_CM * cmH2O_to_Pa);
        initHold = 0;
      }
      else
      {
        plateauPressure = (plateauPressure + (p_sensor.pressure_gauge_CM * cmH2O_to_Pa)) * 0.5;
      }
      break;
    case EXPIRATION_PHASE:
      if (PEEPMesaureFlag == 1)
      {
        if (initMeasurePEEP == 1)
        {
          PEEPressure = (p_sensor.pressure_gauge_CM * cmH2O_to_Pa);
          initMeasurePEEP = 0;
        }
        else
        {
          PEEPressure = (PEEPressure + (p_sensor.pressure_gauge_CM * cmH2O_to_Pa)) * 0.5;
        }
      }
      initHold = 1;
      break;
    default: //WAIT AND INSPIRATION PHASE
      initHold = 1;
      initMeasurePEEP = 1;
      break;
  }

}

void meas() //print all sensoe values on Serial Port Monitor
{
}

/*
     This is the main Alarm Control.
        Sensor Monitoring.
        Alarm Triggering.
*/
void alarmControl() // Read Values from Installed Sensor
{
}

bool timer3InterruptTriggered = false;

void timer3ISR()
{
  timer3InterruptTriggered = true;
}
void setup()
{
  // put your setup code here, to run once:

  unsigned long pre_millis = 0;
  unsigned long prePressedTimestamp = 0;
  unsigned int isPressedTime = 0;

  plateauPressure = 0.0;
  PEEPressure = 0.0;


  //    timCnt = 0,                // Timer interrupt cycle counter used to spread the load over time
  timerLvl = 0;

  CVmode = VOL_CONT_MODE; //Volume Controlled Mode

#ifdef DC_MOTOR_VNH2SP30
  megaServo.attach(pin_M1_CW, pin_M1_CCW, pin_M1_PWM, pin_M1_POT);
#endif

  Timer3.initialize(100000);   //microseconds //0.10sec
  //Timer3.attachInterrupt(userInterface);
  Timer3.attachInterrupt(timer3ISR);

#ifdef StepGen
  //    pinMode(pin_Stepper_DIR, OUTPUT);
  //    pinMode(pin_Stepper_STEP, OUTPUT);
  pinMode(pin_Stepper_MS1, OUTPUT);
  pinMode(pin_Stepper_MS2, OUTPUT);
  pinMode(pin_Stepper_MS3, OUTPUT);
  //   pinMode(pin_Stepper_EN,    OUTPUT);
  pinMode(pin_Stepper_SLP, OUTPUT);
  //   pinMode(pin_Stepper_RST, OUTPUT);

  /*Stepper Motor Detailed Guide with Driver: https://lastminuteengineers.com/a4988-stepper-motor-driver-arduino-tutorial */

  // Reset the position to 0:
#ifndef AUTO_HOME
  stepper.setCurrentPosition(0); //if limit switch not installed.
#endif
  //if limit switch is installed then move motor to limit switch position and set that position as zero.
  stepper.setMaxSpeed(defMotorMaxSpeed);
  stepper.setAcceleration(defMotorMaxAcceleration);

  //Set Motor Step Size using MS1-MS2-MS3 of motor driver
  /*
      MS1   MS2   MS3   Microstep Resolution
      Low   Low   Low   Full step
      High  Low   Low   Half step
      Low   High  Low   Quarter step
      High  High  Low   Eighth step
      High  High  High  Sixteenth step

  */
  //Full Step
  //    digitalWrite(pin_Stepper_STEP, LOW);
  //    digitalWrite(pin_Stepper_DIR, LOW);
  digitalWrite(pin_Stepper_MS1, LOW);
  digitalWrite(pin_Stepper_MS2, LOW);
  digitalWrite(pin_Stepper_MS3, LOW);
  stopMove = true;
  runAtConstantSpeed = false;

  Timer1.initialize(200);
  Timer1.attachInterrupt(Timer);

#endif
  pinMode(pin_Button_OK, INPUT);
  pinMode(pin_Button_SNZ, INPUT);
  pinMode(pin_Switch_START, INPUT);
  pinMode(pin_Switch_MODE, INPUT);

#ifdef AUTO_HOME
  pinMode(pin_LmtSWT_OP1, INPUT);
#endif
  pinMode(pin_LmtSWT_OP2, INPUT);
  //    pinMode(pin_LmtSWT_CL1, INPUT);
  //    pinMode(pin_LmtSWT_CL2, INPUT);

  pinMode(pin_Knob_1, INPUT);
  pinMode(pin_Knob_2, INPUT);
  pinMode(pin_Knob_3, INPUT);
  pinMode(pin_Knob_4, INPUT);
#ifdef Beeper
  pinMode(pin_Beep, OUTPUT);
#endif
#ifdef Keypad_4x3
  //     keypad.addEventListener(keypadEvent);  // Add an event listener.
  keypad.setHoldTime(500);               // Default is 1000mS
  keypad.setDebounceTime(250);           // Default is 50mS
#endif
  Wire.begin();
  lcd.init(); //intializing lcd
  //lcd.begin(20, 4);
  lcd.backlight(); //setting lcd backlight

#ifdef BMP_180
  bmp_180.begin();//initialiazing BMP180
#ifdef __DEBUG
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.
    Serial.println("BMP180 init fail\n\n");
  }
#endif
#endif

  Serial.begin(SERIAL_BAUD);
  //#ifdef TX_SERIAL_TELEMETRY
  //    Serial1.begin(SERIAL_BAUD);
  //#endif
  //Serial.print("Open Source Ventilator Pakistan Prottype 1 SW Ver: "); Serial.println(CodeVer);
  //sprintf(tempChar, "*OpenVentPk Ver %1d.%1d*",CODE_VER_MAJOR,CODE_VER_MINOR);;Serial.print(tempChar);


  //    noInterrupts();
  pre_millis = millis();
  //  Serial.println("Entering Warmpup");

  while ((millis() - pre_millis) < WARM_UP_TIME)
  {
    Serial.println(millis() - pre_millis);
    prePressedTimestamp = millis();
    isPressedTime = millis() - prePressedTimestamp;

    //    Serial.print("Motor Current Pos: "); Serial.println(stepper.currentPosition());


    while (OkButton == HIGH && SnoozeButton == HIGH && isPressedTime < 2000)
    {
      isPressedTime = millis() - prePressedTimestamp;
      if (timer3InterruptTriggered)
      {
        DevModeDetectionInProg = 1;
        userInterface();
        timer3InterruptTriggered = false;
      }
    }
    DevModeDetectionInProg = 0;

    if (isPressedTime >= 2000)
    {
      devMode = 1;
      WarmUpFlag = 0;
      devModeFunc(); //MOVE LCD_DISPLAY to Userinterface
    }
    if (timer3InterruptTriggered)
    {
      userInterface();
      timer3InterruptTriggered = false;
    }
  }
  //    Serial.println("Exiting Warmpup");

  //    interrupts();

  noInterrupts();
  //Fetch Motor Speed and Volume displace SF from EEPROM
  eeget();    // read startup parameters (either from EEPROM or default value)

  spStatus.curI_E_Section = reqExpirationRatioIndex;
  spStatus.curBPM = reqBPM;
  spStatus.curTV = reqVolume;
  spStatus.curOP = reqPressure;

  bpmSetpoint = reqBPM;                 // Start from these values without sweep
  volumeSetpoint = reqVolume;
  pressureSetpoint = reqPressure;
  expirationRatioSetpoint = I_E_SampleSet[reqExpirationRatioIndex - 1];

  interrupts();

  tick1 = millis();
  tick2 = millis();

#ifdef StepGen
  //    stopMove = false;
#endif
}

void devModeFunc() //Developer Mode
{
  bpmSetpoint = reqBPM; // Start from these values without sweep
  volumeSetpoint = reqVolume;
  pressureSetpoint = reqPressure;
  expirationRatioSetpoint = I_E_SampleSet[reqExpirationRatioIndex - 1];

  while (devMode == 1)
  {
    userInterface();
    delay(1000);
  }
}

void limitValues(float &B, float &V, float &P) // prevent values from going outside of their design limits
{
}
void Ventilator_Control()
{
  static unsigned int Tin = 0;
  static unsigned int Tex = 0;
  static unsigned int Th = 150; //ms
  static unsigned long Tcur = 0;
  static unsigned long BreathStartTimestamp = 0;

  static float BVin = 0;
  static float BVex = 0;

  static int Vin = 0;
  static int Vex = 0;
  static int reqMotorPosInspiration = 0;

  static unsigned int init = 1;

  //    noInterrupts();
  CVmode = VOL_CONT_MODE; //Proto-1
  if (init == 1)
  {
    breathLength = (int)(60000 / bpmSetpoint);
    // Take the hold time out of the exhale cycle. Do this to ensure respitory rate is correct.
    Tex = (int)((breathLength - Th) / (1 + expirationRatioSetpoint)); // if I/E ratio = 0.5 ; it means expiration is twice as long as inspiration
    Tin = (int)(Tex * expirationRatioSetpoint);
    init = 0;
    Tcur = breathLength;
  }
#ifdef StepGen
  digitalWrite(pin_Stepper_MS1, LOW); //Full Step
  digitalWrite(pin_Stepper_MS2, LOW);
  digitalWrite(pin_Stepper_MS3, LOW);
#endif
  if (activateVentilatorOperation == 1)
  {
    VentilatorOperationON = 1;

    if (Tcur >= breathLength)
    {
      Tcur = 0;
      bpmSetpoint = reqBPM;                 // Load Fresh User Settings
      volumeSetpoint = reqVolume;
      pressureSetpoint = reqPressure;
      expirationRatioSetpoint = I_E_SampleSet[(int)reqExpirationRatioIndex - 1];

      breathLength = (int)(60000 / bpmSetpoint);
      // Take the hold time out of the exhale cycle. Do this to ensure respitory rate is correct.
      Tex = (int)((breathLength - Th) / (1 + expirationRatioSetpoint)); // if I/E ratio = 0.5 ; it means expiration is twice as long as inspiration
      Tin = (int)(Tex * expirationRatioSetpoint);
      BVin = (volumeSetpoint / Tin); //Breathe In Speed in ml/s
      BVex = (volumeSetpoint / Tex); //Breathe Out Speed in ml/s
      Vin = (int)(BVin * MotorSpeedSF); //Breathe In Motor Speed in steps/s
      Vex = (int)(BVex * MotorSpeedSF); //Breathe Out Motor Speed in steps/s
#ifdef StepGen
      reqMotorPosInspiration = (int)(volumeSetpoint * MotorVolumeSF); //in Steps
      reqMotorPosInspiration = constrain(reqMotorPosInspiration, 0, MaxMotorPosition);
#elif DC_MOTOR_VNH2SP30
      reqMotorPosInspiration = volumeSetpoint; //in mL (mega Servos maps mL to Pot automatically)
#endif
      BreathStartTimestamp = millis();

      static int i = 0;
#ifdef __DEBUG
           Serial.print("In Ventilator Control: "); Serial.println(i++);
           Serial.print("Breathing Length: "); Serial.println(breathLength);
           Serial.print("Inspiration Time: "); Serial.println(Tin);
           Serial.print("Expiration Time: "); Serial.println(Tex);
           Serial.print("Motor Speed: "); Serial.println(Vin);
           Serial.print("Motor Current Pos (Exp): "); Serial.println(stepper.currentPosition());
           Serial.print("targetPosition: "); Serial.println(reqMotorPosInspiration);
           Serial.print("Breathing Phase: "); Serial.println(breathPhase);
#endif
    }
    Tcur = millis() - BreathStartTimestamp;

    if (CVmode == VOL_CONT_MODE)
    {
      if (Tcur <= Tin)
      {
        //              if (breathPhase == EXPIRATION_PHASE)
        //              {// Initializations
        //              }
        breathPhase = INSPIRATION_PHASE;
#ifdef StepGen
        stepper.moveTo(InvertDir * reqMotorPosInspiration);
        stepper.setSpeed(Vin);
        stopMove = false;
        runAtConstantSpeed = true;
#elif DC_MOTOR_VNH2SP30
        megaServo.setSpeed(Vin);
        megaServo.writeVolume(reqMotorPosInspiration);
#endif
        PEEPMesaureFlag = 0;
      }
      else if ((Tcur > Tin) && (Tcur <= (Tin + Th)))
      {
        if (breathPhase == INSPIRATION_PHASE)
        { //Initialization
        }
        breathPhase = HOLD_PHASE;
#ifdef StepGen
        stepper.moveTo(stepper.currentPosition());
        stopMove = true; //Immediately stop the motor by blocking run command
        runAtConstantSpeed = false;
#elif DC_MOTOR_VNH2SP30
        megaServo.setSpeed(0);
        megaServo.stop();
#endif
      }
      else if ((Tcur > (Tin + Th)) && (Tcur < (Tin + Th + Tex)))
      {
        if (breathPhase == HOLD_PHASE)
        { //Initializations
          //                    Serial.print("Motor Current Pos (Hold): "); Serial.println(stepper.currentPosition());
        }
        breathPhase = EXPIRATION_PHASE;
#ifdef StepGen
        stepper.moveTo(0);
        stepper.setSpeed(Vex);
        stopMove = false;
        runAtConstantSpeed = true;
#elif DC_MOTOR_VNH2SP30
        megaServo.setSpeed(Vex);
        megaServo.writeVolume(0);
        if (digitalRead(pin_LmtSWT_OP1) == HIGH) //HOME POS
        {
          megaServo.setSpeed(0);
          megaServo.stop();
        }
#endif
        if ((Tcur >= (Tin + Tex)) && (Tcur < (Tin + Th + Tex)))
        {
          PEEPMesaureFlag = 1;
        }
      }
    }
  }
  else
  {
    //      Serial.println("Ventilator Operation Halt");
    VentilatorOperationON = 0;
    breathPhase = WAIT_PHASE;
    Tcur = breathLength; // This will always start inspiration breath cycle on setting switch to start position
#ifdef StepGen
    //        stepper.moveTo(stepper.currentPosition());
    stepper.moveTo(0); // to remove ambu bag for manual bagging in case of emergency
    stepper.setSpeed(Vex);
    stopMove = false;
    runAtConstantSpeed = true;
    PEEPMesaureFlag = 0;
#elif DC_MOTOR_VNH2SP30
    megaServo.setSpeed(0);
    megaServo.stop();
#endif
  }
  //    interrupts();
}

#ifdef TX_SERIAL_TELEMETRY
void GetTelData()
{
  static int init = 1;
  unsigned int TEL_BYTE = 0x00;
  if (init == 1)
  {
    TEL.Time = 0;
    TEL.txUpdateRate = 0;
    TEL.txPktCtr = 0;
    init = 0;
  }

  TEL.Time += (samplePeriod1);

  TEL.mTV = 390.0;//0.0;
  TEL.mPressure = 20.0;//p_sensor.pressure_gauge_CM;
  TEL.mFlowRate = 70.0;//0.0; //SLPM
  TEL.mPEEP = 5.0;//PEEPressure * Pa2cmH2O;
  TEL.mPltPress = 25.0;//plateauPressure * Pa2cmH2O;
  TEL.mFiO2 = 0.0;

  TEL.spTV = reqVolume;
  TEL.spPressure = reqPressure * Pa2cmH2O;
  TEL.spFiO2 = 50.0;//0.0;
  TEL.spPEEP = 0.0 * Pa2cmH2O;
  TEL.spBPM = reqBPM;
  TEL.spIE_Inhale = I_E_InpsFactor[reqExpirationRatioIndex - 1];
  TEL.spIE_Exhale = I_E_ExpFactor[reqExpirationRatioIndex - 1];
  TEL.patientWeight = patientWeight;

  TEL.statusByteError = ErrorNumber;
  TEL_BYTE = 0x00;
  TEL_BYTE |= breathPhase & 0x03;
  if (CVmode == PRESS_CONT_MODE)  TEL_BYTE |= 0x04;
  TEL_BYTE &= 0xF7; //Mandatory Mode : D3 = 0
  if (VentilatorOperationON == 1)  TEL_BYTE |= 0x10;
  TEL_BYTE |= (selfTestProg & 0x03) << 5;
  if (selfTestStatus == ST_PASS)  TEL_BYTE |= 0x80;
  TEL.statusByte1 = TEL_BYTE;
}
#endif
void loop()
{
  WarmUpFlag = 0;
#ifdef StepGen
  digitalWrite(pin_Stepper_SLP, HIGH);
#endif
  if (millis() > (tick1 + samplePeriod1))
  {
    tick1 = millis();
    readSensors();

    if (selfTestProg != ST_COMPLETE)
      selfTest();
    else
    {
      alarmControl();
    }
#ifdef ActiveBeeper
    beep(); //alarmAction = RING_ALARM, SNOOZE_ALARM; alarmSeverity = SEVERITY_HIGH, SEVERITY_MED, SEVERITY_LOW, SEVERITY_MUTE
#endif

#ifdef TX_SERIAL_TELEMETRY
    GetTelData(); //Called at 100Hz
    Prepare_Tx_Telemetry(); //Called at 100Hz
#endif
  }
  if ((selfTestProg == ST_COMPLETE) && (selfTestStatus == ST_PASS)) // I am not writing control loop inside 100Hz loop to keep both loop rates independant
  {
    if (millis() > (tick2 + samplePeriod2))
    {
      tick2 = millis();
      Ventilator_Control(); //Mandatory Volume Controlled Mode Only
    }
  }
  if (timer3InterruptTriggered)
  {
    userInterface();
    timer3InterruptTriggered = false;
  }
}