#include "header.h"
#include "curveFitting.h"

//#define ALTERNATE_CALIBRATION_SCHEME

extern struct Slave slave;
extern struct TidalVolume TV;
extern int Homing_Done_F;

byte calibStatus = ST_COMPLETE;//ST_NOT_INIT;//ST_COMPLETE;//
byte estimateVolume = false;

double VolCoeffs[ORDER+1];

void calibrate(){

  unsigned int period = 1000; //us
 
  static double steps[(STEPPERRANGE/stepSize)+1]; //mm
  static double volume[(STEPPERRANGE/stepSize)+1]; 

  static unsigned long reqMotorSteps = 0;
  static int i = stepSize;
  static int j = 1;
  static bool init = true;

  if (init)
  {
    slave.runAck = 0;
    slave.homeAck = 0;
    slave.lastCMD_ID = HOME;
    steps[0] = 0.0;
    volume[0] = 0.0;
    TV.measured = 0.0;
    #ifndef TX_SERIAL_TELEMETRY
    Serial.println("Inside Calibration Routine");
    delay(5000);//for testing only
    #endif
    init = false;    
  }

  //Load Data
  if (calibStatus == ST_IN_PROG) {
    if (i<=STEPPERRANGE){
      if (slave.runAck == 0 && slave.lastCMD_ID != RUN)
      {
        reqMotorSteps = (long)((i / LIN_MECH_mm_per_rev) * STEPPER_MICROSTEP * STEPPER_PULSES_PER_REV);
        txSlaveCMD(RUN, period, reqMotorSteps, "1"); //Move Forward
        slave.lastCMD_ID = RUN;
        #ifndef ALTERNATE_CALIBRATION_SCHEME
        TV.measured = 0.0;
        #endif
      }
      else if (slave.runAck == CMD_RECEIVED)
      {
     //   Serial.println("CMD_RECEIVED");
        estimateVolume = true;
      }
      else if (slave.runAck == CMD_COMPLETE && slave.lastCMD_ID == RUN)
      {
        estimateVolume = false;
        steps[j]= (double)(i);
        volume[j]=(double)(TV.measured);
      #ifndef ALTERNATE_CALIBRATION_SCHEME
        txSlaveCMD(HOME, period);
        slave.lastCMD_ID = HOME;
      #else
        #ifndef TX_SERIAL_TELEMETRY
        Serial.print("Distance: ");Serial.print(steps[j]);Serial.println("mm");
        Serial.print("Volume: ");Serial.print(volume[j]);Serial.println("ml");
        #endif
        slave.runAck = 0;
        slave.lastCMD_ID = NO_CMD;
        i +=stepSize;
        j++;
      #endif
      }    
      #ifndef ALTERNATE_CALIBRATION_SCHEME
      else if (slave.homeAck == 2)
      {
        slave.runAck = 0;
        slave.homeAck = 0;
        TV.measured = 0.0;
        #ifndef TX_SERIAL_TELEMETRY
        Serial.print("Distance: ");Serial.print(steps[j]);Serial.println("mm");
        Serial.print("Volume: ");Serial.print(volume[j]);Serial.println("ml");
        #endif
        i +=stepSize;
        j++;
      }
      #endif
    }
    else
    {
      #ifndef ALTERNATE_CALIBRATION_SCHEME
      calibStatus = ST_COMPLETE;
      #else
      if (slave.lastCMD_ID != HOME)
      {
        txSlaveCMD(HOME, period);
        slave.lastCMD_ID = HOME;       
      }
      else if (slave.homeAck == 2)
      {
        calibStatus = ST_COMPLETE;
      }
      #endif
    }
  }
  
  if (calibStatus == ST_COMPLETE)
  {
    int ret = fitCurve(ORDER, sizeof(volume)/sizeof(double), volume, steps, sizeof(VolCoeffs)/sizeof(double), VolCoeffs);

    //Highest to lowest for 3rd order y=ax^2+bx+c where x is volume and y is step in mm. for 3rd order equation. 
    if (ret == 0){ //Returned value is 0 if no error
      
      #ifdef E2PROM
      int eeAddress = ee_Vol_Coef_a;
      EEPROM.put(eeAddress, VolCoeffs);
//      eeAddress += sizeof(VolCoeffs);
      #endif
    Serial.println("Highest to lowest for 3rd order y=ax^2+bx+c where x is volume and y is step in mm. for 3rd order equation");
      for (int k = 0; k < sizeof(VolCoeffs)/sizeof(double); k++){
        Serial.print(VolCoeffs[k], 8);
        Serial.print('\t');
      }
      delay(20000);//for testing only
    }
  }
}