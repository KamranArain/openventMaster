#include "header.h"
#include "curveFitting.h"

extern struct Slave slave;
extern struct TidalVolume TV;
extern int Homing_Done_F;

byte calibStatus = ST_NOT_INIT;
byte estimateVolume = false;

#define STEPPERRANGE 40 //mm

void calibrate(){

  int stepSize=5; //mm
  unsigned int period = 2000; //us
  double steps[STEPPERRANGE/stepSize]; //mm
  double volume[STEPPERRANGE/stepSize]; 
 
  static unsigned long reqMotorSteps = 0;
  static int i = stepSize;
  static int j = 0;
  static bool init = true;

  if (init)
  {
    slave.runAck = 0;
    slave.homeAck = 0;
    init = false;    
  }

  //Load Data
  if (calibStatus == ST_IN_PROG) {
    if (i<STEPPERRANGE){
      if (slave.runAck == 0)
      {
        reqMotorSteps = (long)((i / LIN_MECH_mm_per_rev) * STEPPER_MICROSTEP * STEPPER_PULSES_PER_REV);
        txSlaveCMD(RUN, period, reqMotorSteps, "1"); //Move Forward
        slave.lastCMD_ID = RUN;
        TV.measured = 0.0;
      }
      else if (slave.runAck == CMD_RECEIVED)
      {
        estimateVolume = true;
      }
      else if (slave.runAck == CMD_COMPLETE && slave.homeAck == 0)
      {
        txSlaveCMD(HOME, 2000);
        slave.lastCMD_ID = HOME;
        estimateVolume = false;
        steps[j]=i;
        volume[j]=TV.measured;
      }    
      else if (slave.homeAck == 2)
      {
        slave.runAck = 0;
        slave.homeAck = 0;
        TV.measured = 0.0;
        i +=stepSize;
        j++;
      }
    }
    else
    {
      calibStatus = ST_COMPLETE;
//      Homing_Done_F = 0;
//      slave.runAck = 0;
//      slave.homeAck = 0;
    }
  }
  
  if (calibStatus == ST_COMPLETE)
  {
    int order = 3; //DO not exceed 20.
    double coeffs[order+1];
    int ret = fitCurve(order, sizeof(volume)/sizeof(double), steps, volume, sizeof(coeffs)/sizeof(double), coeffs);

    //Highest to lowest for 3rd order y=ax^2+bx+c where x is volume and y is step in mm. for 3rd order equation. 
    if (ret == 0){ //Returned value is 0 if no error
    Serial.println("Highest to lowest for 3rd order y=ax^2+bx+c where x is volume and y is step in mm. for 3rd order equation");
      for (int i = 0; i < sizeof(coeffs)/sizeof(double); i++){
        Serial.print(coeffs[i]);
        Serial.print('\t');
      }
    }
  }
}