#ifndef Control_h
#define Control_h
#include "Arduino.h"

class Control{

  public:
  
  float compensateVolumeError(float setPoint,float measured);
  void setConstants(float kp,float ki, float kd,float coeff[4],float deadBand); 
  
    
  private:

  float kP;
  float kI;
  float kD;

  float volDeadBand; //ml
  float volEqCoeff[4];
  float discreteIntegral;
  float oldError;
  float oldPredicatedSteps=0;
};




#endif //Control_h
