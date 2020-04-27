#include "control.h"

float Control::compensateVolumeError(float setPoint,float measured){

  float error=setPoint-measured;
  float absError=abs(error);
  float valuePredicted=0;
  float stepsPredicted=0;

  if(absError<=volDeadBand){
    discreteIntegral=0;
    return oldPredicatedSteps;
  }
  else{

      discreteIntegral+=error;
      
      valuePredicted=kP*absError+kI*discreteIntegral;
      
      stepsPredicted=(volEqCoeff[0]*pow(valuePredicted,3))+(volEqCoeff[1]*pow(valuePredicted,2))+(volEqCoeff[2]*valuePredicted)+volEqCoeff[3];
    
  }
  
  if(error>=0){ // Need to add more steps

      oldPredicatedSteps=stepsPredicted; 
      return stepsPredicted;
  }
  else{      // Need to sub more steps
      oldPredicatedSteps = -stepsPredicted;
      return -stepsPredicted;
  } 

}

void Control::setConstants(float kp,float ki, float kd,float coeff[4],float deadBand){

     kP=kp;
     kI=ki;
     kD=kd;

     for(int i=0;i<4;i++)
     volEqCoeff[i]=coeff[i];
     volDeadBand=deadBand;    
}
