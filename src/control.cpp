#include "control.h"

float Control::compensateError(float setPoint,float measured){

  error=setPoint-measured;
  float absError=fabs(error);
  float currentValuePredicted=0;


      if(absError>integralStartBand){
      discreteIntegral=0;
      }
      else{
      discreteIntegral+=error;    
      }

      currentValuePredicted=kP*error+kI*discreteIntegral; //Weight of Error will reduce with Kp reaching its point + Weight of Error increase by time to reach goal with Ki.
  
      if(valuePredicted<=errorLimit)
      currentValuePredicted=currentValuePredicted+valuePredicted; //Total Steps added/subtracted From begining of cycle.
      else{
      currentValuePredicted=0;
      }
  
      valuePredicted=currentValuePredicted;

      return valuePredicted;
}

void Control::setConstants(float kp,float ki,float bandIntegral,float eLimit){

     kP=kp;
     kI=ki;

     integralStartBand=integralStartBand;
     errorLimit=eLimit;     
     valuePredicted=0;
               
}

void Control::resetController(){
  discreteIntegral=0;
  valuePredicted=0;  
}