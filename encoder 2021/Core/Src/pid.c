/****************************************
*                                       *
*       Title:  PID LIBRARY             *
*       Author: Mateusz Kryszczak       *
*       Date: 19.01.2017                *
*       MCU: STM32F401                  *
*                                       *
*****************************************/

#include "pid.h"

PID_Properties_t pidProperties;

void init_pid() { 
  pidProperties.kp = 12.0;
  pidProperties.ki = 6.0f;
  pidProperties.kd = 0.0f;
  pidProperties.period = 5; // interval between PID computations [ms]
  pidProperties.deadband = 2.5f; // [deg]
  pidProperties.error = 0.0f;
  pidProperties.integralSum = 0.0f;
  pidProperties.posIntegralLimit = 1000.0f;       //integral anti-windup - same as motor max pwm
  pidProperties.negIntegralLimit = -1000.0f;       //integral anti-windup
  pidProperties.lastError = 0.0f;
  pidProperties.lastFeedback = 0.0f;
  pidProperties.posOutputLimit = 1000.0f;         //bound the output
  pidProperties.negOutputLimit = -1000.0f;
}

uint8_t PID(PID_Properties_t* props, float setpoint, float feedback, float* pOutput, derivative_t derivativeType, bool reverse){
  
  if(props == 0 || pOutput == 0) return 1;
  
  // we are doing the calculations in the PU (-1 : 1)
  const float scale_factor = 360.0f;
  
  float error = (setpoint/scale_factor) - (feedback/scale_factor);
  float derivativeOutput = 0.0f;
  float output;
  
  // if we are not in the deadband
  float degreeDelta = setpoint - feedback;
  float absoluteDelta = fabs(degreeDelta);
  if (absoluteDelta > props->deadband) {
    //proportional part
    float proportionalOutput = props->kp * error;
    
    //integral part
    float ki = props->ki * ((float)props->period / 1000.0f); // we need to take period into consideration (divide by 1000 to get seconds)
    props->integralSum += ki * error;
    //anti wind-up
    if (props->integralSum > props->posIntegralLimit) props->integralSum = props->posIntegralLimit;
    else if (props->integralSum < props->negIntegralLimit) props->integralSum = props->negIntegralLimit;
    
    //derivative part
    
    if(derivativeType != noDerivative){
      float kd = props->kd / ((float)props->period / 1000.0f); // we need to take period into consideration (divide by 1000 to get seconds)
      
      switch(derivativeType){
      case derivativeOnFeedback:
        derivativeOutput = kd * (-1.0f) * (feedback - props->lastFeedback);
        break;
        
      case derivativeOnError:
        derivativeOutput = kd * (error - props->lastError);
        break;
        
      default:
        break;
      }
    }
    output = (proportionalOutput + props->integralSum + derivativeOutput);
  } else {
    props->integralSum = 0.0f;
    output = 0;
  }
  
  // reverse direction if needed
  if(reverse == true) output *= -1.0f;
  
  //check if output is within bounds
  if(output > props->posOutputLimit) output = props->posOutputLimit;
  else if(output < props->negOutputLimit) output = props->negOutputLimit;
  
  // map to full scale
  output *= scale_factor;
  
  *pOutput = output;
  
  props->lastFeedback = feedback;
  props->lastError = error;
  
  return 0;
}

uint8_t ResetIntegrator(PID_Properties_t* props){
  if(props == 0) return 0;
  
  props->integralSum = 0;
  return 1;
}