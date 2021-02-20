#include "IIRFilter.h"

float IIR(float value, float coeff) {
  static float aggregate = 0.0f;
    
  float secCoeff = 1.0f - coeff;
  value = coeff * aggregate + secCoeff * value;
  
  // last value
  aggregate = value;
  
  //infinite response problem solution
  if (value < 0.5f && value > -0.5f) value = 0.0f; 
  
  return value;
}