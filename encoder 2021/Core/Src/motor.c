#include "motor.h"

extern TIM_HandleTypeDef htim1;

void start_pwm() {
  HAL_GPIO_WritePin(HB_INA_GPIO_Port, HB_INA_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(HB_INB_GPIO_Port, HB_INB_Pin, GPIO_PIN_RESET);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void stop_pwm() {
  HAL_GPIO_WritePin(HB_INA_GPIO_Port, HB_INA_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(HB_INB_GPIO_Port, HB_INB_Pin, GPIO_PIN_RESET);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}

static int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void set_speed(int32_t speed, MotorConfig *mconf ) {
  if(speed > mconf->PWM_Max) speed = mconf->PWM_Max;
  if(speed < -mconf->PWM_Max) speed = -mconf->PWM_Max;
  
  if((speed > 0)&&(mconf->polarization==Reverse)||(speed < 0)&&(mconf->polarization==DubleReverse)){
    HAL_GPIO_WritePin(HB_INA_GPIO_Port, HB_INA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HB_INB_GPIO_Port, HB_INB_Pin, GPIO_PIN_SET);
    
    int16_t mappedSpeed = map(speed, mconf->PWM_Min, mconf->PWM_Max, mconf->Motor_Min, mconf->PWM_Max); // the motor doesnt rotate at low PWMs, so map o some higher range
    TIM1->CCR1 = mappedSpeed;
  }else {
    HAL_GPIO_WritePin(HB_INA_GPIO_Port, HB_INA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HB_INB_GPIO_Port, HB_INB_Pin, GPIO_PIN_RESET);
    
    int16_t mappedSpeed = map(-speed, mconf->PWM_Min, mconf->PWM_Max, mconf->Motor_Min, mconf->PWM_Max); // the motor doesnt rotate at low PWMs, so map o some higher range
    TIM1->CCR1 = mappedSpeed;
  } 
}
