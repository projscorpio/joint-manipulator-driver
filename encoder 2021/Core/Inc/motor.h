#pragma once
#include "main.h"


#define PWM_MIN 0
#define PWM_MAX 900
#define MOTOR_MIN 10


typedef enum {
	Reverse,
	DubleReverse,

} Wiring;

typedef struct{
	Wiring polarization;
	int32_t PWM_Min;
	int32_t PWM_Max;
	uint8_t Motor_Min;

}MotorConfig;



void start_pwm();
void stop_pwm();
void set_speed(int32_t speed,MotorConfig *conf); // -PWM_MAX : PWM_MAX range
