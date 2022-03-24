//
// Created by Lay on 3/23/2022.
//

#ifndef SOURCE_MOTOR_H
#define SOURCE_MOTOR_H

#include "Arduino.h"

#define MOTOR_PIN1 23 // typo:
#define MOTOR_PIN2
#define MOTOR_PIN3
#define MOTOR_PIN4
#define ESC_CALI_DELAY
#define UPPER_DUTY  500
#define SUBER_DUTY   1000
#define Motor_PWM_MIN		SUBER_DUTY
#define Motor_PWM_MAX		UPPER_DUTY
#define Motor_PWM_INIT	    0
#define LAMDA_PITCH  23000.0
#define LAMDA_ROLL 23000.0
#define LAMDA_YAW 25000.0



void motor_init();
void ESC_Cali();
void motor_set_speed();
void pwm_limit(uint32_t *pwm);
void motor_pwm_calculate(float f_ut_Pitch, float f_ut_Roll, float f_ut_Yaw);


extern uint16_t duty[4];

#endif //SOURCE_MOTOR_H
