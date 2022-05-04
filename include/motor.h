//
// Created by Lay on 3/23/2022.
//

#ifndef SOURCE_MOTOR_H
#define SOURCE_MOTOR_H

#include "Arduino.h"

//电调行程校准为700-2600，死区700-980，有效区间980-2500，650-700可被识别为有信号输入；（ESC brand： HobbyWing Skywalker 30A）
#define MOTOR_PIN1 23
#define MOTOR_PIN2 22
#define MOTOR_PIN3 21
#define MOTOR_PIN4 32
#define ESC_CALI_DELAY 2000
#define UPPER_DUTY  2300
#define SUBER_DUTY   1000
#define Motor_PWM_MIN		1000// 980
#define Motor_PWM_MAX		2500
#define Motor_PWM_INIT	     1300//1300
#define LAMDA_PITCH   15000.0 //23000.0
#define LAMDA_ROLL   15000.0 //23000.0
#define LAMDA_YAW    15000.0 //25000.0



void motor_init();
void esc_cali();
void motor_set_speed();
void pwm_limit(uint16_t *pwm);
void motor_pwm_calculate(float f_ut_Pitch, float f_ut_Roll, float f_ut_Yaw);


extern uint16_t duty[4];
extern uint16_t M1, M2, M3, M4;

#endif //SOURCE_MOTOR_H
