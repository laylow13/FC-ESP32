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
#define UPPER_DUTY
#define SUBER_DUTY

void motor_init();
void ESC_Cali();
void set_motor_speed();

extern uint16_t duty[4];

#endif //SOURCE_MOTOR_H
