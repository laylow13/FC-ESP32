//
// Created by Lay on 3/23/2022.
//

#include "motor.h"

uint16_t duty[4];
float Pitch_Motor_1, Pitch_Motor_2, Pitch_Motor_3, Pitch_Motor_4,
        Roll_Motor_1, Roll_Motor_2, Roll_Motor_3, Roll_Motor_4,
        Yaw_Motor_1, Yaw_Motor_2, Yaw_Motor_3, Yaw_Motor_4;
uint32_t M1, M2, M3, M4;
// TODO:set Macros about ESC_PWM_in and Motor
void motor_init() {
//    ledcSetup(0,500,10);
//    ledcSetup(1,500,10);
//    ledcSetup(2,500,10);
//    ledcSetup(3,500,10);
//
//    ledcAttachPin(0,MOTOR_PIN1);
//    ledcAttachPin(1,MOTOR_PIN2);
//    ledcAttachPin(2,MOTOR_PIN3);
//    ledcAttachPin(3,);
}

void ESC_Cali() {
//    ledcWrite(0,UPPER_DUTY);
//    ledcWrite(1,UPPER_DUTY);
//    ledcWrite(2,UPPER_DUTY);
//    ledcWrite(3,UPPER_DUTY);
//    delay(ESC_CALI_DELAY);
//    ledcWrite(0,SUBER_DUTY);
//    ledcWrite(1,SUBER_DUTY);
//    ledcWrite(2,SUBER_DUTY);
//    ledcWrite(3,SUBER_DUTY);
}

void motor_set_speed() {
    pwm_limit(&M1);
    pwm_limit(&M2);
    pwm_limit(&M3);
    pwm_limit(&M4);
    ledcWrite(0, M1);
    ledcWrite(1, M2);
    ledcWrite(2, M3);
    ledcWrite(3, M4);
}


void pwm_limit(uint32_t *pwm) {
    if (*pwm > Motor_PWM_MAX)
        *pwm = Motor_PWM_MAX;
    else if (*pwm < Motor_PWM_MIN)
        *pwm = Motor_PWM_MIN;
}

void motor_pwm_calculate(float f_ut_Pitch, float f_ut_Roll, float f_ut_Yaw) {
    Pitch_Motor_1 = Pitch_Motor_3 =
            sqrt((Motor_PWM_INIT - Motor_PWM_MIN) * (Motor_PWM_INIT - Motor_PWM_MIN) + LAMDA_PITCH * f_ut_Pitch) +
            Motor_PWM_MIN;
    Pitch_Motor_2 = Pitch_Motor_4 =
            sqrt((Motor_PWM_INIT - Motor_PWM_MIN) * (Motor_PWM_INIT - Motor_PWM_MIN) - LAMDA_PITCH * f_ut_Pitch) +
            Motor_PWM_MIN;
    Roll_Motor_1 = Roll_Motor_2 =
            sqrt((Motor_PWM_INIT - Motor_PWM_MIN) * (Motor_PWM_INIT - Motor_PWM_MIN) - LAMDA_ROLL * f_ut_Roll) +
            Motor_PWM_MIN;
    Roll_Motor_3 = Roll_Motor_4 =
            sqrt((Motor_PWM_INIT - Motor_PWM_MIN) * (Motor_PWM_INIT - Motor_PWM_MIN) + LAMDA_ROLL * f_ut_Roll) +
            Motor_PWM_MIN;
    Yaw_Motor_1 = Yaw_Motor_4 =
            sqrt((Motor_PWM_INIT - Motor_PWM_MIN) * (Motor_PWM_INIT - Motor_PWM_MIN) - LAMDA_YAW * f_ut_Yaw) +
            Motor_PWM_MIN;
    Yaw_Motor_2 = Yaw_Motor_3 =
            sqrt((Motor_PWM_INIT - Motor_PWM_MIN) * (Motor_PWM_INIT - Motor_PWM_MIN) + LAMDA_YAW * f_ut_Yaw) +
            Motor_PWM_MIN;
    M1 = (unsigned int) (Pitch_Motor_1 + Roll_Motor_1 + Yaw_Motor_1 - 2 * Motor_PWM_INIT);
    M2 = (unsigned int) (Pitch_Motor_2 + Roll_Motor_2 + Yaw_Motor_2 - 2 * Motor_PWM_INIT);
    M3 = (unsigned int) (Pitch_Motor_3 + Roll_Motor_3 + Yaw_Motor_3 - 2 * Motor_PWM_INIT);
    M4 = (unsigned int) (Pitch_Motor_4 + Roll_Motor_4 + Yaw_Motor_4 - 2 * Motor_PWM_INIT);
}