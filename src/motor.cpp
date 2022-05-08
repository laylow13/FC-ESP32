//
// Created by Lay on 3/23/2022.
//

#include "motor.h"


uint16_t duty[4];
float Pitch_Motor_1, Pitch_Motor_2, Pitch_Motor_3, Pitch_Motor_4,
        Roll_Motor_1, Roll_Motor_2, Roll_Motor_3, Roll_Motor_4,
        Yaw_Motor_1, Yaw_Motor_2, Yaw_Motor_3, Yaw_Motor_4;
uint16_t M1, M2, M3, M4;
float reLu(float input){return input >0 ? input :  0.0;}

void motor_init() {
    ledcSetup(0,250,12);
    ledcSetup(1,250,12);
    ledcSetup(2,250,12);
    ledcSetup(3,250,12);

    ledcAttachPin(MOTOR_PIN1,0);
    ledcAttachPin(MOTOR_PIN2,1);
    ledcAttachPin(MOTOR_PIN3,2);
    ledcAttachPin(MOTOR_PIN4,3);
//    esc_cali();
    delay(1);
    ledcWrite(0,SUBER_DUTY);
    ledcWrite(1,SUBER_DUTY);
    ledcWrite(2,SUBER_DUTY);
    ledcWrite(3,SUBER_DUTY);
}

void esc_cali() {
    ledcWrite(0,UPPER_DUTY);
    ledcWrite(1,UPPER_DUTY);
    ledcWrite(2,UPPER_DUTY);
    ledcWrite(3,UPPER_DUTY);
    delay(ESC_CALI_DELAY);
    ledcWrite(0,SUBER_DUTY);
    ledcWrite(1,SUBER_DUTY);
    ledcWrite(2,SUBER_DUTY);
    ledcWrite(3,SUBER_DUTY);
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


void pwm_limit(uint16_t *pwm) {
    if (*pwm > 2*Motor_PWM_MIN)
        *pwm = 2*Motor_PWM_MIN;
    else if (*pwm < Motor_PWM_MIN)
        *pwm = Motor_PWM_MIN;
}
//x:roll  right-		y:pitch		forward-		z://yaw		clockwise-
/***********
               M1 Clockwise  axis-y Pitch    M3 counterclockwise
				  \             |           /
				    \           |         /
				      \         |       /
				                __ __ __ __ __axis-x Roll
				         /            \
				        /              \
				       /                 \
				      /                   \
				    /                      \
				M2 counterclockwise        M4 Clockwise

***********/
void motor_pwm_calculate(float f_ut_Pitch, float f_ut_Roll, float f_ut_Yaw) {
    Pitch_Motor_1 = Pitch_Motor_2 =
            sqrt(reLu((Motor_PWM_INIT - Motor_PWM_MIN) * (Motor_PWM_INIT - Motor_PWM_MIN) + LAMDA_PITCH * f_ut_Pitch))
            +Motor_PWM_MIN;
    Pitch_Motor_3 = Pitch_Motor_4 =
            sqrt(reLu((Motor_PWM_INIT - Motor_PWM_MIN) * (Motor_PWM_INIT - Motor_PWM_MIN) - LAMDA_PITCH * f_ut_Pitch)) +
            Motor_PWM_MIN;
    Roll_Motor_1 = Roll_Motor_3 =
            sqrt(reLu((Motor_PWM_INIT - Motor_PWM_MIN) * (Motor_PWM_INIT - Motor_PWM_MIN) + LAMDA_ROLL * f_ut_Roll)) +
            Motor_PWM_MIN;
    Roll_Motor_2 = Roll_Motor_4 =
            sqrt(reLu((Motor_PWM_INIT - Motor_PWM_MIN) * (Motor_PWM_INIT - Motor_PWM_MIN) - LAMDA_ROLL * f_ut_Roll)) +
            Motor_PWM_MIN;
    Yaw_Motor_1 = Yaw_Motor_4 =
            sqrt(reLu((Motor_PWM_INIT - Motor_PWM_MIN) * (Motor_PWM_INIT - Motor_PWM_MIN) - LAMDA_YAW * f_ut_Yaw)) +
            Motor_PWM_MIN;
    Yaw_Motor_2 = Yaw_Motor_3 =
            sqrt(reLu((Motor_PWM_INIT - Motor_PWM_MIN) * (Motor_PWM_INIT - Motor_PWM_MIN) + LAMDA_YAW * f_ut_Yaw)) +
            Motor_PWM_MIN;
    M1 = (uint16_t) (Pitch_Motor_1 + Roll_Motor_1 + Yaw_Motor_1 - 2 * Motor_PWM_INIT);
    M2 = (uint16_t) (Pitch_Motor_2 + Roll_Motor_2 + Yaw_Motor_2 - 2 * Motor_PWM_INIT);
    M3 = (uint16_t) (Pitch_Motor_3 + Roll_Motor_3 + Yaw_Motor_3 - 2 * Motor_PWM_INIT);
    M4 = (uint16_t) (Pitch_Motor_4 + Roll_Motor_4 + Yaw_Motor_4 - 2 * Motor_PWM_INIT);
}
