//
// Created by Lay on 3/20/2022.
//

#include "tasks.h"
#include "imu.h"
#include "pid.h"
#include "motor.h"

#define PID_CTRL_PERIOD 5 //:ms
Mahony mahony;

void pid_ctrl_init()
{
//    TODO:1.tune kP kI kD
    anglePitch_ctrl.init(1,0,0,0,PID_CTRL_PERIOD);
    angleRoll_ctrl.init(1,0,0,0,PID_CTRL_PERIOD);
    angleYaw_ctrl.init(1,0,0,0,PID_CTRL_PERIOD);
    angularVelPitch_ctrl.init(1,0,0,0,PID_CTRL_PERIOD);
    angularVelRoll_ctrl.init(1,0,0,0,PID_CTRL_PERIOD);
    angularVelYaw_ctrl.init(1,0,0,0,PID_CTRL_PERIOD);
}

void att_calc_init()
{
    imu.init();
//    -324.00,99.00,-96.00
//   -31.00 12.00 -8.00
    imu.gx_error=-38.00;
    imu.gy_error=8.00;
    imu.gz_error=-12.00;
//    imu.getGyroStaticError();
    Serial.print(imu.gx_error);
    Serial.print(imu.gy_error);
    Serial.println(imu.gz_error);
    mahony.begin(200);
}

[[noreturn]] void att_update_task(void *pvParameters)
{
    //TODO:solve attitude strange error...
    TickType_t lastwaketime;
    float gyroScale = 0.061;
    while (1)
    {
        imu.update();
        LPFUpdate6axis((imu.gx-imu.gx_error)*gyroScale,(imu.gy-imu.gy_error)*gyroScale,
                       (imu.gz-imu.gz_error)*gyroScale,imu.ax,imu.ay,imu.az);
        mahony.updateIMU(Filters.GyroxLPF.output/57.3,Filters.GyroyLPF.output/57.3,Filters.GyrozLPF.output/57.3,
                         Filters.AccxLPF.output,Filters.AccyLPF.output,Filters.AcczLPF.output);
//        Serial.println(mahony.getPitch());
        vTaskDelayUntil(&lastwaketime,5/portTICK_PERIOD_MS);
    }
}

[[noreturn]] void serial_print_task(void *pvParameters)
{
    while (1)
    {
        Serial.print("[P,R,Y]:");
        Serial.print(mahony.getPitch());
        Serial.print(",");
        Serial.print(mahony.getRoll());
        Serial.print(",");
        Serial.println(mahony.getYaw());
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

[[noreturn]] void pid_calculate_task(void *pvParameters)
{
    float angleYaw,angleRoll,anglePitch,angularVelYaw,angularVelRoll,angularVelPitch;
    while (1)
    {
        angularVelPitch=Filters.GyroxLPF.output*0.01744;
        angularVelRoll=Filters.GyroyLPF.output*0.01744;
        angularVelYaw=Filters.GyrozLPF.output*0.01744;
        anglePitch = mahony.getPitch()*0.01744;  //deg to rad
        angleRoll = mahony.getRoll()*0.01744;
        angleYaw = mahony.getYaw()*0.01744;

        anglePitch_ctrl.calculate(anglePitch);
        angularVelPitch_ctrl.setDes(anglePitch_ctrl.output);
        angularVelPitch_ctrl.calculate(angularVelPitch);

        angleRoll_ctrl.calculate(angleRoll);
        angularVelRoll_ctrl.setDes(angleRoll_ctrl.output);
        angularVelRoll_ctrl.calculate(angularVelRoll);

        angleYaw_ctrl.calculate(angleYaw);
        angularVelYaw_ctrl.setDes(angleYaw_ctrl.output);
        angularVelYaw_ctrl.calculate(angularVelYaw);

        motor_pwm_calculate(angularVelPitch_ctrl.output,angularVelRoll_ctrl.output,angularVelYaw_ctrl.output);
        motor_set_speed();
    }
}

void nonRtosTask()
{
    float gyroScale = 0.061;
    imu.update();
    LPFUpdate6axis((imu.gx-imu.gx_error)*gyroScale,(imu.gy-imu.gy_error)*gyroScale,
                   (imu.gz-imu.gz_error)*gyroScale,imu.ax,imu.ay,imu.az);
    mahony.updateIMU(Filters.GyroxLPF.output/57.3,Filters.GyroyLPF.output/57.3,Filters.GyrozLPF.output/57.3,
                     Filters.AccxLPF.output,Filters.AccyLPF.output,Filters.AcczLPF.output);
    Serial.println(mahony.getYaw());
}