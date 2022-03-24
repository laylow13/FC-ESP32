#include <Arduino.h>
#include "tasks.h"
#include "imu.h"
#include "motor.h"

/*** Macros ***/

/**** global defines ****/

/**** global function ****/

void setup() {
    Serial.begin(921600);
    att_calc_init();
    pid_ctrl_init();
    motor_init();
    Serial.print(imu.gx_error);
    Serial.print(imu.gy_error);
    Serial.println(imu.gz_error);

    xTaskCreatePinnedToCore(att_update_task, "att_updata", 2048, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(serial_print_task, "serial_print", 1024, nullptr, 1, nullptr, 0);
    xTaskCreatePinnedToCore(pid_ctrl_task, "pid_ctrl", 2048, nullptr,2 , nullptr, 1);

}

void loop() {

}