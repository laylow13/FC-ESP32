#include <Arduino.h>
#include "imu.h"
#include "Wire.h"
#include "tasks.h"
#include "EEPROM.h"
#include "motor.h"

/*** Macros ***/

/**** global defines ****/

/**** global function ****/

void setup() {
    Serial.begin(921600);
    calc_att_init();
    motor_init();
    xTaskCreatePinnedToCore(att_update_task, "att_updata", 2048, nullptr, 1, nullptr, 0);
    xTaskCreatePinnedToCore(serial_print_task, "serial_print", 1024, nullptr, 1, nullptr, 1);
}

void loop() {

}