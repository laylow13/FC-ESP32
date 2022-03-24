#include <Arduino.h>
#include "imu.h"
#include "Wire.h"
#include "tasks.h"
#include "EEPROM.h"
#include "motor.h"

/*** Macros ***/
#define TASKNUM 3

/**** global defines ****/
unsigned long task_time_tag[TASKNUM] = {0};
uint8_t task_time_period[TASKNUM] = {5};
hw_timer_t *timer = NULL;

/**** global function ****/

void setup() {
    Serial.begin(9600);
    calc_att_init();
    motor_init();
    xTaskCreatePinnedToCore(att_update_task, "att_updata", 2048, nullptr, 1, nullptr, 0);
    xTaskCreatePinnedToCore(serial_print_task, "serial_print", 1024, nullptr, 1, nullptr, 1);
}

void loop() {

}