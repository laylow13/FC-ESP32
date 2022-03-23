#include <Arduino.h>
#include "imu.h"
#include "Wire.h"
#include "tasks.h"
#include "EEPROM.h"

/*** Macros ***/
#define TASKNUM 3

/**** global defines ****/
unsigned long task_time_tag[TASKNUM]={0};
uint8_t task_time_period[TASKNUM]={5};
hw_timer_t* timer=NULL;
/**** global function ****/

void setup() {
Serial.begin(9600);
calc_att_init();
}
void loop() {
    if(millis()-task_time_tag[0]>task_time_period[0])
{

    task0();
    Serial.println(mahony.getPitch());
    task_time_tag[0]=millis();
}
}