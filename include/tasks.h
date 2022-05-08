//
// Created by Lay on 3/20/2022.
//

#ifndef SOURCE_TASKS_H
#define SOURCE_TASKS_H

#include <freertos/FreeRTOS.h>
#include "MahonyAHRS.h"
#include "AsyncUDP.h"
#include "WiFi.h"

typedef struct {
    char *type;
    float data[3]={0};
    char string[10]={0};
    uint32_t longData=0;
} serialPrintData_t;

typedef struct {
    float gx;
    float gy;
    float gz;
    float pitch;
    float roll;
    float yaw;
} attFeedbackData_t ;

typedef struct {
    char param[10]={0};
    float value;
} paramChange_t;
typedef struct
{
    uint8_t data[10]={0};
    IPAddress remoteIP;
}udpRecData_t;

[[noreturn]] void att_update_task(void *pvParameters);
[[noreturn]] void serial_print_task(void *pvParameters);
[[noreturn]] void pid_calculate_task(void *pvParameters);
[[noreturn]] void udp_parser_data_task(void *pvParameters);
[[noreturn]] void udp_send_task(void *pvParameters);

void att_calc_init();
void pid_ctrl_init();
uint8_t wifi_udp_init();
void nonRtosTask();
static void dataParser(paramChange_t newParam);
static void print_controller_info(float angle);

extern Mahony mahony;
#endif //SOURCE_TASKS_H
