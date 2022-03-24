//
// Created by Lay on 3/20/2022.
//

#ifndef SOURCE_TASKS_H
#define SOURCE_TASKS_H

#include "MahonyAHRS.h"

[[noreturn]] void att_update_task(void *pvParameters);
[[noreturn]] void serial_print_task(void *pvParameters);

[[noreturn]] void pid_ctrl_task(void *pvParameters);

void att_calc_init();
void pid_ctrl_init();

extern Mahony mahony;

#endif //SOURCE_TASKS_H
