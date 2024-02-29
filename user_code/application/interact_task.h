#ifndef INTERACT_TASK_H
#define INTERACT_TASK_H

#include "cmsis_os.h"
#include "interact.h"
#include "struct_typedef.h"

#define INTERACT_TASK_INIT_TIME 357
#define INTERACT_CONTROL_TIME_MS 2


extern void interact_task(void *pvParameters);

#endif
