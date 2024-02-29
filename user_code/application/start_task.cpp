//
// Created by WSJ on 2021/11/2.
//

#include "start_task.h"
#ifdef  __cplusplus
extern "C" {
#endif

#include "freertos.h"
#include "task.h"
#include "bsp_delay.h"

#ifdef  __cplusplus
}
#endif
#include "communicate_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "ins_task.h"
#include "calibrate_task.h"
#include "detect_task.h"
#include "interact_task.h"

#define Tiny_Stack_Size       64
#define Small_Stack_Size      128
#define Normal_Stack_Size     256
#define Large_Stack_Size      512
#define Huge_Stack_Size       1024
#define PriorityVeryLow       1
#define PriorityLow           2
#define PriorityBelowNormal   3
#define PriorityNormal        4
#define PriorityAboveNormal   5
#define PriorityHigh          6
#define PrioritySuperHigh     7
#define PriorityRealtime      8

TaskHandle_t ins_task_handle;
TaskHandle_t gimbal_task_handle;
TaskHandle_t shoot_task_handle;
TaskHandle_t cali_task_handle;
TaskHandle_t communicate_task_handle;
TaskHandle_t oled_task_handle;
TaskHandle_t led_flow_task_handle;
TaskHandle_t detect_task_handle;
TaskHandle_t interact_task_handle;

void System_Resource_Init(void)
{
    /* Syetem Service init --------------*/
    delay_init();
    cali_param_init();

    // buzzer_on(10, 10000);

    // vTaskDelay(3000);

    // buzzer_off();

    /* Applications Init ----------------*/
}

/**
* @brief Load and start User Tasks.
* @note  Edit this function to add tasks into the activated tasks list.
*/
void Task_start(void) {
    /* Syetem Service init --------------*/
    /* Applications Init ----------------*/

    xTaskCreate(ins_task, "ins_task", Huge_Stack_Size, NULL, PriorityRealtime, &ins_task_handle);

    xTaskCreate(gimbal_task, "gimbal_task", Normal_Stack_Size, NULL, PriorityHigh, &gimbal_task_handle);

    xTaskCreate(shoot_task, "shoot_task", Normal_Stack_Size, NULL, PriorityHigh, &shoot_task_handle);

    xTaskCreate(communicate_task, "communicate_task", Large_Stack_Size, NULL, PriorityHigh, &communicate_task_handle);

    //xTaskCreate(calibrate_task, "calibrate_task", Normal_Stack_Size, NULL, PriorityHigh, &cali_task_handle);

    xTaskCreate(detect_task, "detect_task", Normal_Stack_Size, NULL, PriorityHigh, &detect_task_handle);

    xTaskCreate(interact_task, "interact_task", Normal_Stack_Size, NULL, PriorityNormal, &interact_task_handle);
}
