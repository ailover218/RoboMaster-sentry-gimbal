/**
  *************************(C) COPYRIGHT 2021 SUMMERPRAY************************
  * @file       interact_task.c/h
  * @brief      用户交互任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     NOV-06-2021     summerpray      1. doing
  *
  @verbatim
  ==============================================================================
  ==============================================================================
 *      ┌─┐       ┌─┐
 *   ┌──┘ ┴───────┘ ┴──┐
 *   │                 │
 *   │       ───       │
 *   │  ─┬┘       └┬─  │
 *   │                 │
 *   │       ─┴─       │
 *   │                 │
 *   └───┐         ┌───┘
 *       │         │
 *       │         │
 *       │         │
 *       │         └──────────────┐
 *       │                        │
 *       │                        ├─┐
 *       │                        ┌─┘
 *       │                        │
 *       └─┐  ┐  ┌───────┬──┐  ┌──┘
 *         │ ─┤ ─┤       │ ─┤ ─┤
 *         └──┴──┘       └──┴──┘
 *                神兽保佑
 *               代码无BUG!
  ==============================================================================
  ==============================================================================
  @endverbatim
  *************************(C) COPYRIGHT 2021 SUMMERPRAY************************
  */

#include "interact_task.h"
#include "detect_task.h"


#ifdef __cplusplus
extern "C"
{
#endif

#include "bsp_buzzer.h"

#ifdef __cplusplus
}
#endif

void interact_task(void *pvParameters)
{
    vTaskDelay(INTERACT_TASK_INIT_TIME);

    led.init();

    while (1)
    {
        led.RGB_flow();
        vTaskDelay(INTERACT_CONTROL_TIME_MS);
    }
}
