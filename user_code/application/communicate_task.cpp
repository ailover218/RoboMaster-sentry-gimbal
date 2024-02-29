#include "communicate_task.h"

#include "Communicate.h"

/**
* @brief          communucate_task
* @param[in]      pvParameters: NULL
* @retval         none
*/
void communicate_task(void *pvParameters)
{
  vTaskDelay(COMMUNICATE_TASK_INIT_TIME);

  communicate.init();

  while (1)
  {
    communicate.run();

    vTaskDelay(COMMUNICATE_CONTROL_TIME_MS);
  }
}
