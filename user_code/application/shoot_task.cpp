/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot_task.c/h
  * @brief      shoot control task, because use the euler angle calculated by

  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     JUN-07-2022     方兆俊      1. doing
  *
  *
  @verbatim
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
  @endverbatim
  ****************************(C) COPYRIGHT 2021 *******************************
  */


#include "shoot_task.h"

void shoot_task(void *pvParameters)
{
    vTaskDelay(SHOOT_TASK_INIT_TIME);
    //发射机构初始化
    shoot.init();
    while (1)
    {
        //设置发射机构状态机
        shoot.set_mode();
        //发射机构数据反馈
        shoot.feedback_update();
        //设置发射机构控制量
        shoot.set_control();
        //设置PID计算
        shoot.solve();
        //输出电流
        shoot.output();
        //系统延时
        vTaskDelay(SHOOT_CONTROL_TIME);
    }
}
