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
#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H
#include "cmsis_os.h"
#include "Shoot.h"

//任务初始化 空闲一段时间
#define SHOOT_TASK_INIT_TIME 201
#define SHOOT_CONTROL_TIME 1


extern void shoot_task(void *pvParameters);

#endif 
