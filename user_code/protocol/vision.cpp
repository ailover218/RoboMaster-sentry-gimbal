#include "vision.h"
#include "Remote_control.h"
#include "struct_typedef.h"
#include "string.h"
#include "INS.h"
#include "tim.h"
#include "CRC8_CRC16.h"
#include "gimbal.h"

// #include "referee.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t Vision_Buffer[2][VISION_BUFFER_LEN]; // 视觉数据暂存

extern RC_ctrl_t rc_ctrl;
extern INS imu;
extern Gimbal gimbal;

// 角度初始化补偿
float Vision_Comps_Yaw = COMPENSATION_YAW;
float Vision_Comps_Pitch = COMPENSATION_PITCH;           // 固定补偿，减小距离的影响
float Vision_Comps_Pitch_Dist = COMPENSATION_PITCH_DIST; // 根据距离补偿

VisionSendHeader_t VisionSendHeader; // 帧头

VisionActData_t VisionActData; // 行动模式结构体

VisionRecvData_t VisionRecvData; // 接收数据结构体

VisionSendData_t VisionSendData; // 发送数据结构体

uint8_t Attack_Color_Choose = ATTACK_NONE; // 默认不识别

// 打符是否换装甲了
uint8_t Vision_Armor = FALSE;

// 是否识别到装甲板
bool_t if_identify_target = 0;

// 角度补偿,发送给视觉
float Vision_Comps_Yaw_Send = COMPENSATION_YAW;
float Vision_Comps_Pitch_Send = COMPENSATION_PITCH;
float SB_K_comps = 3.f;

static uint64_t send_cnt = 0;

// 滴答定时器计数变量 ,49天后溢出
volatile uint32_t sysTickUptime = 0;

void vision_init()
{
  usart1_init(Vision_Buffer[0], Vision_Buffer[1], VISION_BUFFER_LEN);

  VisionRecvData.pitch_angle = 0;
  VisionRecvData.yaw_angle = 0;
  VisionRecvData.distance = 0;        // 距离
  VisionRecvData.centre_lock = 0;     // 是否瞄准到了中间  0没有  1瞄准到了
  VisionRecvData.identify_target = 0; // 视野内是否有目标/是否识别到了目标   0否  1是
  VisionRecvData.identify_buff = 0;   // 打符时是否识别到了目标，1是，2识别到切换了装甲，0没识别到
}

/**
 * @brief  读取视觉信息
 * @param  uart1缓存数据
 * @retval void
 * @attention  IRQ执行
 */
uint8_t Vision_Time_Test[2] = {0}; // 当前数据和上一次数据
uint8_t Vision_Ping = 0;           // 发送时间间隔

/**
 * @brief          定时器周期给视觉发送陀螺仪数据
 * @param[in]      htim:定时器指针
 * @retval         none
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim1)
  {
    HAL_GPIO_TogglePin(CRAMA_TRI_GPIO_Port, CRAMA_TRI_Pin);
    if (send_cnt++ % 2 == 0)
    {
      vision_send_data(0x02);
    }
  }
}

void vision_read_data(uint8_t *ReadFormUart)
{

  // 判断帧头数据是否为0xA5
  if (ReadFormUart[0] == VISION_BEGIN)
  {
    // 判断帧头数据是否为0xff
    if (ReadFormUart[17] == VISION_END)
    {

      // 接收数据拷贝
      memcpy(&VisionRecvData, ReadFormUart, VISION_READ_LEN_PACKED);

      if (VisionRecvData.identify_target == TRUE)
        if_identify_target = TRUE; // 识别到装甲板
      else
        if_identify_target = FALSE; // 未识别到装甲板

      // //帧计算
      // Vision_Time_Test[NOW] = xTaskGetTickCount();
      // Vision_Ping = Vision_Time_Test[NOW] - Vision_Time_Test[LAST];//计算时间间隔
      // Vision_Time_Test[LAST] = Vision_Time_Test[NOW];
    }
  }
}

/**
 * @brief  发送视觉指令
 * @param  CmdID
 * @retval void
 * @attention  按协议打包好数据发送
 *				CmdID   0x00   关闭视觉
 *				CmdID   0x01   识别红色装甲
 *				CmdID   0x02   识别蓝色装甲
 *				CmdID   0x03   小符
 *				CmdID   0x04   大符
 */
uint8_t vision_send_pack[50] = {0};
uint8_t CmdID = 0;
void vision_send_data(uint8_t CmdID)
{
  int i; // 循环发送次数
  uint16_t shoot_speed_limit;
  uint16_t bullet_speed;
  // get_shooter_shoot_speed_limit_and_bullet_speed(&shoot_speed_limit, &bullet_speed);

  VisionSendData.BEGIN = VISION_BEGIN;

  VisionSendData.CmdID = CmdID;
  VisionSendData.speed = 3;
  VisionSendData.yaw = imu.INS_angle[0];
  VisionSendData.roll = imu.INS_angle[1];
  VisionSendData.pitch = imu.INS_angle[2];

  VisionSendData.END = 0xFF;

  memcpy(vision_send_pack, &VisionSendData, VISION_SEND_LEN_PACKED);

  // 将打包好的数据通过串口移位发送到上位机
  for (i = 0; i < VISION_SEND_LEN_PACKED; i++)
  {
    HAL_UART_Transmit(&huart1, &vision_send_pack[i], sizeof(vision_send_pack[0]), 0xFFF);
  }
  // HAL_UART_Transmit(&huart1, vision_send_pack, VISION_SEND_LEN_PACKED, 0xFFF);

  memset(vision_send_pack, 0, 50);
}

// 调解自瞄的跟随速度
uint16_t yaw_para = 80;
uint16_t pitch_para = 150;

void vision_error_angle(float *yaw_angle_error, float *pitch_angle_error)
{
  *yaw_angle_error = VisionRecvData.yaw_angle / yaw_para;
  *pitch_angle_error = -VisionRecvData.pitch_angle / pitch_para;

  if (VisionRecvData.yaw_angle == 0)
  {
    *yaw_angle_error = 0;
  }
  if (VisionRecvData.pitch_angle == 0)
  {
    *pitch_angle_error = 0;
  }
}

/**
 * @brief  判断是否识别到装甲板
 * @param  void
 * @retval TRUE识别到   FALSE未识别到
 * @attention  为自瞄做准备
 */
bool_t vision_if_find_target(void)
{
  return if_identify_target;
}

/**
 * @brief  判断换装甲板了吗
 * @param  void
 * @retval TRUE换了   FALSE没换
 * @attention  为自动打符做准备,串口空闲中断每触发一次且通过校验,则Vision_Armor置TRUE
 */
bool_t vision_if_armor(void)
{
  return Vision_Armor;
}

/**
 * @brief  换装甲标志位手动置0(false)
 * @param  void
 * @retval void
 * @attention  记得要清零,在哪清零自己选,调用这个函数就行
 */
void vision_clean_ammorflag(void)
{
  Vision_Armor = FALSE;
}

/**
 * @brief 获取us级时间
 * @return us级时间
 * @note 滴答定时器分频在此函数处理好像会损失精度
 */
uint64_t getSysTimeUs(void)
{
  uint64_t ms;
  uint64_t value;
  ms = sysTickUptime;
  value = (ms * TICK_US + (SysTick->LOAD - SysTick->VAL) * TICK_US / SysTick->LOAD) / MPRE;
  return value;
}