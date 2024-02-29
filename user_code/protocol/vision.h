#ifndef __VISION_H
#define __VISION_H

#include "main.h"
#include "struct_typedef.h"

#include "bsp_usart.h"

#define VISION_BUFFER_LEN 200
#define NOW 0
#define LAST 1

#define ATTACK_NONE 0 //不识别
#define ATTACK_RED 1  //识别红方
#define ATTACK_BLUE 2 //识别蓝方

#define VISION_DATA_ERROR 0	  //视觉数据错误
#define VISION_DATA_CORRECT 1 //视觉数据错误

#define VISION_LEN_HEADER 3		  //帧头长
#define VISION_LEN_DATA 15		  //数据段长度,可自定义
#define VISION_SEND_LEN_PACKED 16 //发送数据包长度
#define VISION_READ_LEN_PACKED 18 //接受数据包长度

#define VISION_OFF (0x00)			  //关闭视觉
#define VISION_RED (0x01)			  //识别红色
#define VISION_BLUE (0x02)			  //识别蓝色
#define VISION_RBUFF_ANTI (0x03)	  //红逆 大符
#define VISION_BBUFF_ANTI (0x04)	  //蓝逆 大符
#define VISION_RBUFF_CLOCKWISE (0x05) //红顺 大符
#define VISION_BBUFF_CLOCKWISE (0x06) //蓝顺 大符
#define VISION_RBUFF_STAND (0x07)	  //红 小符
#define VISION_BBUFF_STAND (0x08)	  //蓝 小符

//起始字节,协议固定为0xA5
#define VISION_BEGIN (0xA5) //可更改
#define VISION_END (0xFF)	//帧尾

/*-------视觉分辨率预编译--------*/
#define VISION_MID_YAW 444	 // 640
#define VISION_MID_PITCH 500 // 360

/*------------------自瞄预编译,角度初始化补偿------------------------*/
#define COMPENSATION_YAW 0
#define COMPENSATION_PITCH 0
#define COMPENSATION_PITCH_DIST 0


#define TICK_PER_SECOND 1000
#define TICK_US	(1000000/TICK_PER_SECOND)

//滴答定时器再分频，MPRE次中断为1ms
#define MPRE 4
/* 	STM32 -> PC

	CmdID   0x00   关闭视觉
	CmdID   0x01   识别红色装甲
	CmdID   0x02   识别蓝色装甲
	CmdID   0x03   红符
	CmdID   0x04   蓝符
*/

/* 	PC -> STM32

	CmdID   0x00   关闭视觉
	CmdID   0x01   识别红色装甲
	CmdID   0x02   识别蓝色装甲
	CmdID   0x03   小符
	CmdID   0x04   大符
*/

//可利用收和发的指令码进行比较,当收和发的指令码相同时,可判定为数据可用

// PC收发与STM32收发成镜像关系,以下结构体适用于STM32,PC需稍作修改

typedef enum
{
	VISION_MANU = 0,
	VISION_BUFF = 1,
	VISION_AUTO = 2,
} VisionActData_t; //视觉模式选择

typedef __packed struct // 3 Byte
{
	/* 头 */
	uint8_t BEGIN; //帧头起始位,暂定0xA5
	uint8_t CmdID; //指令

} VisionSendHeader_t;

// STM32接收,直接将串口接收到的数据拷贝进结构体 18帧
typedef __packed struct // 18 Byte
{
	/* 头 */
	uint8_t BEGIN; //帧头起始位,暂定0xA5
	uint8_t CmdID; //指令

	/* 数据 */
	float pitch_angle;
	float yaw_angle;
	float distance;			 //距离
	uint8_t centre_lock;	 //是否瞄准到了中间  0没有  1瞄准到了
	uint8_t identify_target; //视野内是否有目标/是否识别到了目标   0否  1是
	uint8_t identify_buff;	 //打符时是否识别到了目标，1是，2识别到切换了装甲，0没识别到

	uint8_t END;

} VisionRecvData_t;

// STM32发送,直接将打包好的数据一个字节一个字节地发送出去
typedef __packed struct
{
	uint8_t BEGIN; //帧头起始位,暂定0xA5
	uint8_t CmdID; //指令

	uint8_t speed; //射速

	fp32 yaw;

	fp32 pitch;

	fp32 roll;

	uint8_t END;

} VisionSendData_t;

extern uint8_t CmdID;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

extern uint8_t Vision_Buffer[2][VISION_BUFFER_LEN]; //视觉数据暂存

//命令码ID,用来判断接收的是什么数据
void vision_read_data(uint8_t *ReadFormUart1);

#ifdef __cplusplus
extern "C" {
#endif
void vision_send_data(uint8_t CmdID);
#ifdef __cplusplus
};
#endif
void vision_error_angle(float *yaw_angle_error, float *pitch_angle_error);

extern void vision_init();

bool_t vision_if_find_target();
bool_t vision_if_armor(void);
void vision_clean_ammorflag(void);

uint64_t getSysTimeUs(void);

#endif
