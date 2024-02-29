#include "communicate.h"

#include "main.h"
#include "string.h"

#include "bsp_usart.h"
#include "bsp_led.h"

#include "detect_task.h"
#include "Gimbal.h"

#include "Remote_control.h"
#include "Can_receive.h"
#include "vision.h"
#include "shoot.h"

Remote_control remote_control;

Can_receive can_receive;

Communicate communicate;

extern bool_t if_identify_target;
extern bool_t auto_switch;
extern Shoot shoot;

void Communicate::init()
{
    remote_control.init();
    can_receive.init();
    vision_init();
}

void Communicate::run()
{
    //向底盘发送遥控器和云台数据
    int16_t temp_ch0, temp_ch2, temp_ch3;
    uint16_t temp_v;
    uint8_t temp_s0, temp_gimbal_behaviour_mode;
    fp32 temp_gimbal_yaw_angle;

    fp32 temp_gimbal_pitch_angle;
    bool_t temp_auto = auto_switch;
    bool_t temp_aim = if_identify_target;
    bool_t temp_fric = shoot.fric_status;

    temp_ch0 = remote_control.rc_ctrl.rc.ch[0];
    temp_ch2 = remote_control.rc_ctrl.rc.ch[2];
    temp_ch3 = remote_control.rc_ctrl.rc.ch[3];
    temp_v   = remote_control.rc_ctrl.key.v;
    temp_s0  = remote_control.rc_ctrl.rc.s[0];

    temp_gimbal_behaviour_mode = gimbal.gimbal_mode;
    temp_gimbal_yaw_angle = gimbal.gimbal_yaw_motor.encode_angle;
    temp_gimbal_pitch_angle = gimbal.gimbal_pitch_motor.encode_angle;

    can_receive.send_gimbal_board_com(temp_s0, temp_gimbal_behaviour_mode, temp_gimbal_yaw_angle);
    if (game_start())
    {
        can_receive.send_UI_com(temp_auto, temp_aim, temp_fric, temp_gimbal_pitch_angle, temp_v);
    }
    else
    {
        can_receive.send_rc_board_com(temp_ch0, temp_ch2, temp_ch3, temp_v);
    }
}
bool Communicate::game_start()
{
    return can_receive.gimbal_receive.game_progress != 0;
}

#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{

    //遥控器串口
    void USART3_IRQHandler(void)
    {
        if (huart3.Instance->SR & UART_FLAG_RXNE) //接收到数据
        {
            __HAL_UART_CLEAR_PEFLAG(&huart3);
        }
        else if (USART3->SR & UART_FLAG_IDLE)
        {
            static uint16_t this_time_rx_len = 0;

            __HAL_UART_CLEAR_PEFLAG(&huart3);

            if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
            {
                /* Current memory buffer used is Memory 0 */

                // disable DMA
                //失效DMA
                __HAL_DMA_DISABLE(&hdma_usart3_rx);

                // get receive data length, length = set_data_length - remain_length
                //获取接收数据长度,长度 = 设定长度 - 剩余长度
                this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

                // reset set_data_lenght
                //重新设定数据长度
                hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

                // set memory buffer 1
                //设定缓冲区1
                hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

                // enable DMA
                //使能DMA
                __HAL_DMA_ENABLE(&hdma_usart3_rx);

                if (this_time_rx_len == RC_FRAME_LENGTH)
                {
                    remote_control.unpack(0);
                    //记录数据接收时间
                    detect_hook(DBUS_TOE);
                }
            }
            else
            {
                /* Current memory buffer used is Memory 1 */
                // disable DMA
                //失效DMA
                __HAL_DMA_DISABLE(&hdma_usart3_rx);

                // get receive data length, length = set_data_length - remain_length
                //获取接收数据长度,长度 = 设定长度 - 剩余长度
                this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

                // reset set_data_lenght
                //重新设定数据长度
                hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

                // set memory buffer 0
                //设定缓冲区0
                DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

                // enable DMA
                //使能DMA
                __HAL_DMA_ENABLE(&hdma_usart3_rx);

                if (this_time_rx_len == RC_FRAME_LENGTH)
                {
                    //处理遥控器数据
                    remote_control.unpack(1);
                    //记录数据接收时间
                    detect_hook(DBUS_TOE);
                }
            }
        }
    }

    //视觉接收中断
    void USART1_IRQHandler(void)
    {
        static volatile uint8_t res;
        if (USART1->SR & UART_FLAG_IDLE)
        {
            __HAL_UART_CLEAR_PEFLAG(&huart1);

            static uint16_t this_time_rx_len = 0;

            if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
            {
                __HAL_DMA_DISABLE(huart1.hdmarx);
                this_time_rx_len = VISION_BUFFER_LEN - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
                __HAL_DMA_SET_COUNTER(huart1.hdmarx, VISION_BUFFER_LEN);
                huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
                __HAL_DMA_ENABLE(huart1.hdmarx);

                vision_read_data(Vision_Buffer[0]); //读取视觉数据
                memset(Vision_Buffer[0], 0, 200);
                detect_hook(VISION_TOE);
            }
            else
            {
                __HAL_DMA_DISABLE(huart1.hdmarx);
                this_time_rx_len = VISION_BUFFER_LEN - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
                __HAL_DMA_SET_COUNTER(huart1.hdmarx, VISION_BUFFER_LEN);
                huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
                __HAL_DMA_ENABLE(huart1.hdmarx);

                vision_read_data(Vision_Buffer[1]); //读取视觉数据
                memset(Vision_Buffer[1], 0, 200);   //对象   内容  长度
                detect_hook(VISION_TOE);
            }
        }
    }

    /**
     * @brief          hal库CAN回调函数,接收电机数据
     * @param[in]      hcan:CAN句柄指针
     * @retval         none
     */
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
    {

        if (hcan == &CAN_1) //接云台CAN 信息
        {
            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
            switch (rx_header.StdId)
            {
            //云台机构电机
            case CAN_YAW_MOTOR_ID:
                can_receive.get_gimbal_motor_measure(0, rx_data);
                detect_hook(GIMBAL_YAW_MOTOR_TOE);
                break;

            case CAN_COOLING_BOARM_COM_ID:
                can_receive.receive_cooling_and_id_board_com(rx_data);
                detect_hook(BOARD_COM);
                break;

            case CAN_SHOOT_SPEED_BOARD_COM_ID:
                can_receive.receive_shoot_speed_and_mode_board_com(rx_data);
                detect_hook(BOARD_COM);
                break;

            default:
            {
                break;
            }
            }
        }
        else if (hcan == &CAN_2)
        {
            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
            switch (rx_header.StdId)
            {
            //发射机构电机
            case CAN_LEFT_FRIC_MOTOR_ID:
                can_receive.get_shoot_motor_measure(0, rx_data);
                detect_hook(CAN_LEFT_FRIC_MOTOR_ID);
                break;

            case CAN_RIGHT_FRIC_MOTOR_ID:
                can_receive.get_shoot_motor_measure(1, rx_data);
                detect_hook(CAN_RIGHT_FRIC_MOTOR_ID);
                break;

            case CAN_PITCH_MOTOR_ID:
                can_receive.get_gimbal_motor_measure(1, rx_data);
                detect_hook(GIMBAL_PITCH_MOTOR_TOE);
					    	break;
            case CAN_TRIGGER_MOTOR_ID:
                can_receive.get_shoot_motor_measure(2, rx_data);
                detect_hook(CAN_TRIGGER_MOTOR_ID);
                break;
            case CAN_COVER_MOTOR_ID:
                can_receive.get_shoot_motor_measure(3, rx_data);
                detect_hook(CAN_COVER_MOTOR_ID);
                break;
            default:
            {
                break;
            }
            }
        }
    }
}

#endif
