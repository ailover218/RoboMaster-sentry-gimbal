#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"

#include "struct_typedef.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define SHOOT_CAN hcan2
#define GIMBAL_PITCH_CAN hcan2
#define GIMBAL_YAW_CAN hcan1
#define TRIGGER_CAN hcan2
#define BOARD_COM_CAN hcan1
#define CAN_1 hcan1
#define CAN_2 hcan2

//云台电机编号
enum gimbal_motor_id_e
{
    //底盘动力电机接收
    YAW_MOTOR = 0,
    PITCH_MOTOR,
};

//发射机构电机编号
enum shoot_motor_id_e
{
    //底盘动力电机接收
    LEFT_FRIC_MOTOR = 0,
    RIGHT_FRIC_MOTOR,
    TRIGGER_MOTOR,
    COVER_MOTOR,
};

typedef enum
{
    //发射机构电机接受ID can2
    CAN_LEFT_FRIC_MOTOR_ID = 0x201,
    CAN_RIGHT_FRIC_MOTOR_ID = 0x202,
    CAN_TRIGGER_MOTOR_ID = 0x203,
    CAN_COVER_MOTOR_ID = 0X204,
    CAN_SHOOT_ALL_ID = 0x200,

    //云台电机接收ID   
    CAN_YAW_MOTOR_ID = 0x205,//can1
    CAN_PITCH_MOTOR_ID = 0x206,//can2
    CAN_GIMBAL_ALL_ID = 0x1FF,

    //板间通信ID   can1
  CAN_RC_BOARM_COM_ID = 0x101,
  CAN_GIMBAL_BOARD_COM_ID = 0x102,
    CAN_COOLING_BOARM_COM_ID = 0x303,
    CAN_SHOOT_SPEED_BOARD_COM_ID = 0x304,
    CAN_UI_COM_ID = 0x305,
} can_msg_id_e;

// rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

//云台发送数据结构体
typedef struct
{
    //遥控器数据
    int16_t ch_0;
    int16_t ch_2;
    int16_t ch_3;
    uint16_t v;

    //云台状态
    uint8_t s0;
    uint8_t gimbal_behaviour;
    fp32 gimbal_yaw_angle;

    fp32 gimbal_pitch_angle;
    bool_t auto_state;
    bool_t aim_state;
    bool_t fric_state;
} gimbal_send_t;

//云台接收数据结构体
typedef struct
{
    //测试热量及ID
    uint16_t shoot_cooling_limit; // shoot测速热量上限
    uint16_t shoot_cooling_rate;  // shoot测速热量冷却
    uint16_t shoot_cooling_heat;  // shoot测速实时热量
    uint8_t color;                //判断红蓝方
    uint8_t robot_id;             //机器人编号

    //测速速度及底盘模式
    uint16_t shoot_speed_limit; // shoot测速射速上限
    uint16_t bullet_speed;      // shoot测速实时射速

    uint8_t chassis_behaviour;
    uint8_t game_progress;

} gimbal_receive_t;

class Can_receive
{

public:
    //云台电机反馈数据结构体
    motor_measure_t gimbal_motor[2];
    //发射机构电机反馈数据结构体
    motor_measure_t shoot_motor[4];

    //发送数据结构体
    CAN_TxHeaderTypeDef can_tx_message;
    uint8_t can_send_data[8];

    //板间通信
    //云台接收信息
    gimbal_receive_t gimbal_receive;
    //云台发送
    gimbal_send_t gimbal_send;

    void init();

    /*--------------------云台电机数据接收----------------------*/
    void get_gimbal_motor_measure(uint8_t num, uint8_t data[8]);
    void can_cmd_yaw_motor(int16_t yaw);
    void can_cmd_pitch_motor(int16_t pitch);
    const motor_measure_t *get_gimbal_motor_measure_point(uint8_t i);

    /*-------------------发射机构电机数据接收--------------------*/
    void get_shoot_motor_measure(uint8_t num, uint8_t data[8]);
    void can_cmd_shoot_motor(int16_t left_fric, int16_t right_fric, int16_t trigger, int16_t cover); //动力电机数据
    void can_cmd_trigger_motor(int16_t trigger);
    void can_cmd_shoot_motor_reset_ID();
    const motor_measure_t *get_shoot_motor_measure_point(uint8_t i);

    /*-----------------------板间通信函数------------------------*/
    void receive_cooling_and_id_board_com(uint8_t data[8]);
    void receive_shoot_speed_and_mode_board_com(uint8_t data[8]);
    void send_rc_board_com(int16_t ch_0, int16_t ch_2, int16_t ch_3, uint16_t v);            //发送遥控器数据
    void send_gimbal_board_com(uint8_t s0, uint8_t gimbal_behaviour, fp32 gimbal_yaw_angle); //发送云台模式及状态
    void send_UI_com(bool_t auto_s, bool_t aim_s, bool_t fric_s, fp32 gimbal_pitch_angle,uint16_t v);   //发送UI数据
};

#endif
