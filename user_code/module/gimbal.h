//
// Created by WSJ on 2021/11/2.
//

#ifndef GIMBAL_H
#define GIMBAL_H

#include "Motor.h"
#include "struct_typedef.h"
#include "First_high_pass_filter.h"
#include "Remote_control.h"
#include "Can_receive.h"
#include "user_lib.h"
#include "INS.h"
#include "Communicate.h"

#include "config.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "bsp_buzzer.h"

#ifdef __cplusplus
}
#endif

#include "gimbal_task.h"

/*------------------------速度环pid-----------------------------*/
// yaw 速度环
#define YAW_SPEED_PID_KP 8000.0f
#define YAW_SPEED_PID_KI 0.0f
#define YAW_SPEED_PID_KD 20.0f
#define YAW_SPEED_PID_MAX_IOUT 25.0f

#define YAW_SPEED_PID_MAX_OUT 20000.0f

// pitch 速度环
#define PITCH_SPEED_PID_KP 8000.0f // 2900
#define PITCH_SPEED_PID_KI 0.0f
#define PITCH_SPEED_PID_KD 0.0f
#define PITCH_SPEED_PID_MAX_IOUT 10.0f
#define PITCH_SPEED_PID_MAX_OUT 20000.0f // 15000

/*------------------------------陀螺仪PID------------------------*/
// yaw轴陀螺仪PID 由陀螺仪角度控制
#define YAW_GYRO_PID_KP 30.0f
#define YAW_GYRO_PID_KI 0.1f
#define YAW_GYRO_PID_KD 12.0f
#define YAW_GYRO_PID_MAX_IOUT 0.2f
#define YAW_GYRO_PID_MAX_OUT 120.0f

// pitch轴陀螺仪PID 由陀螺仪角度控制
#define PITCH_GYRO_PID_KP 24.0f
#define PITCH_GYRO_PID_KI 0.02f
#define PITCH_GYRO_PID_KD 240.0f
#define PITCH_GYRO_PID_MAX_IOUT 10.0f
#define PITCH_GYRO_PID_MAX_OUT 500.0f

/*------------------------------编码器PID------------------------*/
// yaw轴编码器PID 由编码器角度控制
#define YAW_ENCODE_PID_KP 5.0f
#define YAW_ENCODE_PID_KI 0.0f
#define YAW_ENCODE_PID_KD 0.0f
#define YAW_ENCODE_PID_MAX_IOUT 5.0f
#define YAW_ENCODE_PID_MAX_OUT 20.0f

// pitch轴编码器PID 由编码器角度控制
#define PITCH_ENCODE_PID_KP 30.0f
#define PITCH_ENCODE_PID_KI 0.01f
#define PITCH_ENCODE_PID_KD 3.0f
#define PITCH_ENCODE_PID_MAX_IOUT 1.0f
#define PITCH_ENCODE_PID_MAX_OUT 100.0f

/*------------------------------自瞄PID------------------------*/
// yaw轴自瞄PID 由陀螺仪角度控制
#define YAW_AUTO_PID_KP 30.0f
#define YAW_AUTO_PID_KI 0.0f
#define YAW_AUTO_PID_KD 2.0f
#define YAW_AUTO_PID_MAX_IOUT 1.0f
#define YAW_AUTO_PID_MAX_OUT 40.0f

// Pitch轴自瞄PID 由编码器角度控制
#define PITCH_AUTO_PID_KP 100.0f
#define PITCH_AUTO_PID_KI 0.0f
#define PITCH_AUTO_PID_KD 3.0f // 0.1
#define PITCH_AUTO_PID_MAX_IOUT 1.0f
#define PITCH_AUTO_PID_MAX_OUT 150.0f

/*---------------------按键--------------------*/
// yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL 0
#define PITCH_CHANNEL 1
#define GIMBAL_MODE_CHANNEL 0

// 掉头云台速度
#define TURN_SPEED 0.04f

// 遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND 10
#define GIMBAL_LEFT_CHANNEL 1  // 左拨杆
#define GIMBAL_RIGHT_CHANNEL 0 // 右拨杆

// 云台 遥控器速度
#define YAW_RC_SEN -0.00002f // 右手系 z轴逆时针为正 但是遥控器通道向右为正 故加负号
#define PITCH_RC_SEN 0.000010f

// 云台 鼠标速度
#define YAW_MOUSE_SEN -0.0001f
#define PITCH_MOUSE_SEN -0.00005f

#define YAW_ENCODE_SEN 0.01f
#define PITCH_ENCODE_SEN 0.01f

#define GIMBAL_CONTROL_TIME 1

// 云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE 0

// 一阶高通滤波参数
#define GIMBAL_ACCEL_YAW_NUM 0.1666666667f
#define GIMBAL_ACCEL_PITCH_NUM 0.3333333333f

#define GIMBAL_CONTROL_TIME 0.001f

/*---------------------云台限幅与安装参数--------------------*/
// 电机measure数组的索引
#define YAW 0
#define PITCH 1

// 电机正反装所对应的
#define YAW_TURN 1
#define PITCH_TURN 0

// 电机码盘值半圈的编码值
#define HALF_ECD_RANGE 4096
// 电机码盘值一圈的编码值
#define ECD_RANGE 8191

// 云台中值(中值所对应的编码器编码值)
#define ECD_YAW_MID 6015
#define ECD_PITCH_MID 2898

// 限幅
#define MAX_GYRO_YAW PI
#define MIN_GYRO_YAW -PI

#define MAX_GYRO_PITCH 0.60f
#define MIN_GYRO_PITCH -0.20f

#define MAX_ENCODE_YAW 2 * PI
#define MIN_ENCODE_YAW -2 * PI

#define MAX_ENCODE_PITCH 0.60f
#define MIN_ENCODE_PITCH -0.20f

// 云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_TO_MID_ERROR 0.05f
#define GIMBAL_TO_MID_STOP_TIME 200
#define GIMBAL_TO_MID_TIME 6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
// //云台初始化回中值的速度以及控制到的角度
#define GIMBAL_TO_MID_PITCH_SPEED 0.001f
#define GIMBAL_TO_MID_YAW_SPEED 0.003f

#define YAW_TO_MID_SET 0.0f
#define PITCH_TO_MID_SET 0.0f

// 云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET 8000
#define GIMBAL_CALI_STEP_TIME 2000
#define GIMBAL_CALI_STEP_TIME 0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP 1
#define GIMBAL_CALI_PITCH_MIN_STEP 2
#define GIMBAL_CALI_YAW_MAX_STEP 3
#define GIMBAL_CALI_YAW_MIN_STEP 4

#define GIMBAL_CALI_START_STEP GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP 5

// 判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 3000

// 电机编码值转化成角度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

// gm6020转化成底盘速度(m/s)的比例，
#define GM6020_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f * 187 / 3591

// 云台自锁C键长按判断
#define PRESS_STOP_LONG_TIME 100
#define KEY_STOP_GIMBAL if_key_singal_pessed(gimbal_RC, last_gimbal_RC, KEY_PRESSED_STOP_GIMBAL)

/*---------------------云台自瞄摇摆参数--------------------*/
#define GIMBAL_SWING_MAXTIME 2000 // 左右摆头摸下摆头幅度限制
#define GIMBAL_SWING_ADJUST 0.005 // 摆头模式下pitch轴固定增量大小

// 云台行为模式
typedef enum
{
    GIMBAL_ZERO_FORCE = 0,
    GIMBAL_TO_MID,
    GIMBAL_CALI,
    GIMBAL_CHASSIS,
    GIMBAL_FREE,
} gimbal_mode_e;

// 云台校准结构体
typedef struct
{
    fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;

class Gimbal
{
public:
    const RC_ctrl_t *gimbal_RC; // 云台使用的遥控器指针
    RC_ctrl_t *last_gimbal_RC;  // 云台使用的遥控器指针
    // 鼠标右键状态(用于自瞄判断)
    bool_t press_r;
    bool_t last_press_r;
    uint16_t press_r_time;

    uint16_t gimbal_last_key_v; // 遥控器上次按键

    gimbal_mode_e gimbal_mode;      // 云台行为模式
    gimbal_mode_e last_gimbal_mode; // 云台上次控制状态机

    Gimbal_motor gimbal_yaw_motor;   // 云台yaw电机数据
    Gimbal_motor gimbal_pitch_motor; // 云台pitch电机数据

    First_high_pass_filter gimbal_yaw_high_pass_filter;   // 云台yaw电机一阶高通滤波
    First_high_pass_filter gimbal_pitch_high_pass_filter; // 云台pitch电机一阶高通滤波

    // 陀螺仪接口
    const fp32 *gimbal_INT_angle_point; // 获取陀螺仪角度值
    const fp32 *gimbal_INT_gyro_point;  // 获取陀螺仪角速度值
    uint8_t step;

    bool_t gimbal_stop_flag; // 云台自锁Flag

    // 云台自锁按键状态
    bool_t press_stop;
    bool_t last_press_C;
    uint16_t press_stop_time; // 按键时间

    bool_t vision_target_flag; // 看到目标标志位,1为看到

    void init();                               // 云台初始化
    void set_mode();                           // 设置云台控制模式
    void feedback_update();                    // 云台数据反馈
    void key_state_update();                   // 按键信息更新
    bool_t need_cali();                        // 判断云台是否要进行校准
    bool_t gimbal_to_mid();                    // 云台归中
    void switch_control();                     // 拨杆控制模式
    void pitch_up_control();                   // 摩擦轮上电抬头控制
    void mode_change_save();                   // 模式切换数据保存
    void gimbal_data_update();                 // 云台数据计算更新
    void set_control();                        // 设置云台控制量
    void update_auto_pid();                    // 更新自瞄模式PID
    void recover_normal_pid();                 // 返回正常PID
    void turn_around_control(fp32 *yaw);       // 掉头控制
    void gimbal_swing(fp32 *yaw, fp32 *pitch); // 云台摆头
    void solve();                              // 云台控制PID计算
    void output();                             // 输出电流

    /***************************(C) GIMBAL control *******************************/
    void gimbal_to_mid_control(fp32 *yaw, fp32 *pitch);  // 初始化模式
    void gimbal_chassis_control(fp32 *yaw, fp32 *pitch); // 陀螺仪模式
    void gimbal_free_control(fp32 *yaw, fp32 *pitch);    // 编码器模式
    /***************************(C) GIMBAL control *******************************/

    static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

    /***************************(C) GIMBAL CALI *******************************/
    gimbal_step_cali_t gimbal_cali;
    void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);

    void set_hand_operator_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);

    // 云台校准设置
    bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);
    // 云台校准发送
    static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);
    // 云台校准计算
    /***************************(C) GIMBAL CALI *******************************/
};

bool_t gimbal_cmd_to_shoot_stop(void);

extern Gimbal gimbal;

#endif
