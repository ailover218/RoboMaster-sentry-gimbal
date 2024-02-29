#ifndef MOTOR_H
#define MOTOR_H

#include "Pid.h"
#include "Can_receive.h"

//传感器类型
#define ENCODE 0
#define GYRO 1

// gimbal电机
class Gimbal_motor
{
public:
    //电机数据
    const motor_measure_t *motor_measure;

    //速度环PID
    Pid speed_pid;
    //陀螺仪角度环PID
    Pid gyro_angle_pid;
    //编码器角度环PID
    Pid encode_angle_pid;

    //用户定义的初始中值
    uint16_t offset_ecd;

    //编码器角度限幅和中值
    fp32 max_encode_angle; // rad
    fp32 min_encode_angle; // rad
    fp32 mid_encode_angle; // rad

    //陀螺仪角度限幅和中值
    fp32 max_gyro_angle; // rad
    fp32 min_gyro_angle; // rad
    fp32 mid_gyro_angle; // rad

    //编码器计算出来的角度及设定值
    fp32 encode_angle;     // rad
    fp32 encode_angle_set; // rad
    //陀螺仪计算出来的角度及设定值
    fp32 gyro_angle;     // rad
    fp32 gyro_angle_set; // rad
    //云台电机速度和速度设置
    fp32 speed;
    fp32 speed_set;
    //云台PID计算得到的电流值
    fp32 current_set;
    //最终Can发送的电流值
    int16_t current_give;

    void init_data_point(const motor_measure_t *motor_measure_); //初始化云台数据指针
    void init_pid_param(int gimbal);                             //初始化PID参数
    void init_gimbal_offset(int gimbal);                         //初始化中值和限幅
    void angle_limit(fp32 add, int angle_mode);                  //角度控制总函数
    void gyro_angle_limit(fp32 add);                             //陀螺仪角度控制
    void encode_angle_limit(fp32 add);                           //编码器角度控制

    void motor_gyro_angle_control();   //云台陀螺仪角度电流计算
    void motor_encode_angle_control(); //云台编码器角度电流计算
};

//摩擦轮电机
class Firc_motor
{
public:
    const motor_measure_t *motor_measure;
    //速度环pid和角度环pid, 用户可以选择性开启
    Pid speed_pid;
    Pid angle_pid;

    fp32 accel;
    fp32 speed;
    fp32 speed_set;

    fp32 max_speed;     //摩擦轮旋转最大速度
    fp32 min_speed;     //摩擦轮旋转最小速度
    fp32 require_speed; //允许拨盘开启的最低速度

    fp32 current_set;
    int16_t current_give;

    void init(const motor_measure_t *motor_measure_);
};

//拨弹电机
class Trigger_motor
{
public:
    const motor_measure_t *motor_measure;
    //速度环pid和角度环pid, 用户可以选择性开启
    Pid speed_pid;
    Pid angle_pid;

    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 angle_set;

    fp32 current_set;
    int16_t current_give;

    int8_t ecd_count; ///编码值计数

    void init(const motor_measure_t *motor_measure_);
};

//弹仓开合电机
class Cover_motor
{
public:
    const motor_measure_t *motor_measure;
    //速度环pid和角度环pid, 用户可以选择性开启
    Pid speed_pid;
    Pid angle_pid;

    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 angle_set;

    fp32 current_set;
    int16_t current_give;

    int8_t ecd_count; ///编码值计数

    void init(const motor_measure_t *motor_measure_);
};

#endif
