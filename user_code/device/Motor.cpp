#include "Motor.h"
#include "gimbal.h"
#include "Pid.h"
#include "Can_receive.h"

void Gimbal_motor::init_data_point(const motor_measure_t *motor_measure_)
{
    motor_measure = motor_measure_;
}
void Gimbal_motor::init_pid_param(int gimbal)
{
    if (gimbal == YAW)
    {
        //初始化速度环PID
        fp32 yaw_speed_pid_parm[5] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD, YAW_SPEED_PID_MAX_IOUT, YAW_SPEED_PID_MAX_OUT};
        speed_pid.init(PID_SPEED, yaw_speed_pid_parm, &speed, &speed_set, NULL);
        //初始化yaw轴陀螺仪角度环PID
        fp32 yaw_gyro_angle_pid_parm[5] = {YAW_GYRO_PID_KP, YAW_GYRO_PID_KI, YAW_GYRO_PID_KD, YAW_GYRO_PID_MAX_IOUT, YAW_GYRO_PID_MAX_OUT};
        gyro_angle_pid.init(PID_ANGLE, yaw_gyro_angle_pid_parm, &gyro_angle, &gyro_angle_set, 0);
        //初始化yaw轴编码器角度环PID
        fp32 yaw_encode_angle_pid_parm[5] = {YAW_ENCODE_PID_KP, YAW_ENCODE_PID_KI, YAW_ENCODE_PID_KD, YAW_ENCODE_PID_MAX_IOUT, YAW_ENCODE_PID_MAX_OUT};
        encode_angle_pid.init(PID_ANGLE, yaw_encode_angle_pid_parm, &encode_angle, &encode_angle_set, 0);
        // PID清零
        speed_pid.pid_clear();
        gyro_angle_pid.pid_clear();
        encode_angle_pid.pid_clear();
    }
    else if (gimbal == PITCH)
    {
        //初始化速度环PID
        fp32 pitch_speed_pid_parm[5] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD, PITCH_SPEED_PID_MAX_IOUT, PITCH_SPEED_PID_MAX_OUT};
        speed_pid.init(PID_SPEED, pitch_speed_pid_parm, &speed, &speed_set, NULL);
        
        //初始化pitch轴陀螺仪角度环PID
        fp32 pitch_gyro_angle_pid_parm[5] = {PITCH_GYRO_PID_KP,PITCH_GYRO_PID_KI,PITCH_GYRO_PID_KD,PITCH_GYRO_PID_MAX_IOUT,PITCH_GYRO_PID_MAX_OUT};
        gyro_angle_pid.init(PID_ANGLE,pitch_gyro_angle_pid_parm,&gyro_angle,&gyro_angle_set,0);
        
        //初始化pitch轴编码器角度环PID
        fp32 pitch_encode_angle_pid_parm[5] = {PITCH_ENCODE_PID_KP, PITCH_ENCODE_PID_KI, PITCH_ENCODE_PID_KD, PITCH_ENCODE_PID_MAX_IOUT, PITCH_ENCODE_PID_MAX_OUT};
        encode_angle_pid.init(PID_ANGLE, pitch_encode_angle_pid_parm, &encode_angle, &encode_angle_set, 0);
        // PID清零
        speed_pid.pid_clear();
        gyro_angle_pid.pid_clear();
        encode_angle_pid.pid_clear();
    }
}
void Gimbal_motor::init_gimbal_offset(int gimbal)
{
    if (gimbal == YAW)
    {
        //设置陀螺仪角度限幅
        max_gyro_angle = MAX_GYRO_YAW;
        min_gyro_angle = MIN_GYRO_YAW;
        //设置编码器角度限幅
        max_encode_angle = MAX_ENCODE_YAW;
        min_encode_angle = MIN_ENCODE_YAW;
        //设置初始编码器中值
        offset_ecd = ECD_YAW_MID;
    }
    else if (gimbal == PITCH)
    {
        //设置陀螺仪角度限幅
        max_gyro_angle = MAX_GYRO_PITCH;
        min_gyro_angle = MIN_GYRO_PITCH;
        //设置编码器角度限幅
        max_encode_angle = MAX_ENCODE_PITCH;
        min_encode_angle = MIN_ENCODE_PITCH;
        //设置初始编码器中值
        offset_ecd = ECD_PITCH_MID;
    }
}
/**
 * @brief          角度限幅总函数
 * @param[out]     angle_mode:使用什么传感器来进行角度控制
 * @retval         none
 */
void Gimbal_motor::angle_limit(fp32 add, int angle_mode)
{
    if (angle_mode == ENCODE)
    {
        encode_angle_limit(add);
    }
    else if (angle_mode == GYRO)
    {
        gyro_angle_limit(add);
    }
}
/**
 * @brief          云台控制模式:GIMBAL_speed，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
void Gimbal_motor::gyro_angle_limit(fp32 add)
{
    static fp32 bias_angle;
    static fp32 angle_set;
    //当前控制误差角度
    bias_angle = rad_format(gyro_angle_set - gyro_angle);
    //云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
    if (encode_angle + bias_angle + add > max_encode_angle)
    {
        //如果是往最大机械角度控制方向
        if (add > 0.0f)
        {
            // calculate max add_angle
            //计算出一个最大的添加角度，
            add = max_encode_angle - encode_angle - bias_angle;
        }
    }
    else if (encode_angle + bias_angle + add < min_encode_angle)
    {
        if (add < 0.0f)
        {
            add = min_encode_angle - encode_angle - bias_angle;
        }
    }
    angle_set = gyro_angle_set;
    gyro_angle_set = rad_format(angle_set + add);
    //是否超过最大 最小值
    if (gyro_angle_set > max_gyro_angle)
    {
        gyro_angle_set = max_gyro_angle;
    }
    else if (gyro_angle_set < min_gyro_angle)
    {
        gyro_angle_set = min_gyro_angle;
    }
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */

void Gimbal_motor::encode_angle_limit(fp32 add)
{

    encode_angle_set += add;
    //是否超过最大 最小值
    if (encode_angle_set > max_encode_angle)
    {
        encode_angle_set = max_encode_angle;
    }
    else if (encode_angle_set < min_encode_angle)
    {
        encode_angle_set = min_encode_angle;
    }
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
void Gimbal_motor::motor_encode_angle_control()
{
    //角度环，速度环串级pid调试
    speed_set = encode_angle_pid.pid_calc();
    current_set = speed_pid.pid_calc();
}

/**
 * @brief          云台控制模式:GIMBAL_speed，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
void Gimbal_motor::motor_gyro_angle_control()
{
    //角度环，速度环串级pid调试
    speed_set = gyro_angle_pid.pid_calc();
    current_set = speed_pid.pid_calc();
}
void Firc_motor::init(const motor_measure_t *motor_measure_)
{
    motor_measure = motor_measure_;
}

void Trigger_motor::init(const motor_measure_t *motor_measure_)
{
    motor_measure = motor_measure_;
}

void Cover_motor::init(const motor_measure_t *motor_measure_)
{
    motor_measure = motor_measure_;
}
