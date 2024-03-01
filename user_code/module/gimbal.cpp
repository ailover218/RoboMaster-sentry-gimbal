#include "Gimbal.h"
#include "Shoot.h"
#include "Communicate.h"
#include "INS.h"
#include "detect_task.h"

#include "math.h"
#include "First_order_filter.h"
#include "Motor.h"
#include "vision.h"

// 电机编码值规整 0—8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

/**
 * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
 * @param          输入的遥控器值
 * @param          输出的死区处理后遥控器值
 * @param          死区值
 */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

// 自瞄相关数据
bool_t auto_switch = 0; // 自瞄开关

// 云台模块 对象
Gimbal gimbal;

#if IF_REMOTE_CONTROL

/**
 * @brief          初始化云台
 * @Author         WSJ
 */
void Gimbal::init()
{
    /*---------------------------遥控器---------------------------------*/
    // 遥控器数据指针获取
    gimbal_RC = remote_control.get_remote_control_point();
    last_gimbal_RC = remote_control.get_last_remote_control_point();
    // 初始化上一次的遥控器按键值
    gimbal_last_key_v = gimbal_RC->key.v;

    /*------------------------云台状态机初始化----------------------------*/
    // 初始化初始状态为无力模式
    gimbal_mode = GIMBAL_ZERO_FORCE;
    last_gimbal_mode = gimbal_mode;

    /*---------------------------yaw电机---------------------------------*/
    // 获取Yaw轴电机数据指针
    gimbal_yaw_motor.init_data_point(can_receive.get_gimbal_motor_measure_point(YAW));

    // 初始化Yaw轴PID数据
    gimbal_yaw_motor.init_pid_param(YAW);

    // 设置Yaw轴电机角度限幅和中值
    gimbal_yaw_motor.init_gimbal_offset(YAW);

    /*--------------------------pitch电机--------------------------------*/
    // 获取Pitch轴电机数据指针
    gimbal_pitch_motor.init_data_point(can_receive.get_gimbal_motor_measure_point(PITCH));

    // 初始化Pitch轴PID数据
    gimbal_pitch_motor.init_pid_param(PITCH);

    // 设置Pitch轴电机角度限幅和中值
    gimbal_pitch_motor.init_gimbal_offset(PITCH);

    /*--------------------------滤波值初始化-----------------------------*/
    const static fp32 gimbal_yaw_high_pass_filter_para[1] = {GIMBAL_ACCEL_YAW_NUM};
    const static fp32 gimbal_pitch_high_pass_filter_para[1] = {GIMBAL_ACCEL_PITCH_NUM};
    // 一阶高通滤波初始化
    gimbal_yaw_high_pass_filter.init(GIMBAL_CONTROL_TIME, gimbal_yaw_high_pass_filter_para);
    gimbal_pitch_high_pass_filter.init(GIMBAL_CONTROL_TIME, gimbal_pitch_high_pass_filter_para);

    /*----------------------------陀螺仪指针-----------------------------*/
    // 陀螺仪数据指针获取
    gimbal_INT_angle_point = imu.get_INS_angle_point();
    gimbal_INT_gyro_point = imu.get_gyro_data_point();

    /*--------------------------设定初始设置值----------------------------*/
    gimbal_yaw_motor.gyro_angle_set = gimbal_yaw_motor.gyro_angle;
    gimbal_yaw_motor.encode_angle_set = gimbal_yaw_motor.encode_angle;
    gimbal_yaw_motor.speed_set = gimbal_yaw_motor.speed_set;

    gimbal_pitch_motor.encode_angle_set = gimbal_pitch_motor.encode_angle;
    gimbal_pitch_motor.speed_set = gimbal_pitch_motor.speed_set;

    /*--------------------------Flag变量初始化----------------------------*/
    // 云台自锁Flag初始化
    gimbal_stop_flag = false;
    // 更新云台数据
    feedback_update();
}

/*
 * @brief          更新云台数据
 * @Author         WSJ
 */
void Gimbal::feedback_update()
{
    // 按键数据更新
    key_state_update();
    // 模式切换数据保存
    mode_change_save();
    last_gimbal_mode = gimbal_mode;
    gimbal_data_update();
}
/**
 * @brief          模式切换数据保存
 * @Author         WSJ
 */
void Gimbal::mode_change_save()
{
    // 切入底盘跟随云台模式
    if (last_gimbal_mode != GIMBAL_CHASSIS && gimbal_mode == GIMBAL_CHASSIS)
    {
        gimbal_yaw_motor.gyro_angle_set = gimbal_yaw_motor.gyro_angle;
        gimbal_pitch_motor.gyro_angle_set = gimbal_pitch_motor.gyro_angle;
    }
    // 切入自由控制模式
    else if (last_gimbal_mode != GIMBAL_FREE && gimbal_mode == GIMBAL_FREE)
    {
        gimbal_yaw_motor.encode_angle_set = gimbal_yaw_motor.encode_angle;
        gimbal_pitch_motor.encode_angle_set = gimbal_pitch_motor.encode_angle;
    }
}
/**
 * @brief          云台数据计算更新
 * @Author         WSJ
 */
void Gimbal::gimbal_data_update()
{
    /*------------------------yaw电机数据更新----------------------------  */
    gimbal_yaw_motor.gyro_angle = gimbal_INT_angle_point[INS_YAW_ADDRESS_OFFSET];

#if YAW_TURN
    gimbal_yaw_motor.encode_angle = -motor_ecd_to_angle_change(gimbal_yaw_motor.motor_measure->ecd,
                                                               gimbal_yaw_motor.offset_ecd);
#else
    gimbal_yaw_motor.encode_angle = motor_ecd_to_angle_change(gimbal_yaw_motor.motor_measure->ecd,
                                                              gimbal_yaw_motor.offset_ecd);
#endif

    // 在云台归中时,读取的速度为编码器反馈的
    if (gimbal_mode == GIMBAL_TO_MID)
        gimbal_yaw_motor.speed = GM6020_MOTOR_RPM_TO_VECTOR * gimbal_yaw_motor.motor_measure->speed_rpm;
    else
        gimbal_yaw_motor.speed = cos(gimbal_pitch_motor.encode_angle) * (gimbal_INT_gyro_point[INS_GYRO_Z_ADDRESS_OFFSET]) - sin(gimbal_pitch_motor.encode_angle) * (gimbal_INT_gyro_point[INS_GYRO_X_ADDRESS_OFFSET]);

    /*------------------------pitch电机数据更新----------------------------  */
    // pitch电机
    gimbal_pitch_motor.gyro_angle = -gimbal_INT_angle_point[INS_PITCH_ADDRESS_OFFSET];

#if PITCH_TURN
    gimbal_pitch_motor.encode_angle = -motor_ecd_to_angle_change(gimbal_pitch_motor.motor_measure->ecd,
                                                                 gimbal_pitch_motor.offset_ecd);
#else
    gimbal_pitch_motor.encode_angle = motor_ecd_to_angle_change(gimbal_pitch_motor.motor_measure->ecd,
                                                                gimbal_pitch_motor.offset_ecd);
#endif

    // 在云台归中时,读取的速度为编码器反馈的
    if (gimbal_mode == GIMBAL_TO_MID || gimbal_mode == GIMBAL_FREE)
        gimbal_pitch_motor.speed = GM6020_MOTOR_RPM_TO_VECTOR * gimbal_pitch_motor.motor_measure->speed_rpm;
    else
        gimbal_pitch_motor.speed = gimbal_INT_gyro_point[INS_GYRO_Y_ADDRESS_OFFSET];
}
/**
 * @brief          按键数据更新
 * @Author         WSJ
 */
void Gimbal::key_state_update()
{
    /*--------------------------按键状态更新------------------------------*/
    // 获取上一次的按键状态
    gimbal_last_key_v = gimbal_RC->key.v;
    last_press_r = press_r;
    press_r = gimbal_RC->mouse.press_r;

    // 鼠标右键长按计时
    if (press_r)
    {
        if (press_r_time < PRESS_R_LONG_TIME)
        {
            press_r_time++;
        }
    }
    else
    {
        press_r_time = 0;
    }

    press_stop = if_key_pessed(gimbal_RC->key.v, KEY_PRESSED_STOP_GIMBAL);

    // 长按自锁按键长按计时
    if (press_stop)
    {
        if (press_stop_time < PRESS_STOP_LONG_TIME)
        {
            press_stop_time++;
        }
    }
    else
    {
        press_stop_time = 0;
    }

    if (press_stop && gimbal_stop_flag == true) // 单击自锁按键并且开启完毕
    {
        gimbal_stop_flag = false;
    }
    if (press_stop_time == PRESS_STOP_LONG_TIME && gimbal_stop_flag == false) // 长按自锁按键并且关闭完毕
    {
        gimbal_stop_flag = true;
    }

    if (gimbal_RC->rc.ch[4] > 500)
    {
        gimbal_stop_flag = true;
    }
    else
    {
        gimbal_stop_flag = false;
    }
    // 更新按键状态
    last_gimbal_RC->key.v = gimbal_RC->key.v;
}
/**
 * @brief          设置云台控制模式,主要在'behaviour_mode_set'函数中改变
 * @Author         WSJ
 */
void Gimbal::set_mode()
{
    // 记录上一次的模式
    last_gimbal_mode = gimbal_mode;
    // 判断是否需要进入校准模式
    if (need_cali())
        return;

    // 归中模式
    if (gimbal_mode == GIMBAL_TO_MID)
    {
        if (!gimbal_to_mid())
        {
            return;
        }
    }
    // 拨杆改变模式(在这里修改拨杆值对应的模式)
    switch_control();
    // 判断进入to_mid状态机
    if (last_gimbal_mode == GIMBAL_ZERO_FORCE && gimbal_mode != GIMBAL_ZERO_FORCE)
    {
        gimbal_mode = GIMBAL_TO_MID;
    }
    // 判断退出了to_mid状态机,开启陀螺仪
    if (last_gimbal_mode == GIMBAL_TO_MID && gimbal_mode != GIMBAL_TO_MID)
    {
        gimbal_imu_open_flag = true;
    }

#if GIMBAL_FRIC_OPEN_PITCH_UP
    pitch_up_control();
#endif
}
/**
 * @brief          判断是否需要进行校准行为
 * @Author         WSJ
 */
bool_t Gimbal::need_cali()
{
    // 校准行为，return 不会设置其他的模式
    if (gimbal_mode == GIMBAL_CALI && gimbal_cali.step != GIMBAL_CALI_END_STEP)
    {
        return true;
    }
    // 如果外部使得校准步骤从0 变成 start，则进入校准模式
    if (gimbal_cali.step == GIMBAL_CALI_START_STEP && !toe_is_error(DBUS_TOE))
    {
        gimbal_mode = GIMBAL_CALI;
        return true;
    }
    return false;
}
/**
 * @brief          云台归中
 * @Author         WSJ
 * @return         完成返回true
 */
bool_t Gimbal::gimbal_to_mid()
{
    static uint16_t init_time = 0;
    static uint16_t init_stop_time = 0;
    init_time++;

    if ((fabs(gimbal_yaw_motor.encode_angle - YAW_TO_MID_SET) < GIMBAL_TO_MID_ERROR &&
         fabs(gimbal_pitch_motor.encode_angle - PITCH_TO_MID_SET) < GIMBAL_TO_MID_ERROR))
    {
        if (init_stop_time < GIMBAL_TO_MID_STOP_TIME)
        {
            init_stop_time++;
        }
    }
    else
    {
        if (init_time < GIMBAL_TO_MID_TIME)
        {
            init_time++;
        }
    }

    // 超过初始化最大时间，或者已经稳定到中值一段时间，退出初始化状态开关打下档，或者掉线
    if (init_time < GIMBAL_TO_MID_TIME && init_stop_time < GIMBAL_TO_MID_STOP_TIME &&
        !switch_is_down(gimbal_RC->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        return false;
    }
    else
    {
        init_stop_time = 0;
        init_time = 0;
        return true;
    }
}
/**
 * @brief          拨杆控制模式
 * @Author         WSJ
 */
void Gimbal::switch_control()
{
    if (switch_is_up(gimbal_RC->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        gimbal_mode = GIMBAL_CHASSIS;
    }
    else if (switch_is_mid(gimbal_RC->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        gimbal_mode = GIMBAL_FREE;
    }
    else if (switch_is_down(gimbal_RC->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        gimbal_mode = GIMBAL_ZERO_FORCE;
    }
}
/**
 * @brief          摩擦轮上电抬头控制
 * @Author         WSJ
 */
void Gimbal::pitch_up_control()
{
    // 云台抬头时间
    static uint16_t gimbal_pitch_up_time = 0;
    // 当摩擦轮刚开启,云台抬头
    if (shoot_open_fric_cmd_to_gimbal_up() == 1 && gimbal_pitch_up_time < 400)
    {
        gimbal.gimbal_pitch_motor.encode_angle_set = PITCH_TO_MID_SET + 0.3f;
        gimbal_pitch_up_time++;
    }
    else if (shoot_open_fric_cmd_to_gimbal_up() == 0)
    {
        gimbal_pitch_up_time = 0;
    }
}
/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 * @param[in]      ecd: 电机当前编码
 * @param[in]      offset_ecd: 电机中值编码
 * @retval         相对角度，单位rad
 * @Author         summerpray
 */
fp32 Gimbal::motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
 * @brief          设置云台控制设定值，控制值是通过behaviour_control_set函数设置的
 * @retval         none
 * Author          summerpray
 */
void Gimbal::set_control()
{
    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;

    if (gimbal_mode == GIMBAL_TO_MID)
    {
        // 归中模式控制量计算
        gimbal_to_mid_control(&add_yaw_angle, &add_pitch_angle);
        // 归中模式下yaw和pitch角度都由编码器控制
        gimbal_yaw_motor.angle_limit(add_yaw_angle, ENCODE);
        gimbal_pitch_motor.angle_limit(add_pitch_angle, ENCODE);
    }
    else if (gimbal_mode == GIMBAL_CHASSIS)
    {
        // 底盘跟随模式控制量计算
        gimbal_chassis_control(&add_yaw_angle, &add_pitch_angle);
        // 底盘跟随模式下yaw使用陀螺仪控制,pitch使用陀螺仪控制
        gimbal_yaw_motor.angle_limit(add_yaw_angle, GYRO);
        gimbal_pitch_motor.angle_limit(add_pitch_angle, GYRO);
    }
    else if (gimbal_mode == GIMBAL_FREE)
    {
        // 自由模式控制量计算
        gimbal_free_control(&add_yaw_angle, &add_pitch_angle);
        // 自由模式下yaw和pitch角度都由编码器控制
        gimbal_yaw_motor.angle_limit(add_yaw_angle, ENCODE);
        gimbal_pitch_motor.angle_limit(add_pitch_angle, ENCODE);
    }
}
/**
 * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
 * @param[out]     yaw轴角度控制，为角度的增量 单位 rad
 * @param[out]     pitch轴角度控制，为角度的增量 单位 rad
 * @retval         返回空
 */
void Gimbal::gimbal_to_mid_control(fp32 *yaw, fp32 *pitch)
{
    if (yaw == NULL || pitch == NULL)
    {
        return;
    }

    // 使用自己的写法 摒弃了初始化时使用陀螺仪数据
    // 初始化状态控制量计算
    if (fabs(PITCH_TO_MID_SET - gimbal_pitch_motor.encode_angle) > GIMBAL_TO_MID_ERROR)
    {
        *pitch = (PITCH_TO_MID_SET - gimbal_pitch_motor.encode_angle) * GIMBAL_TO_MID_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else
    {
        *pitch = (PITCH_TO_MID_SET - gimbal_pitch_motor.encode_angle) * GIMBAL_TO_MID_PITCH_SPEED;
        *yaw = (YAW_TO_MID_SET - gimbal_yaw_motor.encode_angle) * GIMBAL_TO_MID_YAW_SPEED;
    }
}

/**
 * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
 * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
 * @retval         none
 */
void Gimbal::gimbal_chassis_control(fp32 *yaw, fp32 *pitch)
{
    if (yaw == NULL || pitch == NULL)
    {
        return;
    }

    // 长按右键 打开自瞄 松开关闭自瞄
    if (press_r_time == PRESS_R_LONG_TIME && auto_switch == FALSE)
    {
        // 更新自瞄PID
        update_auto_pid();
        auto_switch = TRUE;
    }
    else if (press_r_time != PRESS_R_LONG_TIME && auto_switch == TRUE)
    {
        // 恢复之前的PID
        recover_normal_pid();
        auto_switch = FALSE;
    }
#if GIMABL_VISION_DEBUG
    update_auto_pid();
    auto_switch = TRUE;
#endif
    // 当在自瞄模式下且识别到目标,云台控制权交给mini pc
    if (auto_switch == TRUE && vision_if_find_target() == TRUE)
    {
        vision_error_angle(yaw, pitch); // 获取yaw 和 pitch的偏移量
    }
    else
    {
        static int16_t yaw_channel = 0, pitch_channel = 0;

        rc_deadband_limit(gimbal_RC->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
        rc_deadband_limit(gimbal_RC->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

        *yaw = yaw_channel * YAW_RC_SEN + gimbal_RC->mouse.x * YAW_MOUSE_SEN;
        *pitch = pitch_channel * PITCH_RC_SEN + gimbal_RC->mouse.y * PITCH_MOUSE_SEN;

        // 掉头控制
        turn_around_control(yaw);
    }
    // 云台自锁功能
    if (gimbal_stop_flag)
    {
        *yaw = 0;
        *pitch = 0;
    }

#if GIMBAL_HIGH_PASS_FILTER
    // 一阶高通滤波作为云台角度输入减缓小范围漂移
    gimbal_yaw_high_pass_filter.first_high_pass_filter_cali(*yaw);
    gimbal_pitch_high_pass_filter.first_high_pass_filter_cali(*pitch);

    *yaw = gimbal_yaw_high_pass_filter.out;
    *pitch = gimbal_pitch_high_pass_filter.out;
#endif

    last_gimbal_RC->key.v = gimbal_RC->key.v;
}
/**
 * @brief          更新自瞄模式PID
 * @retval         none
 */
void Gimbal::update_auto_pid()
{
    // 更新自瞄pid系数
    fp32 yaw_gyro_angle_pid_parm[5] = {YAW_AUTO_PID_KP, YAW_AUTO_PID_KI, YAW_AUTO_PID_KD, YAW_AUTO_PID_MAX_IOUT, YAW_AUTO_PID_MAX_OUT};
    gimbal_yaw_motor.gyro_angle_pid.init(PID_ANGLE, yaw_gyro_angle_pid_parm, &gimbal_yaw_motor.gyro_angle, &gimbal_yaw_motor.gyro_angle_set, 0);

    fp32 pitch_gyro_angle_pid_parm[5] = {PITCH_AUTO_PID_KP, PITCH_AUTO_PID_KI, PITCH_AUTO_PID_KD, PITCH_AUTO_PID_MAX_IOUT, PITCH_AUTO_PID_MAX_OUT};
    gimbal_pitch_motor.gyro_angle_pid.init(PID_ANGLE, pitch_gyro_angle_pid_parm, &gimbal_pitch_motor.gyro_angle, &gimbal_pitch_motor.gyro_angle_set, 0);
}
/**
 * @brief          恢复之前的PID
 * @retval         none
 */
void Gimbal::recover_normal_pid()
{
    // 恢复原来的PID系数
    fp32 yaw_gyro_angle_pid_parm[5] = {YAW_GYRO_PID_KP, YAW_GYRO_PID_KI, YAW_GYRO_PID_KD, YAW_GYRO_PID_MAX_IOUT, YAW_GYRO_PID_MAX_OUT};
    gimbal_yaw_motor.gyro_angle_pid.init(PID_ANGLE, yaw_gyro_angle_pid_parm, &gimbal_yaw_motor.gyro_angle, &gimbal_yaw_motor.gyro_angle_set, 0);

    fp32 pitch_gyro_angle_pid_parm[5] = {PITCH_GYRO_PID_KP, PITCH_GYRO_PID_KI, PITCH_GYRO_PID_KD, PITCH_GYRO_PID_MAX_IOUT, PITCH_GYRO_PID_MAX_OUT};
    gimbal_pitch_motor.gyro_angle_pid.init(PID_ANGLE, pitch_gyro_angle_pid_parm, &gimbal_pitch_motor.gyro_angle, &gimbal_pitch_motor.gyro_angle_set, 0);
}
/**
 * @brief          掉头控制
 * @retval         none
 */
void Gimbal::turn_around_control(fp32 *yaw)
{
    static uint16_t last_turn_key_value = 0;
    static uint8_t gimbal_turn_flag = 0;
    static fp32 gimbal_end_angle = 0.0f;

    if (if_key_singal_pessed(gimbal_RC->key.v, last_turn_key_value, KEY_PRESSED_GIMBAL_TURN_180))
    {
        if (gimbal_turn_flag == 0)
        {
            gimbal_turn_flag = 1;
            // 保存掉头的目标值
            gimbal_end_angle = rad_format(gimbal_yaw_motor.gyro_angle + PI);
        }
    }

    last_turn_key_value = gimbal_RC->key.v;

    if (gimbal_turn_flag)
    {
        // 不断控制到掉头的目标值，正转，反装是随机
        if (rad_format(gimbal_end_angle - gimbal_yaw_motor.gyro_angle) > 0.0f)
        {
            *yaw += TURN_SPEED;
        }
        else
        {
            *yaw -= TURN_SPEED;
        }
    }
    // 到达pi （180°）后停止
    if (gimbal_turn_flag && fabs(rad_format(gimbal_end_angle - gimbal_yaw_motor.gyro_angle)) < 0.01f)
    {
        gimbal_turn_flag = 0;
    }
}
/**
 * @brief          云台编码值控制，电机是相对角度控制，
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @retval         none
 */
void Gimbal::gimbal_free_control(fp32 *yaw, fp32 *pitch)
{
    if (yaw == NULL || pitch == NULL)
    {
        return;
    }

    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(gimbal_RC->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_RC->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

    *yaw = yaw_channel * YAW_RC_SEN + gimbal_RC->mouse.x * YAW_MOUSE_SEN;
    *pitch = pitch_channel * PITCH_RC_SEN + gimbal_RC->mouse.y * PITCH_MOUSE_SEN;
}

/**
 * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @retval         none
 */
void Gimbal::gimbal_motionless_control(fp32 *yaw, fp32 *pitch)
{
    if (yaw == NULL || pitch == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}

/***************************(C) GIMBAL control *******************************/

/***************************(C)  MOTOR control *******************************/
/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @retval         none
 * @Author         summerpray
 */
void Gimbal::solve()
{
    if (gimbal_mode == GIMBAL_TO_MID)
    {
        // 归中模式下yaw和pitch角度都由编码器控制
        gimbal_yaw_motor.motor_encode_angle_control();
        gimbal_pitch_motor.motor_encode_angle_control();
    }
    else if (gimbal_mode == GIMBAL_CHASSIS)
    {
        // 底盘跟随模式下yaw使用陀螺仪控制,pitch使用陀螺仪控制
        gimbal_yaw_motor.motor_gyro_angle_control();
        gimbal_pitch_motor.motor_gyro_angle_control();
    }
    else if (gimbal_mode == GIMBAL_FREE)
    {
        // 自由模式下yaw和pitch角度都由编码器控制
        gimbal_yaw_motor.motor_encode_angle_control();
        gimbal_pitch_motor.motor_encode_angle_control();
    }
}

/**
 * @brief         输出电流
 * @retval         none
 * @Author         summerpray
 */
void Gimbal::output()
{

// 根据电机正反转,调解电流发送
#if YAW_TURN
    gimbal_yaw_motor.current_give = -(int16_t)(gimbal_yaw_motor.current_set);
#else
    gimbal_yaw_motor.current_give = (int16_t)(gimbal_yaw_motor.current_set);
#endif
#if PITCH_TURN
    gimbal_pitch_motor.current_give = -(int16_t)(gimbal_pitch_motor.current_set);
#else
    gimbal_pitch_motor.current_give = (int16_t)(gimbal_pitch_motor.current_set);
#endif

    if (gimbal_mode == GIMBAL_ZERO_FORCE)
    {
        gimbal_yaw_motor.current_give = 0;
        gimbal_pitch_motor.current_give = 0;
    }

// 电流控制
#if !GIMBAL_YAW_MOTOR_HAVE_CURRENT
    gimbal_yaw_motor.current_give = 0;
#endif

#if !GIMBAL_PITCH_MOTOR_HAVE_CURRENT
    gimbal_pitch_motor.current_give = 0;
#endif

    can_receive.can_cmd_yaw_motor(gimbal_yaw_motor.current_give);
    can_receive.can_cmd_pitch_motor(gimbal_pitch_motor.current_give);
}
/*****************************(C) CALI GIMBAL *******************************/
/**
 * @brief          云台校准计算，将校准记录的中值,最大 最小值
 * @param[out]     yaw 中值 指针
 * @param[out]     pitch 中值 指针
 * @param[out]     yaw 最大相对角度 指针
 * @param[out]     yaw 最小相对角度 指针
 * @param[out]     pitch 最大相对角度 指针
 * @param[out]     pitch 最小相对角度 指针
 * @retval         none
 */

void Gimbal::calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL || max_pitch == NULL || min_pitch == NULL)
    {
        return;
    }

    int16_t temp_max_ecd = 0, temp_min_ecd = 0, temp_ecd = 0;

#if YAW_TURN
    temp_ecd = gimbal_cali->min_yaw_ecd - gimbal_cali->max_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ECD_RANGE;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd + (temp_ecd / 2);

    ecd_format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = -motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = -motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#else

    temp_ecd = gimbal_cali->max_yaw_ecd - gimbal_cali->min_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ECD_RANGE;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd - (temp_ecd / 2);

    ecd_format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#endif

#if PITCH_TURN

    temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
    temp_max_ecd = gimbal_cali->max_pitch_ecd + temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
    temp_min_ecd = gimbal_cali->min_pitch_ecd + temp_ecd;

    ecd_format(temp_max_ecd);
    ecd_format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > HALF_ECD_RANGE)
    {
        temp_ecd -= ECD_RANGE;
    }
    else if (temp_ecd < -HALF_ECD_RANGE)
    {
        temp_ecd += ECD_RANGE;
    }

    if (temp_max_ecd > temp_min_ecd)
    {
        temp_min_ecd += ECD_RANGE;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ecd_format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = -motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = -motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);

#else
    temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
    temp_max_ecd = gimbal_cali->max_pitch_ecd - temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
    temp_min_ecd = gimbal_cali->min_pitch_ecd - temp_ecd;

    ecd_format(temp_max_ecd);
    ecd_format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > HALF_ECD_RANGE)
    {
        temp_ecd -= ECD_RANGE;
    }
    else if (temp_ecd < -HALF_ECD_RANGE)
    {
        temp_ecd += ECD_RANGE;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ecd_format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
#endif
}

/**
  * @brief          云台设备校准
  * @param[in][out] cali:指针指向云台数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
void Gimbal::set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
    gimbal_yaw_motor.offset_ecd = yaw_offset;
    gimbal_yaw_motor.max_encode_angle = max_yaw;
    gimbal_yaw_motor.min_encode_angle = min_yaw;

    gimbal_pitch_motor.offset_ecd = pitch_offset;
    gimbal_pitch_motor.max_encode_angle = max_pitch;
    gimbal_pitch_motor.min_encode_angle = min_pitch;
}

/*
 * @brief          手动设置云台编码器中值，最小最大机械相对角度
 * @param[in]      yaw_offse:yaw 中值
 * @param[in]      pitch_offset:pitch 中值
 * @param[in]      max_yaw:max_yaw:yaw 最大相对角度
 * @param[in]      min_yaw:yaw 最小相对角度
 * @param[in]      max_yaw:pitch 最大相对角度
 * @param[in]      min_yaw:pitch 最小相对角度
 * @retval         返回空
 * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
 */
void Gimbal::set_hand_operator_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
    gimbal_yaw_motor.offset_ecd = yaw_offset;
    gimbal_yaw_motor.max_encode_angle = max_yaw;
    gimbal_yaw_motor.min_encode_angle = min_yaw;

    gimbal_pitch_motor.offset_ecd = pitch_offset;
    gimbal_pitch_motor.max_encode_angle = max_pitch;
    gimbal_pitch_motor.min_encode_angle = min_pitch;
}

/**
 * @brief          云台校准计算，将校准记录的中值,最大 最小值返回
 * @param[out]     yaw 中 指针
 * @param[out]     pitch 中值 指针
 * @param[out]     yaw 最大相对角度 指针
 * @param[out]     yaw 最小相对角度 指针
 * @param[out]     pitch 最大相对角度 指针
 * @param[out]     pitch 最小相对角度 指针
 * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
 * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
 */
bool_t Gimbal::cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_cali.step == 0)
    {
        gimbal_cali.step = GIMBAL_CALI_START_STEP;
        // 保存进入时候的数据，作为起始数据，来判断最大，最小值
        gimbal_cali.max_pitch = gimbal_pitch_motor.gyro_angle;
        gimbal_cali.max_pitch_ecd = gimbal_pitch_motor.motor_measure->ecd;
        gimbal_cali.max_yaw = gimbal_yaw_motor.gyro_angle;
        gimbal_cali.max_yaw_ecd = gimbal_yaw_motor.motor_measure->ecd;
        gimbal_cali.min_pitch = gimbal_pitch_motor.gyro_angle;
        gimbal_cali.min_pitch_ecd = gimbal_pitch_motor.motor_measure->ecd;
        gimbal_cali.min_yaw = gimbal_yaw_motor.gyro_angle;
        gimbal_cali.min_yaw_ecd = gimbal_yaw_motor.motor_measure->ecd;
        return 0;
    }
    else if (gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        calc_gimbal_cali(&gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
        (*max_yaw) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_yaw) += GIMBAL_CALI_REDUNDANT_ANGLE;
        (*max_pitch) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_pitch) += GIMBAL_CALI_REDUNDANT_ANGLE;
        gimbal_yaw_motor.offset_ecd = *yaw_offset;
        gimbal_yaw_motor.max_encode_angle = *max_yaw;
        gimbal_yaw_motor.min_encode_angle = *min_yaw;
        gimbal_pitch_motor.offset_ecd = *pitch_offset;
        gimbal_pitch_motor.max_encode_angle = *max_pitch;
        gimbal_pitch_motor.min_encode_angle = *min_pitch;
        gimbal_cali.step = 0;
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief          云台在某些行为下，需要射击停止
 * @param[in]      none
 * @retval         1: no move 0:normal
 */

bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal.gimbal_mode == GIMBAL_TO_MID || gimbal.gimbal_mode == GIMBAL_CALI || gimbal.gimbal_mode == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

#else

/**
 * @brief          初始化云台
 * @Author         WSJ
 */
void Gimbal::init()
{
    /*------------------------云台状态机初始化----------------------------*/
    // 初始化初始状态为归中模式
    gimbal_mode = GIMBAL_TO_MID;

    /*---------------------------yaw电机---------------------------------*/
    // 获取Yaw轴电机数据指针
    gimbal_yaw_motor.init_data_point(can_receive.get_gimbal_motor_measure_point(YAW));

    // 初始化Yaw轴PID数据
    gimbal_yaw_motor.init_pid_param(YAW);

    // 设置Yaw轴电机角度限幅和中值
    gimbal_yaw_motor.init_gimbal_offset(YAW);

    /*--------------------------pitch电机--------------------------------*/
    // 获取Pitch轴电机数据指针
    gimbal_pitch_motor.init_data_point(can_receive.get_gimbal_motor_measure_point(PITCH));

    // 初始化Pitch轴PID数据
    gimbal_pitch_motor.init_pid_param(PITCH);

    // 设置Pitch轴电机角度限幅和中值
    gimbal_pitch_motor.init_gimbal_offset(PITCH);

    /*--------------------------滤波值初始化-----------------------------*/
    const static fp32 gimbal_yaw_high_pass_filter_para[1] = {GIMBAL_ACCEL_YAW_NUM};
    const static fp32 gimbal_pitch_high_pass_filter_para[1] = {GIMBAL_ACCEL_PITCH_NUM};
    // 一阶高通滤波初始化
    gimbal_yaw_high_pass_filter.init(GIMBAL_CONTROL_TIME, gimbal_yaw_high_pass_filter_para);
    gimbal_pitch_high_pass_filter.init(GIMBAL_CONTROL_TIME, gimbal_pitch_high_pass_filter_para);

    /*----------------------------陀螺仪指针-----------------------------*/
    // 陀螺仪数据指针获取
    gimbal_INT_angle_point = imu.get_INS_angle_point();
    gimbal_INT_gyro_point = imu.get_gyro_data_point();

    /*--------------------------设定初始设置值----------------------------*/
    gimbal_yaw_motor.gyro_angle_set = gimbal_yaw_motor.gyro_angle;
    gimbal_yaw_motor.encode_angle_set = gimbal_yaw_motor.encode_angle;
    gimbal_yaw_motor.speed_set = gimbal_yaw_motor.speed_set;

    gimbal_pitch_motor.encode_angle_set = gimbal_pitch_motor.encode_angle;
    gimbal_pitch_motor.speed_set = gimbal_pitch_motor.speed_set;

    /*--------------------------Flag变量初始化----------------------------*/
    // 云台自锁Flag初始化
    gimbal_stop_flag = true;

    vision_target_flag = 0; // 没有看到敌方目标
    // 更新云台数据
    feedback_update();
}

/*
 * @brief          更新云台数据
 * @Author         WSJ
 */
void Gimbal::feedback_update()
{
    gimbal_data_update();
}
/**
 * @brief          云台数据计算更新
 * @Author         WSJ
 */
void Gimbal::gimbal_data_update()
{
    /*------------------------yaw电机数据更新----------------------------  */
    gimbal_yaw_motor.gyro_angle = gimbal_INT_angle_point[INS_YAW_ADDRESS_OFFSET];

#if YAW_TURN
    gimbal_yaw_motor.encode_angle = -motor_ecd_to_angle_change(gimbal_yaw_motor.motor_measure->ecd,
                                                               gimbal_yaw_motor.offset_ecd);
#else
    gimbal_yaw_motor.encode_angle = motor_ecd_to_angle_change(gimbal_yaw_motor.motor_measure->ecd,
                                                              gimbal_yaw_motor.offset_ecd);
#endif

    // 在云台归中时,读取的速度为编码器反馈的
    if (gimbal_mode == GIMBAL_TO_MID)
        gimbal_yaw_motor.speed = GM6020_MOTOR_RPM_TO_VECTOR * gimbal_yaw_motor.motor_measure->speed_rpm;
    else
        gimbal_yaw_motor.speed = cos(gimbal_pitch_motor.encode_angle) * (gimbal_INT_gyro_point[INS_GYRO_Z_ADDRESS_OFFSET]) - sin(gimbal_pitch_motor.encode_angle) * (gimbal_INT_gyro_point[INS_GYRO_X_ADDRESS_OFFSET]);

    /*------------------------pitch电机数据更新----------------------------  */
    // pitch电机
    gimbal_pitch_motor.gyro_angle = -gimbal_INT_angle_point[INS_PITCH_ADDRESS_OFFSET];

#if PITCH_TURN
    gimbal_pitch_motor.encode_angle = -motor_ecd_to_angle_change(gimbal_pitch_motor.motor_measure->ecd,
                                                                 gimbal_pitch_motor.offset_ecd);
#else
    gimbal_pitch_motor.encode_angle = motor_ecd_to_angle_change(gimbal_pitch_motor.motor_measure->ecd,
                                                                gimbal_pitch_motor.offset_ecd);
#endif

    // 在云台归中时,读取的速度为编码器反馈的
    if (gimbal_mode == GIMBAL_TO_MID || gimbal_mode == GIMBAL_FREE)
        gimbal_pitch_motor.speed = GM6020_MOTOR_RPM_TO_VECTOR * gimbal_pitch_motor.motor_measure->speed_rpm;
    else
        gimbal_pitch_motor.speed = gimbal_INT_gyro_point[INS_GYRO_Y_ADDRESS_OFFSET];
}

/**
 * @brief          设置云台控制模式,主要在'behaviour_mode_set'函数中改变
 * @Author         WSJ
 */
void Gimbal::set_mode()
{
    // 判断是否需要进入校准模式
    if (need_cali())
        return;

    // 归中模式
    if (gimbal_mode == GIMBAL_TO_MID)
    {
        if (!gimbal_to_mid())
        {
            return;
        }
    }

#if GIMBAL_FRIC_OPEN_PITCH_UP
    pitch_up_control();
#endif
}
/**
 * @brief          判断是否需要进行校准行为
 * @Author         WSJ
 */
bool_t Gimbal::need_cali()
{
    // 校准行为，return 不会设置其他的模式
    if (gimbal_mode == GIMBAL_CALI && gimbal_cali.step != GIMBAL_CALI_END_STEP)
    {
        return true;
    }
    // 如果外部使得校准步骤从0 变成 start，则进入校准模式
    if (gimbal_cali.step == GIMBAL_CALI_START_STEP && !toe_is_error(DBUS_TOE))
    {
        gimbal_mode = GIMBAL_CALI;
        return true;
    }
    return false;
}
/**
 * @brief          云台归中
 * @Author         WSJ
 * @return         完成返回true
 */
bool_t Gimbal::gimbal_to_mid()
{
    static uint16_t init_time = 0;
    static uint16_t init_stop_time = 0;
    init_time++;

    if ((fabs(gimbal_yaw_motor.encode_angle - YAW_TO_MID_SET) < GIMBAL_TO_MID_ERROR &&
         fabs(gimbal_pitch_motor.encode_angle - PITCH_TO_MID_SET) < GIMBAL_TO_MID_ERROR))
    {
        if (init_stop_time < GIMBAL_TO_MID_STOP_TIME)
        {
            init_stop_time++;
        }
    }
    else
    {
        if (init_time < GIMBAL_TO_MID_TIME)
        {
            init_time++;
        }
    }

    // 超过初始化最大时间，或者已经稳定到中值一段时间，退出初始化状态开关打下档，或者掉线
    if (init_time < GIMBAL_TO_MID_TIME && init_stop_time < GIMBAL_TO_MID_STOP_TIME &&
        !switch_is_down(gimbal_RC->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        return false;
    }
    else
    {
        init_stop_time = 0;
        init_time = 0;
        return true;
    }
}

/**
 * @brief          摩擦轮上电抬头控制
 * @Author         WSJ
 */
void Gimbal::pitch_up_control()
{
    // 云台抬头时间
    static uint16_t gimbal_pitch_up_time = 0;
    // 当摩擦轮刚开启,云台抬头
    if (shoot_open_fric_cmd_to_gimbal_up() == 1 && gimbal_pitch_up_time < 400)
    {
        gimbal.gimbal_pitch_motor.encode_angle_set = PITCH_TO_MID_SET + 0.3f;
        gimbal_pitch_up_time++;
    }
    else if (shoot_open_fric_cmd_to_gimbal_up() == 0)
    {
        gimbal_pitch_up_time = 0;
    }
}
/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 * @param[in]      ecd: 电机当前编码
 * @param[in]      offset_ecd: 电机中值编码
 * @retval         相对角度，单位rad
 * @Author         summerpray
 */
fp32 Gimbal::motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
 * @brief          设置云台控制设定值，控制值是通过behaviour_control_set函数设置的
 * @retval         none
 * Author          summerpray
 */
void Gimbal::set_control()
{
    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;

    if (gimbal_mode == GIMBAL_TO_MID)
    {
        // 归中模式控制量计算
        gimbal_to_mid_control(&add_yaw_angle, &add_pitch_angle);
        // 归中模式下yaw和pitch角度都由编码器控制
        gimbal_yaw_motor.angle_limit(add_yaw_angle, ENCODE);
        gimbal_pitch_motor.angle_limit(add_pitch_angle, ENCODE);

        // 完成归中模式后，云台模式自动改为自由控制模式，并开启寻找敌方机器人
        gimbal_mode == GIMBAL_FREE;
    }
    else if (gimbal_mode == GIMBAL_FREE)
    {
        // 自由模式控制量计算
        gimbal_free_control(&add_yaw_angle, &add_pitch_angle);
        // 自由模式下yaw和pitch角度都由编码器控制
        gimbal_yaw_motor.angle_limit(add_yaw_angle, ENCODE);
        gimbal_pitch_motor.angle_limit(add_pitch_angle, ENCODE);
    }
}
/**
 * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
 * @param[out]     yaw轴角度控制，为角度的增量 单位 rad
 * @param[out]     pitch轴角度控制，为角度的增量 单位 rad
 * @retval         返回空
 */
void Gimbal::gimbal_to_mid_control(fp32 *yaw, fp32 *pitch)
{
    if (yaw == NULL || pitch == NULL)
    {
        return;
    }

    // 使用自己的写法 摒弃了初始化时使用陀螺仪数据
    // 初始化状态控制量计算
    if (fabs(PITCH_TO_MID_SET - gimbal_pitch_motor.encode_angle) > GIMBAL_TO_MID_ERROR)
    {
        *pitch = (PITCH_TO_MID_SET - gimbal_pitch_motor.encode_angle) * GIMBAL_TO_MID_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else
    {
        *pitch = (PITCH_TO_MID_SET - gimbal_pitch_motor.encode_angle) * GIMBAL_TO_MID_PITCH_SPEED;
        *yaw = (YAW_TO_MID_SET - gimbal_yaw_motor.encode_angle) * GIMBAL_TO_MID_YAW_SPEED;
    }
}

/**
 * @brief          对int型数据做绝对值
 * @param[out]
 * @retval         none
 */
int Gimbal::int_fabs(int gimbal_swing_time)
{
    gimbal_swing_time < 0 ? gimbal_swing_time = -gimbal_swing_time : gimbal_swing_time = gimbal_swing_time;
}

/**
 * @brief          云台摆头
 * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
 * @retval         none
 */
void Gimbal::gimbal_swing(fp32 *yaw, fp32 *pitch)
{
    static int gimbal_swing_time = 0;   // 云台摆头时间计时
    static uint8_t gimbal_swing_lr = 1; // 云台摆头方向，1为右摆，-1为左摆

    if (int_fabs(gimbal_swing_time) < GIMBAL_SWING_MAXTIME) // 通过计时的方式控制摆动角度
    {
        *pitch = gimbal_swing_lr * GIMBAL_SWING_ADJUST; // 改变pitch值
        *yaw = 0;                                       // yaw不变

        gimbal_swing_lr ? gimbal_swing_time++ : gimbal_swing_time--; // 判断摆头方向
    }
    else if (int_fabs(gimbal_swing_time) == GIMBAL_SWING_MAXTIME)
    {
        gimbal_swing_lr == 1 ? gimbal_swing_lr = -1 : gimbal_swing_lr = 1; // 改变摆头方向
        gimbal_swing_time > 0 ? gimbal_swing_time-- : gimbal_swing_time++;
    }
    else
    {
        gimbal_swing_time = 0;
        gimbal_swing_lr = 0;
    }
}

/**
 * @brief          更新自瞄模式PID
 * @retval         none
 */
void Gimbal::update_auto_pid()
{
    // 更新自瞄pid系数
    fp32 yaw_gyro_angle_pid_parm[5] = {YAW_AUTO_PID_KP, YAW_AUTO_PID_KI, YAW_AUTO_PID_KD, YAW_AUTO_PID_MAX_IOUT, YAW_AUTO_PID_MAX_OUT};
    gimbal_yaw_motor.gyro_angle_pid.init(PID_ANGLE, yaw_gyro_angle_pid_parm, &gimbal_yaw_motor.gyro_angle, &gimbal_yaw_motor.gyro_angle_set, 0);

    fp32 pitch_gyro_angle_pid_parm[5] = {PITCH_AUTO_PID_KP, PITCH_AUTO_PID_KI, PITCH_AUTO_PID_KD, PITCH_AUTO_PID_MAX_IOUT, PITCH_AUTO_PID_MAX_OUT};
    gimbal_pitch_motor.gyro_angle_pid.init(PID_ANGLE, pitch_gyro_angle_pid_parm, &gimbal_pitch_motor.gyro_angle, &gimbal_pitch_motor.gyro_angle_set, 0);
}
/**
 * @brief          恢复之前的PID
 * @retval         none
 */
void Gimbal::recover_normal_pid()
{
    // 恢复原来的PID系数
    fp32 yaw_gyro_angle_pid_parm[5] = {YAW_GYRO_PID_KP, YAW_GYRO_PID_KI, YAW_GYRO_PID_KD, YAW_GYRO_PID_MAX_IOUT, YAW_GYRO_PID_MAX_OUT};
    gimbal_yaw_motor.gyro_angle_pid.init(PID_ANGLE, yaw_gyro_angle_pid_parm, &gimbal_yaw_motor.gyro_angle, &gimbal_yaw_motor.gyro_angle_set, 0);

    fp32 pitch_gyro_angle_pid_parm[5] = {PITCH_GYRO_PID_KP, PITCH_GYRO_PID_KI, PITCH_GYRO_PID_KD, PITCH_GYRO_PID_MAX_IOUT, PITCH_GYRO_PID_MAX_OUT};
    gimbal_pitch_motor.gyro_angle_pid.init(PID_ANGLE, pitch_gyro_angle_pid_parm, &gimbal_pitch_motor.gyro_angle, &gimbal_pitch_motor.gyro_angle_set, 0);
}

/**
 * @brief          云台编码值控制，电机是相对角度控制，
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @retval         none
 */
void Gimbal::gimbal_free_control(fp32 *yaw, fp32 *pitch)
{
    if (yaw == NULL || pitch == NULL)
    {
        return;
    }

#if GIMABL_VISION_DEBUG
    update_auto_pid();
    auto_switch = TRUE;
#endif
    // 当识别到目标,开始瞄准
    if (vision_if_find_target() == TRUE)
    {
        vision_target_flag = 1;         // 检测到目标
        update_auto_pid();              // 更新自瞄pid
        vision_error_angle(yaw, pitch); // 获取yaw 和 pitch的偏移量
    }
    else
    {
        vision_target_flag = 0;
        recover_normal_pid(); // 返回原来的pid
        // TODO 云台开始摆头寻找目标
        gimbal_swing(yaw, pitch);
    }
}

/**
 * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @retval         none
 */
void Gimbal::gimbal_motionless_control(fp32 *yaw, fp32 *pitch)
{
    if (yaw == NULL || pitch == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}

#endif

/***************************(C) GIMBAL control *******************************/

/***************************(C)  MOTOR control *******************************/
/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @retval         none
 * @Author         summerpray
 */
void Gimbal::solve()
{
    if (gimbal_mode == GIMBAL_TO_MID)
    {
        // 归中模式下yaw和pitch角度都由编码器控制
        gimbal_yaw_motor.motor_encode_angle_control();
        gimbal_pitch_motor.motor_encode_angle_control();
    }
    else if (gimbal_mode == GIMBAL_FREE)
    {
        // 自由模式下yaw和pitch角度都由编码器控制
        gimbal_yaw_motor.motor_encode_angle_control();
        gimbal_pitch_motor.motor_encode_angle_control();
    }
}

/**
 * @brief         输出电流
 * @retval         none
 * @Author         summerpray
 */
void Gimbal::output()
{

// 根据电机正反转,调解电流发送
#if YAW_TURN
    gimbal_yaw_motor.current_give = -(int16_t)(gimbal_yaw_motor.current_set);
#else
    gimbal_yaw_motor.current_give = (int16_t)(gimbal_yaw_motor.current_set);
#endif
#if PITCH_TURN
    gimbal_pitch_motor.current_give = -(int16_t)(gimbal_pitch_motor.current_set);
#else
    gimbal_pitch_motor.current_give = (int16_t)(gimbal_pitch_motor.current_set);
#endif

    if (gimbal_mode == GIMBAL_ZERO_FORCE)
    {
        gimbal_yaw_motor.current_give = 0;
        gimbal_pitch_motor.current_give = 0;
    }

// 电流控制
#if !GIMBAL_YAW_MOTOR_HAVE_CURRENT
    gimbal_yaw_motor.current_give = 0;
#endif

#if !GIMBAL_PITCH_MOTOR_HAVE_CURRENT
    gimbal_pitch_motor.current_give = 0;
#endif

    can_receive.can_cmd_yaw_motor(gimbal_yaw_motor.current_give);
    can_receive.can_cmd_pitch_motor(gimbal_pitch_motor.current_give);
}
/*****************************(C) CALI GIMBAL *******************************/
/**
 * @brief          云台校准计算，将校准记录的中值,最大 最小值
 * @param[out]     yaw 中值 指针
 * @param[out]     pitch 中值 指针
 * @param[out]     yaw 最大相对角度 指针
 * @param[out]     yaw 最小相对角度 指针
 * @param[out]     pitch 最大相对角度 指针
 * @param[out]     pitch 最小相对角度 指针
 * @retval         none
 */

void Gimbal::calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL || max_pitch == NULL || min_pitch == NULL)
    {
        return;
    }

    int16_t temp_max_ecd = 0, temp_min_ecd = 0, temp_ecd = 0;

#if YAW_TURN
    temp_ecd = gimbal_cali->min_yaw_ecd - gimbal_cali->max_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ECD_RANGE;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd + (temp_ecd / 2);

    ecd_format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = -motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = -motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#else

    temp_ecd = gimbal_cali->max_yaw_ecd - gimbal_cali->min_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ECD_RANGE;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd - (temp_ecd / 2);

    ecd_format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#endif

#if PITCH_TURN

    temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
    temp_max_ecd = gimbal_cali->max_pitch_ecd + temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
    temp_min_ecd = gimbal_cali->min_pitch_ecd + temp_ecd;

    ecd_format(temp_max_ecd);
    ecd_format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > HALF_ECD_RANGE)
    {
        temp_ecd -= ECD_RANGE;
    }
    else if (temp_ecd < -HALF_ECD_RANGE)
    {
        temp_ecd += ECD_RANGE;
    }

    if (temp_max_ecd > temp_min_ecd)
    {
        temp_min_ecd += ECD_RANGE;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ecd_format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = -motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = -motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);

#else
    temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
    temp_max_ecd = gimbal_cali->max_pitch_ecd - temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
    temp_min_ecd = gimbal_cali->min_pitch_ecd - temp_ecd;

    ecd_format(temp_max_ecd);
    ecd_format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > HALF_ECD_RANGE)
    {
        temp_ecd -= ECD_RANGE;
    }
    else if (temp_ecd < -HALF_ECD_RANGE)
    {
        temp_ecd += ECD_RANGE;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ecd_format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
#endif
}

/**
  * @brief          云台设备校准
  * @param[in][out] cali:指针指向云台数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
void Gimbal::set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
    gimbal_yaw_motor.offset_ecd = yaw_offset;
    gimbal_yaw_motor.max_encode_angle = max_yaw;
    gimbal_yaw_motor.min_encode_angle = min_yaw;

    gimbal_pitch_motor.offset_ecd = pitch_offset;
    gimbal_pitch_motor.max_encode_angle = max_pitch;
    gimbal_pitch_motor.min_encode_angle = min_pitch;
}

/*
 * @brief          手动设置云台编码器中值，最小最大机械相对角度
 * @param[in]      yaw_offse:yaw 中值
 * @param[in]      pitch_offset:pitch 中值
 * @param[in]      max_yaw:max_yaw:yaw 最大相对角度
 * @param[in]      min_yaw:yaw 最小相对角度
 * @param[in]      max_yaw:pitch 最大相对角度
 * @param[in]      min_yaw:pitch 最小相对角度
 * @retval         返回空
 * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
 */
void Gimbal::set_hand_operator_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
    gimbal_yaw_motor.offset_ecd = yaw_offset;
    gimbal_yaw_motor.max_encode_angle = max_yaw;
    gimbal_yaw_motor.min_encode_angle = min_yaw;

    gimbal_pitch_motor.offset_ecd = pitch_offset;
    gimbal_pitch_motor.max_encode_angle = max_pitch;
    gimbal_pitch_motor.min_encode_angle = min_pitch;
}

/**
 * @brief          云台校准计算，将校准记录的中值,最大 最小值返回
 * @param[out]     yaw 中 指针
 * @param[out]     pitch 中值 指针
 * @param[out]     yaw 最大相对角度 指针
 * @param[out]     yaw 最小相对角度 指针
 * @param[out]     pitch 最大相对角度 指针
 * @param[out]     pitch 最小相对角度 指针
 * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
 * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
 */
bool_t Gimbal::cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_cali.step == 0)
    {
        gimbal_cali.step = GIMBAL_CALI_START_STEP;
        // 保存进入时候的数据，作为起始数据，来判断最大，最小值
        gimbal_cali.max_pitch = gimbal_pitch_motor.gyro_angle;
        gimbal_cali.max_pitch_ecd = gimbal_pitch_motor.motor_measure->ecd;
        gimbal_cali.max_yaw = gimbal_yaw_motor.gyro_angle;
        gimbal_cali.max_yaw_ecd = gimbal_yaw_motor.motor_measure->ecd;
        gimbal_cali.min_pitch = gimbal_pitch_motor.gyro_angle;
        gimbal_cali.min_pitch_ecd = gimbal_pitch_motor.motor_measure->ecd;
        gimbal_cali.min_yaw = gimbal_yaw_motor.gyro_angle;
        gimbal_cali.min_yaw_ecd = gimbal_yaw_motor.motor_measure->ecd;
        return 0;
    }
    else if (gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        calc_gimbal_cali(&gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
        (*max_yaw) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_yaw) += GIMBAL_CALI_REDUNDANT_ANGLE;
        (*max_pitch) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_pitch) += GIMBAL_CALI_REDUNDANT_ANGLE;
        gimbal_yaw_motor.offset_ecd = *yaw_offset;
        gimbal_yaw_motor.max_encode_angle = *max_yaw;
        gimbal_yaw_motor.min_encode_angle = *min_yaw;
        gimbal_pitch_motor.offset_ecd = *pitch_offset;
        gimbal_pitch_motor.max_encode_angle = *max_pitch;
        gimbal_pitch_motor.min_encode_angle = *min_pitch;
        gimbal_cali.step = 0;
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief          云台在某些行为下，需要射击停止
 * @param[in]      none
 * @retval         1: no move 0:normal
 */

bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal.gimbal_mode == GIMBAL_TO_MID || gimbal.gimbal_mode == GIMBAL_CALI || gimbal.gimbal_mode == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
