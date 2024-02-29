#ifndef PID_H
#define PID_H

#ifdef __cplusplus
#include "struct_typedef.h"
#include "vision.h"
typedef enum
{
    PID_SPEED = 0, //速度环
    PID_ANGLE,     //角度环

} pid_mode_e;

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_iout; //最大积分输出
    fp32 max_out;  //最大输出


    fp32 *set;
    fp32 *ref;
    fp32 error;
    fp32 last_error;

    fp32 error_delta; //微分项,速度环下为error之间的差值,角度环下为陀螺仪角速度值

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
} pid_data_t;



class Pid {
public:
    uint8_t mode;
    pid_data_t data;

    void init(pid_mode_e mode_, const fp32 *pid_parm, fp32 *ref_, fp32 *set_, fp32 erro_delta_);

    fp32 pid_calc(); 

    void pid_clear();

    void Clear();

    fp32 cycleTime[3];

	fp32 getCycleT();
};

#endif
#endif //CLASSIS_BOARD_PID_H
