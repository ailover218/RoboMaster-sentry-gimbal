//
// Created by WSJ on 2021/11/2.
//

#ifndef CLASSIS_BOARD_USER_LIB_H
#define CLASSIS_BOARD_USER_LIB_H

#include "struct_typedef.h"
#include "arm_math.h"
#include "struct_typedef.h"

#define PI 3.14159265358979f

typedef __packed struct
{
    fp32 input;        //输入数据
    fp32 out;          //输出数据
    fp32 min_value;    //限幅最小值
    fp32 max_value;    //限幅最大值
    fp32 frame_period; //时间间隔
} ramp_function_source_t;


//循环限幅函数
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
//弧度格式化为-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

//限幅函数
extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
//最大值限幅函数
extern void abs_limit(fp32 Value, fp32 MaxValue);

extern fp32 abs_fp32(fp32 Value);

extern int16_t abs_int16(int16_t Value);

void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);
#endif //CLASSIS_BOARD_USER_LIB_H
