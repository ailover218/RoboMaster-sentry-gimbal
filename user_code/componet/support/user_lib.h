//
// Created by WSJ on 2021/11/2.
//

#ifndef CLASSIS_BOARD_USER_LIB_H
#define CLASSIS_BOARD_USER_LIB_H

#include "struct_typedef.h"
#include "arm_math.h"

#define PI 3.14159265358979f
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
#endif //CLASSIS_BOARD_USER_LIB_H
