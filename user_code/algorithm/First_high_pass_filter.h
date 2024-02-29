//
// Created by WSJ on 2021/11/2.
//

#ifndef FIRST_HIGH_PASS_FILTER_H
#define FIRST_HIGH_PASS_FILTER_H

#include "struct_typedef.h"
class First_high_pass_filter
{
public:
    fp32 input;        //输入数据
    fp32 out;          //滤波输出的数据
    fp32 num[1];       //滤波参数
    fp32 frame_period; //滤波的时间间隔 单位 s
    void init(fp32 frame_period, const fp32 num[1]);
    void first_high_pass_filter_cali(fp32 input);
};

#endif 
