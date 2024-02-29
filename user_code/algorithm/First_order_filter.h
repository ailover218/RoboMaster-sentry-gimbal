//
// Created by WSJ on 2021/11/2.
//

#ifndef FIRST_ORDER_FILTER_H
#define FIRST_ORDER_FILTER_H


#include "struct_typedef.h"
class First_order_filter
{
public:
    fp32 input;        //输入数据
    fp32 out;          //滤波输出的数据
    fp32 num[1];       //滤波参数
    fp32 frame_period; //滤波的时间间隔 单位 s
    void init(fp32 frame_period, const fp32 num[1]);
    void first_order_filter_cali(fp32 input);
};


#endif //CLASSIS_BOARD_FIRST_ORDER_FILTER_H
