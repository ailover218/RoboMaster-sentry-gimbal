//
// Created by WSJ on 2021/11/2.
//

#include "First_high_pass_filter.h"

void First_high_pass_filter::init(fp32 frame_period_, const fp32 *num_)
{
    frame_period = frame_period_;
    num[0] = num_[0];
    input = 0.0f;
    out = 0.0f;
}

void First_high_pass_filter::first_high_pass_filter_cali(fp32 input_)
{
    input = input_;
    out =
        num[0] /
            (num[0] + frame_period) * out +
        frame_period /
            (num[0] + frame_period) * input;
    out = input - out;
}
