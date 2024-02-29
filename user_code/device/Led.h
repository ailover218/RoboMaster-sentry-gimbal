#ifndef LED_H
#define LED_H

#include "struct_typedef.h"

#define RGB_FLOW_COLOR_CHANGE_TIME 1000
#define RGB_FLOW_COLOR_LENGHT 6


class Led
{
public:
    uint16_t i, j;
    fp32 delta_alpha, delta_red, delta_green, delta_blue;
    fp32 alpha, red, green, blue;
    uint32_t aRGB;

    uint32_t RGB_flow_color[RGB_FLOW_COLOR_LENGHT + 1];
    

    void init();

    void run();

    void breath_led();
};


#endif
