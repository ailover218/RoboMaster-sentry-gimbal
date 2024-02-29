#include "Led.h"


#ifdef __cplusplus
extern "C"
{
#endif

#include "bsp_led.h"

#ifdef __cplusplus
}
#endif

#include "cmsis_os.h"
#include "main.h"


void Led::init()
{
    RGB_flow_color[0] = 0xFF0000FF;
    RGB_flow_color[1] = 0x0000FF00;
    RGB_flow_color[2] = 0xFFFF0000;
    RGB_flow_color[3] = 0x000000FF;
    RGB_flow_color[4] = 0xFF00FF00;
    RGB_flow_color[5] = 0x00FF0000;
    RGB_flow_color[6] = 0xFF0000FF;
}

void Led::run()
{
    breath_led();
}

//呼吸灯
void Led::breath_led()
{
    for (i = 0; i < RGB_FLOW_COLOR_LENGHT; i++)
    {
        alpha = (RGB_flow_color[i] & 0xFF000000) >> 24;
        red = ((RGB_flow_color[i] & 0x00FF0000) >> 16);
        green = ((RGB_flow_color[i] & 0x0000FF00) >> 8);
        blue = ((RGB_flow_color[i] & 0x000000FF) >> 0);

        delta_alpha = (fp32)((RGB_flow_color[i + 1] & 0xFF000000) >> 24) - (fp32)((RGB_flow_color[i] & 0xFF000000) >> 24);
        delta_red = (fp32)((RGB_flow_color[i + 1] & 0x00FF0000) >> 16) - (fp32)((RGB_flow_color[i] & 0x00FF0000) >> 16);
        delta_green = (fp32)((RGB_flow_color[i + 1] & 0x0000FF00) >> 8) - (fp32)((RGB_flow_color[i] & 0x0000FF00) >> 8);
        delta_blue = (fp32)((RGB_flow_color[i + 1] & 0x000000FF) >> 0) - (fp32)((RGB_flow_color[i] & 0x000000FF) >> 0);

        delta_alpha /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;
        for (j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; j++)
        {
            alpha += delta_alpha;
            red += delta_red;
            green += delta_green;
            blue += delta_blue;

            aRGB = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 | ((uint32_t)(green)) << 8 | ((uint32_t)(blue)) << 0;
            aRGB_led_show(aRGB);
            osDelay(1);
        }
    }
}
