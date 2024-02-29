#ifndef BSP_RC_H
#define BSP_RC_H
#include "struct_typedef.h"



#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{
#include "struct_typedef.h"

    extern void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
    extern void RC_unable(void);
    extern void RC_restart(uint16_t dma_buf_num);
}
#endif
#endif
