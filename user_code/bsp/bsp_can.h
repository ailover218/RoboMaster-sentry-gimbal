#ifndef BSP_CAN_H
#define BSP_CAN_H

#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{
#include "struct_typedef.h"

extern void can_filter_init(void);
}
#endif
#endif
