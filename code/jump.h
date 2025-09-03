#ifndef __JUMP_H__
#define __JUMP_H__

#include "zf_common_headfile.h"

typedef void (*HandlerFunc)(int value);

typedef struct
{
    int16 min;
    int16 max;
    HandlerFunc handler;
    const char *description;
}jump_control_struct;

//extern int barrier_flag;

void dynamic_jump_control(void);

#endif
