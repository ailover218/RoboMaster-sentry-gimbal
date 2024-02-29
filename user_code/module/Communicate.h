#ifndef COMMUNICAT_H
#define COMMUNICAT_H

#include "cmsis_os.h"
#include "main.h"

#include "Remote_control.h"
#include "Can_receive.h"


class Communicate
{
public:
    void init();

    void run();

    bool game_start();
};

extern Remote_control remote_control;

extern Can_receive can_receive;

extern Communicate communicate;

#endif

