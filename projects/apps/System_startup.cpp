/**
 * @file System_startup.cpp
 * @author qingyu
 * @brief
 * @version 0.1
 * @date 2026-04-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "System_startup.h"
#include "trd_gpio.hpp"
#ifdef CONFIG_TRD_CHASSIS
#include "trd_chassis.hpp"
#endif
#ifdef CONFIG_TRD_GIMBAL
#include "trd_gimbal.hpp"
#endif
#ifdef CONFIG_TRD_CAN_TX
#include "trd_can_tx.hpp"
#endif
#ifdef CONFIG_TRD_REMOTE
#include "remote.hpp"
#endif
#ifdef CONFIG_TRD_TFLM
#include "trd_tflm.hpp"
#endif

using namespace thread;

void System_Bsp_Init()
{
#ifdef CONFIG_TRD_CAN_TX
    can    ::thread_init();
#endif
}


void System_Modules_Init()
{
    output ::thread_init();
#ifdef CONFIG_TRD_CHASSIS
    chassis::thread_init();
#endif
#ifdef CONFIG_TRD_GIMBAL
    gimbal ::thread_init();
#endif
#ifdef CONFIG_TRD_REMOTE
    remote ::thread_init();
#endif
#ifdef CONFIG_TRD_TFLM
    ml     ::thread_init();
#endif
}

void System_Thread_Start()
{
#ifdef CONFIG_TRD_REMOTE
    remote  ::thread_start(4);
#endif
#ifdef CONFIG_TRD_CAN_TX
    can     ::thread_start(4);
#endif
#ifdef CONFIG_TRD_CHASSIS
    chassis ::thread_start(5);
#endif
#ifdef CONFIG_TRD_GIMBAL
    gimbal  ::thread_start(5);
#endif
    output  ::thread_start(6);
#ifdef CONFIG_TRD_TFLM
    // ml      ::thread_start(6);
#endif
}
