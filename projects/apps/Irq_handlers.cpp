/**
 * @file Irq_handlers.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-05-12
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <zephyr/drivers/can.h>

#ifdef CONFIG_TRD_CHASSIS
#include "trd_chassis.hpp"
#endif
#ifdef CONFIG_TRD_GIMBAL
#include "trd_gimbal.hpp"
#endif

void user_can1_rx_callback(struct can_frame &frame, void *)
{
    switch (frame.id)
    {
    #ifdef CONFIG_TRD_CHASSIS
        using namespace instance::chassis;

        case kSteerCanId[0]:
            chassis_wheel[0].steer_motor.CanCpltRxCallback(frame.data);
            break;

        case kDriveCanId[0]:
            chassis_wheel[0].drive_motor.CanCpltRxCallback(frame.data);
            break;
            
        case kSteerCanId[1]:
            chassis_wheel[1].steer_motor.CanCpltRxCallback(frame.data);
            break;

        case kDriveCanId[1]:
            chassis_wheel[1].drive_motor.CanCpltRxCallback(frame.data);
            break;
            
        #if USE_POWERMETER
        case KSteerPwrMeterId:
            SteerPwrMeter.CanCpltRxCallback(frame.data);
            break;
        case KDrivePwrMeterId:
            DrivePwrMeter.CanCpltRxCallback(frame.data);
            break;
        #endif
    #endif

    #ifdef CONFIG_TRD_GIMBAL
        using namespace instance::gimbal;

        case kBYawMasterId:
            big_yaw_.motor.CanCpltRxCallback(frame.data);
            break;
            
    #endif
            
        default:
            break;
    }
}
