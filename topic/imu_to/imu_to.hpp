/**
 * @file imu_to.hpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-06-02
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <zephyr/zbus/zbus.h>

namespace topic::imu_to {

struct Message
{
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float gyro[3]       = {0.0f, 0.0f, 0.0f};

    float temperature = 0.0f;
    float roll        = 0.0f;
    float pitch       = 0.0f;
    float yaw         = 0.0f;
    float yaw_total   = 0.0f;
};

} // namespace topic::imu_to

ZBUS_CHAN_DECLARE(pub_imu_to);
ZBUS_OBS_DECLARE(sub_imu_to);
