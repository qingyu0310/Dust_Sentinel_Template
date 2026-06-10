/**
 * @file imu_to.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-06-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "imu_to.hpp"

ZBUS_CHAN_DEFINE(pub_imu_to,
                 topic::imu_to::Message,
                 NULL,
                 NULL,
                 ZBUS_OBSERVERS(sub_imu_to),
                 ZBUS_MSG_INIT({}));

ZBUS_SUBSCRIBER_DEFINE(sub_imu_to, 10);
