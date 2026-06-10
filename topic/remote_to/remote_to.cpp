/**
 * @file remote_to.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-06-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <zephyr/zbus/zbus.h>
#include "remote_to.hpp"

ZBUS_CHAN_DEFINE(pub_remote_to,
                 topic::remote_to::Message,
                 NULL,
                 NULL,
                 ZBUS_OBSERVERS(sub_remote_to),
                 ZBUS_MSG_INIT({}));

ZBUS_SUBSCRIBER_DEFINE(sub_remote_to, 10);
