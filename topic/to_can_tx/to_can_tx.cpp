/**
 * @file to_can_tx.cpp
 * @author qingyu
 * @brief
 * @version 0.1
 * @date 2026-05-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <zephyr/kernel.h>
#include "to_can_tx.hpp"

K_MSGQ_DEFINE(user_can1_msgq, sizeof(topic::to_can_tx::Message), 16, 4);  // 深度 16，4 字节对齐，满时 K_NO_WAIT 直接丢帧


