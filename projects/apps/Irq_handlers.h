/**
 * @file Irq_handlers.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-05-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */

 
#include <zephyr/drivers/can.h>

void user_can1_rx_callback(struct can_frame &frame, void *);
void user_can2_rx_callback(struct can_frame &frame, void *);
void user_can3_rx_callback(struct can_frame &frame, void *);















