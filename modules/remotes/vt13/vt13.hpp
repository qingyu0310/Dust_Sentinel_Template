/**
 * @file vt13.hpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-05-12
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cstdint>
#include "remote_to.hpp"

namespace vt13 {

static constexpr uint16_t kFrameSizeVT13 = 21;
bool validate(const uint8_t* buffer, uint8_t len);
bool decode(const uint8_t* buffer, uint8_t len, topic::remote_to::Message& pub);
bool dataprocess(const uint8_t* buffer, uint8_t len, topic::remote_to::Message& pub);

}
