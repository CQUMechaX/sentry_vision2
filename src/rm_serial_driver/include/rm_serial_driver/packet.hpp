// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>
#include <iostream>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0x5A; 
  uint8_t detect_color : 1;  // 0-red 1-blue 发1        
  float leftyaw;                   
  float leftpitch;                            
  float rightyaw;               
  float rightpitch;                 
  uint16_t checksum = 0;     // crc16校验位 https://blog.csdn.net/ydyuse/article/details/105395368
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
  bool is_lefttracking;
  float leftyaw;
  float leftpitch;
  bool is_righttracking;
  float rightyaw;
  float rightpitch;
  uint8_t naving;
  float nav_vx;
  float nav_vy;
  float leftdistance;
  float rightdistance;
  uint16_t checksum = 0;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet; 
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
