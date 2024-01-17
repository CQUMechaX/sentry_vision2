// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0x5A; 
  uint8_t detect_color : 1;  // 0-red 1-blue 发1
  bool reset_tracker : 1;    // 发0
  uint8_t reserved : 6;      // 发6
  float roll;                // rad 发0
  float pitch;               // rad       
  float yaw;                 // rad
  float aim_x;               // 发0.5
  float aim_y;               // 发0.5
  float aim_z;               // 发0.5
  uint16_t checksum = 0;     // crc16校验位 https://blog.csdn.net/ydyuse/article/details/105395368
} __attribute__((packed));

// struct ReceivePacket
// {
//   uint8_t header = 0x5A;
//   // int id; //判断是谁发的 目前只有1，代表导航发的
//   float  x;
//   float  y;
//   float  z;
//   float  orientation_x;
//   float  orientation_y;
//   float  orientation_z;
//   float  orientation_w;
//   uint16_t checksum = 0;
// } __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
  bool is_tracking = false;
  bool naving = false;
  float yaw;
  float pitch;
  float nav_vx;
  float nav_vy;
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
