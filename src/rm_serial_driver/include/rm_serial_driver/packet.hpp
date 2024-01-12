// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
/*struct ReceivePacket
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
} __attribute__((packed));*/

struct ReceivePacket
{
  uint8_t header = 0x5A;
  // int id; //判断是谁发的 目前只有1，代表导航发的
  float  x;
  float  y;
  float  z;
  float  orientation_x;
  float  orientation_y;
  float  orientation_z;
  float  orientation_w;
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
  uint8_t frame_id;        //识别码 视觉信息发0 导航信息发1
  bool tracking : 1;
  uint8_t id : 3;          // 0-outpost 6-guard 7-base
  uint8_t armors_num : 3;  // 2-balance 3-outpost 4-normal
  uint8_t reserved : 1;
  float x;
  float y;
  float z;
  float yaw;
  float vx;
  float vy;
  float vz;
  float v_yaw; 
  float r1;
  float r2;
  float dz;
  float nav_vx;             //nav前缀的是导航信息
  float nav_vy;
  float nav_yaw;
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
