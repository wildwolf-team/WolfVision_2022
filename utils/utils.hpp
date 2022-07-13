#pragma once
#include <atomic>

struct RoboCmd {
  std::atomic<float> yaw_angle;
  std::atomic<float> pitch_angle;
  std::atomic<uint16_t> depth;
  std::atomic<uint8_t> data_type;
  std::atomic<uint8_t> auto_shoot;
};

struct RoboCmdUartBuff{
  uint8_t  start       = (unsigned)'S';
  uint8_t  data_type   = 0;
  uint8_t  auto_shoot  = 0;
  float    yaw_angle   = 0.f;
  float    pitch_angle = 0.f;
  // uint16_t depth       = 0;
  uint8_t  crc         = 0;
  uint8_t  end         = (unsigned)'E';
} __attribute__((packed));

struct RoboInf {
  std::atomic<float>   yaw_angle;
  std::atomic<float>   pitch_angle;
  std::atomic<int> bullet_velocity;
  std::atomic<int> robot_color;
  std::atomic<int> model;
  // std::atomic<uint16_t> crc;
};

struct RoboInfUartBuff {
  uint8_t  robot_id;
  uint8_t  model;
  float    yaw_angle;
  float    pitch_angle;
  uint8_t  bullet_velocity;
  uint8_t  end;
} __attribute__((packed));

// The color of our teamc
enum Color {
  ALL,
  RED,
  BLUE,
};
// Description of operation mode information
enum Mode {
  DEFAULT_MODE,
  // Self-Scanning Mode
  SUP_SHOOT,
  ENERGY_AGENCY,
  // Tradition_vision
  TRADITION_MODE = 4,
  //Top_Mode
  TOP_MODE = 5,
  // Forecast Mode
  FORECAST_MODE = 6,
  // Hit Spin Armor Mode
  SPINARMOR_MODE
};