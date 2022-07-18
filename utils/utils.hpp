#pragma once
#include <atomic>

struct RoboCmd {
  std::atomic<float> yaw_angle;
  std::atomic<float> pitch_angle;
  std::atomic<uint16_t> depth;
  std::atomic<uint8_t> data_type;
  std::atomic<uint8_t> auto_shoot;
};

struct RoboInf {
  std::atomic<float>   yaw_angle;
  std::atomic<float>   pitch_angle;
  std::atomic<int> bullet_velocity;
  std::atomic<int> robot_color;
  std::atomic<int> model;
  std::atomic<uint8_t> spin_armor_offset; // hero spin armor offset
};

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