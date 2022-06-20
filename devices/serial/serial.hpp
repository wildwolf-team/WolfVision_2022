#pragma once
#include "serial/serial.h"
#include "utils/utils.hpp"
#include <iostream>
auto idntifier_green = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "serial");
auto idntifier_red   = fmt::format(fg(fmt::color::red)   | fmt::emphasis::bold, "serial");

class RoboSerial : public serial::Serial {
 public:
  RoboSerial(std::string port, unsigned long baud) {
    auto timeout = serial::Timeout::simpleTimeout(serial::Timeout::max());
    this->setPort(port);
    this->setBaudrate(baud);
    this->setTimeout(timeout);
    try {
      this->open();
      fmt::print("[{}] Serial init successed.\n", idntifier_green);
    } catch(const std::exception& e) {
      fmt::print("[{}] Serial init failed, {}.\n", idntifier_red, e.what());
    }
  }

  void WriteInfo(RoboCmd &robo_cmd) {
    RoboCmdUartBuff robo_cmd_uart_temp;
    robo_cmd_uart_temp.yaw_angle   = robo_cmd.yaw_angle.load();
    robo_cmd_uart_temp.pitch_angle = robo_cmd.pitch_angle.load();
    robo_cmd_uart_temp.depth       = robo_cmd.depth.load();
    robo_cmd_uart_temp.data_type   = robo_cmd.data_type.load();
    robo_cmd_uart_temp.auto_shoot  = robo_cmd.auto_shoot.load();
    robo_cmd_uart_temp.crc = checksumCRC((uint8_t*)&(robo_cmd_uart_temp) + 1, sizeof(robo_cmd_uart_temp) - 3);
    this->write((uint8_t*)&robo_cmd_uart_temp, sizeof(robo_cmd_uart_temp));
  }

  void ReceiveInfo(RoboInf &robo_inf) {
    RoboInfUartBuff robo_inf_uart_temp;
    uint8_t temp;
    this->read(&temp, 1);
    while (temp != 'S')
      this->read(&temp, 1);
    this->read((uint8_t*)&robo_inf_uart_temp, sizeof(robo_inf_uart_temp));
    if (robo_inf_uart_temp.end == 'E') {
      robo_inf.yaw_angle.store(robo_inf_uart_temp.yaw_angle);
      robo_inf.pitch_angle.store(robo_inf_uart_temp.pitch_angle);
      robo_inf.model.store(int(robo_inf_uart_temp.model));
      if (int(robo_inf_uart_temp.robot_id) < 100) {
        robo_inf.robot_color.store(Color::RED);
      } else {
        robo_inf.robot_color.store(Color::BLUE);
      }
      robo_inf.bullet_velocity.store(int(robo_inf_uart_temp.bullet_velocity)-1);
      if (robo_inf.bullet_velocity < 10 || robo_inf.bullet_velocity > 30) {
        robo_inf.bullet_velocity.store(15);
      }
    }
  }

 private:
  static constexpr unsigned char CRC8_Table[] = {
    0,   94,  188, 226, 97,  63,  221, 131, 194, 156, 126, 32,  163, 253, 31, 65,  157, 195, 33,  127, 252, 162, 64,  30,  95,  1,   227, 189, 62,  96,  130, 220, 35,  125, 159, 193, 66,
    28,  254, 160, 225, 191, 93,  3,   128, 222, 60,  98,  190, 224, 2,   92, 223, 129, 99,  61,  124, 34,  192, 158, 29,  67,  161, 255, 70,  24,  250, 164, 39,  121, 155, 197, 132, 218,
    56,  102, 229, 187, 89,  7,   219, 133, 103, 57,  186, 228, 6,   88,  25, 71,  165, 251, 120, 38,  196, 154, 101, 59,  217, 135, 4,   90,  184, 230, 167, 249, 27,  69,  198, 152, 122,
    36,  248, 166, 68,  26,  153, 199, 37,  123, 58,  100, 134, 216, 91,  5,  231, 185, 140, 210, 48,  110, 237, 179, 81,  15,  78,  16,  242, 172, 47,  113, 147, 205, 17,  79,  173, 243,
    112, 46,  204, 146, 211, 141, 111, 49,  178, 236, 14,  80,  175, 241, 19, 77,  206, 144, 114, 44,  109, 51,  209, 143, 12,  82,  176, 238, 50,  108, 142, 208, 83,  13,  239, 177, 240,
    174, 76,  18,  145, 207, 45,  115, 202, 148, 118, 40,  171, 245, 23,  73, 8,   86,  180, 234, 105, 55,  213, 139, 87,  9,   235, 181, 54,  104, 138, 212, 149, 203, 41,  119, 244, 170,
    72,  22,  233, 183, 85,  11,  136, 214, 52,  106, 43,  117, 151, 201, 74, 20,  246, 168, 116, 42,  200, 150, 21,  75,  169, 247, 182, 232, 10,  84,  215, 137, 107, 53};

  inline uint8_t checksumCRC(unsigned char* buf, uint16_t len) {
    uint8_t check = 0;
    while (len--) {
      check = CRC8_Table[check ^ (*buf++)];
    }
    return check;
  }
};