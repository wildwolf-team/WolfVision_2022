#include "serial.hpp"

Ser::Serial::Serial(std::string port, unsigned long baud) {
  auto timeout = serial::Timeout::simpleTimeout(serial::Timeout::max());
  this->setPort(port);
  this->setBaudrate(baud);
  this->setTimeout(timeout);
  try {
    this->open();
    fmt::print("[{}] Serial init successed.\n", idntifier_green);
  } catch (const std::exception& e) {
    fmt::print("[{}] Serial init failed, {}.\n", idntifier_red, e.what());
  }
}

void Ser::Serial::ReceiveInfo(RoboInf& _robo_inf) try {
  Recive  inf;
  uint8_t head_flag;
  this->read(&head_flag, sizeof(head_flag));
  while (head_flag != 'S') this->read(&head_flag, sizeof(head_flag));
  uint8_t id;
  this->read(&id, sizeof(id));
  if (static_cast<int>(id) < 100) {
    _robo_inf.robot_color.store(Color::RED);
  } else {
    _robo_inf.robot_color.store(Color::BLUE);
  }
  // hero spin armor offset
  if (static_cast<int>(id) == 1 || static_cast<int>(id) == 101) {
    int8_t spin_armor_offset;
    this->read((uint8_t*)&spin_armor_offset, sizeof(spin_armor_offset));
    _robo_inf.spin_armor_offset.store(spin_armor_offset);
  }
  this->read((uint8_t*)&(inf.mode), sizeof(inf) - 1);
  if (inf.end == 'E') {
    _robo_inf.yaw_angle.store(inf.yaw_angle);
    _robo_inf.pitch_angle.store(inf.pitch_angle);
    _robo_inf.model.store(int(inf.mode));
    if (_robo_inf.bullet_velocity < 10 || _robo_inf.bullet_velocity > 30) {
      _robo_inf.bullet_velocity.store(15);
    } else {
      _robo_inf.bullet_velocity.store(static_cast<int>(inf.bullet_velocity) - 2);
    }
  }
} catch (const std::exception& e) {
  fmt::print("[{}] Serial read failed, {}.\n", idntifier_red, e.what());
}

void Ser::Serial::WriteInfo(RoboCmd& _robo_cmd) try {
  Command cmd;
  cmd.yaw_angle   = _robo_cmd.yaw_angle.load();
  cmd.pitch_angle = _robo_cmd.pitch_angle.load();
  cmd.data_type   = _robo_cmd.data_type.load();
  cmd.auto_shoot  = _robo_cmd.auto_shoot.load();
  cmd.crc         = checksumCRC((uint8_t*)&(cmd.data_type), sizeof(cmd) - 3);
  if (this->isOpen()) {
    this->write((uint8_t*)&cmd, sizeof(cmd));
  } else {
    this->open();
  }
} catch (const std::exception& e) {
  this->close();
  fmt::print("[{}] Serial write failed, {}.\n", idntifier_red, e.what());
}