#pragma once
#include <iostream>
#include <chrono>
#include "utils/utils.hpp"

namespace hero {
  class spinArmor {
  private:
    std::string config_path_;
    std::map<std::string, float> debug_info_;
    std::chrono::_V2::system_clock::time_point start_point_;
    int interval_armor_time_ = 1400; // 最佳装甲板出现的间隔时间，即 1/3 圈时间（单位 ms）
    float auto_shoot_range_ {20}; // 自动击打的范围，单位 ms
    float shoot_delay_ {50}; // 电控延迟，单位 ms
    int best_armor_tp; // 上一次法向量装甲板出现时间点
    int frame_now_ {0}; // 图像的时间戳
    float best_armor_pitch_range_ {1.f}; // 选择前哨站装甲板 pitch 范围，单位角度
    float best_armor_selected_yaw_range_ {0.3f}; // 最佳装甲板的选择范围
    float best_armor_depth_ {0.f}; // 最佳装甲板的深度
    float best_armor_imu_pitch_ {0.f}; // 最佳装甲板对应的 pitch

  public:
      spinArmor(const std::string &_config_path); //初始化参数
      void readConfig();
      bool run(RoboInf &_robo_inf);
      void add(const float &_outpost_armor_yaw, const float _best_armor_depth,
        const float _target_imu_pitch);
      std::map<std::string, float> returnDebugInfo();
      float getTargetImuPitch();
      ~spinArmor();
  };
  
  inline spinArmor::spinArmor(const std::string &_config_path) {
    start_point_ = std::chrono::system_clock::now();
    config_path_ = _config_path;
    readConfig();
  }

  inline void spinArmor::readConfig() {
    cv::FileStorage fs;
    fs.open(config_path_, cv::FileStorage::READ);
    interval_armor_time_ = static_cast<int>(fs["interval_armor_time"]);
    auto_shoot_range_ = static_cast<float>(fs["auto_shoot_range"]);
    shoot_delay_ = static_cast<float>(fs["shoot_delay"]);
    best_armor_pitch_range_ = static_cast<float>(fs["best_armor_pitch_range"]);
    best_armor_selected_yaw_range_ = static_cast<float>(fs["best_armor_selected_yaw_range"]);
    fs.release();
  }

/**
 * @brief 检测前对最佳装甲板出现时刻进行更新
 * 
 * @param _robo_inf 
 * @return true 
 * @return false 
 */
  inline bool spinArmor::run(RoboInf &_robo_inf) {
    float down_t = best_armor_depth_ / _robo_inf.bullet_velocity.load() * 1000;
    frame_now_ = std::chrono::duration_cast<std::chrono::milliseconds>
      (std::chrono::system_clock::now() - start_point_).count();
    int next_best_armor_dur = interval_armor_time_ -
      (frame_now_ - best_armor_tp) - shoot_delay_ - down_t; // 距离下一次最佳装甲板出现时刻
    // 退出模式后一段时间再进入的情况
    if(next_best_armor_dur < -3000) {
      best_armor_imu_pitch_ = 0.f;
    }

    debug_info_["down_t"] = down_t;
    debug_info_["next_best_armor_dur"] = static_cast<float>(next_best_armor_dur);

    if(next_best_armor_dur < 10 && next_best_armor_dur > -10) {
      debug_info_["spin_armor_shot"] = 1;
      return true;
    } else {
      debug_info_["spin_armor_shot"] = 0;
      return false;
    }

  }

/**
 * @brief 检测到装甲板时进行更新
 * 
 * @param _outpost_armor_yaw 
 * @param _best_armor_depth 
 */
  inline void spinArmor::add(const float &_outpost_armor_yaw,
      const float _best_armor_depth, const float _best_armor_imu_pitch) {
    if(_outpost_armor_yaw < best_armor_selected_yaw_range_ &&
       _outpost_armor_yaw > -best_armor_selected_yaw_range_) {
      // 如果本次与上一次更新时间差大于最佳装甲板间隔的一半则更新时间点
      if(frame_now_ - best_armor_tp > interval_armor_time_ / 2) {
        best_armor_tp = frame_now_;
        best_armor_depth_ = _best_armor_depth;
        best_armor_imu_pitch_ = _best_armor_imu_pitch;

        debug_info_["best_armor_imu_pitch_"] = best_armor_imu_pitch_;
        debug_info_["best_armor"] = 1.f;
      }
    } else {
      debug_info_["best_armor"] = 0.f;
    }
    debug_info_["outpost_armor_yaw"] = _outpost_armor_yaw;
  }

  inline std::map<std::string, float> spinArmor::returnDebugInfo() {
    return debug_info_;
  }

  inline float spinArmor::getTargetImuPitch() {
    if(frame_now_ - best_armor_tp > interval_armor_time_ * 2) {
      return 0.f;
    } else {
      return best_armor_imu_pitch_;
    }
  }
  
  inline spinArmor::~spinArmor() {
  }
  
}