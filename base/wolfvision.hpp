/**
 * @file wolfvision.hpp
 * @author XX (2796393320@qq.com)
 *         WCJ (1767851382@qq.com)
 *         SMS (2436210442@qq.com)
 *         SHL (2694359979@qq.com)
 * @brief 主函数
 * @date 2021-08-28
 *
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 *
 */
#pragma once

#include <fmt/color.h>
#include <fmt/core.h>

#include <atomic>
#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "devices/camera/mv_video_capture.hpp"
#include "devices/serial/serial.hpp"
#include "module/armor/basic_armor.hpp"
#include "module/buff/basic_buff.hpp"
#include "module/net/basic_net.h"
#include "unistd.h"
#include "utils/angle_solve/basic_pnp.hpp"
#include "utils/fps.hpp"
#include "utils/mjpeg_streamer.hpp"
#include "utils/reset_mv_camera.hpp"
#include "utils/utils.hpp"
#include "utils/json.hpp"
#include "utils/simple_cpp_sockets.hpp"

auto idntifier = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "wolfvision");

using MJPEGStreamer = nadjieb::MJPEGStreamer;
using namespace std::chrono_literals;
/**
 * @brief 串口发送赋值
 * 
 * @param _robo_cmd 
 * @param _yaw 
 * @param _pitch 
 * @param _depth 
 * @param _data_type 
 * @param _auto_shoot 
 */
void updataWriteData(RoboCmd& _robo_cmd, const float _yaw, const float _pitch, const int _depth, const int _data_type, const int _auto_shoot) {
  _robo_cmd.yaw_angle.store(-_yaw);
  _robo_cmd.pitch_angle.store(_pitch);
  _robo_cmd.depth.store(_depth);
  _robo_cmd.data_type.store(_data_type > 1 ? 1 : _data_type);
  _robo_cmd.auto_shoot.store(_auto_shoot);
}
/**
 * @brief 网页端发送图像
 * 
 * @param _streamer_ptr 
 * @param _src_img 
 * @param _params 
 */
void webImage(const std::shared_ptr<nadjieb::MJPEGStreamer>& _streamer_ptr, const cv::Mat _src_img, const std::vector<int> _params) {
  if (!_src_img.empty()) {
    std::vector<uchar> buff_bgr;
    cv::Mat _dst_img;
    cv::resize(_src_img, _dst_img, cv::Size(640, 384));
    cv::imencode(".jpg", _dst_img, buff_bgr, _params);
    _streamer_ptr->publish("/pc", std::string(buff_bgr.begin(), buff_bgr.end()));
  }
}
/**
 * @brief 主线程
 * 
 * @param _robo_cmd 
 * @param _robo_inf 
 * @param _streamer_ptr 
 */
void PTZCameraThread(RoboCmd& _robo_cmd, RoboInf& _robo_inf, const std::shared_ptr<nadjieb::MJPEGStreamer>& _streamer_ptr) {
  std::vector<int> params       = {cv::IMWRITE_JPEG_QUALITY, 100};
  mindvision::VideoCapture* mv_capture_  = new mindvision::VideoCapture(mindvision::CameraParam(0, mindvision::RESOLUTION_960_X_600, mindvision::EXPOSURE_15000));
  auto    pnp_         = std::make_shared<basic_pnp::PnP>(
                              fmt::format("{}{}", CONFIG_FILE_PATH, "/camera/mv_camera_config_4912.xml"),
                              fmt::format("{}{}", CONFIG_FILE_PATH, "/angle_solve/basic_pnp_config.xml"));
  auto    start        = std::chrono::system_clock::now();
  cv::Mat src_img_;
  fps::FPS global_fps_;
  basic_net::armor_detection armor;
  armor.rst.reserve(128);
  auto basic_buff_ = std::make_shared<basic_buff::Detector>(fmt::format("{}{}", CONFIG_FILE_PATH, "/buff/basic_buff_config.xml"));
  auto detector = std::make_shared<basic_net::Detector>();
  detector->detection_init(fmt::format("{}{}", CONFIG_FILE_PATH, "/net/opt4_FP16.xml"), "GPU");
  RoboInf inf;
  auto              t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::stringstream ss;
  ss << std::put_time(std::localtime(&t), "/%Y_%m_%d_%H_%M_%S");
  
  std::string     s_v = CONFIG_FILE_PATH + ss.str() + ".avi";
  cv::Mat draw_img;
  // cv::VideoWriter vw_src;
  // vw_src.open(s_v, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(960, 600), true);  // 记得打开
  int buff_num = 0;
  int other_num = 0;
  nlohmann::json config_json;
  std::ifstream config_is(fmt::format("{}{}", CONFIG_FILE_PATH,
                  "/robo_config.json"));
  config_is >> config_json;
  nlohmann::json debug_info_;
  std::unique_ptr<UDPClient> pj_udp_cl_ = std::make_unique<UDPClient>(
      config_json["pj_udp_cl_port"].get<int>(),
      config_json["pj_udp_cl_ip"].get<std::string>());

  while (true) try {
      bool is_shoot = false;
      global_fps_.getTick();
      if (mv_capture_->isindustryimgInput()) {
        mv_capture_->cameraReleasebuff();
        src_img_ = mv_capture_->image();
        draw_img = src_img_.clone();
        auto end  = std::chrono::system_clock::now();
        float time = static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() * 0.001);
        inf.yaw_angle = _robo_inf.yaw_angle.load();
        inf.pitch_angle = _robo_inf.pitch_angle.load();
        detector->process_frame(src_img_, armor);
        armor.armor_t = time;
        static float pitch = 0.f;
        switch (_robo_inf.model) {
        case Mode::SUP_SHOOT:
          // std::cout << "Mode::SUP_SHOOT" << "\n";
          if (detector->screen_armor(_robo_inf, armor, src_img_)) {
            if (armor.rst[0].tag_id == 1 || armor.rst[0].tag_id == 0) {
              pnp_->solvePnP(_robo_inf.bullet_velocity.load(), 1, armor.rst[0].pts);
            } else {
              pnp_->solvePnP(_robo_inf.bullet_velocity.load(), 0, armor.rst[0].pts);
            }
          }
          updataWriteData(_robo_cmd, pnp_->returnYawAngle(), pnp_->returnPitchAngle(), pnp_->returnDepth(), armor.rst.size(), 0);
          break;
        case Mode::ENERGY_AGENCY:
          std::cout << " is ENERGY_AGENCY mode " << std::endl;
          cv::resize(src_img_, src_img_, cv::Size(1280, 800));
          basic_buff_->runTask(src_img_, _robo_inf, _robo_cmd, time);
          // webImage(_streamer_ptr, basic_buff_->returnBinImage(), params);
          // cv::imshow("dst_img", basic_buff_->returnDstImage());
          // cv::imshow("bin_img", basic_buff_->returnBinImage());
          break;
        case Mode::TOP_MODE:
          // std::cout << "Mode::TOP_MODE" << "\n";
          if (detector->screen_top_armor(_robo_inf, armor, src_img_)) {
            if (armor.rst[0].tag_id == 1 || armor.rst[0].tag_id == 0) {
              pnp_->solvePnP(_robo_inf.bullet_velocity.load(), 1, armor.rst[0].pts);
            } else {
              pnp_->solvePnP(_robo_inf.bullet_velocity.load(), 0, armor.rst[0].pts);
            }
            pitch = pnp_->returnPitchAngle();
            detector->forecastFlagV(armor.armor_t, inf.yaw_angle.load() - pnp_->returnYawAngle(), inf.pitch_angle.load() + pnp_->returnPitchAngle());
            // is_shoot = detector->topAutoShoot(pnp_->returnDepth(), _robo_inf.bullet_velocity.load(), armor.rst[0].pts, detector->returnArmorRotatedRect(), src_img_);
            is_shoot = detector->topAutoShoot(pnp_->returnDepth(), _robo_inf.bullet_velocity.load(), armor.rst[0].pts, detector->returnArmorRotatedRect(), src_img_);
            if (armor.rst[0].tag_id == 1 || armor.rst[0].tag_id == 0) {
              pnp_->solvePnP(_robo_inf.bullet_velocity.load(), 1, detector->returnArmorRotatedRect());
            } else {
              pnp_->solvePnP(_robo_inf.bullet_velocity.load(), 0, detector->returnArmorRotatedRect());
            }
          }
          updataWriteData(_robo_cmd, pnp_->returnYawAngle(), pitch, pnp_->returnDepth(), armor.rst.size(), is_shoot);
          break;
        case Mode::FORECAST_MODE:
          // std::cout << "Mode::FORECAST_MODE" << "\n";
          if (detector->screen_armor(_robo_inf, armor, src_img_)) {
            if (armor.rst[0].tag_id == 1 || armor.rst[0].tag_id == 0) {
              pnp_->solvePnP(_robo_inf.bullet_velocity.load(), 1, armor.rst[0].pts);
            } else {
              pnp_->solvePnP(_robo_inf.bullet_velocity.load(), 0, armor.rst[0].pts);
            }
            detector->forecastFlagV(armor.armor_t, inf.yaw_angle.load() - pnp_->returnYawAngle(), inf.pitch_angle.load() + pnp_->returnPitchAngle());
            // std::cout << "inf.yaw_angle.load() - pnp_->returnYawAngle() = " << inf.yaw_angle.load() - pnp_->returnYawAngle() << "\n";
            detector->forecast_armor(pnp_->returnDepth(), _robo_inf.bullet_velocity.load(), armor.rst[0].pts, src_img_);
            // detector->defense_tower(pnp_->returnDepth(), _robo_inf.bullet_velocity.load(), armor.rst[0].pts, src_img_);
            if (armor.rst[0].tag_id == 1 || armor.rst[0].tag_id == 0) {
              pnp_->solvePnP(_robo_inf.bullet_velocity.load(), 1, armor.rst[0].pts);
            } else {
              pnp_->solvePnP(_robo_inf.bullet_velocity.load(), 0, armor.rst[0].pts);
            }
          } else {
            detector->forecastFlagV(armor.armor_t, inf.yaw_angle.load(), inf.pitch_angle.load());
          }
          updataWriteData(_robo_cmd, pnp_->returnYawAngle(), pnp_->returnPitchAngle(), pnp_->returnDepth(), armor.rst.size(), 0);
          // cv::imshow("armor_dst", src_img_);
          break;
        default:
          // std::cout << "Mode::SUP_SHOOT" << "\n";
          if (detector->screen_armor(_robo_inf, armor, src_img_)) {
            if (armor.rst[0].tag_id == 1 || armor.rst[0].tag_id == 0) {
              pnp_->solvePnP(_robo_inf.bullet_velocity.load(), 1, armor.rst[0].pts);
            } else {
              pnp_->solvePnP(_robo_inf.bullet_velocity.load(), 0, armor.rst[0].pts);
            }
          }
          updataWriteData(_robo_cmd, pnp_->returnYawAngle(), pnp_->returnPitchAngle(), pnp_->returnDepth(), armor.rst.size(), 0);
          break;
        }
        // cv::imshow("src", src_img_);
        global_fps_.calculateFPSGlobal();
        debug_info_["imu_yaw"] = _robo_inf.yaw_angle.load();
        pj_udp_cl_->send_message(debug_info_.dump());
        debug_info_.empty();
        webImage(_streamer_ptr, src_img_, params);
        // _streamer_ptr->publish_text_value("top_yaw", _robo_inf.yaw_angle.load());
        // _streamer_ptr->publish_text_value("yaw_angle", _robo_cmd.yaw_angle.load());
        // _streamer_ptr->publish_text_value("pitch_angle", _robo_cmd.pitch_angle.load());
        // _streamer_ptr->publish_text_value("depth", _robo_cmd.depth.load());
        // _streamer_ptr->publish_text_value("time", time);
        // vw_src.write(draw_img);
        // sync();
        armor.rst.clear();
        memset(armor.quantity, 0, sizeof(armor.quantity));
        if (_robo_inf.model == Mode::ENERGY_AGENCY) {
          buff_num += 1;
          other_num = 0;
        } else {
          buff_num = 0;
          other_num += 1;
        }
        if (buff_num == 1) {
          mv_capture_->~VideoCapture();
          mv_capture_ = new mindvision::VideoCapture(mindvision::CameraParam(0, mindvision::RESOLUTION_960_X_600, mindvision::EXPOSURE_1200));
        }
        if (other_num == 1) {
          mv_capture_->~VideoCapture();
          mv_capture_ = new mindvision::VideoCapture(mindvision::CameraParam(0, mindvision::RESOLUTION_960_X_600, mindvision::EXPOSURE_15000));
        }
      } else {
        mv_capture_->~VideoCapture();
        mv_capture_ = new mindvision::VideoCapture(mindvision::CameraParam(0, mindvision::RESOLUTION_960_X_600, mindvision::EXPOSURE_15000));
      }
      cv::waitKey(1);
    } catch (const std::exception& e) {
      fmt::print("{}\n", e.what());
    }
}
/**
 * @brief 串口发送线程
 * 
 * @param serial 
 * @param robo_cmd 
 */
void uartWriteThread(const std::shared_ptr<RoboSerial>& serial, RoboCmd& robo_cmd) {
  while (true) try {
      if (serial->isOpen()) {
        serial->WriteInfo(robo_cmd);
        // fmt::print("[{}] WriteInfo success.\n", idntifier_red);
      } else {
        serial->open();
      }
      std::this_thread::sleep_for(10ms);
    } catch (const std::exception& e) {
      serial->close();
      static int serial_read_excepted_times{0};
      if (serial_read_excepted_times++ > 3) {
        std::this_thread::sleep_for(10000ms);
        fmt::print("[{}] read serial excepted to many times, sleep 10s.\n", idntifier_red);
        serial_read_excepted_times = 0;
      }
      fmt::print("[{}] serial exception: {}\n", idntifier_red, e.what());
      std::this_thread::sleep_for(1000ms);
    }
}
/**
 * @brief 串口接受线程
 * 
 * @param serial 
 * @param robo_inf 
 */
void uartReadThread(const std::shared_ptr<RoboSerial>& serial, RoboInf& robo_inf) {
  while (true) try {
      if (serial->isOpen()) {
        serial->ReceiveInfo(robo_inf);
      }
      std::this_thread::sleep_for(2ms);
    } catch (const std::exception& e) {
      static int serial_read_excepted_times{0};
      if (serial_read_excepted_times++ > 3) {
        std::this_thread::sleep_for(10000ms);
        fmt::print("[{}] read serial excepted to many times, sleep 10s.\n", idntifier_red);
        serial_read_excepted_times = 0;
      }
      fmt::print("[{}] serial exception: {}\n", idntifier_red, e.what());
      std::this_thread::sleep_for(1000ms);
    }
}

/**
 * @brief 看门狗线程
 * 
 * @param ptz_camera_thread 
 * @param uartWriteThread 
 */
void watchDogThread(std::thread& ptz_camera_thread, std::thread& uartWriteThread) {
  while (true) {
    if (ptz_camera_thread.joinable()) {
      ptz_camera_thread.detach();
    }
    if (uartWriteThread.joinable()) {
      uartWriteThread.detach();
    }
    sleep(1);
  }
}
