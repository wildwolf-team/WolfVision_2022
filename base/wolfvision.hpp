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

class WolfVision
{
  private:
    void serialWrite();
    void serialRead();
    void updataWriteData(RoboCmd& _robo_cmd, const float _yaw, const float _pitch, const int _depth, const int _data_type, const int _auto_shoot);
    RoboCmd     robo_cmd_;
    RoboInf     robo_inf_;
    RoboInf     inf_;
    std::unique_ptr<RoboSerial> serial_;

  private:
    void webImage(const cv::Mat _src_img);
    void disData();
    std::unique_ptr<nadjieb::MJPEGStreamer> streamer_ptr_;
    std::unique_ptr<UDPClient> pj_udp_cl_;
    nlohmann::json debug_info_;
    nlohmann::json config_json_;
    std::vector<int> params_;

  private:
    void autoAim();
    void switchMode();
    std::unique_ptr<basic_pnp::PnP> pnp_;
    std::unique_ptr<basic_buff::Detector> buff_;
    std::unique_ptr<basic_net::Detector> net_armor_;
    std::shared_ptr<mindvision::VideoCapture> capture_;
    basic_net::armor_detection armor_;
    int buff_num_        = 0;
    int other_num_       = 0;
    int camera_exposure_ = 0;
    int buff_exposure_   = 0;
    float yaw_power_     = 0;
    bool debug_mode_     = false;
    
  private:
    cv::VideoWriter vw_src_;
    time_t t_;
    std::stringstream vw_t_ss_;
    std::string       vw_t_str_;
    bool vw_mode_ = false;

  private:
    cv::Mat src_img_;
    cv::Mat write_img_;
    bool is_shoot_ = false;

  public:
    void spin();
    WolfVision();
    ~WolfVision();
};


