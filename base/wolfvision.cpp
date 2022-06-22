#include "wolfvision.hpp"

int main() {
  auto node = std::make_unique<WolfVision>();
  node->spin();
  return 0;
}

WolfVision::WolfVision() try {
	serial_ = std::make_unique<RoboSerial>("/dev/ttyUSB0", 115200);

  streamer_ptr_ = std::make_unique<nadjieb::MJPEGStreamer>();
  streamer_ptr_->start(8080, fmt::format("{}{}", SOURCE_PATH, "/utils/streamer.html"));
  std::ifstream config_is(fmt::format("{}{}", CONFIG_FILE_PATH, "/robo_config.json"));
  config_is >> config_json_;
  pj_udp_cl_ = std::make_unique<UDPClient>(
      config_json_["pj_udp_cl_port"].get<int>(),
      config_json_["pj_udp_cl_ip"].get<std::string>());
  params_ = {cv::IMWRITE_JPEG_QUALITY, 100};

  vw_mode_ = config_json_["vw_mode"];
  debug_mode_ = config_json_["debug_mode"];

  camera_exposure_ = config_json_["camera_exposure_time"].get<int>();
  buff_exposure_   = config_json_["buff_exposure"].get<int>();
  yaw_power_       = config_json_["yaw_power"].get<float>();
  mindvision::CameraParam camera_params(0, mindvision::RESOLUTION_960_X_600, camera_exposure_);
  capture_ = std::make_shared<mindvision::VideoCapture>(camera_params);
  if(!capture_->isOpen())
    capture_->open();
  pnp_     = std::make_unique<basic_pnp::PnP>(fmt::format("{}{}", CONFIG_FILE_PATH, "/camera/mv_camera_config_4912.xml"),
                                              fmt::format("{}{}", CONFIG_FILE_PATH, "/angle_solve/basic_pnp_config.xml"));
  buff_    = std::make_unique<basic_buff::Detector>(fmt::format("{}{}", CONFIG_FILE_PATH, "/buff/basic_buff_config.xml"));
  net_armor_ = std::make_unique<basic_net::Detector>();
  net_armor_->detection_init(fmt::format("{}{}", CONFIG_FILE_PATH, "/net/opt4_FP16.xml"), "GPU");
  armor_.rst.reserve(128);
  pnp_->serYawPower(yaw_power_);
  if (vw_mode_) {
    t_ = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    vw_t_ss_ << std::put_time(std::localtime(&t_), "/video/%Y_%m_%d_%H_%M_%S");
    vw_t_str_ = CONFIG_FILE_PATH + vw_t_ss_.str() + ".avi";
    vw_src_.open(vw_t_str_, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(960, 600), true);  // 记得打开  
  }

} catch(const std::exception& e) {
  fmt::print("{}\n", e.what());
}

WolfVision::~WolfVision() {}

void WolfVision::autoAim() {
  auto start = std::chrono::system_clock::now();
  while (true) {
    is_shoot_ = false;
    if (capture_->isOpen()) {

      *capture_>>src_img_;

      auto end  = std::chrono::system_clock::now();
      float time = static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() * 0.001);
      if (debug_mode_) {
        static float last_time = 0;
        fmt::print("[{}] fps == : {} \n", idntifier, 1/(time - last_time));
        last_time = time;
      }
      armor_.armor_t = time;     
      inf_.yaw_angle = robo_inf_.yaw_angle.load();
      inf_.pitch_angle = robo_inf_.pitch_angle.load();

      if (vw_mode_) {
        write_img_ = src_img_.clone();
        vw_src_.write(write_img_);
        sync();
      }

      switch (robo_inf_.model) {
          case Mode::ENERGY_AGENCY: {
            // std::cout << " is ENERGY_AGENCY mode " << std::endl;
            cv::resize(src_img_, src_img_, cv::Size(1280, 800));
            buff_->runTask(src_img_, robo_inf_, robo_cmd_, time);
            // webImage(_streamer_ptr, buff_->returnBinImage(), params);
            break;
          }
          case Mode::TOP_MODE: {
            // std::cout << "Mode::TOP_MODE" << "\n";
            net_armor_->process_frame(src_img_, armor_);
            if (net_armor_->screen_top_armor(robo_inf_, armor_, src_img_)) {
              if (armor_.rst[0].tag_id == 1 || armor_.rst[0].tag_id == 0) {
                pnp_->solvePnP(robo_inf_.bullet_velocity.load(), 1, armor_.rst[0].pts);
              } else {
                pnp_->solvePnP(robo_inf_.bullet_velocity.load(), 0, armor_.rst[0].pts);
              }
              net_armor_->forecastFlagV(armor_.armor_t, inf_.yaw_angle.load() - pnp_->returnYawAngle(), inf_.pitch_angle.load() + pnp_->returnPitchAngle());
              // is_shoot_ = net_armor_->topAutoShoot(pnp_->returnDepth(), robo_inf_.bullet_velocity.load(), armor_.rst[0].pts, net_armor_->returnArmorRotatedRect(), src_img_);
              is_shoot_ = net_armor_->topAutoShoot(pnp_->returnDepth(), robo_inf_.bullet_velocity.load(), armor_.rst[0].pts, net_armor_->returnArmorRotatedRect(), src_img_);
              if (armor_.rst[0].tag_id == 1 || armor_.rst[0].tag_id == 0) {
                pnp_->solvePnP(robo_inf_.bullet_velocity.load(), 1, net_armor_->returnArmorRotatedRect());
              } else {
                pnp_->solvePnP(robo_inf_.bullet_velocity.load(), 0, net_armor_->returnArmorRotatedRect());
              }
            }
            updataWriteData(robo_cmd_, pnp_->returnYawAngle(), pnp_->returnPitchAngle(), pnp_->returnDepth(), armor_.rst.size(), is_shoot_);
            break;
          }
          case Mode::FORECAST_MODE: {
            // std::cout << "Mode::FORECAST_MODE" << "\n";
            net_armor_->process_frame(src_img_, armor_);
            if (net_armor_->screen_armor(robo_inf_, armor_, src_img_)) {
              if (armor_.rst[0].tag_id == 1 || armor_.rst[0].tag_id == 0) {
                pnp_->solvePnP(robo_inf_.bullet_velocity.load(), 1, armor_.rst[0].pts);
              } else {
                pnp_->solvePnP(robo_inf_.bullet_velocity.load(), 0, armor_.rst[0].pts);
              }
              net_armor_->forecastFlagV(armor_.armor_t, inf_.yaw_angle.load() - pnp_->returnYawAngle(), inf_.pitch_angle.load() + pnp_->returnPitchAngle());
              // std::cout << "inf_.yaw_angle.load() - pnp_->returnYawAngle() = " << inf_.yaw_angle.load() - pnp_->returnYawAngle() << "\n";
              net_armor_->forecast_armor(pnp_->returnDepth(), robo_inf_.bullet_velocity.load(), armor_.rst[0].pts, src_img_);
              // net_armor_->defense_tower(pnp_->returnDepth(), robo_inf_.bullet_velocity.load(), armor_.rst[0].pts, src_img_);
              // pnp_->solvePnPx(0, src_img_, net_armor_->forecast_armor(robo_inf_.bullet_velocity.load(), pnp_->returnTvec(), pnp_->returnYawAngle(), pnp_->returnPitchAngle()));
              if (armor_.rst[0].tag_id == 1 || armor_.rst[0].tag_id == 0) {
                pnp_->solvePnP(robo_inf_.bullet_velocity.load(), 1, armor_.rst[0].pts);
              } else {
                pnp_->solvePnP(robo_inf_.bullet_velocity.load(), 0, armor_.rst[0].pts);
              }
            }
            updataWriteData(robo_cmd_, pnp_->returnYawAngle(), pnp_->returnPitchAngle(), pnp_->returnDepth(), armor_.rst.size(), 0);
            break;
          }
          default: {
            // std::cout << "Mode::SUP_SHOOT" << "\n";
            net_armor_->process_frame(src_img_, armor_);
            if (net_armor_->screen_armor(robo_inf_, armor_, src_img_)) {
              if (armor_.rst[0].tag_id == 1 || armor_.rst[0].tag_id == 0) {
                pnp_->solvePnP(robo_inf_.bullet_velocity.load(), 1, armor_.rst[0].pts);
              } else {
                pnp_->solvePnP(robo_inf_.bullet_velocity.load(), 0, armor_.rst[0].pts);
              }
            }
            updataWriteData(robo_cmd_, pnp_->returnYawAngle(), pnp_->returnPitchAngle(), pnp_->returnDepth(), armor_.rst.size(), 0);
            break;
          }
      }
      if (debug_mode_) {
        disData();
        webImage(src_img_);
      }
      armor_.rst.clear();
      memset(armor_.quantity, 0, sizeof(armor_.quantity));
      switchMode();
    } else {
      capture_->open();
    }
  }
}

void WolfVision::spin() {
  std::thread uartWriteThread(std::bind(&WolfVision::serialWrite,this));
  std::thread uartReadThread(std::bind(&WolfVision::serialRead,this));
  std::thread detectionThread(std::bind(&WolfVision::autoAim,this));
  while (true) {
    if (uartWriteThread.joinable())
      uartWriteThread.detach();
    if (uartReadThread.joinable())
      uartReadThread.detach();
    if (detectionThread.joinable())
      detectionThread.detach();
    std::this_thread::sleep_for(1000ms);
  }
}

void WolfVision::disData() {
    debug_info_["imu_yaw"] = inf_.yaw_angle.load();
    debug_info_["yaw"] = pnp_->returnYawAngle();
    debug_info_["pitch"] = pnp_->returnPitchAngle();
    debug_info_["tagret_yaw"] = inf_.yaw_angle.load() - pnp_->returnYawAngle();
    pj_udp_cl_->send_message(debug_info_.dump());
    debug_info_.empty();
}

void WolfVision::switchMode() {
  if (robo_inf_.model == Mode::ENERGY_AGENCY) {
    buff_num_ += 1;
    other_num_ = 0;
  } else {
    buff_num_ = 0;
    other_num_ += 1;
  }
  if (buff_num_ == 1) {
    capture_->setCameraExposureTime(buff_exposure_);
  }
  if (other_num_ == 1) {
    capture_->setCameraExposureTime(camera_exposure_);
  }
}

void WolfVision::webImage(const cv::Mat _src_img) {
  if (!_src_img.empty()) {
    std::vector<uchar> buff_bgr;
    cv::Mat _dst_img;
    cv::resize(_src_img, _dst_img, cv::Size(640, 384));
    cv::imencode(".jpg", _dst_img, buff_bgr, params_);
    streamer_ptr_->publish("/pc", std::string(buff_bgr.begin(), buff_bgr.end()));
  }
}

void WolfVision::updataWriteData(RoboCmd& _robo_cmd, const float _yaw, const float _pitch, const int _depth, const int _data_type, const int _auto_shoot) {
  _robo_cmd.yaw_angle.store(-_yaw);
  _robo_cmd.pitch_angle.store(_pitch);
  _robo_cmd.depth.store(_depth);
  _robo_cmd.data_type.store(_data_type > 1 ? 1 : _data_type);
  _robo_cmd.auto_shoot.store(_auto_shoot);
}

/**
 * @brief 串口发送
 * 
 */
void WolfVision::serialWrite() {
  while (true) try {
      if (serial_->isOpen()) {
        serial_->WriteInfo(robo_cmd_);
        // fmt::print("[{}] WriteInfo success.\n", idntifier_red);
      } else {
        serial_->open();
      }
      std::this_thread::sleep_for(10ms);
    } catch (const std::exception& e) {
      serial_->close();
      static int serial_read_excepted_times{0};
      if (serial_read_excepted_times++ > 3) {
        std::this_thread::sleep_for(10000ms);
        fmt::print("[{}] read serial_ excepted to many times, sleep 10s.\n", idntifier_red);
        serial_read_excepted_times = 0;
      }
      fmt::print("[{}] serial_ exception: {}\n", idntifier_red, e.what());
      std::this_thread::sleep_for(1000ms);
    }
}

/**
 * @brief 接受串口
 * 
 */
void WolfVision::serialRead() {
  while (true) try {
      if (serial_->isOpen()) {
        serial_->ReceiveInfo(robo_inf_);
      }
      std::this_thread::sleep_for(2ms);
    } catch (const std::exception& e) {
      static int serial_read_excepted_times{0};
      if (serial_read_excepted_times++ > 3) {
        std::this_thread::sleep_for(10000ms);
        fmt::print("[{}] read serial_ excepted to many times, sleep 10s.\n", idntifier_red);
        serial_read_excepted_times = 0;
      }
      fmt::print("[{}] serial_ exception: {}\n", idntifier_red, e.what());
      std::this_thread::sleep_for(1000ms);
    }
}