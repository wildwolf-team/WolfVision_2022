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
  capture_ = std::make_shared<mindvision::VideoCapture>(mindvision::RESOLUTION::RESOLUTION_1280_X_768,
    fmt::format("{}{}", CONFIG_FILE_PATH, "/camera/camera_param.yaml"));
  if(!capture_->isOpen())
    capture_->open();
  pnp_     = std::make_unique<basic_pnp::PnP>(fmt::format("{}{}", CONFIG_FILE_PATH, "/camera/mv_camera_config_555.xml"),
                                              fmt::format("{}{}", CONFIG_FILE_PATH, "/angle_solve/basic_pnp_config.xml"));
  basic_armor_   = std::make_shared<basic_armor::Detector>(fmt::format("{}{}", CONFIG_FILE_PATH, "/armor/basic_armor_config.xml"));
  buff_    = std::make_unique<basic_buff::Detector>(fmt::format("{}{}", CONFIG_FILE_PATH, "/buff/basic_buff_config.xml"));
  net_armor_ = std::make_unique<basic_net::Detector>();
  net_armor_->detection_init(fmt::format("{}{}", CONFIG_FILE_PATH, "/net/opt4_FP16.xml"), "GPU");
  spin_armor_ = std::make_unique<hero::spinArmor>(fmt::format("{}{}", CONFIG_FILE_PATH, "/armor/spin_armor.yaml"));
  armor_.rst.reserve(128);
  pnp_->serYawPower(yaw_power_);
  if (vw_mode_) {
    t_ = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    vw_t_ss_ << std::put_time(std::localtime(&t_), "/video/%Y_%m_%d_%H_%M_%S");
    vw_t_str_ = CONFIG_FILE_PATH + vw_t_ss_.str() + ".avi";
    vw_src_.open(vw_t_str_, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, capture_->getImageSize(), true);  // 记得打开  
  }
} catch(const std::exception& e) {
  fmt::print("{}\n", e.what());
}

WolfVision::~WolfVision() {}

void WolfVision::autoAim() {
  auto start = std::chrono::system_clock::now();
  ThreadPool pool(4);
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
            case Mode::TRADITION_MODE: {
            // std::cout << "is TRADITION_MODE" << std::endl;
            basic_armor_->freeMemory();
            if(basic_armor_->runBasicArmor(src_img_, robo_inf_)){
              pnp_->solvePnP(robo_inf_.bullet_velocity, basic_armor_->returnFinalArmorDistinguish(0), basic_armor_->returnFinalArmorRotatedRect(0));
              yaw   = pnp_->returnYawAngle();
              depth = pnp_->returnDepth();
              pitch = pnp_->returnPitchAngle();
              basic_armor_->forecast_armor_flag(armor_.armor_t, robo_inf_.yaw_angle, last_yaw);
              target_2d = basic_armor_->forecast_armor(depth * 1000, robo_inf_.bullet_velocity.load(), 0);
              pnp_->solvePnP(robo_inf_.bullet_velocity.load(), basic_armor_->returnFinalArmorDistinguish(0), target_2d);
              updataWriteData(robo_cmd_, pnp_->returnYawAngle(), pitch, depth, basic_armor_->returnArmorNum(), 0);
            }else{
              basic_armor_->forecast_armor_flag(armor_.armor_t, robo_inf_.yaw_angle, last_yaw);
              yaw = 0;
              if(basic_armor_->returnLostCnt() > 0){
                updataWriteData(robo_cmd_, pnp_->returnYawAngle(), pitch, depth, 1, 0);
              }else{
                updataWriteData(robo_cmd_, pnp_->returnYawAngle(), pitch, depth, 0, 0);
              }
            }
            last_yaw = yaw;
            last_pitch = pitch;
            break;
          }
          case Mode::ENERGY_AGENCY: {
            // std::cout << " is ENERGY_AGENCY mode " << std::endl;
            buff_->runTask(src_img_, robo_inf_, robo_cmd_, time);
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
              // 当装甲板正面时更新深度
              float error_angle = atan((armor_.rst[0].pts[3].y - armor_.rst[0].pts[0].y) /
                                       (armor_.rst[0].pts[3].x - armor_.rst[0].pts[0].x));
              error_angle = atan(error_angle) * 180 / M_PI;
              if (fabs(error_angle) < 0.5f) {
                depth_ = pnp_->returnDepth();
              }
              net_armor_->forecastFlagV(armor_.armor_t, inf_.yaw_angle.load() - pnp_->returnYawAngle(), inf_.pitch_angle.load() + pnp_->returnPitchAngle());
              is_shoot_ = net_armor_->topAutoShoot(pnp_->returnDepth(), robo_inf_.bullet_velocity.load(), armor_.rst[0].pts, net_armor_->returnArmorRotatedRect(), src_img_);
              if (armor_.rst[0].tag_id == 1 || armor_.rst[0].tag_id == 0) {
                pnp_->solvePnP(robo_inf_.bullet_velocity.load(), 1, net_armor_->returnArmorRotatedRect(), depth_);
              } else {
                pnp_->solvePnP(robo_inf_.bullet_velocity.load(), 0, net_armor_->returnArmorRotatedRect(), depth_);
              }
            }
            updataWriteData(robo_cmd_, pnp_->returnYawAngle(), pnp_->returnPitchAngle(), depth_, armor_.rst.size(), is_shoot_);
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
              net_armor_->forecast_armor(pnp_->returnDepth(), robo_inf_.bullet_velocity.load(), armor_.rst[0].pts, src_img_);
              if (armor_.rst[0].tag_id == 1 || armor_.rst[0].tag_id == 0) {
                pnp_->solvePnP(robo_inf_.bullet_velocity.load(), 1, armor_.rst[0].pts);
              } else {
                pnp_->solvePnP(robo_inf_.bullet_velocity.load(), 0, armor_.rst[0].pts);
              }
            }
            updataWriteData(robo_cmd_, pnp_->returnYawAngle(), pnp_->returnPitchAngle(), pnp_->returnDepth(), armor_.rst.size(), 0);
            break;
          }
          case Mode::SPINARMOR_MODE: {
            if(spin_armor_->run(robo_inf_)) {
              updataWriteData(robo_cmd_, 0.f, spin_armor_->getTargetImuPitch() == 0.f ? 0.f : 
                robo_inf_.pitch_angle.load() - spin_armor_->getTargetImuPitch() - 0.25f,
                0.f, true, true);
              std::this_thread::sleep_for(15ms);
            } else {
              updataWriteData(robo_cmd_, 0.f, spin_armor_->getTargetImuPitch() == 0.f ? 0.f :
                robo_inf_.pitch_angle.load() - spin_armor_->getTargetImuPitch() - 0.25f,
                0.f, true, false);
            }
            net_armor_->process_frame(src_img_, armor_);
            if(net_armor_->screen_top_armor(robo_inf_, armor_, src_img_)) {
              for(auto &rst : armor_.rst) {
                pnp_->solvePnP(robo_inf_.bullet_velocity.load(), 1, rst.pts);
                if(pnp_->returnPitchAngle() < 3.f && pnp_->returnPitchAngle() > -3.f) {
                  spin_armor_->add(pnp_->returnYawAngle(), pnp_->returnDepth(),
                    robo_inf_.pitch_angle.load() - pnp_->returnPitchAngle());
                  break;
                }
              }
            }
            debug_info_["spin_armor_mode"] = nlohmann::json(spin_armor_->returnDebugInfo());
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
      pool.enqueue([=]() {
          disData();
          webImage(src_img_);
      });
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
    debug_info_["imu_pitch"] = inf_.pitch_angle.load();
    debug_info_["vision_yaw"] = -robo_cmd_.yaw_angle.load();
    debug_info_["vision_pitch"] = robo_cmd_.pitch_angle.load();
    debug_info_["depth"] = robo_cmd_.depth.load();
    debug_info_["tagret_yaw"] = inf_.yaw_angle.load() - pnp_->returnYawAngle();
    debug_info_["mode"] = robo_cmd_.data_type.load();
    pj_udp_cl_->send_message(debug_info_.dump());
    debug_info_.empty();
}

void WolfVision::switchMode() {
 if (robo_inf_.model == Mode::ENERGY_AGENCY || robo_inf_.model == Mode::TRADITION_MODE) {
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
  _robo_cmd.depth.store(_depth*1000);
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
      // std::this_thread::sleep_for(1ms);
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