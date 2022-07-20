#include <chrono>
#include <opencv2/opencv.hpp>
#include <fmt/core.h>

#include "devices/camera/mv_video_capture.hpp"
#include "devices/serial/serial.hpp"
#include "utils/utils.hpp"

int main() {
  auto start = std::chrono::system_clock::now();
  auto end = std::chrono::system_clock::now();

  auto capture = std::make_shared<mindvision::VideoCapture>(mindvision::RESOLUTION::RESOLUTION_1280_X_768,
    fmt::format("{}{}", CONFIG_FILE_PATH, "/camera/camera_param.yaml"));
  capture->open();

  auto serial_ = std::make_unique<RoboSerial>("/dev/ttyUSB0", 115200);

  cv::Mat img;
  cv::VideoWriter vw(fmt::format("{}{}", SOURCE_PATH, "/test.avi"),
                     cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                     30, cv::Size(capture->getImageCols(),
                     capture->getImageRows()));

  while (true) {
    if(capture->isOpen())
      *capture >> img;
    if(!img.empty()) {
      end = std::chrono::system_clock::now();
      cv::putText(img, std::to_string(
        std::chrono::duration_cast<std::chrono::milliseconds>
        (end - start).count()), cv::Point(img.cols * 0.25, img.rows * 0.25),
        1, 2, cv::Scalar(0, 255, 0));
      vw.write(img);
      cv::imshow("shootDelay", img);
      int key = cv::waitKey(1);
      if(key == 's') {
        start = std::chrono::system_clock::now();
        RoboCmd robo_cmd;
        robo_cmd.data_type.store(true);
        robo_cmd.auto_shoot.store(true);
        serial_->WriteInfo(robo_cmd);
      } else if (key == 'e') {
        RoboCmd robo_cmd;
        robo_cmd.data_type.store(false);
        robo_cmd.auto_shoot.store(false);
        serial_->WriteInfo(robo_cmd);
        break;
      }
    }
  }
  vw.release();
  return 0;
} 