#include <filesystem>

#include <opencv2/opencv.hpp>
#include <fmt/core.h>

#include "devices/camera/mv_video_capture.hpp"

namespace fs = std::filesystem;

int main() {
  auto capture = std::make_shared<mindvision::VideoCapture>(mindvision::RESOLUTION::RESOLUTION_1280_X_768,
    fmt::format("{}{}", CONFIG_FILE_PATH, "/camera/camera_param.yaml"));
  capture->open();

  int img_id {0};

  std::string file_path_string;
  auto current_path = fs::current_path();
  if(!file_path_string.empty()) {
    current_path = file_path_string;
  } else {
    for (size_t i = 0; i < 99; i++) {
      const auto path = current_path / std::to_string(i);
      fs::directory_entry dir {path};
      if (!dir.exists()) {
        current_path = path;
        break;
      }
    }
  }

  cv::Mat img;
  int key {0};

  while (key != 'q' && capture->isOpen()) {
    key = cv::waitKey(1);
    *capture >> img;
    if(key == 'w') {
      cv::imwrite(current_path.string() + "/" + std::to_string(img_id) +
        "jpg", img);
      img_id++;
    }
    cv::imshow("img", img);
  }

  return 0;
}