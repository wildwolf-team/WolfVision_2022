/**
 * @file mv_video_capture.cpp
 * @author RCX
 * @brief 相机读取类
 * @date 2021-08-29
 *
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 *
 */
#include "mv_video_capture.hpp"

namespace mindvision {

auto idntifier_green = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "mv_video_capture");
auto idntifier_red   = fmt::format(fg(fmt::color::red)   | fmt::emphasis::bold, "mv_video_capture");

VideoCapture::VideoCapture(const RESOLUTION _resolution, const std::string _config_path){
  config_path_ = _config_path;
  camera_param_.resolution = Camera_Resolution(_resolution);
  readConfig();
}

VideoCapture::~VideoCapture() {
  close();
}

void VideoCapture::operator>>(cv::Mat& img) {
  if (is_open_ == true &&
      CameraReleaseImageBuffer(hCamera, pbyBuffer) == CAMERA_STATUS_SUCCESS &&
      CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 5000) == CAMERA_STATUS_SUCCESS) {
    CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);

    if (iplImage) {
      cvReleaseImageHeader(&iplImage);
    }
    iplImage =
      cvCreateImageHeader(cvSize(sFrameInfo.iWidth, sFrameInfo.iHeight), IPL_DEPTH_8U, channel);
    cvSetData(iplImage, g_pRgbBuffer, sFrameInfo.iWidth * channel);
    img = cv::cvarrToMat(iplImage, true);
  } else {
    fmt::print("[{}] read img from camera failed.\n", idntifier_red);
    is_open_ = false;
  }
}

void VideoCapture::readConfig() {
  cv::FileStorage fs;
  fs.open(config_path_, cv::FileStorage::READ);

  camera_param_.analog_gain = static_cast<int>(fs["AnalogGain"]);
  camera_param_.exposure_time = static_cast<int>(fs["ExposureTime"]);
  camera_param_.sharpen = static_cast<int>(fs["Sharpen"]);
  camera_param_.gamma = static_cast<int>(fs["Gamma"]);
  camera_param_.contrast = static_cast<int>(fs["Contrast"]);
  camera_param_.red_gain = static_cast<int>(fs["RedGain"]);
  camera_param_.green_gain = static_cast<int>(fs["GreenGain"]);
  camera_param_.blue_gain = static_cast<int>(fs["BlueGain"]);
  camera_param_.saturation = static_cast<int>(fs["Saturation"]);

  fs.release();
}

void VideoCapture::open() {
  if(is_open_) {
    fmt::print("[{}] Error, mindvision industrial camera already open.\n", idntifier_red);
  }
  CameraSdkInit(1);

  tSdkCameraDevInfo pCameraList;
  int piNums {1};
  int cameraStatus = CameraEnumerateDevice(&pCameraList, &piNums);

  if (piNums == 0) {
    fmt::print("[{}] Error, no mindvision industrial camera detected.\n", idntifier_red);

    is_open_ = false;
  } else {
    // 相机初始化
    cameraStatus = CameraInit(&pCameraList, -1, -1, &hCamera);

    if (cameraStatus != CAMERA_STATUS_SUCCESS && cameraStatus != CAMERA_STATUS_DEVICE_IS_OPENED) {
      fmt::print("[{}] Error, Init mindvision industrial camera failed: {}\n", idntifier_red, cameraStatus);

      is_open_ = false;
    } else {
      CameraGetCapability(hCamera, &tCapability);

      g_pRgbBuffer =
        static_cast<unsigned char*>(
          malloc(tCapability.sResolutionRange.iHeightMax *
                tCapability.sResolutionRange.iWidthMax  * 3));

      // 设置相机分辨率
      CameraGetImageResolution(hCamera, &pImageResolution);

      pImageResolution.iIndex      = 0xFF;
      pImageResolution.iWidthFOV   = camera_param_.resolution.cols;
      pImageResolution.iHeightFOV  = camera_param_.resolution.rows;
      pImageResolution.iWidth      = camera_param_.resolution.cols;
      pImageResolution.iHeight     = camera_param_.resolution.rows;
      pImageResolution.iHOffsetFOV = static_cast<int>((1280 - camera_param_.resolution.cols) * 0.5);
      pImageResolution.iVOffsetFOV = static_cast<int>((1024 - camera_param_.resolution.rows) * 0.5);

      CameraSetImageResolution(hCamera, &pImageResolution);

      // 设置曝光时间
      CameraSetAeState(hCamera, FALSE);
      CameraSetExposureTime(hCamera, camera_param_.exposure_time);

      // 关闭自动白平衡
      CameraSetWbMode(hCamera, FALSE);

      // 设置增益
      CameraSetGain(hCamera, camera_param_.red_gain, camera_param_.green_gain, camera_param_.blue_gain);
      CameraSetAnalogGain(hCamera, camera_param_.analog_gain);

      // 设置 Gamma 与对比度
      CameraSetGamma(hCamera, camera_param_.gamma);
      CameraSetContrast(hCamera, camera_param_.contrast);

      // 让SDK进入工作模式
      CameraPlay(hCamera);
      CameraReleaseImageBuffer(hCamera, pbyBuffer);

      if (tCapability.sIspCapacity.bMonoSensor) {
        channel = 1;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
      } else {
        channel = 3;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
      }

      fmt::print("[{}] Info, Init mindvision industrial camera success: {}\n", idntifier_green, cameraStatus);

      is_open_ = true;
    }
  }
}

void VideoCapture::setCameraExposureTime(const int _camera_exposure_time) {
  camera_param_.exposure_time = _camera_exposure_time;
  if (is_open_) {
    std::lock_guard<std::mutex> lk(mtx);
    CameraSetExposureTime(hCamera, camera_param_.exposure_time);
  }
}

// 白平衡校准
void VideoCapture::setCameraOnceWB() {
  if (is_open_) {
    std::lock_guard<std::mutex> lk(mtx);
    CameraSetOnceWB(hCamera);
  }
}

void VideoCapture::setCameraColorGain(int _iRGain, int _iGGain, int _iBGain) {
  camera_param_.red_gain = _iRGain;
  camera_param_.green_gain = _iGGain;
  camera_param_.blue_gain = _iBGain;
  if (is_open_) {
    std::lock_guard<std::mutex> lk(mtx);
    CameraSetGain(hCamera, _iRGain, _iGGain, _iBGain);
  }
}

void VideoCapture::setCameraAnalogGrain(int _iAnalogGain) {
  camera_param_.analog_gain = _iAnalogGain;
  if (is_open_) {
    std::lock_guard<std::mutex> lk(mtx);
    CameraSetAnalogGain(hCamera, _iAnalogGain);
  }
}

void VideoCapture::close() {
  if (is_open_) {
    int camera_status = CameraUnInit(hCamera);
    free(g_pRgbBuffer);

    fmt::print("[{}] Released mindvision industrial camera: {}\n", idntifier_green, camera_status);
  }
}

bool VideoCapture::isOpen() {
  return is_open_;
}

int VideoCapture::getImageCols() {
  return camera_param_.resolution.cols;
}

int VideoCapture::getImageRows() {
  return camera_param_.resolution.rows;
}

cv::Size VideoCapture::getImageSize() {
  return cv::Size(camera_param_.resolution.cols, camera_param_.resolution.rows);
}

}  // namespace mindvision
