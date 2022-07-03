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

VideoCapture::VideoCapture(const CameraParam &_camera_param) :
  camera_exposuretime_(_camera_param.camera_exposuretime),
  camera_resolution_(_camera_param.resolution) {
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
      pImageResolution.iWidthFOV   = camera_resolution_.cols;
      pImageResolution.iHeightFOV  = camera_resolution_.rows;
      pImageResolution.iWidth      = camera_resolution_.cols;
      pImageResolution.iHeight     = camera_resolution_.rows;
      pImageResolution.iHOffsetFOV = static_cast<int>((1280 - camera_resolution_.cols) * 0.5);
      pImageResolution.iVOffsetFOV = static_cast<int>((1024 - camera_resolution_.rows) * 0.5);

      CameraSetImageResolution(hCamera, &pImageResolution);

      // 设置曝光时间
      CameraSetAeState(hCamera, FALSE);
      CameraSetExposureTime(hCamera, camera_exposuretime_);

      CameraSetGain(hCamera, 100, 100, 105);
      //red  145 100 95
      //blue 100 100 105
      CameraSetAnalogGain(hCamera, 20);

      CameraSetGamma(hCamera, 60);
      // CameraSetContrast(hCamera, 80);
      // 关闭自动白平衡
      CameraSetWbMode(hCamera, FALSE);
      // CameraSetOnceWB(hCamera);
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
  camera_exposuretime_ = _camera_exposure_time;
  if (is_open_) {
    std::lock_guard<std::mutex> lk(mtx);
    CameraSetExposureTime(hCamera, camera_exposuretime_);
  }
}

// 白平衡校准
void VideoCapture::setCameraOnceWB() {
  if (is_open_) {
    std::lock_guard<std::mutex> lk(mtx);
    CameraSetOnceWB(hCamera);
  }
}

void VideoCapture::setCameraColorGain(int iRGain, int iGGain, int iBGain) {
  if (is_open_) {
    std::lock_guard<std::mutex> lk(mtx);
    CameraSetGain(hCamera, iRGain, iGGain, iBGain);
  }
}

void VideoCapture::setCameraAnalogGrain(int iAnalogGain) {
  if (is_open_) {
    std::lock_guard<std::mutex> lk(mtx);
    CameraSetAnalogGain(hCamera, iAnalogGain);
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
  return camera_resolution_.cols;
}

int VideoCapture::getImageRows() {
  return camera_resolution_.rows;
}

cv::Size VideoCapture::getImageSize() {
  return cv::Size(camera_resolution_.cols, camera_resolution_.rows);
}

}  // namespace mindvision
