/**
 * @file mv_video_capture.hpp
 * @author RCX
 * @brief 相机读取类
 * @date 2021-08-29
 *
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 *
 */
#pragma once

#include <mutex>

#include <fmt/core.h>
#include <fmt/color.h>

#include <utils.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include <CameraApi.h>

namespace mindvision {

enum MyColor {
  ALL,
  RED,
  BLUE,
};

enum EXPOSURETIME {
  // 相机曝光时间
  EXPOSURE_15000 = 15000,
  EXPOSURE_10000 = 10000,
  EXPOSURE_7500 = 7500,
  EXPOSURE_5000 = 5000,
  EXPOSURE_2500 = 2500,
  EXPOSURE_1200 = 1200,
  EXPOSURE_800 = 800,
  EXPOSURE_600 = 600,
  EXPOSURE_400 = 400,
};

enum RESOLUTION {
  // 相机分辨率
  RESOLUTION_1280_X_1024,
  RESOLUTION_1280_X_800,
  RESOLUTION_1280_X_768,
  RESOLUTION_960_X_600,
  RESOLUTION_640_X_384,
};

struct Camera_Resolution {
  int cols;
  int rows;
  // 设置相机分辨率
  explicit Camera_Resolution(const mindvision::RESOLUTION _resolution) {
    switch (_resolution) {
      case mindvision::RESOLUTION::RESOLUTION_1280_X_1024:
        cols = 1280;
        rows = 1024;
        break;
      case mindvision::RESOLUTION::RESOLUTION_1280_X_800:
        cols = 1280;
        rows = 800;
        break;
      case mindvision::RESOLUTION::RESOLUTION_1280_X_768:
        cols = 1280;
        rows = 768;
        break;
      case mindvision::RESOLUTION::RESOLUTION_960_X_600:
        cols = 960;
        rows = 600;
        break;
      case mindvision::RESOLUTION::RESOLUTION_640_X_384:
        cols = 640;
        rows = 384;
        break;
      default:
        cols = 1280;
        rows = 800;
        break;
    }
  }
};

struct CameraParam {
  int camera_mode;
  int camera_exposuretime;

  mindvision::Camera_Resolution resolution;

  CameraParam(const int                      _camera_mode,
              const mindvision::RESOLUTION   _resolution,
              int _camera_exposuretime)
    : camera_mode(_camera_mode),
      camera_exposuretime(_camera_exposuretime),
      resolution(_resolution) {}
};

class VideoCapture {
 public:
  VideoCapture() = default;
  explicit VideoCapture(const mindvision::CameraParam &_camera_param);

  ~VideoCapture();

  void operator>>(cv::Mat& img);

  /**
   * @brief 判断工业相机是否在线
   *
   * @return true   检测到工业相机
   * @return false  没检测到工业相机
   */
  bool isindustryimgInput();
  /**
   * @brief 清空相机内存（每次读取相机后进行清空）
   * 
   */
  void cameraReleasebuff();

  void open();
  /**
   * @brief 返回相机读取图片
   * 
   * @return cv::Mat 
   */
  inline cv::Mat image() const { return cv::cvarrToMat(iplImage, true); }

  void setCameraExposureTime(int _camera_exposure_time);

  void setCameraOnceWB();

  void setCameraColorGain(int iRGain, int iGGain, int iBGain);

  void setCameraAnalogGrain(int iAnalogGain);

  void close();

  bool isOpen();

  int getImageCols();

  int getImageRows();

  cv::Size getImageSize();

  int myColor(const RoboInf& _robo_inf);


 private:
  std::mutex mtx;

  unsigned char* g_pRgbBuffer;

  int camera_exposuretime_ = 0;
  mindvision::Camera_Resolution camera_resolution_;

  int  hCamera;
  int  channel        = 3;
  bool is_open_       = false;


  tSdkCameraDevInfo   tCameraEnumList;
  tSdkCameraCapbility tCapability;
  tSdkFrameHead       sFrameInfo;
  tSdkImageResolution pImageResolution;
  BYTE*               pbyBuffer;
  BOOL                AEstate  = FALSE;
  IplImage*           iplImage = nullptr;
};

}  // namespace mindvision
