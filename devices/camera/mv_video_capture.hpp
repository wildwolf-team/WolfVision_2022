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

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include <CameraApi.h>

namespace mindvision {

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
  explicit Camera_Resolution(const mindvision::RESOLUTION _resolution =
      RESOLUTION::RESOLUTION_1280_X_1024) {
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
  int analog_gain;
  int exposure_time;
  int sharpen;
  int gamma;
  int contrast;
  int red_gain;
  int green_gain;
  int blue_gain;
  int saturation;

  Camera_Resolution resolution;
};

class VideoCapture {
 public:
  VideoCapture() = default;
  explicit VideoCapture(const RESOLUTION _resolution, 
                        const std::string _config_path);

  ~VideoCapture();

  void operator>>(cv::Mat& img);

  void readConfig();

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

  void setCameraColorGain(int _iRGain, int _iGGain, int _iBGain);

  void setCameraAnalogGrain(int _iAnalogGain);

  void close();

  bool isOpen();

  int getImageCols();

  int getImageRows();

  cv::Size getImageSize();


 private:
  std::mutex mtx;

  unsigned char* g_pRgbBuffer;

  std::string config_path_;
  CameraParam camera_param_;

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
