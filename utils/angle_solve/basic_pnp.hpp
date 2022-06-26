/**
 * @file basic_pnp.hpp
 * @author XX (2796393320@qq.com)
 * @brief 角度结算
 * @date 2021-08-27
 *
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 *
 */
#pragma once

#include <string>
#include <vector>

#include <fmt/core.h>
#include <fmt/color.h>

#include "abstract_pnp.hpp"

namespace basic_pnp {

auto idntifier_green = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "basic_pnp");
auto idntifier_red   = fmt::format(fg(fmt::color::red)   | fmt::emphasis::bold, "basic_pnp");

class PnP : public abstract_pnp::PnP {
 public:
  PnP() = default;
  explicit PnP(std::string _camera_path, std::string _pnp_config_path) {
    cv::FileStorage fs_camera(_camera_path, cv::FileStorage::READ);

    fs_camera["camera-matrix"] >> cameraMatrix_;
    fs_camera["distortion"] >> distCoeffs_;

    cv::FileStorage fs_config(_pnp_config_path, cv::FileStorage::READ);

    fs_config["PTZ_CAMERA_X"] >> pnp_config_.ptz_camera_x;
    fs_config["PTZ_CAMERA_Y"] >> pnp_config_.ptz_camera_y;
    fs_config["PTZ_CAMERA_Z"] >> pnp_config_.ptz_camera_z;

    fs_config["PTZ_BARREL_X"] >> pnp_config_.barrel_ptz_offset_x;
    fs_config["PTZ_BARREL_Y"] >> pnp_config_.barrel_ptz_offset_y;

    fs_config["OFFSET_ARMOR_YAW"] >> pnp_config_.offset_armor_yaw;
    fs_config["OFFSET_ARMOR_PITCH"] >> pnp_config_.offset_armor_pitch;

    fmt::print("[{}] Info, ptz_camera_x,y,z: {}, {}, {}\n", idntifier_green, pnp_config_.ptz_camera_x, pnp_config_.ptz_camera_y, pnp_config_.ptz_camera_z);

    fmt::print("[{}] Info, ptz_barrel_x,y: {}, {}\n", idntifier_green, pnp_config_.barrel_ptz_offset_x, pnp_config_.barrel_ptz_offset_y);

    fmt::print("[{}] Info, offset_armor_yaw,pitch: {}, {}\n", idntifier_green, pnp_config_.offset_armor_yaw, pnp_config_.offset_armor_pitch);


    fmt::print("[{}] Init PnP configuration finished\n", idntifier_green);
  }

  ~PnP() = default;
  /**
   * @brief 返回 Yaw 轴角度
   *
   * @return float
   * @author XX
   */
  inline float  returnYawAngle()   { return pnp_info_.x; }
  /**
   * @brief 返回 Pitch 轴角度
   *
   * @return float
   * @author XX
   */
  inline float  returnPitchAngle() { return pnp_info_.y; }
  /**
   * @brief 返回深度
   *
   * @return float
   * @author XX
   */
  inline float  returnDepth()      { return pnp_info_.z; }
  /**
   * @brief 返回目标旋转向量
   *
   * @return double
   * @author XX
   */
  inline double returnTvecTx()     { return tvec_.ptr<double>(0)[0]; }
  inline double returnTvecTy()     { return tvec_.ptr<double>(0)[1]; }
  inline double returnTvecTz()     { return tvec_.ptr<double>(0)[2]; }
  inline cv::Point3f returnPnpInfo() {return pnp_info_;}
  /**
   * @brief 角度结算
   *
   * @param _ballet_speed 子弹速度
   * @param _armor_type   装甲板类型
   * @param _src_img      原图（ CV_8UC3 ）
   * @param _rect         目标旋转矩形
   * @author XX
   */
  void solvePnP(const int _ballet_speed, const int _armor_type, const cv::Point2f p[4], const int _depth = 0) {
    object_3d_ = initialize3DPoints(_armor_type);
    std::vector<cv::Point2d> pu(p, p + 4);

    cv::solvePnP(object_3d_, pu, cameraMatrix_, distCoeffs_, rvec_, tvec_, false, cv::SOLVEPNP_ITERATIVE);

    cv::Mat     ptz       = cameraPtz(tvec_);
    cv::Point3f angle     = getAngle(ptz, _ballet_speed, 1, _depth);
    pnp_info_.x *= yaw_power_;
    pnp_info_.x = angle.x + pnp_config_.offset_armor_yaw;
    pnp_info_.y = angle.y + pnp_config_.offset_armor_pitch;
    pnp_info_.z = angle.z;
    
    if (fabs(pnp_info_.x) > 30) {
      pnp_info_.x = 0;
    }
    if (fabs(pnp_info_.y) > 15) {
      pnp_info_.y = 0;
    }

    object_3d_.clear();
    object_3d_.shrink_to_fit();
    target_2d_.clear();
    target_2d_.shrink_to_fit();
  }
  /**
   * @brief 角度结算
   *
   * @param _ballet_speed 子弹速度
   * @param _armor_type   装甲板类型
   * @param _src_img      原图（ CV_8UC3 ）
   * @param _rect         目标旋转矩形
   * @author XX
   */
  void solvePnP(const int             _ballet_speed,
                const int             _armor_type,
                const cv::RotatedRect _rect, 
                const int             _depth = 0) {
    object_3d_ = initialize3DPoints(_armor_type);
    target_2d_ = initialize2DPoints(_rect);

    cv::solvePnP(object_3d_, target_2d_, cameraMatrix_, distCoeffs_, rvec_, tvec_);

    cv::Mat     ptz   = cameraPtz(tvec_);
    cv::Point3f angle = getAngle(ptz, _ballet_speed, 1, _depth);
    pnp_info_.x *= yaw_power_;
    pnp_info_.x = angle.x + pnp_config_.offset_armor_yaw;
    pnp_info_.y = angle.y + pnp_config_.offset_armor_pitch;
    pnp_info_.z = angle.z;
    
    if (fabs(pnp_info_.x) > 30) {
      pnp_info_.x = 0;
    }
    if (fabs(pnp_info_.y) > 15) {
      pnp_info_.y = 0;
    }
    object_3d_.clear();
    object_3d_.shrink_to_fit();
    target_2d_.clear();
    target_2d_.shrink_to_fit();
  }
  
  /**
   * @brief 角度结算
   *
   * @param _ballet_speed 子弹速度
   * @param _armor_type   装甲板类型
   * @param _target_2d    目标 2d 点坐标
   * @author XX
   */
  void solvePnP(const int                      _ballet_speed,
                const int                      _armor_type,
                const std::vector<cv::Point2f> _target_2d,
                const int                      _depth = 0) {
    object_3d_ = initialize3DPoints(_armor_type);
    target_2d_ = _target_2d;

    cv::solvePnP(object_3d_, target_2d_, cameraMatrix_, distCoeffs_, rvec_, tvec_);

    cv::Mat     ptz   = cameraPtz(tvec_);
    cv::Point3f angle = getAngle(ptz, _ballet_speed, 1, _depth);
    pnp_info_.x *= yaw_power_;
    pnp_info_.x = angle.x + pnp_config_.offset_armor_yaw;
    pnp_info_.y = angle.y + pnp_config_.offset_armor_pitch;
    pnp_info_.z = angle.z;
    
    object_3d_.clear();
    object_3d_.shrink_to_fit();
    target_2d_.clear();
    target_2d_.shrink_to_fit();
  }
  void serYawPower(const float _yaw_power) {yaw_power_ = _yaw_power;}
 private:
  // PnP_Information pnp_info_;
  cv::Point3f pnp_info_;
  cv::Mat cameraMatrix_, distCoeffs_;
  cv::Mat rvec_ = cv::Mat::zeros(3, 3, CV_64FC1);
  cv::Mat tvec_ = cv::Mat::zeros(3, 1, CV_64FC1);
  float yaw_power_ = 0.f;
  std::vector<cv::Point2f> target_2d_;
  std::vector<cv::Point3f> object_3d_;
};

}  // namespace basic_pnp
