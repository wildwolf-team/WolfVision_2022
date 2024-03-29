/**
 * @file abstract_pnp.hpp
 * @author XX (2796393320@qq.com)
 * @brief 角度结算基类
 * @date 2021-08-27
 *
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 *
 */

#pragma once

#include <algorithm>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace abstract_pnp {

struct PnP_Config {
  double ptz_camera_x = 0.0;
  double ptz_camera_y = 0.0;
  double ptz_camera_z = 0.0;

  float  barrel_ptz_offset_x = 0.0;
  float  barrel_ptz_offset_y = 0.0;
  float  offset_armor_pitch  = 0.0;
  float  offset_armor_yaw    = 0.0;
};

class PnP {
 public:
  PnP_Config pnp_config_;

  PnP() {
    reference_Obj_.emplace_back(cv::Point3f(0.0, 0.0, 0.0));
    reference_Obj_.emplace_back(cv::Point3f(100, 0.0, 0.0));
    reference_Obj_.emplace_back(cv::Point3f(0.0, 100, 0.0));
    reference_Obj_.emplace_back(cv::Point3f(0.0, 0.0, 100));

    static double theta = 0;
    static double r_data[] = {1, 0,           0,
                              0, cos(theta),  sin(theta),
                              0, -sin(theta), cos(theta)};
    static double t_data[] = {static_cast<double>(pnp_config_.ptz_camera_x),
                              static_cast<double>(pnp_config_.ptz_camera_y),
                              static_cast<double>(pnp_config_.ptz_camera_z)};

    r_camera_ptz = cv::Mat(3, 3, CV_64FC1, r_data);
    t_camera_ptz = cv::Mat(3, 1, CV_64FC1, t_data);

    small_object_3d_ = {// 单位：m
                        {-0.066, 0.029,  0.},
                        {-0.066, -0.029, 0.},
                        {0.066,  -0.029, 0.},
                        {0.066,  0.029,  0.}};
    big_object_3d_   = {// 单位：m
                      {-0.115, 0.029,  0.},
                      {-0.115, -0.029, 0.},
                      {0.115,  -0.029, 0.},
                      {0.115,  0.029,  0.}};
    buff_object_3d_ = {// 单位：m
                      {-0.115, 0.029,  0.},
                      {-0.115, -0.029, 0.},
                      {0.115,  -0.029, 0.},
                      {0.115,  0.029,  0.}};
  }

  ~PnP() = default;
  /**
   * @brief 初始化目标 3d 点
   *
   * @param _armor_type               装甲板类型
   * @return std::vector<cv::Point3f> 返回目标 3d 点
   * @author XX
   */
  std::vector<cv::Point3f> initialize3DPoints(int _armor_type) {
    // 选择装甲板类型
    switch (_armor_type) {
      case 0:
        return small_object_3d_;
        break;
      case 1:
        return big_object_3d_;
        break;
      case 2:
        return buff_object_3d_;
        break;
      default:
        return small_object_3d_;
        break;
    }
  }
  /**
   * @brief 初始化目标 3d 点
   *
   * @param _width                    目标实际宽度
   * @param _heigth                   目标实际高度
   * @return std::vector<cv::Point3f> 返回目标 3d 点
   * @author XX
   */
  std::vector<cv::Point3f> initialize3DPoints(int _width, int _heigth) {
    float half_x = _width  * 0.5;
    float half_y = _heigth * 0.5;

    // 3d点模型赋值
    std::vector<cv::Point3f> object_3d = {
      cv::Point3f(-half_x, -half_y, 0),
      cv::Point3f(half_x,  -half_y, 0),
      cv::Point3f(half_x,  half_y,  0),
      cv::Point3f(-half_x, half_y,  0)
    };

    return object_3d;
  }
  /**
   * @brief 初始化目标 2d 点
   *
   * @param _rect                     目标旋转矩形
   * @return std::vector<cv::Point2f> 返回目标 2d 点
   * @author XX
   */
  std::vector<cv::Point2f> initialize2DPoints(cv::RotatedRect _rect) {
    std::vector<cv::Point2f> target2d;

    static cv::Point2f vertex[4];
    static cv::Point2f lu, ld, ru, rd;

    _rect.points(vertex);
    // 排序 左上->右上->右下->左下
    std::sort(vertex, vertex + 4,
      [](const cv::Point2f& p1, const cv::Point2f& p2) {
          return p1.x < p2.x;
      });

    if (vertex[0].y < vertex[1].y) {
      lu = vertex[0];
      ld = vertex[1];
    } else {
      lu = vertex[1];
      ld = vertex[0];
    }
    if (vertex[2].y < vertex[3].y) {
      ru = vertex[2];
      rd = vertex[3];
    } else {
      ru = vertex[3];
      rd = vertex[2];
    }
    target2d.emplace_back(lu);
    target2d.emplace_back(ru);
    target2d.emplace_back(rd);
    target2d.emplace_back(ld);

    return target2d;
  }
  /**
   * @brief 初始化目标 2d 点
   *
   * @param _rect                     目标外接矩形
   * @return std::vector<cv::Point2f> 返回目标 2d 点
   * @author XX
   */
  std::vector<cv::Point2f> initialize2DPoints(cv::Rect _rect) {
    cv::RotatedRect box = rectChangeRotatedrect(_rect);

    return initialize2DPoints(box);
  }
  /**
   * @brief 外接矩形转旋转矩形
   *
   * @param _rect            目标外接矩形
   * @return cv::RotatedRect 返回旋转矩形
   * @author XX
   */
  cv::RotatedRect rectChangeRotatedrect(cv::Rect _rect) {
    cv::RotatedRect box =
      cv::RotatedRect((_rect.tl() + _rect.br()) / 2,
                      cv::Size(_rect.width / 2, _rect.height / 2),
                      0);

    return box;
  }
  /**
   * @brief 转换坐标系
   *
   * @param _t       旋转向量
   * @return cv::Mat 返回转化后的旋转向量
   * @author XX
   */
  cv::Mat cameraPtz(cv::Mat& _t) {
    double t_data[] = {static_cast<double>(pnp_config_.ptz_camera_x),
                       static_cast<double>(pnp_config_.ptz_camera_y),
                       static_cast<double>(pnp_config_.ptz_camera_z)};
    t_camera_ptz = cv::Mat(3, 1, CV_64FC1, t_data);
    return r_camera_ptz * _t - t_camera_ptz;
  }
  /**
   * @brief 绘制坐标系
   *
   * @param _draw_img     画板
   * @param _rvec         旋转矩阵
   * @param _tvec         旋转向量
   * @param _cameraMatrix 相机内参
   * @param _distcoeffs   相机外参
   * @author XX
   */
  void drawCoordinate(cv::Mat& _draw_img,
                      cv::Mat& _rvec,         cv::Mat& _tvec,
                      cv::Mat& _cameraMatrix, cv::Mat& _distcoeffs) {
    std::vector<cv::Point2f> reference_Img;

    cv::projectPoints(reference_Obj_,
                      _rvec,         _tvec,
                      _cameraMatrix, _distcoeffs,
                      reference_Img);
    // 绘制坐标系
    cv::line(_draw_img, reference_Img[0], reference_Img[1],
             cv::Scalar(0, 0, 255), 2);
    cv::line(_draw_img, reference_Img[0], reference_Img[2],
             cv::Scalar(0, 255, 0), 2);
    cv::line(_draw_img, reference_Img[0], reference_Img[3],
             cv::Scalar(255, 0, 0), 2);

    // //cv::imshow("[abstract_pnp] drawCoordinate() -> _draw_img", _draw_img);
  }
  /**
   * @brief 计算子弹下坠
   *
   * @param _dist         目标深度
   * @param _tvec_y       目标高度
   * @param _ballet_speed 子弹速度
   * @param _company      计算单位
   * @return float        返回补偿角度
   * @author XX
   */
  float getPitch(float       _dist,
                 float       _tvec_y,
                 float       _ballet_speed,
                 const int   _company = 1) {
    // 选择计算单位
    _dist         /= _company;
    _tvec_y       /= _company;
    _ballet_speed /= _company;
    
    float       y_temp   = _tvec_y;
    float       y_actual = 0.f;
    float       dy       = 0.f;
    float       a        = 0.f;
    const float gravity  = 9800.f / _company;
    
    for (size_t i = 0; i != 5; ++i) {
      a = static_cast<float>(atan2(y_temp, _dist));
      // 子弹飞行时间
      float t =  (float((exp( 0.001 * _dist)-1) / ( 0.001 * _dist * 0.001 * _ballet_speed * cos(a))));
      // float t = _dist / _ballet_speed * cos(a);

      y_actual  = _ballet_speed * sin(a) * t - gravity * t * t / 2;
      dy        = _tvec_y - y_actual;
      y_temp   += dy;

      if (fabsf(dy) < 1e-2) { break; }
    }

    return a;
  }

  /**
   * @brief 计算云台偏差角度
   *
   * @param _pos_in_ptz   旋转向量
   * @param _bullet_speed 子弹速度
   * @param _company      子弹下坠单位
   * @param _depth        距目标的水平深度(m)
   * @return cv::Point3f  返回 Yaw Pitch 轴的偏移量和深度（m）
   * @author XX
   */
  cv::Point3f getAngle(const cv::Mat& _pos_in_ptz,
                       const int      _bullet_speed,
                       const int      _company,
                       const float    _depth) {
    cv::Point3f angle;
    
    const double *_xyz  = reinterpret_cast<const double *>(_pos_in_ptz.data);
    double xyz[3]         = {_xyz[0], _xyz[1], _xyz[2]};
    // 更新水平深度
    if (_depth != 0) {
      xyz[2] = _depth;
    }
    // 计算直线距离
    double down_t   = 0.f;
    double distance = sqrt(xyz[2] * xyz[2] + xyz[1] * xyz[1] + xyz[0] * xyz[0]);
    // std::cout << "distance == " << distance << std::endl;
    if (_bullet_speed < 20) {
      down_t = distance / static_cast<double>(_bullet_speed);
      down_t = down_t * down_t;
    } else {
      double distance_xy = sqrt(xyz[1] * xyz[1] + xyz[0] * xyz[0]);
      double p_pitch = std::atan2(xyz[2], distance_xy);
      // 抛物线计算
      double a = 9.8 * 9.8 * 0.25;
      double b = -_bullet_speed * _bullet_speed - distance * 9.8 * cos(M_PI_2 + p_pitch);
      double c = distance * distance;
      // 带入求根公式，解出down_t = t^2
      down_t = (- sqrt(b * b - 4 * a * c) - b) / (2 * a);
    }
    // std::cout << "down_t == " << down_t << std::endl;
    // h = 0.5 * g * t^2
    double offset_gravity = 0.5 * 9.8 * down_t;
    // std::cout << "offset_gravity == " << offset_gravity << std::endl;
    // 添加补偿高度
    xyz[1] = xyz[1] - offset_gravity;

    if (pnp_config_.barrel_ptz_offset_y != 0.f) {
      double alpha =
        asin(static_cast<double>(pnp_config_.barrel_ptz_offset_y) /
             sqrt(xyz[1] * xyz[1] + xyz[2] * xyz[2]));
      double beta  = 0.f;

      if (xyz[1] < 0) {
        beta    = atan(-xyz[1] / xyz[2]);
        angle.y = static_cast<float>(-(alpha + beta));  // camera coordinate
      } else if (xyz[1] < static_cast<double>(pnp_config_.barrel_ptz_offset_y)) {
        beta    = atan(xyz[1] / xyz[2]);
        angle.y = static_cast<float>(-(alpha - beta));
      } else {
        beta    = atan(xyz[1] / xyz[2]);
        angle.y = static_cast<float>((beta - alpha));  // camera coordinate
      }
    } else {
      angle.y = static_cast<float>(atan2(xyz[1], xyz[2]));
    }

    if (pnp_config_.barrel_ptz_offset_x != 0.f) {
      double alpha =
        asin(static_cast<double>(pnp_config_.barrel_ptz_offset_x) /
             sqrt(xyz[0] * xyz[0] + xyz[2] * xyz[2]));
      double beta  = 0.f;

      if (xyz[0] > 0) {
        beta    = atan(-xyz[0] / xyz[2]);
        angle.x = static_cast<float>(-(alpha + beta));  // camera coordinate
      } else if (xyz[0] <
                 static_cast<double>(pnp_config_.barrel_ptz_offset_x)) {
        beta    = atan(xyz[0] / xyz[2]);
        angle.x = static_cast<float>(-(alpha - beta));
      } else {
        beta    = atan(xyz[0] / xyz[2]);
        angle.x = static_cast<float>(beta - alpha);  // camera coordinate
      }
    } else {
      angle.x = static_cast<float>(atan2(xyz[0], xyz[2]));
    }
    angle.z  = static_cast<float>(xyz[2]);
    angle.x  = static_cast<float>(angle.x) * 180 / M_PI;
    angle.y  = static_cast<float>(angle.y) * 180 / M_PI;
    return angle;
  }

 private:
  cv::Mat pnp_config_trackbar_ = cv::Mat::zeros(1, 300, CV_8UC1);

  std::vector<cv::Point3f> reference_Obj_;
  std::vector<cv::Point3f> big_object_3d_;
  std::vector<cv::Point3f> small_object_3d_;
  std::vector<cv::Point3f> buff_object_3d_;

  double theta = 0.0;
  double r_data[9];
  double t_data[3];

  cv::Mat r_camera_ptz;
  cv::Mat t_camera_ptz;
};

}  // namespace abstract_pnp
