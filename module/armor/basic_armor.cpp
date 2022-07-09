/**
 * @file basic_armor.cpp
 * @author XX (2796393320@qq.com)
 * @brief 装甲板识别
 * @date 2021-08-28
 *
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 *
 */
#include "basic_armor.hpp"

namespace basic_armor {

Detector::Detector(const std::string _armor_config) {
  cv::FileStorage fs_armor(_armor_config, cv::FileStorage::READ);
  // 初始化基本参数
  fs_armor["GRAY_EDIT"]          >> image_config_.gray_edit;
  fs_armor["COLOR_EDIT"]         >> image_config_.color_edit;
  fs_armor["METHOD"]             >> image_config_.method;
  fs_armor["BLUE_ARMOR_GRAY_TH"] >> image_config_.blue_armor_gray_th;
  fs_armor["RED_ARMOR_GRAY_TH"]  >> image_config_.red_armor_gray_th;
  if (image_config_.method == 0) {
    fs_armor["RED_ARMOR_COLOR_TH"]   >> image_config_.red_armor_color_th;
    fs_armor["BLUE_ARMOR_COLOR_TH"]  >> image_config_.blue_armor_color_th;
    fs_armor["GREEN_ARMOR_COLOR_TH"] >> image_config_.green_armor_color_th;
    fs_armor["WHILE_ARMOR_COLOR_TH"] >> image_config_.while_armor_color_th;
  } else {
    fs_armor["H_RED_MIN"] >> image_config_.h_red_min;
    fs_armor["H_RED_MAX"] >> image_config_.h_red_max;
    fs_armor["S_RED_MIN"] >> image_config_.s_red_min;
    fs_armor["S_RED_MAX"] >> image_config_.s_red_max;
    fs_armor["V_RED_MIN"] >> image_config_.v_red_min;
    fs_armor["V_RED_MAX"] >> image_config_.v_red_max;

    fs_armor["H_BLUE_MIN"] >> image_config_.h_blue_min;
    fs_armor["H_BLUE_MAX"] >> image_config_.h_blue_max;
    fs_armor["S_BLUE_MIN"] >> image_config_.s_blue_min;
    fs_armor["S_BLUE_MAX"] >> image_config_.s_blue_max;
    fs_armor["V_BLUE_MIN"] >> image_config_.v_blue_min;
    fs_armor["V_BLUE_MAX"] >> image_config_.v_blue_max;
  }

  fs_armor["LIGHT_DRAW"]             >> light_config_.light_draw;
  fs_armor["LIGHT_EDTI"]             >> light_config_.light_edit;

  fs_armor["LIGHT_RATIO_W_H_MIN"]    >> light_config_.ratio_w_h_min;
  fs_armor["LIGHT_RATIO_W_H_MAX"]    >> light_config_.ratio_w_h_max;

  fs_armor["LIGHT_ANGLE_MIN"]        >> light_config_.angle_min;
  fs_armor["LIGHT_ANGLE_MAX"]        >> light_config_.angle_max;

  fs_armor["LIGHT_PERIMETER_MIN"]    >> light_config_.perimeter_min;
  fs_armor["LIGHT_PERIMETER_MAX"]    >> light_config_.perimeter_max;

  fs_armor["ARMOR_EDIT"]             >> armor_config_.armor_edit;
  fs_armor["ARMOR_DRAW"]             >> armor_config_.armor_draw;
  fs_armor["ARMOR_FORECAST"]         >> armor_config_.armor_forecast;
  fs_armor["ARMOR_HEIGHT_RATIO_MIN"] >> armor_config_.light_height_ratio_min;
  fs_armor["ARMOR_HEIGHT_RATIO_MAX"] >> armor_config_.light_height_ratio_max;

  fs_armor["ARMOR_WIDTH_RATIO_MIN"]  >> armor_config_.light_width_ratio_min;
  fs_armor["ARMOR_WIDTH_RATIO_MAX"]  >> armor_config_.light_width_ratio_max;

  fs_armor["ARMOR_Y_DIFFERENT"]      >> armor_config_.light_y_different;
  fs_armor["ARMOR_HEIGHT_DIFFERENT"] >> armor_config_.light_height_different;
  fs_armor["ARMOR_ANGLE_DIFFERENT"]  >> armor_config_.armor_angle_different;

  fs_armor["ARMOR_SMALL_ASPECT_MIN"] >> armor_config_.small_armor_aspect_min;
  fs_armor["ARMOR_TYPE_TH"]          >> armor_config_.armor_type_th;
  fs_armor["ARMOR_BIG_ASPECT_MAX"]   >> armor_config_.big_armor_aspect_max;

  fmt::print("[{}] Info, Armor configuration initial success\n", idntifier_green);
  this->kalman_init();
}

float Detector::getDistance(const cv::Point a, const cv::Point b) {
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

void Detector::freeMemory() {
  lost_armor_success = armor_success;
  armor_success      = false;

  light_.clear();
  light_.shrink_to_fit();
  armor_.clear();
  armor_.shrink_to_fit();
}

bool Detector::findLight() {
  int                                 perimeter = 0;
  cv::RotatedRect                     box;
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(bin_color_img,
                   contours,
                   cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_NONE);

  if (contours.size() < 2) {
    fmt::print("[{}] Info, quantity of contours less than 2\n", idntifier_green);
    return false;
  }
  // 调参开关
  if (light_config_.light_edit == 1) {
    std::string window_name = {"[basic_armor] findLight() -> light_trackbar"};
    cv::namedWindow(window_name);

    cv::createTrackbar("angle_min", window_name,
                       &light_config_.angle_min, 1800, NULL);
    cv::createTrackbar("angle_max", window_name,
                      &light_config_.angle_max, 1800, NULL);

    cv::createTrackbar("perimeter_min", window_name,
                       &light_config_.perimeter_min, 100000, NULL);
    cv::createTrackbar("perimeter_max", window_name,
                       &light_config_.perimeter_max, 100000, NULL);

    cv::createTrackbar("ratio_w_h_min", window_name,
                       &light_config_.ratio_w_h_min, 1000, NULL);
    cv::createTrackbar("ratio_w_h_max", window_name,
                       &light_config_.ratio_w_h_max, 1000, NULL);

    //cv::imshow(window_name, light_trackbar_);
  }

  for (size_t i = 0; i != contours.size(); ++i) {
    perimeter = arcLength(contours[i], true);

    if (perimeter < light_config_.perimeter_min ||
        perimeter > light_config_.perimeter_max ||
        contours[i].size() < 5) {
      continue;
    }

    box = cv::fitEllipse(cv::Mat(contours[i]));

    if (box.angle > 90.0f) {
      box.angle = box.angle - 180.0f;
    }

    static float _h        = MAX(box.size.width, box.size.height);
    static float _w        = MIN(box.size.width, box.size.height);
    static float light_w_h = _h / _w;
    // 判断灯条的条件
    if (box.angle < light_config_.angle_max     &&
        box.angle > -light_config_.angle_min    &&
        light_w_h < light_config_.ratio_w_h_max &&
        light_w_h > light_config_.ratio_w_h_min) {
      light_.emplace_back(box);
      if (light_config_.light_draw == 1 || light_config_.light_edit == 1) {
        cv::Point2f vertex[4];
        box.points(vertex);

        for (size_t l = 0; l != 4; ++l) {
          cv::line(draw_img_,
                   vertex[l], vertex[(l + 1) % 4],
                   cv::Scalar(0, 255, 255), 3, 8);
        }
      }
    }
  }

  if (light_.size() < 2) {
    fmt::print("[{}] Info, quantity of light bar less than two\n", idntifier_green);

    return false;
  }

  return true;
}

bool Detector::runBasicArmor(const cv::Mat& _src_img, const RoboInf& robo_inf) {
  // 预处理
  runImage(_src_img, robo_inf.robot_color.load());
  draw_img_ = _src_img.clone();
  if (findLight()) {
    if (fittingArmor()) {
      finalArmor();
      for (int i = 0; i < armor_.size(); ++i) {
        if (!armor_[i].armor_rect.center.inside(roi_.getRoi(last_armor_rect_, 1.1))) {
          armor_.erase(armor_.begin() + i);
        } else {
          armor_cnt_ = 0;
        }
      }
      if (armor_.size() == 0) {
        if (armor_cnt_ > 10) {
          last_armor_rect_ = cv::RotatedRect(cv::Point(_src_img.cols * 0.5, _src_img.rows * 0.5), cv::Size(_src_img.cols, _src_img.rows), 0);
          armor_cnt_ = 0;
        }
        armor_cnt_++;
        return false;
      }
      last_armor_rect_ = armor_[0].armor_rect;
      lost_cnt_ = 10;
      return true;
    }
  }
  last_armor_rect_ = cv::RotatedRect(cv::Point(_src_img.cols * 0.5, _src_img.rows * 0.5), cv::Size(_src_img.cols, _src_img.rows), 0);
  return false;
  // runImage(_src_img, robo_inf.robot_color.load());
  // draw_img_ = _src_img.clone();
  // cv::line(draw_img_, cv::Point(_src_img.cols * 0.5, 0), cv::Point(_src_img.cols * 0.5, _src_img.rows), cv::Scalar(255, 0, 255));
  // findLightBarContour();
  // if (light_.size() > 0) {
  //   lightBarFilter();
  //   removeWrongArmor();
  //   if (armor_.size() > 0) {
  //     finalArmor();
  //     for (int i = 0; i < armor_.size(); ++i) {
  //       if (!armor_[i].armor_rect.center.inside(roi_.getRoi(last_armor_rect_, 1.1))) {
  //         armor_.erase(armor_.begin() + i);
  //       } else {
  //         armor_cnt_ = 0;
  //       }
  //     }
  //     if (armor_.size() == 0) {
  //       if (armor_cnt_ > 10) {
  //         last_armor_rect_ = cv::RotatedRect(cv::Point(_src_img.cols * 0.5, _src_img.rows * 0.5), cv::Size(_src_img.cols, _src_img.rows), 0);
  //         armor_cnt_ = 0;
  //       }
  //       armor_cnt_++;
  //       return false;
  //     }
  //     last_armor_rect_ = armor_[0].armor_rect;
  //     lost_cnt_ = 10;
  //     return true;
  //   }
  // }
  // last_armor_rect_ = cv::RotatedRect(cv::Point(_src_img.cols * 0.5, _src_img.rows * 0.5), cv::Size(_src_img.cols, _src_img.rows), 0);
  // return false;
}

void Detector::finalArmor() {
  armor_success = true;
  
  if (armor_.size() == 1) {
    fmt::print("[{}] Info, only one armor\n", idntifier_green);
  } else {
    fmt::print("[{}] Info, multiple armors\n", idntifier_green);
    // 离图像中心点大小排序从小到大
    std::sort(armor_.begin(), armor_.end(),
      [](Armor_Data _a, Armor_Data _b) {
      return _a.distance_center < _b.distance_center;
    });
  }
  cv::rectangle(draw_img_, armor_[0].armor_rect.boundingRect(),
                cv::Scalar(255, 255, 255), 3, 10);
}

void Detector::lightBarFilter () {
  float centre_between = 0, scale_max = 9;
    cv::Point2f shortCenter1[2];
    cv::Point2f shortCenter2[2];
    for (size_t i = 0; i < light_.size(); ++i) {
      for (size_t j = i + 1; j < light_.size(); ++j) {
        float maxLength_1 = 0, maxLength_2 = 0, maxLength_between = 0, minLength_between;
        maxLength_1       = MAX(light_[i].size.width, light_[i].size.height);
        maxLength_2       = MAX(light_[j].size.width, light_[j].size.height);
        maxLength_between = maxLength_1 > maxLength_2 ? maxLength_1 : maxLength_2;
        minLength_between = maxLength_1 < maxLength_2 ? maxLength_1 : maxLength_2;
        if (fabs(light_[i].center.x - light_[j].center.x) > maxLength_between ||
            fabs(light_[i].center.y - light_[j].center.y) > minLength_between * 0.3f) {
          //最长的一边也必须在一定的长度范围内,面积也要在一定范围内
          float heightScale = maxLength_1 > maxLength_2 ? (maxLength_1 / maxLength_2) : (maxLength_2 / maxLength_1);
          if (heightScale < 1.4f) {
            centre_between = sqrtf((float)pow(light_[i].center.x - light_[j].center.x, 2) + (float)pow(light_[i].center.y - light_[j].center.y, 2));
            //限制装甲矩形的长宽比
            if (centre_between < maxLength_between * scale_max) {
              getShortCenter(light_[i], shortCenter1);
              getShortCenter(light_[j], shortCenter2);
              float       tan1 = 0.0f, tan2 = 0.0f;
              cv::Point2f vertices1[4];
              cv::Point2f vertices2[4];
              light_[i].points(vertices1);
              light_[j].points(vertices2);
              if (fabs(vertices1[0].x - vertices1[2].x) > fabs(vertices1[1].x - vertices1[3].x)) {
                if (vertices1[0].y > vertices1[2].y) {
                  tan1 = (vertices1[0].y - vertices1[2].y) / vertices1[0].x - vertices1[2].x;
                } else {
                  tan1 = (vertices1[2].y - vertices1[0].y) / vertices1[2].x - vertices1[0].x;
                }
              } else {
                if (vertices1[1].y > vertices1[3].y) {
                  tan1 = (vertices1[1].y - vertices1[3].y) / vertices1[1].x - vertices1[3].x;
                } else {
                  tan1 = (vertices1[3].y - vertices1[1].y) / vertices1[3].x - vertices1[1].x;
                }
              }

              if (fabs((vertices2[0].y - vertices2[2].y) / (vertices2[0].x - vertices2[2].x)) > fabs((vertices2[1].y - vertices2[3].y) / (vertices2[1].x - vertices2[3].x))) {
                if (vertices2[0].y > vertices2[2].y) {
                  tan2 = (vertices2[0].y - vertices2[2].y) / vertices2[0].x - vertices2[2].x;
                } else {
                  tan2 = (vertices2[2].y - vertices2[0].y) / vertices2[2].x - vertices2[0].x;
                }
              } else {
                if (vertices2[1].y > vertices2[3].y) {
                  tan2 = (vertices2[1].y - vertices2[3].y) / vertices2[1].x - vertices2[3].x;
                } else {
                  tan2 = (vertices2[3].y - vertices2[1].y) / vertices2[3].x - vertices2[1].x;
                }
              }
              float lightAngle = lineToLineAngle(shortCenter1[0], shortCenter1[1], shortCenter2[0], shortCenter2[1]);

              if (tan1 * tan2 < 0.0f) {
                continue;
              }
                
              //两个灯条矩形的角度差都不能小于一定值
              if (lightAngle > 13.0f) {
                continue;
              }

              if (calSkewingAngle(light_[i], light_[j], shortCenter1) < 75.0f || calSkewingAngle(light_[i], light_[j], shortCenter2) < 75.0f) {
                continue;
              }

              cv::RotatedRect armor = boundingRRect(light_[i], light_[j]);
              if (fabs(armor.angle) > 10.0f && fabs(armor.angle) < 170.0f){
                continue;
              }


              // 记录数据
              armor_data_.armor_rect = armor;
              if (light_[i].center.x > light_[j].center.x) {
                armor_data_.left_light = light_[j];
                armor_data_.right_light = light_[i];
              } else {
                armor_data_.left_light  = light_[i];
                armor_data_.right_light = light_[j];
              }
              cv::Point armor_center = cv::Point(armor_data_.armor_rect.center.x, armor_data_.armor_rect.center.y);
              // 装甲板保存灯条离中心点的距离
              armor_data_.distance_center = getDistance(armor_center, cv::Point(draw_img_.cols * 0.5, draw_img_.rows * 0.5));
              cv::putText(draw_img_, std::to_string(armor_data_.distance_center), armor_data_.armor_rect.center, 1, 2, cv::Scalar(0, 255, 0));
              armor_data_.lightAngle = lightAngle;
              distinguishArmorType();
              armor_.push_back(armor_data_);
            }
          }
        }
      }
    }
}

//找到发光体轮廓
void Detector::findLightBarContour() {
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i>              hierarchy;
  //找到灯条轮廓
  cv::findContours(bin_color_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  //直接通过轮廓筛选  然后才通过拟合的旋转矩形筛选
  for (int i = 0; i < contours.size(); ++i) {
    if (hierarchy[i][3] == -1) {
      cv::RotatedRect scanRect = cv::minAreaRect(contours[i]);  //检测最小面积的矩形

      cv::Point2f vertices[4];
      scanRect.points(vertices);

      if (fabs(vertices[1].x - vertices[3].x) > fabs(vertices[1].y - vertices[3].y)) continue;
      //std::cout << "scanRect.size.area():" << scanRect.size.area() << std::endl;
      if (scanRect.size.area() < 20){
        continue;
      }
      //rect的高度、和宽度有一个小于板灯的最小高度就直接跳过本次循环
      float longSide  = MAX(scanRect.size.width, scanRect.size.height);
      float shortSide = MIN(scanRect.size.width, scanRect.size.height);
      //std::cout << "longSide:" << longSide << std::endl;
      //std::cout << "shortSide:" << shortSide << std::endl;
      if (longSide > 200 || longSide < 8 || shortSide > 50 || shortSide < 2) {
        continue;
      }
      if (longSide > shortSide * 8.0f || longSide < shortSide * 1.5f) {
        continue;
      }
      cv::rectangle(draw_img_, scanRect.boundingRect(), cv::Scalar(255, 0, 255), 3, 8);
      light_.push_back(scanRect);
    }
  }
}

void Detector::distinguishArmorType() {
  float longSide_1  = MAX(armor_data_.left_light.size.width, armor_data_.left_light.size.height);
  float longSide_2  = MAX(armor_data_.right_light.size.width, armor_data_.right_light.size.height);
  float heightScale = longSide_1 > longSide_2 ? (longSide_1 / longSide_2) : (longSide_2 / longSide_1);
  float longSide = armor_data_.armor_rect.size.height > armor_data_.armor_rect.size.width ? armor_data_.armor_rect.size.height : armor_data_.armor_rect.size.width;
  float shortSide = armor_data_.armor_rect.size.height < armor_data_.armor_rect.size.width ? armor_data_.armor_rect.size.height : armor_data_.armor_rect.size.width;
  armor_data_.aspect_ratio = longSide / shortSide;
  if (armor_data_.aspect_ratio > 3.3f) {
    armor_data_.distinguish = 1;
  } else if (armor_data_.aspect_ratio > 2.6f) {
    if (heightScale > 1.3f) {
      armor_data_.distinguish = 1;
    } else {
      armor_data_.distinguish = 0;
    }
  } else
    armor_data_.distinguish = 0;
}

bool Detector::embeddedRectJudge(cv::RotatedRect r1, cv::RotatedRect r2) {
  double distance = r1.size.width > r1.size.height ? r1.size.width / 2 : r1.size.height / 2;

  return (getDistance(r1.center, r2.center) < distance);
}

void Detector::removeWrongArmor() {
  float centreDistance[2];

  //去除含有内嵌灯条的装甲板
  for (int i = 0; i < armor_.size(); ++i) {
    for (int j = 0; j < light_.size(); ++j) {
      centreDistance[0] = getDistance(armor_[i].right_light.center, light_[j].center);
      centreDistance[1] = getDistance(armor_[i].left_light.center, light_[j].center);
      if (centreDistance[0] && centreDistance[1]) {
        if (embeddedRectJudge(armor_[i].armor_rect, light_[j])) {
          armor_.erase(std::begin(armor_) + i);
          break;
        }
      }
    }
  }

  if (armor_.size() <= 1) {
    return;
  }

  //有装甲板公用同一灯条，去除长宽比大的或夹角大的
  for (int i = 0; i < armor_.size() - 1; ++i) {
    for (int j = i + 1; j < armor_.size(); ++j) {
      centreDistance[0] = getDistance(armor_[i].right_light.center, armor_[j].left_light.center);
      centreDistance[1] = getDistance(armor_[i].left_light.center, armor_[j].right_light.center);
      if (!centreDistance[0]) {
        if (armor_[i].distinguish == true && armor_[j].distinguish == true) {
          if (armor_[i].lightAngle > armor_[j].lightAngle) {
            armor_.erase(std::begin(armor_) + i);
          } else if (armor_[i].lightAngle < armor_[j].lightAngle) {
            armor_.erase(std::begin(armor_) + j);
          } else {
            if (armor_[i].armor_rect.size.width / armor_[i].armor_rect.size.height > armor_[j].armor_rect.size.width / armor_[j].armor_rect.size.height) {
              armor_.erase(std::begin(armor_) + i);
            } else {
              armor_.erase(std::begin(armor_) + j);
            }
          }
        } else {
          if (armor_[i].armor_rect.size.width / armor_[i].armor_rect.size.height > armor_[j].armor_rect.size.width / armor_[j].armor_rect.size.height) {
            armor_.erase(std::begin(armor_) + i);
          } else {
            armor_.erase(std::begin(armor_) + j);
          }
        }
      } else if (!centreDistance[1]) {
        if (armor_[i].distinguish == true && armor_[j].distinguish == true) {
          if (armor_[i].lightAngle > armor_[j].lightAngle) {
            armor_.erase(std::begin(armor_) + i);
          } else if (armor_[i].lightAngle < armor_[j].lightAngle) {
            armor_.erase(std::begin(armor_) + j);
          } else {
            if (armor_[i].armor_rect.size.width / armor_[i].armor_rect.size.height > armor_[j].armor_rect.size.width / armor_[j].armor_rect.size.height) {
              armor_.erase(std::begin(armor_) + i);
            } else {
              armor_.erase(std::begin(armor_) + j);
            }
          }
        } else {
          if (armor_[i].armor_rect.size.width / armor_[i].armor_rect.size.height > armor_[j].armor_rect.size.width / armor_[j].armor_rect.size.height) {
            armor_.erase(std::begin(armor_) + i);
          } else {
            armor_.erase(std::begin(armor_) + j);
          }
        }
      }
    }
  }
}

inline cv::RotatedRect Detector::boundingRRect(const cv::RotatedRect& rect_1, const cv::RotatedRect& rect_2) {
  const cv::Point2f &pl = rect_1.center, &pr = rect_2.center;
  cv::Point2f        center = (pl + pr) / 2.0;
  cv::RotatedRect    wh_1   = rect_1;
  cv::RotatedRect    wh_2   = rect_2;
  float              long_1 = MAX(wh_1.size.width, wh_1.size.height);
  float              long_2 = MAX(wh_2.size.width, wh_2.size.height);
  float              width  = getDistance(pl, pr);
  float              height = (long_2 + long_1) / (float)2.0;
  float              angle  = std::atan2(rect_2.center.y - rect_1.center.y, rect_2.center.x - rect_1.center.x);
  return cv::RotatedRect(center, cv::Size2f(width, height), angle * 180 / CV_PI);
}

float Detector::calSkewingAngle(cv::RotatedRect rect1, cv::RotatedRect rect2, cv::Point2f* shortCenter) {
  if (rect1.size.width <= 0 || rect1.size.height <= 0) {
    return float();
  }
  return lineToLineAngle(rect1.center, rect2.center, shortCenter[0], shortCenter[1]);
}

inline float Detector::lineToLineAngle(cv::Point2f& p1, cv::Point2f& p2, cv::Point2f& p3, cv::Point2f& p4) {
  if (p2.x == p1.x) {
    p2.x += 1e-10f;
  }
  if (p3.x == p4.x) {
    p3.x += 1e-10f;
  }
  float tan1   = (p2.y - p1.y) / (p2.x - p1.x);
  float tan2   = (p4.y - p3.y) / (p4.x - p3.x);
  float angle1 = atanf(tan1) * 180.0f / CV_PI;
  float angle2 = atanf(tan2) * 180.0f / CV_PI;
  float skew   = fabs(fabs(angle1 - angle2) - 90);
  return 90.0f - skew;
}

void Detector::getShortCenter(cv::RotatedRect rect, cv::Point2f* shortCenter) {
  if (rect.size.width <= 0) {
    return;
  }
  cv::Point2f verts[4];
  rect.points(verts);
  if (getDistance(verts[0], verts[1]) < getDistance(verts[1], verts[2])) {
    shortCenter[0] = (verts[0] + verts[1]) / 2;
    shortCenter[1] = (verts[2] + verts[3]) / 2;
  } else {
    shortCenter[0] = (verts[1] + verts[2]) / 2;
    shortCenter[1] = (verts[3] + verts[0]) / 2;
  }
}

bool Detector::fittingArmor() {
  if (armor_config_.armor_edit == 1) {
    std::string window_name = {"[basic_armor] fittingArmor() -> armor_trackbar"};
    cv::namedWindow(window_name);

    cv::createTrackbar("light_height_aspect_min", window_name,
                       &armor_config_.light_height_ratio_min, 100, NULL);
    cv::createTrackbar("light_height_aspect_max", window_name,
                       &armor_config_.light_height_ratio_max, 100, NULL);
    cv::createTrackbar("light_width_aspect_min", window_name,
                       &armor_config_.light_width_ratio_min, 100, NULL);
    cv::createTrackbar("light_width_ratio_max", window_name,
                       &armor_config_.light_width_ratio_max, 100, NULL);
    cv::createTrackbar("light_y_different", window_name,
                       &armor_config_.light_y_different, 100, NULL);
    cv::createTrackbar("light_height_different", window_name,
                       &armor_config_.light_height_different, 100, NULL);
    cv::createTrackbar("armor_angle_different", window_name,
                       &armor_config_.armor_angle_different, 100, NULL);
    cv::createTrackbar("small_armor_aspect_min", window_name,
                       &armor_config_.small_armor_aspect_min, 100, NULL);
    cv::createTrackbar("armor_type_th", window_name,
                       &armor_config_.armor_type_th, 100, NULL);
    cv::createTrackbar("big_armor_aspect_max", window_name,
                       &armor_config_.big_armor_aspect_max, 100, NULL);

    //cv::imshow(window_name, armor_trackbar_);
  }

  for (size_t i = 0; i != light_.size(); ++i) {
    for (size_t j = i + 1; j != light_.size(); ++j) {
      int light_left = 0, light_right = 0;
      // 区分左右灯条
      if (light_[i].center.x > light_[j].center.x) {
        light_left  = j;
        light_right = i;
      } else {
        light_left  = i;
        light_right = j;
      }

      armor_data_.left_light  = light_[light_left];
      armor_data_.right_light = light_[light_right];
      // 装甲板倾斜弧度
      float error_angle =
          atan((light_[light_right].center.y - light_[light_left].center.y) /
               (light_[light_right].center.x - light_[light_left].center.x));
      armor_data_.tan_angle = atan(error_angle) * 180 / CV_PI;
      if (fabs(armor_data_.tan_angle) < 10.f) {
        // 拟合装甲板条件判断
        if (lightJudge(light_left, light_right)) {
          // 装甲板内颜色平均强度
          if (averageColor() <= 255) {
            // 储存装甲板
            armor_.push_back(armor_data_);
            if (armor_config_.armor_draw == 1 ||
                armor_config_.armor_edit == 1) {
              rectangle(draw_img_, armor_data_.armor_rect.boundingRect(),
                        cv::Scalar(255, 255, 0), 5, 8);
            }
          }
        }
      }
    }
  }
  removeWrongArmor();
  if (armor_.size() < 1) {
    fmt::print("[{}] Info, armor not found\n", idntifier_green);

    return false;
  }

  return true;
}

bool Detector::lightJudge(const int i, const int j) {
  armor_data_.left_light_height   =
      MAX(light_[i].size.height, light_[i].size.width);
  armor_data_.left_light_width    =
      MIN(light_[i].size.height, light_[i].size.width);
  armor_data_.right_light_height  =
      MAX(light_[j].size.height, light_[j].size.width);
  armor_data_.right_light_width   =
      MIN(light_[j].size.height, light_[j].size.width);
  armor_data_.light_height_aspect =
      armor_data_.left_light_height / armor_data_.right_light_height;
  armor_data_.light_width_aspect  =
      armor_data_.left_light_width / armor_data_.right_light_width;
  // 左右灯条高宽比
  if (armor_data_.light_height_aspect < armor_config_.light_height_ratio_max * 0.1 &&
      armor_data_.light_height_aspect > armor_config_.light_height_ratio_min * 0.1 &&
      armor_data_.light_width_aspect  < armor_config_.light_width_ratio_max  * 0.1 &&
      armor_data_.light_width_aspect  > armor_config_.light_height_ratio_min * 0.1) {
    armor_data_.height =
      MIN(armor_data_.left_light.size.height, armor_data_.right_light.size.height);
    // 灯条 y 轴位置差
    if (fabs(armor_data_.left_light.center.y - armor_data_.right_light.center.y) <
        armor_data_.height * armor_config_.light_y_different * 0.1) {
      // 灯条高度差
      if (fabs(armor_data_.left_light.size.height - armor_data_.right_light.size.height) <
          armor_data_.height * armor_config_.light_height_different * 0.1) {
        armor_data_.width =
          getDistance(armor_data_.left_light.center,
                      armor_data_.right_light.center);

        armor_data_.aspect_ratio = armor_data_.width / (MAX(armor_data_.left_light.size.height, armor_data_.right_light.size.height));
        // 灯条角度差
        if (fabs(armor_data_.left_light.angle - armor_data_.right_light.angle) <
            armor_config_.armor_angle_different * 0.1) {
          cv::RotatedRect rects = cv::RotatedRect(
              (armor_data_.left_light.center + armor_data_.right_light.center) / 2,
              cv::Size(armor_data_.width * 0.5 , armor_data_.height * 0.5),
              armor_data_.tan_angle);
          
          armor_data_.armor_rect      = rects;
          cv::Point armor_center = cv::Point(armor_data_.armor_rect.center.x, armor_data_.armor_rect.center.y);
                                   // 装甲板保存灯条离中心点的距离
          armor_data_.distance_center = getDistance(
              armor_center, cv::Point(draw_img_.cols * 0.5, draw_img_.rows * 0.5));
          cv::putText(draw_img_, std::to_string(armor_data_.distance_center),
                      armor_data_.armor_rect.center, 1, 2, cv::Scalar(0, 255, 0));
          // std::cout << "armor_data_.aspect_ratio = "
          //           << armor_data_.aspect_ratio << std::endl; 
                       // 小装甲板比例范围
          if (armor_data_.aspect_ratio >
                  armor_config_.small_armor_aspect_min * 0.1 &&
              armor_data_.aspect_ratio <
                  armor_config_.armor_type_th * 0.1) {
            armor_data_.distinguish = 0;

            return true;
          // 大装甲板比例范围
          }
          else if (armor_data_.aspect_ratio >
                       armor_config_.armor_type_th * 0.1 &&
                   armor_data_.aspect_ratio <
                       armor_config_.big_armor_aspect_max * 0.1) {
            armor_data_.distinguish = 1;

            return true;
          }
        }
      }
    }
  }

  return false;
}

int Detector::averageColor() {
  armor_data_.left_light_height  =
      MAX(armor_data_.left_light_height, armor_data_.left_light_width);
  armor_data_.left_light_width   =
      MIN(armor_data_.left_light_height, armor_data_.left_light_width);
  armor_data_.right_light_height =
      MAX(armor_data_.right_light_height, armor_data_.right_light_width);
  armor_data_.right_light_width  =
      MIN(armor_data_.right_light_height, armor_data_.right_light_width);

  cv::RotatedRect rects =
    cv::RotatedRect((armor_data_.left_light.center + armor_data_.right_light.center) / 2,
      cv::Size(armor_data_.width - (armor_data_.left_light_width + armor_data_.right_light_width),
          ((armor_data_.left_light_height + armor_data_.right_light_height) / 2)),
      armor_data_.tan_angle);
  cv::rectangle(draw_img_, rects.boundingRect(), cv::Scalar(255, 0, 0), 3, 8);

  armor_data_.armor_rect = rects;
  cv::Rect _rect         = rects.boundingRect();
  // ROI 安全条件
  if (_rect.x <= 0) {
    _rect.x = 0;
  }
  if (_rect.y <= 0) {
    _rect.y = 0;
  }
  if (_rect.y + _rect.height >= bin_gray_img.rows) {
    _rect.height = bin_gray_img.rows - _rect.y;
  }
  if (_rect.x + _rect.width >= bin_gray_img.cols) {
    _rect.width = bin_gray_img.cols - _rect.x;
  }
  // 计算颜色平均强度
  static cv::Mat roi    = bin_gray_img(_rect);
  int average_intensity = static_cast<int>(mean(roi).val[0]);

  return average_intensity;
}

void Detector::runImage(const cv::Mat& _src_img, const int _my_color) {
  // 选择预处理类型（ BGR / HSV ）
  switch (image_config_.method) {
  case 0:
    bin_color_img = fuseImage(grayPretreat(_src_img, _my_color), 
                              bgrPretreat(_src_img, _my_color), whilePretreat(_src_img));
    break;
  default:
    bin_color_img = fuseImage(grayPretreat(_src_img, _my_color), 
                              hsvPretreat(_src_img, _my_color), whilePretreat(_src_img));
    break;
  }
}

cv::Mat Detector::fuseImage(const cv::Mat _bin_gray_img, const cv::Mat _bin_color_img, const cv::Mat _while_img) {
  cv::bitwise_and(_bin_color_img, _bin_gray_img, _bin_color_img);
  cv::bitwise_and(_bin_color_img, _while_img, _bin_color_img);
  // cv::morphologyEx(_bin_color_img, _bin_color_img, cv::MORPH_OPEN, ele_);
  cv::medianBlur(_bin_color_img, _bin_color_img, 5);
  return _bin_color_img;
}

inline cv::Mat Detector::whilePretreat(const cv::Mat& _src_img) {
  cv::cvtColor(_src_img, gray_while_img_, cv::COLOR_BGR2GRAY);
  cv::threshold(gray_while_img_, while_img_, image_config_.while_armor_color_th, 255, cv::THRESH_BINARY);
  cv::bitwise_not(while_img_, while_img_);
  return while_img_;
}

inline cv::Mat Detector::grayPretreat(const cv::Mat& _src_img,
                                      const int      _my_color) {
  cv::cvtColor(_src_img, gray_img_, cv::COLOR_BGR2GRAY);

  std::string window_name = {"[basic_armor] grayPretreat() -> gray_trackbar"};
  switch (_my_color) {
    case Color::RED:
      // my_color 为红色，则处理蓝色的情况
      if (image_config_.gray_edit) {
        cv::namedWindow(window_name);
        cv::createTrackbar("blue_gray_th", window_name,
                           &image_config_.blue_armor_gray_th, 255, NULL);
        //cv::imshow(window_name, gray_trackbar_);
      }

      cv::threshold(gray_img_, bin_gray_img, image_config_.blue_armor_gray_th,
                    255, cv::THRESH_BINARY);
      break;
    case Color::BLUE:
      // my_color 为蓝色，则处理红色的情况
      if (image_config_.gray_edit) {
        cv::namedWindow(window_name);
        cv::createTrackbar("red_gray_th", window_name,
                           &image_config_.red_armor_gray_th, 255, NULL);
        //cv::imshow(window_name, gray_trackbar_);
      }

      cv::threshold(gray_img_, bin_gray_img, image_config_.red_armor_gray_th,
                    255, cv::THRESH_BINARY);
      break;
    default:
      // my_color 为默认值，则处理红蓝双色的情况
      if (image_config_.gray_edit) {
        cv::namedWindow(window_name);
        cv::createTrackbar("red_gray_th", window_name,
                           &image_config_.red_armor_gray_th, 255, NULL);
        cv::createTrackbar("blue_gray_th", window_name,
                           &image_config_.blue_armor_gray_th, 255, NULL);
        //cv::imshow(window_name, gray_trackbar_);
      }

      cv::threshold(gray_img_, bin_red_gray_img,
                    image_config_.red_armor_gray_th, 255, cv::THRESH_BINARY);
      cv::threshold(gray_img_, bin_blue_gray_img,
                    image_config_.blue_armor_gray_th, 255, cv::THRESH_BINARY);
      cv::bitwise_or(bin_red_gray_img, bin_blue_gray_img, bin_gray_img);
      break;
  }

  if (image_config_.gray_edit) {
    //cv::imshow(window_name, bin_gray_img);
  }

  return bin_gray_img;
}

inline cv::Mat Detector::bgrPretreat(const cv::Mat& _src_img, const int _my_color) {
  static std::vector<cv::Mat> _split;
  static cv::Mat              bin_color_img;
  static cv::Mat              bin_color_green_img;

  cv::split(_src_img, _split);

  std::string window_name = {"[basic_armor] brgPretreat() -> color_trackbar"};
  switch (_my_color) {
  case Color::RED:
    // my_color 为红色，则处理蓝色的情况
    cv::subtract(_split[0], _split[2], bin_color_img);
    cv::subtract(_split[0], _split[1], bin_color_green_img);

    if (image_config_.color_edit) {
      cv::namedWindow(window_name);
      cv::createTrackbar("blue_color_th", window_name,
                         &image_config_.blue_armor_color_th, 255, NULL);
      //cv::imshow(window_name, this->bgr_trackbar_);
    }

    cv::threshold(bin_color_img, bin_color_img, image_config_.blue_armor_color_th, 255, cv::THRESH_BINARY);
    cv::threshold(bin_color_green_img, bin_color_green_img, image_config_.green_armor_color_th, 255, cv::THRESH_BINARY);
    cv::medianBlur(bin_color_green_img, bin_color_green_img, 3);
    cv::morphologyEx(bin_color_img, bin_color_img, cv::MORPH_OPEN, ele_);
    cv::morphologyEx(bin_color_green_img, bin_color_green_img, cv::MORPH_OPEN, ele_);
    // cv::dilate(bin_color_img, bin_color_img, ele_);
    // cv::dilate(bin_color_green_img, bin_color_green_img, ele_);
    cv::bitwise_and(bin_color_green_img, bin_color_img, bin_color_img);
    break;
  case Color::BLUE:
    // my_color 为蓝色，则处理红色的情况
    cv::subtract(_split[2], _split[0], bin_color_img);
    cv::subtract(_split[2], _split[1], bin_color_green_img);

    if (image_config_.color_edit) {
      cv::namedWindow(window_name);
      cv::createTrackbar("red_color_th", window_name,
                         &image_config_.red_armor_color_th, 255, NULL);
      //cv::imshow(window_name, this->bgr_trackbar_);
    }

    cv::threshold(bin_color_img, bin_color_img, image_config_.red_armor_color_th, 255, cv::THRESH_BINARY);
    cv::threshold(bin_color_green_img, bin_color_green_img, image_config_.green_armor_color_th, 255, cv::THRESH_BINARY);
    cv::medianBlur(bin_color_green_img, bin_color_green_img, 3);
    // cv::dilate(bin_color_img, bin_color_img, ele_);
    // cv::dilate(bin_color_green_img, bin_color_green_img, ele_);
    cv::morphologyEx(bin_color_img, bin_color_img, cv::MORPH_OPEN, ele_);
    cv::morphologyEx(bin_color_green_img, bin_color_green_img, cv::MORPH_OPEN, ele_);
    cv::bitwise_and(bin_color_green_img, bin_color_img, bin_color_img);
    break;
  default:
    // my_color 为默认值，则处理红蓝双色的情况
    cv::subtract(_split[2], _split[0], bin_red_color_img);
    cv::subtract(_split[0], _split[2], bin_blue_color_img);
    cv::subtract(_split[2], _split[1], bin_red_green_img);
    cv::subtract(_split[0], _split[1], bin_blue_green_img);

    if (image_config_.color_edit) {
      cv::namedWindow(window_name);
      cv::createTrackbar("red_color_th", window_name,
                         &image_config_.red_armor_color_th, 255, NULL);
      cv::createTrackbar("blue_color_th", window_name,
                         &image_config_.blue_armor_color_th, 255, NULL);
      //cv::imshow(window_name, this->bgr_trackbar_);
    }
    cv::threshold(bin_blue_color_img, bin_blue_color_img, image_config_.blue_armor_color_th, 255, cv::THRESH_BINARY);
    cv::threshold(bin_blue_green_img, bin_blue_green_img, image_config_.green_armor_color_th, 255, cv::THRESH_BINARY);
    cv::threshold(bin_red_color_img, bin_red_color_img, image_config_.red_armor_color_th, 255, cv::THRESH_BINARY);
    cv::threshold(bin_red_green_img, bin_red_green_img, image_config_.green_armor_color_th, 255, cv::THRESH_BINARY);
    cv::bitwise_or(bin_blue_color_img, bin_red_color_img, bin_color_img);
    cv::bitwise_or(bin_blue_green_img, bin_red_green_img, bin_color_green_img);
    cv::dilate(bin_color_green_img, bin_color_green_img, ele_);
    cv::bitwise_and(bin_color_img, bin_color_green_img, bin_color_img);
    break;
  }

  if (image_config_.color_edit) {
    //cv::imshow(window_name, bin_color_img);
  }

  return bin_color_img;
}

inline cv::Mat Detector::hsvPretreat(const cv::Mat& _src_img,
                                     const int      _my_color) {
  cv::cvtColor(_src_img, hsv_img, cv::COLOR_BGR2HSV_FULL);
  std::string window_name = {"[basic_armor] hsvPretreat() -> hsv_trackbar"};
  switch (_my_color) {
    // my_color 为红色，则处理蓝色的情况
    case Color::RED:
      if (image_config_.color_edit) {
        cv::namedWindow(window_name);
        cv::createTrackbar("blue_h_min:", window_name,
                           &image_config_.h_blue_min, 255, NULL);
        cv::createTrackbar("blue_h_max:", window_name,
                           &image_config_.h_blue_max, 255, NULL);
        cv::createTrackbar("blue_s_min:", window_name,
                           &image_config_.s_blue_min, 255, NULL);
        cv::createTrackbar("blue_s_max:", window_name,
                           &image_config_.s_blue_max, 255, NULL);
        cv::createTrackbar("blue_v_min:", window_name,
                           &image_config_.v_blue_min, 255, NULL);
        cv::createTrackbar("blue_v_max:", window_name,
                           &image_config_.v_red_max, 255, NULL);
        //cv::imshow(window_name, this->hsv_trackbar_);
      }

      cv::inRange(hsv_img,
                  cv::Scalar(image_config_.h_blue_min,
                             image_config_.s_blue_min,
                             image_config_.v_blue_min),
                  cv::Scalar(image_config_.h_blue_max,
                             image_config_.s_blue_max,
                             image_config_.v_blue_max),
                  bin_color_img);
      break;
    case Color::BLUE:
      // my_color 为蓝色，则处理红色的情况
      if (image_config_.color_edit) {
        cv::namedWindow("hsv_trackbar");
        cv::createTrackbar("red_h_min:", window_name,
                           &image_config_.h_red_min, 255, NULL);
        cv::createTrackbar("red_h_max:", window_name,
                           &image_config_.h_red_max, 255, NULL);
        cv::createTrackbar("red_s_min:", window_name,
                           &image_config_.s_red_min, 255, NULL);
        cv::createTrackbar("red_s_max:", window_name,
                           &image_config_.s_red_max, 255, NULL);
        cv::createTrackbar("red_v_min:", window_name,
                           &image_config_.v_red_min, 255, NULL);
        cv::createTrackbar("red_v_max:", window_name,
                           &image_config_.v_red_max, 255, NULL);
        //cv::imshow(window_name, hsv_trackbar_);
      }

      cv::inRange(hsv_img,
                  cv::Scalar(image_config_.h_red_min,
                             image_config_.s_red_min,
                             image_config_.v_red_min),
                  cv::Scalar(image_config_.h_red_max,
                             image_config_.s_red_max,
                             image_config_.v_red_max),
                  bin_color_img);
      break;
    default:
      // my_color 为默认值，则处理红蓝双色的情况
      if (image_config_.color_edit) {
        cv::namedWindow("hsv_trackbar");
        cv::createTrackbar("red_h_min:", window_name,
                           &image_config_.h_red_min, 255, NULL);
        cv::createTrackbar("red_h_max:", window_name,
                           &image_config_.h_red_max, 255, NULL);
        cv::createTrackbar("red_s_min:", window_name,
                           &image_config_.s_red_min, 255, NULL);
        cv::createTrackbar("red_s_max:", window_name,
                           &image_config_.s_red_max, 255, NULL);
        cv::createTrackbar("red_v_min:", window_name,
                           &image_config_.v_red_min, 255, NULL);
        cv::createTrackbar("red_v_max:", window_name,
                           &image_config_.v_red_max, 255, NULL);

        cv::createTrackbar("blue_h_min:", window_name,
                           &image_config_.h_blue_min, 255, NULL);
        cv::createTrackbar("blue_h_max:", window_name,
                           &image_config_.h_blue_max, 255, NULL);
        cv::createTrackbar("blue_s_min:", window_name,
                           &image_config_.s_blue_min, 255, NULL);
        cv::createTrackbar("blue_s_max:", window_name,
                           &image_config_.s_blue_max, 255, NULL);
        cv::createTrackbar("blue_v_min:", window_name,
                           &image_config_.v_blue_min, 255, NULL);
        cv::createTrackbar("blue_v_max:", window_name, &image_config_.v_red_max,
                           255, NULL);

        //cv::imshow(window_name, hsv_trackbar_);
      }

      cv::inRange(hsv_img,
                  cv::Scalar(image_config_.h_red_min,
                             image_config_.s_red_min,
                             image_config_.v_red_min),
                  cv::Scalar(image_config_.h_red_max,
                             image_config_.s_red_max,
                             image_config_.v_red_max),
                  bin_red_color_img);

      cv::inRange(hsv_img,
                  cv::Scalar(image_config_.h_blue_min,
                             image_config_.s_blue_min,
                             image_config_.v_blue_min),
                  cv::Scalar(image_config_.h_blue_max,
                             image_config_.s_blue_max,
                             image_config_.v_blue_max),
                  bin_blue_color_img);
      cv::bitwise_or(bin_blue_color_img, bin_red_color_img, bin_color_img);

      break;
  }

  if (image_config_.gray_edit) {
    //cv::imshow(window_name, bin_color_img);
  }

  return bin_color_img;
}
void Detector::kalman_init() { 
  A = _Kalman::Matrix_xxd::Identity();
  H(0, 0) = 1;
  R(0, 0) = 0.01;
  for (int i = 1; i < S; i++) {
    R(i, i) = 100;
  }
  yaw_v = _Kalman(A, H, R, Q, init, 0);
  yaw_acc = _Kalman(A, H, R, Q, init, 0);
  R(0, 0) = 1;
  _Kalman::Matrix_zzd Q_{20};
  top_angle = _Kalman(A, H, R, Q_, init, 0);
}

void Detector::forecast_armor_flag(float time, double yawangle, double last_yawangle) {
  if (std::fabs(last_yawangle - yawangle) > (40. / 180. * M_PI)) {
    yaw_v.reset(yawangle, time);
    last_yawangle = yawangle;
    std::cout << "reset-yaw" << std::endl;
  } else {
    last_yawangle = yawangle;
    Eigen::Matrix<double, 1, 1> z_k{yawangle};
    _Kalman::Matrix_x1d state = yaw_v.update(z_k, time);
    c_speed    = state(1, 0);
  }

  if (std::fabs(c_last_speed - c_speed) > (10. / 180. * M_PI)) {
    yaw_acc.reset(c_speed, time);
    c_last_speed    = c_speed;
  } else {
    c_last_speed    = c_speed;
    Eigen::Matrix<double, 1, 1> z_k{c_speed};
    _Kalman::Matrix_x1d         state = yaw_acc.update(z_k, time);
    c_acc = state(1, 0);
  }
  cv::putText(draw_img_, std::to_string(c_speed), cv::Point(50, 50), 1, cv::FONT_HERSHEY_COMPLEX, cv::Scalar(0, 255, 0), 1, 8, false);
  cv::putText(draw_img_, std::to_string(c_acc), cv::Point(50, 100), 1, cv::FONT_HERSHEY_COMPLEX, cv::Scalar(0, 255, 0), 1, 8, false);
}

std::vector<cv::Point2f> Detector::forecast_armor(float depth, const int bullet_velocity, int num){
  double predict_time = (depth * 0.001 / (bullet_velocity));
  // c_speed -= 1 * c_acc * predict_time;
  double s_yaw        = atan2(predict_time * c_speed * depth * 0.001, 1);
  compensate_w        = 4.5 * tan(s_yaw);
  compensate_w           = (last_last_compensate_w + last_compensate_w + compensate_w) * 0.333;
  last_last_compensate_w = last_compensate_w;
  last_compensate_w      = compensate_w;
  static cv::Point2f ss  = cv::Point2f(0, 0);
  ss = cv::Point2f(-compensate_w, 0);
  std::vector<cv::Point2f> traget_2d = returnFinalArmor4Point(num);
  traget_2d[0] += ss;
  traget_2d[1] += ss;
  traget_2d[2] += ss;
  traget_2d[3] += ss;

  cv::Point2f center = returnFinalArmorRotatedRect(0).center;
  center += ss;
  int width = returnFinalArmorRotatedRect(0).size.width * 1.5;
  cv::circle(draw_img_, center, 15, cv::Scalar(0, 255, 255), -1);
  cv::line(draw_img_, cv::Point(draw_img_.cols * 0.5 - width, 0), cv::Point(draw_img_.cols * 0.5 - width, draw_img_.rows), cv::Scalar(0, 0, 255));
  cv::line(draw_img_, cv::Point(draw_img_.cols * 0.5 + width, 0), cv::Point(draw_img_.cols * 0.5 + width, draw_img_.rows), cv::Scalar(0, 0, 255));
  is_shoot = false;
  if (center.x < draw_img_.cols * 0.5 + width && center.x > draw_img_.cols * 0.5 - width) {
    is_shoot = true;
  }
  return traget_2d;
}

std::vector<cv::Point2f> Detector::outpost_forecast_armor(float depth, const int bullet_velocity, int num) {
  double predict_time = (depth * 0.001 / bullet_velocity);
  // double T = M_PI / (c_speed * depth * 0.001 * 0.026 * 2);
  fixed_time = fabs(90 / ((c_speed * int(depth * 0.001) * 1000) / 26));
  fixed_time = (last_last_fixed_time + last_fixed_time + fixed_time) * 0.333;
  last_last_fixed_time = last_fixed_time;
  last_fixed_time = fixed_time;

  // while (fixed_time < predict_time) {
  //   fixed_time += fabs(90 / ((c_speed * int(depth * 0.001) * 1000) / 26));
  // }
  double delta_time = fixed_time - predict_time;
  // std::cout << "转动时间 = " << fixed_time << std::endl;
  // std::cout << "子弹飞行时间 = " << predict_time << std::endl;
  // std::cout << "反向预测时间 = " << delta_time << std::endl;
  // if (fixed_time > 1) {
  //   delta_time = -predict_time;
  // }

  double s_yaw = atan2(delta_time * c_speed * depth * 0.001, 1);
  compensate_w = 8 * tan(s_yaw);
  compensate_w           = (last_last_compensate_w + last_compensate_w + compensate_w) * 0.333;
  last_last_compensate_w = last_compensate_w;
  last_compensate_w      = compensate_w;
  static cv::Point2f ss = cv::Point2f(0, 0);
  ss = cv::Point2f(compensate_w, 0);
  std::vector<cv::Point2f> traget_2d = returnFinalArmor4Point(num);
  traget_2d[0] += ss;
  traget_2d[1] += ss;
  traget_2d[2] += ss;
  traget_2d[3] += ss;
  return traget_2d;
}

std::vector<cv::Point2f> Detector::top_forecast_armor(float depth, const int bullet_velocity){
  double predict_time = (depth * 0.001 / bullet_velocity);
  double s_yaw        = atan2(predict_time * c_speed * depth * 0.001, 1);
  // std::cout << "s_yaw=" << s_yaw << std::endl;
  compensate_w           = 8 * tan(s_yaw);
  compensate_w           = (last_last_compensate_w + last_compensate_w + compensate_w) * 0.333;
  last_last_compensate_w = last_compensate_w;
  last_compensate_w      = compensate_w;
  static cv::Point2f ss  = cv::Point2f(0, 0);
  ss                     = cv::Point2f(compensate_w, 0);
  // std::cout << "compensate_w=" << compensate_w << std::endl;
  std::vector<cv::Point2f> traget_2d = returnFinalArmor4Point(0);
  traget_2d[0] += ss;
  traget_2d[1] += ss;
  traget_2d[2] += ss;
  traget_2d[3] += ss;
  return traget_2d;
}

cv::Rect2f Detector::get_armor_fake_Roi(const float coefficient) {
  cv::Point2f center  = armor_[0].armor_rect.center;
  float       width_  = armor_[0].armor_rect.size.width;
  float       height_ = armor_[0].armor_rect.size.height;
  _roi                = cv::Rect2f(center.x - width_ * 0.8, center.y - height_ * 0.8, width_ * coefficient, height_ * coefficient);
  return _roi;
}

bool Detector::is_armor_inside_Roi(cv::Rect2f Rect_, cv::Point2f center_point) {
  bool x_in = (center_point.x > Rect_.tl().x && center_point.x < Rect_.br().x);
  bool y_in = (center_point.y > Rect_.tl().y && center_point.y < Rect_.br().y);
  if (x_in && y_in) {
    return true;
  } else {
    return false;
  }
}

int Detector::fake_roi_judge() {
  int _armorNum;
  if (last_armor_flag && !armor_flag) {
    lost_flag = true;
  }
  if (lost_flag) {
    exist_times++;
  }
  if (exist_times < 20 && lost_flag) {
    _armorNum = armor_.size() + 1;
  } else {
    _armorNum   = armor_.size();
    exist_times = 0;
    lost_flag   = false;
  }
  last_armor_flag = armor_flag;
  return _armorNum;
}

// 小陀螺自动击打
bool Detector::topAutoShoot(const float time, float angle, const int depth, const int bullet_velocity) {
  static double last_angle  = 0.0;
  static double speed       = 0.0;
  static double T           = 0.0;
  static double bullet_time = float(depth) * 0.001 / bullet_velocity;
  angle += 180;
  if (std::fabs(last_angle - angle) > 1) {
    top_angle.reset(angle, time);
    last_angle = angle;
    std::cout << "reset-yaw" << std::endl;
  }
  else {
    last_angle = angle;
    Eigen::Matrix<double, 1, 1> z_k{angle};
    _Kalman::Matrix_x1d         state = top_angle.update(z_k, time);
    speed                             = state(1, 0) * depth;
  }
  T = fabs(90 / (speed * 0.026));
  cv::putText(draw_img_, std::to_string(T), cv::Point(50, 100), 1, 2, cv::Scalar(0, 255, 0));
  int i = 0;
  while (fabs(bullet_time) > 0.5 * fabs(T)) {
    T += fabs(90 / (speed * 0.026));
    ++i;
    if (i > 4) {
      break;
    }
  }
  float diff_time = T - bullet_time;
  cv::putText(draw_img_, std::to_string(diff_time), cv::Point(50, 50), 1, 2, cv::Scalar(0, 255, 0));
  
  cv::putText(draw_img_, std::to_string(bullet_time), cv::Point(50, 150), 1, 2, cv::Scalar(0, 255, 0));
  double s_yaw = atan2(diff_time * speed * 0.001, 1);

  int              forcast = 6 * tan(s_yaw);
  static cv::Point ss      = cv::Point2f(0, 0);
  ss                       = cv::Point2f(forcast, 0);
  cv::Point center = returnFinalArmorRotatedRect(0).center;
  center += ss;
  int width = returnFinalArmorRotatedRect(0).size.width * 1.3;
  cv::circle(draw_img_, center, 15, cv::Scalar(0, 255, 255), -1);
  cv::line(draw_img_, cv::Point(draw_img_.cols * 0.5  - width, 0), cv::Point(draw_img_.cols * 0.5  - width, draw_img_.rows), cv::Scalar(0, 0, 255));
  cv::line(draw_img_, cv::Point(draw_img_.cols * 0.5  + width, 0), cv::Point(draw_img_.cols * 0.5  + width, draw_img_.rows), cv::Scalar(0, 0, 255));
  if (center.x < draw_img_.cols * 0.5 + width && center.x > draw_img_.cols * 0.5 - width) { return true; }
  return false;
}
}  // namespace basic_armor
