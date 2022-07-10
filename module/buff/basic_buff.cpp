#include "basic_buff.hpp"

namespace basic_buff {

Detector::Detector(const std::string& _buff_config_path) {
  cv::FileStorage fs_buff(_buff_config_path, cv::FileStorage::READ);
  // 初始化基本参数
  fs_buff["GRAY_EDIT"] >> image_config_.gray_edit;
  fs_buff["COLOR_EDIT"] >> image_config_.color_edit;
  fs_buff["METHOD"] >> image_config_.method;
  fs_buff["BLUE_ARMOR_GRAY_TH"] >> image_config_.blue_armor_gray_th;
  fs_buff["RED_ARMOR_GRAY_TH"] >> image_config_.red_armor_gray_th;
  if (image_config_.method == 0) {
    fs_buff["RED_ARMOR_COLOR_TH"] >> image_config_.red_armor_color_th;
    fs_buff["BLUE_ARMOR_COLOR_TH"] >> image_config_.blue_armor_color_th;
    fs_buff["GREEN_ARMOR_COLOR_TH"] >> image_config_.green_armor_color_th;
    fs_buff["WHILE_ARMOR_COLOR_TH"] >> image_config_.while_armor_color_th;
  } else {
    fs_buff["H_RED_MIN"] >> image_config_.h_red_min;
    fs_buff["H_RED_MAX"] >> image_config_.h_red_max;
    fs_buff["S_RED_MIN"] >> image_config_.s_red_min;
    fs_buff["S_RED_MAX"] >> image_config_.s_red_max;
    fs_buff["V_RED_MIN"] >> image_config_.v_red_min;
    fs_buff["V_RED_MAX"] >> image_config_.v_red_max;

    fs_buff["H_BLUE_MIN"] >> image_config_.h_blue_min;
    fs_buff["H_BLUE_MAX"] >> image_config_.h_blue_max;
    fs_buff["S_BLUE_MIN"] >> image_config_.s_blue_min;
    fs_buff["S_BLUE_MAX"] >> image_config_.s_blue_max;
    fs_buff["V_BLUE_MIN"] >> image_config_.v_blue_min;
    fs_buff["V_BLUE_MAX"] >> image_config_.v_blue_max;
  }
  // area
  fs_buff["SMALL_TARGET_AREA_MAX"] >> buff_param_.SMALL_TARGET_AREA_MAX;
  fs_buff["SMALL_TARGET_AREA_MIN"] >> buff_param_.SMALL_TARGET_AREA_MIN;
  fs_buff["BIG_TARGET_AREA_MAX"] >> buff_param_.BIG_TARGET_AREA_MAX;
  fs_buff["BIG_TARGET_AREA_MIN"] >> buff_param_.BIG_TARGET_AREA_MIN;

  // length
  fs_buff["SMALL_TARGET_Length_MAX"] >> buff_param_.SMALL_TARGET_Length_MAX;
  fs_buff["SMALL_TARGET_Length_MIN"] >> buff_param_.SMALL_TARGET_Length_MIN;
  fs_buff["BIG_TARGET_Length_MAX"] >> buff_param_.BIG_TARGET_Length_MAX;
  fs_buff["BIG_TARGET_Length_MIN"] >> buff_param_.BIG_TARGET_Length_MIN;

  // diff_angle
  fs_buff["DIFF_ANGLE_MAX"] >> buff_param_.DIFF_ANGLE_MAX;
  fs_buff["DIFF_ANGLE_MIN"] >> buff_param_.DIFF_ANGLE_MIN;

  // aspect_ratio
  fs_buff["SMALL_TARGET_ASPECT_RATIO_MAX"] >> buff_param_.SMALL_TARGET_ASPECT_RATIO_MAX;
  fs_buff["SMALL_TARGET_ASPECT_RATIO_MIN"] >> buff_param_.SMALL_TARGET_ASPECT_RATIO_MIN;

  // area_ratio
  fs_buff["AREA_RATIO_MAX"] >> buff_param_.AREA_RATIO_MAX;
  fs_buff["AREA_RATIO_MIN"] >> buff_param_.AREA_RATIO_MIN;

  // center_r
  fs_buff["BIG_LENTH_R"] >> buff_param_.BIG_LENTH_R;

  fs_buff["CENTER_R_ROI_SIZE"] >> buff_param_.CENTER_R_ROI_SIZE;

  fs_buff["CENTER_R_ASPECT_RATIO_MIN"] >> buff_param_.CENTER_R_ASPECT_RATIO_MIN;
  fs_buff["CENTER_R_ASPECT_RATIO_MAX"] >> buff_param_.CENTER_R_ASPECT_RATIO_MAX;
  buff_param_.CENTER_R_ASPECT_RATIO_MIN *= 0.01;
  buff_param_.CENTER_R_ASPECT_RATIO_MAX *= 0.01;

  fs_buff["CENTER_R_AREA_MIN"] >> buff_param_.CENTER_R_AREA_MIN;
  fs_buff["CENTER_R_AREA_MAX"] >> buff_param_.CENTER_R_AREA_MAX;

  // filter coefficient
  fs_buff["FILTER_COEFFICIENT"] >> buff_param_.FILTER_COEFFICIENT;

  // 能量机关打击模型参数
  fs_buff["BUFF_H"] >> buff_param_.BUFF_H;
  fs_buff["BUFF_RADIUS"] >> buff_param_.BUFF_RADIUS;
  fs_buff["PLATFORM_H"] >> buff_param_.PLATFORM_H;
  fs_buff["BARREL_ROBOT_H"] >> buff_param_.BARREL_ROBOT_H;
  fs_buff["TARGET_X"] >> buff_param_.TARGET_X;

  // 固定预测值的补偿
  fs_buff["OFFSET_FIXED_RADIAN"] >> buff_param_.OFFSET_FIXED_RADIAN;

  // 模型深度补偿（左半边比右半边距离要远）
  fs_buff["OFFSET_TARGET_Z"] >> buff_param_.OFFSET_TARGET_Z;

  // yaw 和 pitch 轴弹道补偿
  fs_buff["OFFSET_ARMOR_YAW"] >> buff_param_.OFFSET_ARMOR_YAW;
  fs_buff["OFFSET_ARMOR_PITCH"] >> buff_param_.OFFSET_ARMOR_PITCH;
  buff_param_.OFFSET_ARMOR_YAW *= 0.01;
  buff_param_.OFFSET_ARMOR_PITCH *= 0.01;

  // 手算pitch 轴弹道补偿
  fs_buff["OFFSET_MANUAL_ARMOR_PITCH"] >> buff_param_.OFFSET_MANUAL_ARMOR_PITCH;
  buff_param_.OFFSET_MANUAL_ARMOR_PITCH *= 0.01;
  kalman_init();
  // 输出提示
  fmt::print("✔️ ✔️ ✔️ 🌈 能量机关初始化参数 读取成功 🌈 ✔️ ✔️ ✔️\n");
}

void Detector::runTask(cv::Mat& _input_img, const RoboInf& _receive_info, RoboCmd& _send_info, float _time) {
  // 获取基本信息
  getInput(_input_img);

  time_ = _time;

  // 预处理
  runImage(src_img_, _receive_info.robot_color.load());

  // 查找目标
  findTarget(_input_img, bin_color_img, target_box_);

  // 判断目标是否为空
  is_find_target_ = isFindTarget(_input_img, target_box_);

  // 查找圆心
  final_center_r_ = findCircleR(src_img_, bin_color_img, _input_img, is_find_target_);

  // 计算运转状态值：速度、方向、角度
  judgeCondition(is_find_target_);
  // 预测速度 and acc
  current_speed_ = forecastFlagV(_time, current_angle_);
  current_acc_   = forecastFlagA(_time, current_speed_);
  if (fabs(current_speed_) < 0.5) {
    current_direction_ = 0;
    current_speed_ *= 0.3;
  }
  fmt::print("[{}] 当前风车转速为: {} rad/s 当前风车加速度为 :{} \n", judgement_yellow, current_speed_, current_acc_);
  
  // 计算预测量 单位为弧度
  final_forecast_quantity_ = doPredict(static_cast<float>(_receive_info.bullet_velocity), is_find_target_);

  // 计算获取最终目标（矩形、顶点）
  calculateTargetPointSet(final_forecast_quantity_, final_center_r_, target_2d_point_, _input_img, is_find_target_);

  // 计算云台角度
  if (is_find_target_) {
    // 计算云台角度
    buff_pnp_.solvePnP(_receive_info.bullet_velocity.load(), 2, target_2d_point_, 7);
    _send_info.yaw_angle.store(-(buff_pnp_.returnYawAngle() - buff_param_.OFFSET_ARMOR_YAW));
    _send_info.pitch_angle.store(buff_pnp_.returnPitchAngle() - buff_param_.OFFSET_ARMOR_PITCH);
    _send_info.depth.store(7);
    _send_info.data_type.store(is_find_target_);
    shoot_time_ = std::chrono::system_clock::now();
    shoot_interval_ = static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(shoot_time_ - last_shoot_time_).count() * 0.001);
    if (fabs(_send_info.yaw_angle.load()) < 0.1 && fabs(_send_info.pitch_angle.load()) < 0.1 && shoot_interval_ > 0.5) {
      _send_info.auto_shoot.store(1);
      last_shoot_time_ = std::chrono::system_clock::now();
    } else {
      _send_info.auto_shoot.store(0);
    }
    
    fmt::print("[{}] Info, yaw: {}, pitch: {}, depth: {}\n", idntifier_yellow, _send_info.yaw_angle, _send_info.pitch_angle, _send_info.depth);
  } else {
    _send_info.yaw_angle.store(0);
    _send_info.depth.store(0);
    _send_info.pitch_angle.store(0);
    _send_info.data_type.store(0);
    _send_info.auto_shoot.store(0);
  }
  // 更新上一帧数据
  updateLastData(is_find_target_);
}

cv::Point2f Detector::angleCalculation(const cv::Point2f& _target_center, const float& _unit_pixel_length, const cv::Size& _image_size, const float& _focal_length) {
  cv::Point2f angle2f;

  float target_projection_x = fabs(_image_size.width * 0.5 - _target_center.x) * _unit_pixel_length;
  angle2f.x                 = atan2(target_projection_x, _focal_length) * 180 / CV_PI;
  if (_target_center.x <= (_image_size.width * 0.5)) {
    angle2f.x = -1 * angle2f.x;
  }

  float target_projection_y = fabs(_image_size.height * 0.5 - _target_center.y) * _unit_pixel_length;
  angle2f.y                 = atan2(target_projection_y, _focal_length) * 180 / CV_PI;
  if (_target_center.y <= (_image_size.height * 0.5)) {
    angle2f.y = -1 * angle2f.y;
  }

  angle2f.y += buff_param_.OFFSET_MANUAL_ARMOR_PITCH;

  return angle2f;
}

void Detector::kalman_init() {
  A       = _Kalman::Matrix_xxd::Identity();
  R       = _Kalman::Matrix_xxd::Identity();
  H(0, 0) = 1;
  R(0, 0) = 0.01;
  for (int i = 1; i < S; i++) {
    R(i, i) = 100;
  }
  forecast_v = _Kalman(A, H, R, Q, init, 0);
  forecast_a = _Kalman(A, H, R, Q, init, 0);
}

double Detector::forecastFlagV(float time, double angle) {
  static double last_angle = 0.f;
  static double c_speed    = 0.f;
  if (std::fabs(last_angle - angle) > 10) {
    forecast_v.reset(angle, time);
    last_angle = angle;
    std::cout << "reset-yaw" << std::endl;
  } else {
    last_angle = angle;
    Eigen::Matrix<double, 1, 1> ck{angle};
    _Kalman::Matrix_x1d         state = forecast_v.update(ck, time);
    c_speed                           = state(1, 0) * CV_PI / 180;
    // std::cout << "c_speed==========================" << c_speed << std::endl;
  }
  return c_speed;
}

double Detector::forecastFlagA(float time, double V) {
  static double last_V = 0.0;
  double        acc    = 0.f;
  if (std::fabs(last_V - V) > 10 * CV_PI / 180) {
    forecast_a.reset(V, time);
    last_V = V;
    std::cout << "reset-yaw" << std::endl;
  } else {
    last_V = V;
    Eigen::Matrix<double, 1, 1> ck{V};
    _Kalman::Matrix_x1d         state = forecast_a.update(ck, time);
    acc                               = state(1, 0);
  }
  return acc;
}

void Detector::updateLastData(const bool& _is_find_target) {
  if (!(_is_find_target)) {
    fmt::print("[{}] 没有目标，不需要更新上一帧数据 XXX\n", idntifier_yellow);
    is_find_last_target_ = _is_find_target;
    target_2d_point_.clear();
    std::vector<cv::Point2f>(target_2d_point_).swap(target_2d_point_);
    target_rect_ = cv::RotatedRect();
    return;
  }

  last_target_         = current_target_;
  last_angle_          = current_angle_;
  is_find_last_target_ = _is_find_target;
  target_2d_point_.clear();
  std::vector<cv::Point2f>(target_2d_point_).swap(target_2d_point_);
  target_rect_ = cv::RotatedRect();

  fmt::print("[{}] 发现目标，已更新上一帧数据 √√√\n", idntifier_yellow);
}

void Detector::calculateTargetPointSet(
  const float& _predict_quantity, const cv::Point2f& _final_center_r, std::vector<cv::Point2f>& _target_2d_point, cv::Mat& _input_dst_img, const bool& _is_find_target) {
  // 判断有无目标，若无则重置参数并提前退出
  if (!(_is_find_target)) {
    _target_2d_point.clear();
    _target_2d_point = std::vector<cv::Point2f>(4, cv::Point2f(0.f, 0.f));
    // 重置参数
    return;
  }

  // 计算theta
  theta_ = current_radian_;

  if (theta_ < 0) {
    theta_ += (2 * CV_PI);
  }

  // 计算最终角度和弧度
  // final_radian_ = theta_ + final_direction_ * _predict_quantity;
  final_radian_ = theta_ + _predict_quantity;

  final_angle_ = final_radian_ * 180 / CV_PI;

  // 计算sin和cos
  sin_calcu_ = sin(final_radian_);
  cos_calcu_ = cos(final_radian_);

  // 计算最终坐标点
  radio_        = abstract_object::centerDistance(_final_center_r, current_target_.Armor().Rect().center) + 3;
  pre_center_.x = radio_ * cos_calcu_ + _final_center_r.x;
  pre_center_.y = radio_ * sin_calcu_ + _final_center_r.y;

  // 计算最终目标的旋转矩形
  target_rect_ = cv::RotatedRect(pre_center_, current_target_.Armor().Rect().size, current_target_.Armor().Rect().angle);

  // 通过模型计算最终目标点的位置信息（预测点）TODO:待优化
  // 计算能量机关的高度
  target_buff_h_ = 800 + sin(final_radian_ - CV_PI) * 800;
  target_y_      = target_buff_h_ + barrel_buff_botton_h_;

  // 计算能量机关目标的直线距离
  final_target_z_ = sqrt((target_y_ * target_y_) + (target_x_ * target_x_)) * 0.001;
  // 通过模型计算最终目标点的位置信息（预测点）

  // 保存最终目标的顶点，暂时用的是排序点的返序存入才比较稳定，正确使用应为0123, pnp内已进行反向放置
  _target_2d_point.clear();
  cv::Point2f target_vertex[4];
  target_rect_.points(target_vertex);

  // _target_2d_point.push_back(target_vertex[3]);
  // _target_2d_point.push_back(target_vertex[2]);
  // _target_2d_point.push_back(target_vertex[1]);
  // _target_2d_point.push_back(target_vertex[0]);

  _target_2d_point.push_back(target_vertex[0]);
  _target_2d_point.push_back(target_vertex[1]);
  _target_2d_point.push_back(target_vertex[2]);
  _target_2d_point.push_back(target_vertex[3]);

  // #ifndef RELEASE
  // 绘制图像
  // 最终目标装甲板（预测值）
  for (int k = 0; k < 4; ++k) {
    cv::line(_input_dst_img, _target_2d_point[k], _target_2d_point[(k + 1) % 4], cv::Scalar(0, 130, 255),
             8);  // orange
  }

  cv::circle(_input_dst_img, _final_center_r, radio_, cv::Scalar(0, 255, 125), 2, 8, 0);                       // 轨迹圆
  cv::circle(_input_dst_img, pre_center_, 3, cv::Scalar(255, 0, 0), 3, 8, 0);                                  // 预测值的中点
  cv::line(_input_dst_img, pre_center_, _final_center_r, cv::Scalar(0, 255, 255), 2);                          // 预测点和圆心的连线
  cv::line(_input_dst_img, current_target_.Armor().Rect().center, _final_center_r, cv::Scalar(0, 255, 0), 2);  // 装甲板和圆心的连线

  // 顺时针表示顶点顺序,红黄蓝绿
  cv::circle(_input_dst_img, _target_2d_point[0], 10, cv::Scalar(0, 0, 255), -1, 8, 0);
  cv::circle(_input_dst_img, _target_2d_point[1], 10, cv::Scalar(0, 255, 255), -1, 8, 0);
  cv::circle(_input_dst_img, _target_2d_point[2], 10, cv::Scalar(255, 0, 0), -1, 8, 0);
  cv::circle(_input_dst_img, _target_2d_point[3], 10, cv::Scalar(0, 255, 0), -1, 8, 0);
  // 绘制图像
// #endif  // RELEASE
}

float Detector::doPredict(const float& _bullet_velocity, const bool& _is_find_target) {
  // 判断是否发现目标，没有返回0，有则进行计算预测
  if (!(_is_find_target)) {
    target_z_ = 0.f;
    // 重置参数
    return 0.f;
  }

  float predict_quantity = 0.f;

  // 计算固定预测量 原来是给0.35弧度 TODO(fqjun) :测一下最快和最慢速度时的提前量，以确定范围 predict_quantity = fixedPredict(_bullet_velocity*1000);
  predict_quantity = fixedPredict(_bullet_velocity * 1000);  // 默认先给28m/s

  // 计算移动预测量

  fmt::print("[{}] Info, 提前了: {} 度 \n", predict_yellow, predict_quantity * 180 / CV_PI);

  return predict_quantity;
}

float Detector::fixedPredict(const float& _bullet_velocity) {
  // 转换为弧度
  current_radian_ = current_angle_ * CV_PI / 180;

  // 通过模型计算当前目标点的位置信息（原扇叶）
  // 计算能量机关的高度
  target_buff_h_ = 800 + sin(current_radian_ - CV_PI) * 800;

  target_y_ = target_buff_h_ + barrel_buff_botton_h_;

  // 计算能量机关的水平直线距离
  target_x_ = buff_param_.TARGET_X;

  // 计算能量机关目标的直线距离
  target_z_ = sqrt((target_y_ * target_y_) + (target_x_ * target_x_));
  // 通过模型计算当前目标点的位置信息（原扇叶）

  // 计算子弹飞行时间
  bullet_tof_ = (target_z_ + buff_param_.OFFSET_TARGET_Z) / _bullet_velocity + buff_param_.OFFSET_FIXED_RADIAN;

  // current_speed_ = current_acc_ * bull;

  // 计算固定提前量（也可以直接给定）
  if (current_direction_ != 0) {
  fixed_forecast_quantity_ = current_speed_ * bullet_tof_ + 0.5 * current_acc_ * bullet_tof_ * bullet_tof_;
  } else {
    fixed_forecast_quantity_ = 0.f;
  }

  return fixed_forecast_quantity_;
}

void Detector::judgeCondition(const bool& _is_find_target) {
  if (!(_is_find_target)) {
    // 没有目标，角度为上一帧的角度，方向重置为零，速度为0
    current_angle_ = last_target_.Angle();
    diff_angle_    = 0.f;
    current_speed_ = 0.f;
    current_acc_   = 0.f;
    return;
  }

  // 计算角度
  calAngle();

  // 计算方向
  calDirection();

  // 计算速度
  calVelocity();

  return;
}

void Detector::calAngle() {
  // 装甲板到圆心的连线所代表的角度
  current_angle_ = atan2((current_target_.Armor().Rect().center.y - final_center_r_.y), (current_target_.Armor().Rect().center.x - final_center_r_.x)) * 180 / static_cast<float>(CV_PI);

  // 过零处理
  if (current_angle_ < 0.f) {
    current_angle_ += 360.f;
  }

  // 角度差
  diff_angle_ = current_angle_ - last_angle_;

  // 过零处理
  if (diff_angle_ > 180) {
    diff_angle_ -= 360;
  } else if (diff_angle_ < -180) {
    diff_angle_ += 360;
  }

  fmt::print("[{}] 当前角度差为: {} 度\n", judgement_yellow, diff_angle_);

  if (fabs(diff_angle_) > 30.f) {
    is_change_blade_ = true;
    diff_angle_      = 0.f;
  } else {
    is_change_blade_ = false;
  }
  // TODO(fqjun) :当变化量大于 30°时，则是切换装甲板，则重置 diff 为 0，last 为当前。
}

void Detector::calDirection() {
  ++find_cnt_;

  if (find_cnt_ % 2 == 0) {
    current_direction_ = getState();
    filter_direction_  = (1 - 0.01) * last_direction_ + 0.01 * current_direction_;
    last_direction_    = filter_direction_;
  }

  if (find_cnt_ == 10) {
    find_cnt_ = 0;
  }

  // 显示当前转动信息
  if (filter_direction_ > 0.1) {
    fmt::print("[{}] 转动方向:顺时针转动\n", judgement_yellow);
    final_direction_      = 1;
    last_final_direction_ = final_direction_;
  } else if (filter_direction_ < -0.1) {
    fmt::print("[{}] 转动方向:逆时针转动\n", judgement_yellow);
    final_direction_      = -1;
    last_final_direction_ = final_direction_;
  } else {
    fmt::print("[{}] 转动方向:不转动\n", judgement_yellow);
    final_direction_ = last_final_direction_;
  }
}

int Detector::getState() {
  if (fabs(diff_angle_) < 10 && fabs(diff_angle_) > 1e-6) {
    d_angle_ = (1 - buff_param_.FILTER_COEFFICIENT) * d_angle_ + buff_param_.FILTER_COEFFICIENT * diff_angle_;
  }
  if (d_angle_ > 0) {
    return 1;
  } else if (d_angle_ < -0) {
    return -1;
  } else {
    return 0;
  }
}

void Detector::calVelocity() {
  // double current_time       = buff_fps_.lastTime() + last_time_ + last_last_time_;
  double current_time = (time_ - last_time_) * 2.5;
  last_time_ = time_;
  float  current_diff_angle = diff_angle_ + last_diff_angle_ + last_last_diff_angle_;
  std::cout << "current_time = " << current_time << std::endl;
  // 默认单位为角度/s
  if (current_time == 0) {
    current_speed_ = 0.f;
  } else {
    // 将单位转为rad/s
    current_speed_ = fabs(current_diff_angle / current_time * CV_PI / 180);
  }
  // last_last_time_       = last_time_;
  // last_time_            = buff_fps_.lastTime();
  last_last_diff_angle_ = last_diff_angle_;
  last_diff_angle_      = diff_angle_;

  // fmt::print("[{}] 当前风车转速为: {} rad/s \n", judgement_yellow, current_speed_);
}

cv::Point2f Detector::findCircleR(cv::Mat& _input_src_img, cv::Mat& _input_bin_img, cv::Mat& _dst_img, const bool& _is_find_target) {
  // 更新图像
  _input_src_img.copyTo(roi_img_);
  _input_bin_img.copyTo(result_img_);

  cv::Point2f center_r_point2f = cv::Point2f(0.f, 0.f);

  // 若没有扇叶目标则提前退出
  if (!(_is_find_target)) {
    is_circle_        = false;
    roi_local_center_ = cv::Point2f(0.f, 0.f);

    // 清理容器
    center_r_box_.clear();
    std::vector<abstract_center_r::Center_R>(center_r_box_).swap(center_r_box_);
    return center_r_point2f;
  }

  // 计算圆心大概位置
  delta_height_point_ = current_target_.deltaPoint();
  roi_global_center_  = current_target_.Armor().Rect().center - buff_param_.BIG_LENTH_R * delta_height_point_;

  // roi中心安全条件
  if (roi_global_center_.x < 0 || roi_global_center_.y < 0 || roi_global_center_.x > _input_src_img.cols || roi_global_center_.y > _input_src_img.rows) {
    if (roi_global_center_.x < 0) {
      roi_global_center_.x = 1;
    }
    if (roi_global_center_.y < 0) {
      roi_global_center_.y = 1;
    }
    if (roi_global_center_.x > _input_src_img.cols) {
      roi_global_center_.x = _input_src_img.cols - 1;
    }
    if (roi_global_center_.y > _input_src_img.rows - 1) {
      roi_global_center_.y = _input_src_img.rows - 1;
    }
  }

  // 画出假定圆心的roi矩形
  cv::RotatedRect roi_R(roi_global_center_, cv::Size(buff_param_.CENTER_R_ROI_SIZE, buff_param_.CENTER_R_ROI_SIZE), 0);
  cv::Rect        roi = roi_R.boundingRect();

  // roi安全条件
  roi = roi_tool_.makeRectSafeTailor(_input_src_img, roi);

  // 截取roi大小的图像，并绘制截取区域
  result_img_ = roi_tool_.cutRoIRect(_input_bin_img, roi);
  roi_img_    = roi_tool_.cutRoIRect(_input_src_img, roi);

  cv::rectangle(_dst_img, roi, cv::Scalar(0, 255, 200), 2, 8, 0);

  is_circle_ = false;

  // 更新roi的中心点
  roi_local_center_ = cv::Point2f(roi_img_.cols * 0.5, roi_img_.rows * 0.5);

  // 查找轮廓
  cv::findContours(result_img_, contours_r_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  fmt::print("[{}] 圆心目标遍历轮廓数量: {} \n", center_yellow, contours_r_.size());

  // 选择并记录合适的圆心目标
  for (size_t i = 0; i != contours_r_.size(); ++i) {
    if (contours_r_[i].size() < 6) {
      continue;
    }

    center_r_.inputParams(contours_r_[i], roi_img_);

    fmt::print("[{}] 矩形 {} 比例:{}\n", center_yellow, i, center_r_.aspectRatio());

    if (center_r_.aspectRatio() < buff_param_.CENTER_R_ASPECT_RATIO_MIN || center_r_.aspectRatio() > buff_param_.CENTER_R_ASPECT_RATIO_MAX) {
      continue;
    }

    fmt::print("[{}] 矩形 {} 面积:{}\n", center_yellow, i, center_r_.Rect().boundingRect().area());

    if (center_r_.Rect().boundingRect().area() < buff_param_.CENTER_R_AREA_MIN || center_r_.Rect().boundingRect().area() > buff_param_.CENTER_R_AREA_MAX) {
      continue;
    }

    fmt::print("[{}] Find center R target success !!!   ", center_yellow);
    fmt::print(" --》 矩形 {}  --》 Ratio: {} / Area: {} ", i, center_r_.aspectRatio(), center_r_.Rect().boundingRect().area());

    center_r_box_.push_back(center_r_);

    for (int k = 0; k < 4; ++k) {
      cv::line(roi_img_, center_r_.Vertex(k), center_r_.Vertex((k + 1) % 4), cv::Scalar(0, 130, 255), 3);
    }
    fmt::print("\n");
  }

  fmt::print("[{}] 符合比例条件的有: {}\n", center_yellow, center_r_box_.size());

  // 如果没有圆心目标，则退出
  if (center_r_box_.size() < 1) {
    fmt::print("[{}] 圆心为:假定圆心 \n", center_yellow);
    is_circle_       = false;
    center_r_point2f = roi_global_center_;

    // 画出小轮廓到假定圆心的距离线
    cv::line(_dst_img, current_target_.Armor().Rect().center, center_r_point2f, cv::Scalar(0, 0, 255), 2);
    // 画出假定圆心
    cv::circle(_dst_img, center_r_point2f, 2, cv::Scalar(0, 0, 255), 2, 8, 0);
  } else {
    std::sort(center_r_box_.begin(), center_r_box_.end(), [](abstract_center_r::Center_R& c1, abstract_center_r::Center_R& c2) { return c1.centerDist() < c2.centerDist(); });

    fmt::print("[{}] 圆心为:真实圆心 \n", center_yellow);
    is_circle_       = true;
    center_r_point2f = center_r_box_[0].Rect().center + roi_R.boundingRect2f().tl();

    // 画出小轮廓到假定圆心的距离线
    cv::line(_dst_img, current_target_.Armor().Rect().center, center_r_point2f, cv::Scalar(0, 255, 0), 2);
    // 画出假定圆心
    cv::circle(_dst_img, center_r_point2f, 2, cv::Scalar(0, 0, 255), 2, 8, 0);
  }

  // 清理容器
  center_r_box_.clear();
  contours_r_.clear();

  std::vector<abstract_center_r::Center_R>(center_r_box_).swap(center_r_box_);
  std::vector<std::vector<cv::Point>>(contours_r_).swap(contours_r_);

  return center_r_point2f;
}

void Detector::findTarget(cv::Mat& _input_dst_img, cv::Mat& _input_bin_img, std::vector<abstract_target::Target>& _target_box) {
  cv::findContours(_input_bin_img, contours_, hierarchy_, 2, cv::CHAIN_APPROX_NONE);

  for (size_t i = 0; i != contours_.size(); ++i) {
    // 用于寻找小轮廓，没有父轮廓的跳过，以及不满足6点拟合椭圆
    if (hierarchy_[i][3] < 0 || contours_[i].size() < 6 || contours_[static_cast<uint>(hierarchy_[i][3])].size() < 6) {
      continue;
    }

    // 小轮廓周长条件
    small_rect_length_ = cv::arcLength(contours_[i], true);
    small_rect_length_2 = (small_rect.size.height + small_rect.size.width) * 2;;
    small_rect = cv::minAreaRect(contours_[i]);
    // small_rect_length_ = cv::arcLength(contours_[i], true);
    //if (small_rect_length_ < buff_param_.SMALL_TARGET_Length_MIN) {
    if (std::fabs(small_rect_length_2 - small_rect_length_) / std::min(small_rect_length_2, small_rect_length_) < 0.001) {
      continue;
    }
    // 小轮廓面积条件
    small_rect_area_ = cv::contourArea(contours_[i]);
    small_rect_size_area_ = small_rect.size.area();
    // small_rect_area_ = cv::contourArea(contours_[i]);
    // if (small_rect_area_ < buff_param_.SMALL_TARGET_AREA_MIN || small_rect_area_ > buff_param_.SMALL_TARGET_AREA_MAX) {
    if(small_rect_size_area_ < small_rect_area_){
      continue;
    }

    // 大轮廓面积条件
    big_rect_area_ = cv::contourArea(contours_[static_cast<uint>(hierarchy_[i][3])]);
    if (big_rect_area_ < buff_param_.BIG_TARGET_AREA_MIN || big_rect_area_ > buff_param_.BIG_TARGET_AREA_MAX) {
      continue;
    }

    // 保存扇叶和装甲板的数据
    small_target_.inputParams(contours_[i]);
    big_target_.inputParams(contours_[static_cast<uint>(hierarchy_[i][3])]);
    candidated_target_.inputParams(big_target_, small_target_);

    // 组合判断角度差
    if (candidated_target_.diffAngle() >= buff_param_.DIFF_ANGLE_MAX || candidated_target_.diffAngle() <= buff_param_.DIFF_ANGLE_MIN) {
      continue;
    }

    // 判断内轮廓的长宽比是否正常
    if (candidated_target_.Armor().aspectRatio() >= buff_param_.SMALL_TARGET_ASPECT_RATIO_MAX || candidated_target_.Armor().aspectRatio() <= buff_param_.SMALL_TARGET_ASPECT_RATIO_MIN) {
      continue;
    }

    // 判断内外轮廓的面积比是否正常
    // if (candidated_target_.areaRatio() <= buff_param_.AREA_RATIO_MIN || candidated_target_.areaRatio() >= buff_param_.AREA_RATIO_MAX) {
    //   continue;
       if ( std::max(small_rect.size.width, small_rect.size.height) / std::min(small_rect.size.width, small_rect.size.height) < 1 )  {    
         continue;
    }

    small_target_.displayFanArmor(_input_dst_img);
    big_target_.displayFanBlade(_input_dst_img);

    // 设置默认扇叶状态
    candidated_target_.setType(abstract_object::ACTION);
    // 更新装甲板的四个顶点编号
    candidated_target_.updateVertex(_input_dst_img);
    // 更新扇叶状态
    candidated_target_.setType(_input_bin_img, _input_dst_img);

    _target_box.push_back(candidated_target_);
  }
  fmt::print("[{}] 扇叶数量: {}\n", target_yellow, _target_box.size());
}

inline void Detector::getInput(cv::Mat& _input_img) {
  src_img_ = _input_img;
  _input_img.copyTo(draw_img_);
  is_find_target_ = false;
}

bool Detector::isFindTarget(cv::Mat& _input_img, std::vector<abstract_target::Target>& _target_box) {
  if (_target_box.size() < 1) {
    fmt::print("[{}] Info, XXX no target detected XXX \n", target_yellow);

    current_target_ = abstract_target::Target();
    contours_.clear();
    hierarchy_.clear();
    _target_box.clear();

    std::vector<std::vector<cv::Point>>(contours_).swap(contours_);
    std::vector<cv::Vec4i>(hierarchy_).swap(hierarchy_);
    std::vector<abstract_target::Target>(_target_box).swap(_target_box);

    return false;
  }

  action_cnt_   = 0;
  inaction_cnt_ = 0;

  // 遍历容器获取未激活目标
  for (auto iter = _target_box.begin(); iter != _target_box.end(); ++iter) {
    if (iter->Type() != abstract_object::INACTION) {
      // TODO(fqjun) 测试是否会多或者少了几个对象，如果没有顺序的话，有可能第一个就退出了

      ++action_cnt_;
      continue;
    }

    ++inaction_cnt_;
    // 获取到未激活对象后退出遍历 TODO(fqjun) 查看是否筛选出想要的内容
    current_target_ = *iter;

    current_target_.displayInactionTarget(_input_img);
  }

  fmt::print("[{}] 未击打数量: {},  已击打数量: {}\n", target_yellow, inaction_cnt_, action_cnt_);

  // 清除容器
  contours_.clear();
  hierarchy_.clear();
  _target_box.clear();

  std::vector<std::vector<cv::Point>>(contours_).swap(contours_);
  std::vector<cv::Vec4i>(hierarchy_).swap(hierarchy_);
  std::vector<abstract_target::Target>(_target_box).swap(_target_box);

  if (inaction_cnt_ > 0) {
    return true;
  } else {
    return false;
  }
}

void Detector::runImage(const cv::Mat& _src_img, const int _my_color) {
  // 选择预处理类型（ BGR / HSV ）
  switch (image_config_.method) {
  case 0:
    bin_color_img = fuseImage(grayPretreat(_src_img, _my_color), bgrPretreat(_src_img, _my_color), whilePretreat(_src_img));
    break;
  default:
    bin_color_img = fuseImage(grayPretreat(_src_img, _my_color), hsvPretreat(_src_img, _my_color), whilePretreat(_src_img));
    break;
  }
}

cv::Mat Detector::fuseImage(const cv::Mat _bin_gray_img, const cv::Mat _bin_color_img, const cv::Mat _while_img) {
  cv::bitwise_and(_bin_color_img, _bin_gray_img, _bin_color_img);
  cv::bitwise_and(_bin_color_img, _while_img, _bin_color_img);
  cv::morphologyEx(_bin_color_img, _bin_color_img, cv::MORPH_OPEN, ele_);
  cv::morphologyEx(_bin_color_img, _bin_color_img, cv::MORPH_DILATE, ele_);
  cv::medianBlur(_bin_color_img, _bin_color_img, 3);
  return _bin_color_img;
}

inline cv::Mat Detector::whilePretreat(const cv::Mat& _src_img) {
  cv::cvtColor(_src_img, gray_while_img_, cv::COLOR_BGR2GRAY);
  cv::threshold(gray_while_img_, while_img_, image_config_.while_armor_color_th, 255, cv::THRESH_BINARY);
  cv::bitwise_not(while_img_, while_img_);
  return while_img_;
}

inline cv::Mat Detector::grayPretreat(const cv::Mat& _src_img, const int _my_color) {
  cv::cvtColor(_src_img, gray_img_, cv::COLOR_BGR2GRAY);

  std::string window_name = {"[basic_armor] grayPretreat() -> gray_trackbar"};
  switch (_my_color) {
  case Color::BLUE:
    // my_color 为红色，则处理蓝色的情况
    if (image_config_.gray_edit) {
      cv::namedWindow(window_name);
      cv::createTrackbar("blue_gray_th", window_name, &image_config_.blue_armor_gray_th, 255, NULL);
      //cv::imshow(window_name, gray_trackbar_);
    }

    cv::threshold(gray_img_, bin_gray_img, image_config_.blue_armor_gray_th, 255, cv::THRESH_BINARY);
    break;
  case Color::RED:
    // my_color 为蓝色，则处理红色的情况
    if (image_config_.gray_edit) {
      cv::namedWindow(window_name);
      cv::createTrackbar("red_gray_th", window_name, &image_config_.red_armor_gray_th, 255, NULL);
      //cv::imshow(window_name, gray_trackbar_);
    }

    cv::threshold(gray_img_, bin_gray_img, image_config_.red_armor_gray_th, 255, cv::THRESH_BINARY);
    break;
  default:
    // my_color 为默认值，则处理红蓝双色的情况
    if (image_config_.gray_edit) {
      cv::namedWindow(window_name);
      cv::createTrackbar("red_gray_th", window_name, &image_config_.red_armor_gray_th, 255, NULL);
      cv::createTrackbar("blue_gray_th", window_name, &image_config_.blue_armor_gray_th, 255, NULL);
      //cv::imshow(window_name, gray_trackbar_);
    }

    cv::threshold(gray_img_, bin_red_gray_img, image_config_.red_armor_gray_th, 255, cv::THRESH_BINARY);
    cv::threshold(gray_img_, bin_blue_gray_img, image_config_.blue_armor_gray_th, 255, cv::THRESH_BINARY);
    cv::bitwise_or(bin_red_gray_img, bin_blue_gray_img, bin_gray_img);
    break;
  }

  if (image_config_.gray_edit) {
    //cv::imshow(window_name, bin_gray_img);
  }
  // cv::imshow(window_name, bin_gray_img);
  return bin_gray_img;
}

inline cv::Mat Detector::bgrPretreat(const cv::Mat& _src_img, const int _my_color) {
  static std::vector<cv::Mat> _split;
  static cv::Mat              bin_color_img;
  static cv::Mat              bin_color_green_img;

  cv::split(_src_img, _split);

  std::string window_name = {"[basic_armor] brgPretreat() -> color_trackbar"};
  switch (_my_color) {
  case Color::BLUE:
    // my_color 为红色，则处理蓝色的情况
    cv::subtract(_split[0], _split[2], bin_color_img);
    cv::subtract(_split[0], _split[1], bin_color_green_img);

    if (image_config_.color_edit) {
      cv::namedWindow(window_name);
      cv::createTrackbar("blue_color_th", window_name, &image_config_.blue_armor_color_th, 255, NULL);
      //cv::imshow(window_name, this->bgr_trackbar_);
    }

    cv::threshold(bin_color_img, bin_color_img, image_config_.blue_armor_color_th, 255, cv::THRESH_BINARY);
    cv::threshold(bin_color_green_img, bin_color_green_img, image_config_.green_armor_color_th, 255, cv::THRESH_BINARY);
    cv::medianBlur(bin_color_green_img, bin_color_green_img, 3);
    cv::dilate(bin_color_img, bin_color_img, ele_5);
    cv::dilate(bin_color_green_img, bin_color_green_img, ele_5);
    cv::bitwise_and(bin_color_green_img, bin_color_img, bin_color_img);
    break;
  case Color::RED:
    // my_color 为蓝色，则处理红色的情况
    cv::subtract(_split[2], _split[0], bin_color_img);
    cv::subtract(_split[2], _split[1], bin_color_green_img);

    if (image_config_.color_edit) {
      cv::namedWindow(window_name);
      cv::createTrackbar("red_color_th", window_name, &image_config_.red_armor_color_th, 255, NULL);
      //cv::imshow(window_name, this->bgr_trackbar_);
    }

    cv::threshold(bin_color_img, bin_color_img, image_config_.red_armor_color_th, 255, cv::THRESH_BINARY);
    cv::threshold(bin_color_green_img, bin_color_green_img, image_config_.green_armor_color_th, 255, cv::THRESH_BINARY);
    cv::medianBlur(bin_color_green_img, bin_color_green_img, 3);
    cv::dilate(bin_color_img, bin_color_img, ele_5);
    cv::dilate(bin_color_green_img, bin_color_green_img, ele_5);
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
      cv::createTrackbar("red_color_th", window_name, &image_config_.red_armor_color_th, 255, NULL);
      cv::createTrackbar("blue_color_th", window_name, &image_config_.blue_armor_color_th, 255, NULL);
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
  // cv::imshow(window_name, bin_color_img);
  return bin_color_img;
}

inline cv::Mat Detector::hsvPretreat(const cv::Mat& _src_img, const int _my_color) {
  cv::cvtColor(_src_img, hsv_img, cv::COLOR_BGR2HSV_FULL);
  std::string window_name = {"[basic_armor] hsvPretreat() -> hsv_trackbar"};
  switch (_my_color) {
  // my_color 为红色，则处理蓝色的情况
  case Color::BLUE:
    if (image_config_.color_edit) {
      cv::namedWindow(window_name);
      cv::createTrackbar("blue_h_min:", window_name, &image_config_.h_blue_min, 255, NULL);
      cv::createTrackbar("blue_h_max:", window_name, &image_config_.h_blue_max, 255, NULL);
      cv::createTrackbar("blue_s_min:", window_name, &image_config_.s_blue_min, 255, NULL);
      cv::createTrackbar("blue_s_max:", window_name, &image_config_.s_blue_max, 255, NULL);
      cv::createTrackbar("blue_v_min:", window_name, &image_config_.v_blue_min, 255, NULL);
      cv::createTrackbar("blue_v_max:", window_name, &image_config_.v_red_max, 255, NULL);
      //cv::imshow(window_name, this->hsv_trackbar_);
    }

    cv::inRange(hsv_img,
                cv::Scalar(image_config_.h_blue_min, image_config_.s_blue_min, image_config_.v_blue_min),
                cv::Scalar(image_config_.h_blue_max, image_config_.s_blue_max, image_config_.v_blue_max),
                bin_color_img);
    break;
  case Color::RED:
    // my_color 为蓝色，则处理红色的情况
    if (image_config_.color_edit) {
      cv::namedWindow("hsv_trackbar");
      cv::createTrackbar("red_h_min:", window_name, &image_config_.h_red_min, 255, NULL);
      cv::createTrackbar("red_h_max:", window_name, &image_config_.h_red_max, 255, NULL);
      cv::createTrackbar("red_s_min:", window_name, &image_config_.s_red_min, 255, NULL);
      cv::createTrackbar("red_s_max:", window_name, &image_config_.s_red_max, 255, NULL);
      cv::createTrackbar("red_v_min:", window_name, &image_config_.v_red_min, 255, NULL);
      cv::createTrackbar("red_v_max:", window_name, &image_config_.v_red_max, 255, NULL);
      //cv::imshow(window_name, hsv_trackbar_);
    }

    cv::inRange(hsv_img,
                cv::Scalar(image_config_.h_red_min, image_config_.s_red_min, image_config_.v_red_min),
                cv::Scalar(image_config_.h_red_max, image_config_.s_red_max, image_config_.v_red_max),
                bin_color_img);
    break;
  default:
    // my_color 为默认值，则处理红蓝双色的情况
    if (image_config_.color_edit) {
      cv::namedWindow("hsv_trackbar");
      cv::createTrackbar("red_h_min:", window_name, &image_config_.h_red_min, 255, NULL);
      cv::createTrackbar("red_h_max:", window_name, &image_config_.h_red_max, 255, NULL);
      cv::createTrackbar("red_s_min:", window_name, &image_config_.s_red_min, 255, NULL);
      cv::createTrackbar("red_s_max:", window_name, &image_config_.s_red_max, 255, NULL);
      cv::createTrackbar("red_v_min:", window_name, &image_config_.v_red_min, 255, NULL);
      cv::createTrackbar("red_v_max:", window_name, &image_config_.v_red_max, 255, NULL);

      cv::createTrackbar("blue_h_min:", window_name, &image_config_.h_blue_min, 255, NULL);
      cv::createTrackbar("blue_h_max:", window_name, &image_config_.h_blue_max, 255, NULL);
      cv::createTrackbar("blue_s_min:", window_name, &image_config_.s_blue_min, 255, NULL);
      cv::createTrackbar("blue_s_max:", window_name, &image_config_.s_blue_max, 255, NULL);
      cv::createTrackbar("blue_v_min:", window_name, &image_config_.v_blue_min, 255, NULL);
      cv::createTrackbar("blue_v_max:", window_name, &image_config_.v_red_max, 255, NULL);

      //cv::imshow(window_name, hsv_trackbar_);
    }

    cv::inRange(hsv_img,
                cv::Scalar(image_config_.h_red_min, image_config_.s_red_min, image_config_.v_red_min),
                cv::Scalar(image_config_.h_red_max, image_config_.s_red_max, image_config_.v_red_max),
                bin_red_color_img);

    cv::inRange(hsv_img,
                cv::Scalar(image_config_.h_blue_min, image_config_.s_blue_min, image_config_.v_blue_min),
                cv::Scalar(image_config_.h_blue_max, image_config_.s_blue_max, image_config_.v_blue_max),
                bin_blue_color_img);
    cv::bitwise_or(bin_blue_color_img, bin_red_color_img, bin_color_img);

    break;
  }

  if (image_config_.gray_edit) {
    //cv::imshow(window_name, bin_color_img);
  }

  return bin_color_img;
}
}  // namespace basic_buff