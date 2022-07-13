#pragma once

#include <fmt/color.h>
#include <fmt/core.h>

#include <algorithm>
#include <string>
#include <vector>

#include "abstract_center_r.hpp"
#include "abstract_target.hpp"
#include "utils/angle_solve/basic_pnp.hpp"
#include "utils/fps.hpp"
#include "utils/kalman.h"
#include "utils/roi.hpp"
#include "utils/utils.hpp"
using namespace std::chrono_literals;
namespace basic_buff {
constexpr int S                   = 2;
auto          idntifier_green     = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "basic_buff");
auto          idntifier_red       = fmt::format(fg(fmt::color::red) | fmt::emphasis::bold, "basic_buff");
auto          idntifier_yellow    = fmt::format(fg(fmt::color::yellow) | fmt::emphasis::bold, "basic_buff");
auto          process_yellow      = fmt::format(fg(fmt::color::yellow) | fmt::emphasis::bold, "processing");
auto          target_yellow       = fmt::format(fg(fmt::color::yellow) | fmt::emphasis::bold, "find_target");
auto          center_yellow       = fmt::format(fg(fmt::color::yellow) | fmt::emphasis::bold, "center_r");
auto          judgement_yellow    = fmt::format(fg(fmt::color::yellow) | fmt::emphasis::bold, "judgement");
auto          predict_yellow      = fmt::format(fg(fmt::color::yellow) | fmt::emphasis::bold, "predict");
auto          final_target_yellow = fmt::format(fg(fmt::color::yellow) | fmt::emphasis::bold, "final_target");

struct Image_Config {
  // 红蓝 BGR 及灰度图参数
  int red_armor_gray_th;
  int red_armor_color_th;
  int blue_armor_gray_th;
  int blue_armor_color_th;
  int while_armor_color_th;
  int green_armor_color_th;
  // 红 HSV 参数
  int h_red_max;
  int h_red_min;
  int s_red_max;
  int s_red_min;
  int v_red_max;
  int v_red_min;
  // 蓝 HSV 参数
  int h_blue_max;
  int h_blue_min;
  int s_blue_max;
  int s_blue_min;
  int v_blue_max;
  int v_blue_min;
  // 调试开关
  int gray_edit  = 0;
  int color_edit = 0;
  int method     = 0;
};

struct Buff_Param {
  // 筛选条件
  // 面积 MAX MIN 大 小
  int SMALL_TARGET_AREA_MAX;
  int SMALL_TARGET_AREA_MIN;
  int BIG_TARGET_AREA_MAX;
  int BIG_TARGET_AREA_MIN;
  // 周长 MAX MIN
  int SMALL_TARGET_Length_MIN;
  int SMALL_TARGET_Length_MAX;
  int BIG_TARGET_Length_MIN;
  int BIG_TARGET_Length_MAX;
  // 角度差 MAX MIN
  int DIFF_ANGLE_MAX;
  int DIFF_ANGLE_MIN;
  // 长宽比 MAX MIN
  float SMALL_TARGET_ASPECT_RATIO_MAX;
  float SMALL_TARGET_ASPECT_RATIO_MIN;
  // 面积比 MAX MIN
  float AREA_RATIO_MAX;
  float AREA_RATIO_MIN;
  // 圆心R距离小轮廓中心的距离系数
  float BIG_LENTH_R;

  // 圆心限制条件
  // 圆心roi矩形大小
  int CENTER_R_ROI_SIZE;

  // 圆心矩形比例
  float CENTER_R_ASPECT_RATIO_MIN;
  float CENTER_R_ASPECT_RATIO_MAX;

  // 圆心矩形面积
  int CENTER_R_AREA_MIN;
  int CENTER_R_AREA_MAX;

  // 滤波器系数
  float FILTER_COEFFICIENT;

  // 能量机关打击模型参数，详情见 buff_config.xml
  float BUFF_H;
  float BUFF_RADIUS;
  float PLATFORM_H;
  float BARREL_ROBOT_H;
  float TARGET_X;

  // 预测量的补偿（弧度）
  float OFFSET_FIXED_RADIAN;

  // 模型深度补偿（左半边比右半边距离要远）
  float OFFSET_TARGET_Z;

  // yaw 和 pitch 轴弹道补偿
  float OFFSET_ARMOR_YAW;
  float OFFSET_ARMOR_PITCH;

  // 手算pitch 轴弹道补偿
  float OFFSET_MANUAL_ARMOR_PITCH;
};

class Detector {
 public:
  Detector() = default;
  explicit Detector(const std::string& _buff_config_path);
  ~Detector() = default;
  /**
       * @brief 总执行函数
       * 
       * @param _input_img    输入图像
       * @param _receive_info 串口接收指针
       * @param _send_info    串口发送指针
       */
  void        runTask(cv::Mat& _input_img, const RoboInf& _receive_info, RoboCmd& _send_info, float _time);
  void        findTarget(cv::Mat& _input_dst_img, cv::Mat& _input_bin_img, std::vector<abstract_target::Target>& _target_box);
  inline void getInput(cv::Mat& _input_img);
  bool        isFindTarget(cv::Mat& _input_img, std::vector<abstract_target::Target>& _target_box);

 private:
  basic_pnp::PnP buff_pnp_ = basic_pnp::PnP(fmt::format("{}{}", CONFIG_FILE_PATH, "/camera/mv_camera_config_407.xml"), fmt::format("{}{}", CONFIG_FILE_PATH, "/angle_solve/basic_pnp_config.xml"));
  // 手动计算云台角度
  /**
       * @brief  手动计算云台角度
       * @param  _target_center   目标中心点
       * @param  _unit_pixel_length 成像像素元件长度 mm
       * @param  _image_size      图像大小
       * @param  _focal_length    相机焦距 mm
       * @author WCJ
       * @return cv::Point2f 
       */
  cv::Point2f angleCalculation(const cv::Point2f& _target_center, const float& _unit_pixel_length, const cv::Size& _image_size, const float& _focal_length);

 private:
  // 更新上一帧数据
  /**
       * @brief 更新上一帧数据
       * @param  _is_find_target  是否有目标
       * @author WCJ
       */
  void updateLastData(const bool& _is_find_target);

 private:
  std::vector<cv::Point2f> target_2d_point_;      // 目标二维点集
  float                    final_target_z_;       // 最终打击目标的深度信息（预测点）
  cv::RotatedRect          target_rect_;          // 目标矩形
  bool                     is_find_last_target_;  // 上一帧是否发现目标 true：发现 false：未发现
  bool                     is_find_target_;       // 是否发现目标 true：发现 false：未发现
  abstract_target::Target  last_target_;          //  上一个打击目标
  fan_armor::Detector      small_target_;         // 内轮廓
  abstract_blade::FanBlade big_target_;           // 外轮廓
  Buff_Param               buff_param_;           // 参数结构体
  // 打击目标队列
  abstract_target::Target              candidated_target_;  // 当前检测得到的打击目标（遍历使用）
  abstract_target::Target              current_target_;     // 当前检测打击目标（现在）
  std::vector<abstract_target::Target> target_box_;
  // 轮廓点集 轮廓关系
  std::vector<std::vector<cv::Point>> contours_;
  std::vector<cv::Vec4i>              hierarchy_;
  cv::RotatedRect                     small_rect;
  // 小轮廓条件(area and length)
  float small_rect_area_;
  float small_rect_size_area_;
  float small_rect_length_;
  float small_rect_length_2;

  // 大轮廓条件(area and length)
  float big_rect_area_;
  float big_rect_length_;
  // 扇叶数量
  int action_cnt_;    // 已打击扇叶数
  int inaction_cnt_;  // 未击打扇叶数
 private:
  //kalman相关
  using _Kalman = Kalman<1, S>;
  _Kalman             forecast_v;
  _Kalman             forecast_a;
  _Kalman::Matrix_xxd A;
  _Kalman::Matrix_zxd H;
  _Kalman::Matrix_xxd R;
  _Kalman::Matrix_zzd Q{1};
  _Kalman::Matrix_x1d init{0, 0};
  std::chrono::_V2::system_clock::time_point shoot_time_;
  std::chrono::_V2::system_clock::time_point last_shoot_time_;
  float  shoot_interval_ = 0.f;
  double c_speed = 0.0;
  /**
       * @brief 卡尔曼参数初始化
       * 
       */
  void kalman_init();
  /**
       * @brief 更新装甲板卡尔曼状态
       * 
       * @param timeend 
       * @param timestart 
       * @param yawangle 
       */
  double forecastFlagV(float time, double angle);
  double forecastFlagA(float time, double V);

 private:
  // 计算预测量

  /**
       * @brief 计算预测量
       * @param[in]  _bullet_velocity 子弹速度
       * @param[in]  _is_find_target  是否发现目标
       * @return float 预测量
       * @author WCJ
       */
  float doPredict(const float& _bullet_velocity, const bool& _is_find_target);

  /**
       * @brief 计算固定预测量
       * @param[in]  _bullet_velocity 子弹速度
       * @return float  预测量
       * @author WCJ
       */
  float fixedPredict(const float& _bullet_velocity);

  // 变化预测量 TODO(fqjun)
  void mutativePredict(const float& _input_predict_quantity, float& _output_predict_quantity);

  float current_radian_;        // 当前弧度
  float barrel_buff_botton_h_;  // 枪口到扇叶底部高度
  float target_buff_h_;         // 目标在扇叶上的高度
  float target_y_;              // 当前扇叶目标高度
  float target_x_;              // 当前扇叶目标到高台的直线距离
  float target_z_;              // 当前扇叶目标直线距离

  float bullet_tof_;               // 子弹飞行时间
  float fixed_forecast_quantity_;  // 固定预测量（扇叶的单帧移动量）
  float final_forecast_quantity_;  // 最终合成的预测量
 private:
  // 计算获取最终目标（矩形、顶点）

  /**
       * @brief 计算最终目标矩形顶点点集
       * @param  _predict_quantity 预测量
       * @param  _final_center_r   圆心坐标（src）
       * @param  _target_2d_point  目标矩形顶点容器
       * @param  _input_dst_img    输入画板
       * @param  _is_find_target   是否有目标
       * @author WCJ
       */
  void calculateTargetPointSet(const float& _predict_quantity, const cv::Point2f& _final_center_r, std::vector<cv::Point2f>& _target_2d_point, cv::Mat& _input_dst_img, const bool& _is_find_target);

  double      theta_;         // 特殊的弧度
  float       final_angle_;   // 最终角度
  float       final_radian_;  // 最终弧度
  float       sin_calcu_;     // 计算的sin值
  float       cos_calcu_;     // 计算的cos值
  cv::Point2f pre_center_;    // 最终预测点
  float       radio_;         // 轨迹圆半径
 private:
  /**
       * @brief  查找圆心
       * @param  _input_src_img   输入src原图
       * @param  _input_bin_img   输入bin二值图
       * @param  _dst_img         输入dst画板图
       * @param  _is_find_target  是否发现扇叶目标
       * @return cv::Point2f          返回圆心R中点坐标
       * @author WCJ HZH
       */
  cv::Point2f findCircleR(cv::Mat& _input_src_img, cv::Mat& _input_bin_img, cv::Mat& _dst_img, const bool& _is_find_target);

  bool        is_circle_;           // 是否找到圆心
  cv::Point2f delta_height_point_;  // 获取装甲板的高度点差
  cv::Point2f roi_global_center_;   // roi 圆心中点位置(在原图中)
  cv::Mat     result_img_;          // 二值图
  cv::Mat     roi_img_;             // 截取原图
  RoI         roi_tool_;            // roi截取工具

  abstract_center_r::Center_R              center_r_;          // 候选圆心R
  std::vector<std::vector<cv::Point>>      contours_r_;        // 中心R的遍历点集
  cv::Point2f                              roi_local_center_;  // 截取roi的图像中心点(在roi_img中)
  std::vector<abstract_center_r::Center_R> center_r_box_;      // 第一次筛选之后得到的待选中心R
  cv::Point2f                              final_center_r_;    // 最终圆心（假定/真实）

 private:
  // 计算运转状态值：速度、方向、角度
  /**
       * @brief 计算运转状态值：速度、方向、角度
       * @param  _is_find_target  是否发现目标
       * @author WCJ
       */
  void judgeCondition(const bool& _is_find_target);

  /**
       * @brief 计算角度和角度差
       * @author WCJ
       */
  void calAngle();

  /**
       * @brief 计算转动方向
       * @details 1：顺时针 -1：逆时针 0：不转动
       * @author WCJ HZH
       */
  void calDirection();

  /**
       * @brief 获取风车转向
       * @return int
       * @author WCJ
       */
  int getState();

  /**
       * @brief 计算当前扇叶转动速度
       * @author WCJ
       */
  void calVelocity();

  // 角度
  float current_angle_;         // 当前目标角度
  float last_angle_;            // 上一次目标角度
  float diff_angle_;            // 两帧之间角度差
  float last_diff_angle_;       // 上一帧扇叶移动的角度
  float last_last_diff_angle_;  // 上上帧扇叶移动的角度
  bool  is_change_blade_;       // 判断是否切换扇叶

  // 方向
  float filter_direction_;      // 第二次滤波方向
  int   final_direction_;       // 最终的方向
  int   last_final_direction_;  // 上一次最终的方向
  float current_direction_;     // 当前方向
  float last_direction_;        // 上一次方向
  int   find_cnt_;              // 发现目标次数
  float d_angle_;               // 滤波器系数
  int   confirm_cnt_;           // 记录达到条件次数
  bool  is_confirm_;            // 判断是否达到条件

  // 速度
  float  current_speed_;  // 当前转速
  float  current_acc_;
  double last_time_;       // 上一帧的时间
  double last_last_time_;  // 上上帧的时间

 private:
  fps::FPS buff_fps_1_{"Part 1"};
  fps::FPS buff_fps_2_{"Part 2"};
  fps::FPS buff_fps_;
  float time_;
//   float last_time_;
 public:
  inline cv::Mat returnDstImage() { return draw_img_; }
  inline cv::Mat returnBinImage() { return bin_color_img; }

 private:
  // 预处理
  /**
       * @brief 预处理
       * 
       * @param _src_img    原图 （ CV_8UC3 ）
       * @param _my_color   自身的颜色
       */
  void runImage(const cv::Mat& _src_img, const int _my_color);
  /**
       * @brief BGR 预处理
       * 
       * @param _src_img    原图 （ CV_8UC3 ）
       * @param _my_color   自身的颜色
       * @return cv::Mat    返回处理后的二值图
      */
  cv::Mat bgrPretreat(const cv::Mat& _src_img, const int _my_color);
  /**
       * @brief HSV 预处理
       * 
       * @param _src_img    原图 （ CV_8UC3 ）
       * @param _my_color   自身的颜色
       * @return cv::Mat    返回处理后的二值图
       */
  cv::Mat hsvPretreat(const cv::Mat& _src_img, const int _my_color);
  /**
       * @brief GRAY 预处理
       * 
       * @param _src_img    原图 （ CV_8UC3 ）
       * @param _my_color   自身的颜色
       * @return cv::Mat    返回处理后的二值图
       */
  cv::Mat grayPretreat(const cv::Mat& _src_img, const int _my_color);
  /**
       * @brief  合并图像（三合一）
       * 
       * @param _bin_gray_img           灰度图
       * @param _bin_color_img          bgr处理二值图
       * @param _bin_while_img          while处理二值图
       * @return cv::Mat 
       */
  cv::Mat fuseImage(const cv::Mat _bin_gray_img, const cv::Mat _bin_color_img, const cv::Mat _bin_while_img);
  /**
       * @brief  白色通道处理 
       * 
       * @param _src_img         原图
       * @return cv::Mat 
       */
  cv::Mat whilePretreat(const cv::Mat& _src_img);

 private:
  // Mat
  cv::Mat src_img_;   // 输入原图
  cv::Mat draw_img_;  // 图像效果展示图
  cv::Mat gray_img_;
  cv::Mat while_img_;
  cv::Mat hsv_img;
  cv::Mat bin_gray_img;
  cv::Mat bin_red_gray_img;
  cv::Mat bin_blue_gray_img;
  cv::Mat bin_color_img;
  cv::Mat bin_red_color_img;
  cv::Mat bin_blue_color_img;
  cv::Mat bin_red_green_img;
  cv::Mat bin_blue_green_img;
  cv::Mat gray_while_img_;
  // 窗口
  const cv::Mat light_trackbar_  = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat armor_trackbar_  = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat sentry_trackbar_ = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat gray_trackbar_   = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat bgr_trackbar_    = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat hsv_trackbar_    = cv::Mat::zeros(1, 300, CV_8UC1);
  // 参数
  const cv::Mat ele_  = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
  const cv::Mat ele_5 = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  Image_Config  image_config_;
};
}  // namespace basic_buff