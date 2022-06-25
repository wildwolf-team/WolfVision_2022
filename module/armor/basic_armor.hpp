/**
 * @file basic_armor.hpp
 * @author XX (2796393320@qq.com)
 * @brief 装甲板识别
 * @date 2021-08-28
 *
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 *
 */
#pragma once

#include <fmt/color.h>
#include <fmt/core.h>

#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>

#include "utils/utils.hpp"
#include "utils/roi.hpp"
#include "utils/kalman.h"
namespace basic_armor {

constexpr int S = 2;

auto idntifier_green = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "basic_armor");
auto idntifier_red   = fmt::format(fg(fmt::color::red)   | fmt::emphasis::bold, "basic_armor");

struct Armor_Data {
  float width        = 0;       // 装甲板宽度
  float height       = 0;       // 装甲板高度
  float aspect_ratio = 0;       // 装甲板宽高比
  float tan_angle    = 0;       // 装甲板tan角度
  float lightAngle   = 0;
  cv::RotatedRect armor_rect;   // 装甲板旋转矩形
  cv::RotatedRect left_light;   // 左灯条旋转矩形
  cv::RotatedRect right_light;  // 右灯条旋转矩形

  int distance_center = 0;      // 装甲板距离中心点距离
  int distinguish     = 0;      // 装甲板类型（ 0 小装甲板 1 大装甲板）

  float left_light_width   = 0;
  float right_light_width  = 0;
  float left_light_height  = 0;
  float right_light_height = 0;

  float light_height_aspect = 0;
  float light_width_aspect  = 0;
};

struct Armor_Config {
  // 装甲板绘制和调试开关
  int armor_edit;
  int armor_draw;
  int armor_forecast;
  // 左右灯条高度范围
  int light_height_ratio_min;
  int light_height_ratio_max;
  // 左右灯条宽度范围
  int light_width_ratio_min;
  int light_width_ratio_max;
  // 左右灯条 y 的差值不超过灯条平均高度的倍数
  int light_y_different;
  // 左右灯条高度差值不超过灯条平均高度的倍数
  int light_height_different;
  // 左右灯条角度差
  int armor_angle_different;
  // 装甲板比例范围
  int small_armor_aspect_min;
  int armor_type_th;
  int big_armor_aspect_max;
};

struct Light_Config {
  // 绘图和调试开关
  int light_draw;
  int light_edit;
  // 灯条高宽比范围
  int ratio_w_h_min;
  int ratio_w_h_max;
  // 灯条角度范围
  int angle_min;
  int angle_max;
  // 灯条周长范围
  int perimeter_max;
  int perimeter_min;
};

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

class Detector {
 public:
  explicit Detector(const std::string _armor_config);
  ~Detector() = default;
  /**
   * @brief 识别装甲板
   * 
   * @param _src_img       原图（ CV_8UC3 ）
   * @param _receive_data  串口接受的数据
   * @return true          识别到装甲板
   * @return false         没有识别到装甲板
   */
  bool runBasicArmor(const cv::Mat& _src_img,
                     const RoboInf& robo_inf);

  bool topAutoShoot(const float time, float angle, const int depth, const int bullet_velocity);
  /**
   * @brief 计算两点之间距离
   * 
   * @param a       点 A
   * @param b       点 B
   * @return float  两点之间距离
   */
  float getDistance(const cv::Point a, const cv::Point b);
  /**
   * @brief 灯条拟合装甲板
   * 
   * @param i       左灯条在 light_ 的位置
   * @param j       右灯条在 light_ 的位置
   * @return true   左右灯条能拟合成装甲板
   * @return false  左右灯条不能拟合成装甲板
   */
  bool  lightJudge(const int i, const int j);
  /**
   * @brief 装甲板匹配
   * 
   * @return true   匹配到装甲板
   * @return false  没有匹配到装甲板
   */
  bool  fittingArmor();
  /**
   * @brief 装甲板匹配
   * 
   * @return true   匹配到装甲板
   * @return false  没有匹配到装甲板
   */
  void lightBarFilter();
  /**
   * 移除误识别装甲板
   * @param armorStructs 装甲板结构体动态数组
   * @return 移除后的装甲板结构体动态数组
   */
  void removeWrongArmor();
  /**
   * 判断装甲板内部有无灯条
   * @param r1 装甲板矩形
   * @param r2 灯条
   * @return true内部含有灯条，false内部不含灯条
   */
  bool embeddedRectJudge(cv::RotatedRect r1, cv::RotatedRect r2);
  /**
   * 区分装甲板类型
   * @param armorStructs 装甲板结构体动态数组
   */
  void distinguishArmorType();
  /**
   * 用一对灯条合成一个装甲板
   * @param rect_1 灯条1
   * @param rect_2 灯条2
   * @return 合成的装甲板
   */
  inline cv::RotatedRect boundingRRect(const cv::RotatedRect &rect_1, const cv::RotatedRect &rect_2);
  /**
   * 找灯光轮廓
   * @param rects 疑似矩形
   */
  void findLightBarContour();
  /**
	 * 获得短边中点
	 * @param rect 输入矩形
	 * @param shortCenter 短边中点
	 */
	void getShortCenter(cv::RotatedRect rect, cv::Point2f* shortCenter);
  /**
   * 两条直线的夹角
   * @return 夹角
   */
  inline float lineToLineAngle(cv::Point2f& p1, cv::Point2f& p2, cv::Point2f& p3, cv::Point2f& p4);
  /**
	 * 计算两个矩形偏移角度
	 * @param rect1 矩形1
	 * @param rect2 矩形2
	 * @param shortCenter 短边中点数组
	 * @return
	 */
  float calSkewingAngle(cv::RotatedRect rect1, cv::RotatedRect rect2, cv::Point2f* shortCenter);
  /**
   * @brief 寻找灯条
   * 
   * @return true   找到灯条
   * @return false  没有找到灯条
   */
  bool  findLight();
  /**
   * @brief 最优装甲板排序
   * 
   */
  void  finalArmor();
  /**
   * @brief 释放内存
   * 
   */
  void  freeMemory();
  /**
   * @brief 计算图像颜色平均强度
   *
   * @return int 返回图像颜色平均强度
   */
  int   averageColor();
  int   motionDirection();
    /**
   * @brief 返回丢失次数
   * 
   * @return int  
   */
    inline int
    returnLostCnt() {
    return lost_cnt_--;
  }
  /**
   * @brief 返回装甲板数量
   * 
   * @return int 
   */
    inline int returnArmorNum() { return armor_.size(); }
    /**
   * @brief 返回是否找到装甲板
   * 
   * @return true   找到装甲板
   * @return false  没找到装甲板
   */
    inline bool returnSuccessArmor() { return armor_success; }
    /**
   * @brief 返回最优装甲板的结构体
   *
   * @param _num         返回第 _num 个装甲板的结构体（第 0 个为最优装甲板）
   * @return Armor_Data  返回装甲板的结构体
   */
    inline Armor_Data returnFinalArmor(const int _num) { return armor_[_num]; }
    /**
   * @brief 返回装甲板类型
   * 
   * @param _num  返回第 _num 个装甲板的类型
   * @return int  返回装甲板类型（ 0 小装甲板 1 大装甲板）
   */
    inline int returnFinalArmorDistinguish(const int _num) { return armor_[_num].distinguish; }
    /**
   * @brief 返回最优装甲板的旋转矩形
   *
   * @param _num             返回第 _num 个装甲板的旋转矩形
   * @return cv::RotatedRect 返回装甲板的旋转矩形
   */
    inline cv::RotatedRect returnFinalArmorRotatedRect(const int _num) { return armor_[_num].armor_rect; }
    /**
   * @brief 返回最优装甲板的中心点
   * 
   * @param _num  返回第 _num 个装甲板的类型
   * @return cv::Point2f  返回最优装甲板中心点
   */
    inline cv::Point2f returnFinalArmorCenterPoint(const int _num) { return armor_[_num].armor_rect.center; }
    /**
   * @brief 返回上一次最优装甲板的旋转矩形
   * 
   * @return cv::RotatedRect 
   */
    inline cv::RotatedRect returnLastFinalArmorRotatefRect() { return last_armor_rect_; }
    inline bool            returnArmorFlag() { return armor_flag; }
    /**
   * @brief 返回最优装甲板的旋转矩形
   *
   * @param _num             返回第 _num 个装甲板的旋转矩形
   * @return cv::RotatedRect 返回装甲板的旋转矩形
   */
    inline std::vector<cv::Point2f> returnFinalArmor4Point(const int _num) {
      std::vector<cv::Point2f> target_2d_;
      cv::Point2f              P[4];
      armor_[_num].armor_rect.points(P);
      target_2d_.push_back(P[0]);
      target_2d_.push_back(P[1]);
      target_2d_.push_back(P[2]);
      target_2d_.push_back(P[3]);
      return target_2d_;
  }
  /**
   * @brief 返回哨兵的深度
   * 
   * @return int 
   */
  inline int             returnSentryDepth()                              { return actual_depth_; }
  /**
   * @brief 修正装甲板的中心点（使用 ROI 的时候）
   * 
   * @param _num  第 _num 个装甲板
   * @param _tl   ROI 的 tl 点
   */
  inline void fixFinalArmorCenter(const int _num, const cv::Point2f _tl) {
    armor_[_num].armor_rect.center += _tl;
  }
  /**
   * @brief 初始化哨兵模式参数 (请在使用完哨兵模式的时候对其清空)
   *
   */
  inline void initializationSentryMode() {
    sentry_cnt_        = 5;
    initial_gyroscope_ = 0.f;
    deviation_angle_   = 0.f;
    actual_z_          = 0;
    actual_depth_      = 0;
    forecast_angle_    = 0.f;
    forecast_pixels_   = 0;
  }
  /**
   * @brief 预处理
   * 
   * @param _src_img    原图 （ CV_8UC3 ）
   * @param _my_color   自身的颜色
   */
  void runImage(const cv::Mat &_src_img, const int _my_color);
  /**
   * @brief BGR 预处理
   * 
   * @param _src_img    原图 （ CV_8UC3 ）
   * @param _my_color   自身的颜色
   * @return cv::Mat    返回处理后的二值图
   */
  cv::Mat bgrPretreat(const cv::Mat &_src_img, const int _my_color);
  /**
   * @brief HSV 预处理
   * 
   * @param _src_img    原图 （ CV_8UC3 ）
   * @param _my_color   自身的颜色
   * @return cv::Mat    返回处理后的二值图
   */
  cv::Mat hsvPretreat(const cv::Mat &_src_img, const int _my_color);
  /**
   * @brief GRAY 预处理
   * 
   * @param _src_img    原图 （ CV_8UC3 ）
   * @param _my_color   自身的颜色
   * @return cv::Mat    返回处理后的二值图
   */
  cv::Mat grayPretreat(const cv::Mat &_src_img, const int _my_color);
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
  void forecast_armor_flag(float time, double yawangle, double last_yawangle);
  /**
   * @brief 装甲板一阶卡尔曼计算
   * 
   * @param depth 
   * @param bullet_velocity 
   * @return std::vector<cv::Point2f> 
   */
  std::vector<cv::Point2f> forecast_armor(float depth, const int bullet_velocity, int num);
  /**
   * @brief 小陀螺预测量计算
   * 
   * @param depth 
   * @param bullet_velocity 
   * @return std::vector<cv::Point2f> 
   */
  std::vector<cv::Point2f> top_forecast_armor(float depth, const int bullet_velocity);
  /**
   * @brief 
   * 
   * @param depth 
   * @param bullet_velocity 
   * @return std::vector<cv::Point2f> 
   */

  std::vector<cv::Point2f> outpost_forecast_armor(float depth, const int bullet_velocity, int num);
  /**
 * @brief 获得装甲板假ROI
 * 
 * @param coefficient  系数
 * @return cv::Rect2f  返回一个ROI矩形
 */
  cv::Rect2f get_armor_fake_Roi(const float coefficient);
  /**
 * @brief 判断此帧装甲板是否在伪ROI内部
 * 
 * @param Rect_        伪ROI矩形 
 * @param center_point 此帧装甲板中心点
 * @return true 
 * @return false 
 */
  bool is_armor_inside_Roi(cv::Rect2f Rect_, cv::Point2f center_point);
  /**
  * @brief 
  * 
  * @return int 
  */
  int fake_roi_judge();
  inline cv::Mat returnBinImage() {return bin_color_img;}
  inline cv::Mat returnDrawImage() { return draw_img_; }
  inline bool returnShootImage() { return is_shoot; }

 private:
  // 读取 xml 文件参数
  Armor_Config armor_config_;
  Image_Config image_config_;
  Light_Config light_config_;
  Armor_Data   armor_data_;
  RoI    roi_;
  cv::Mat frame;
  cv::Mat draw_img_;
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
  // 滑动条窗口
  const cv::Mat light_trackbar_  = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat armor_trackbar_  = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat sentry_trackbar_ = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat gray_trackbar_   = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat bgr_trackbar_    = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat hsv_trackbar_    = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat ele_ = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
  const cv::Mat ele_5 = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  //假ROI相关
  cv::Rect2f _roi;
  cv::Rect2f fake_roi;
  cv::RotatedRect last_armor_rect_ = cv::RotatedRect(cv::Point(320, 240), cv::Size(640, 480), 0);
  cv::Point       armor_center;
  bool       is_shoot        = false;
  bool       armor_flag      = false;
  bool       last_armor_flag = false;
  bool       lost_flag       = false;
  int        exist_times     = 0;
  int        armorNum        = 0;
  // 所有装甲板数据 （每帧清空）
  std::vector<Armor_Data>      armor_;
  // 所有灯条数据 （每帧清空）
  std::vector<cv::RotatedRect> light_;
  bool lost_armor_success = false;
  bool armor_success      = false;
  bool switch_armor       = false;
  int  armor_cnt_         = 0;
  int lost_cnt_           = 10;
  int lost_distance_armor = 0;
  int amplitude           = 0;
  int optimal_armor       = 0;
  int armor_position      = 0;
  int armor_direction     = 0;
  int Q_                  = 10;
  int R_                  = 5;
  int num                 = 0;

  //kalman相关
  using _Kalman = Kalman<1, S>;
  _Kalman             yaw_v;
  _Kalman             yaw_acc;
  _Kalman             top_angle;
  _Kalman::Matrix_xxd A;
  _Kalman::Matrix_zxd H;
  _Kalman::Matrix_xxd R;
  _Kalman::Matrix_zzd Q{4};
  _Kalman::Matrix_x1d init{0, 0};
  float               compensate_w           = 0;
  float               last_compensate_w      = 0;
  float               last_last_compensate_w = 0;
  float               compensate_p           = 0;
  float               last_compensate_p      = 0;
  float               last_last_compensate_p = 0;
  double              fixed_time             = 0;
  double              last_fixed_time        = 0;
  double              last_last_fixed_time   = 0;
  double              c_speed                = 0.0;
  double              c_last_speed           = 0.0;
  double              c_acc                    = 0.0;
  double              p_last_speed           = 0.0;
  double              last_predict_time      = 0.0;
  bool                inside_flag             = 0;
  bool                fire_flag                = 0;
  bool time_flag  = 0;

  int num_cnt_ = 0;
  // 哨兵模型初始化计数器
  int sentry_cnt_                = 15;
  // 哨兵装甲板到枪管高度
  const int sentry_height_       = 350;
  // 哨兵到环形高度水平面垂直深度
  const int sentry_dist_         = 3380;
  // 哨兵到初始化位置实际水平深度
  int actual_z_                  = 0;
  // 哨兵到枪管实际深度
  int actual_depth_              = 0;
  // 预测像素点数量
  int forecast_pixels_           = 0;
  // 上一帧预测像素点数量
  int last_forecast_pixels_      = 0;
  // 上上帧预测像素点数量
  int last_last_forecast_pixels_ = 0;
  // 相机焦距
  const int camera_focal_        = 8;
  // 延时滤波占比
  int proportion_direction_      = 15;
  // 预测效果大小 * 0.1
  int forecast_size_             = 3000;
  // 预测最大效果 * 装甲板宽度 * 0.1
  int forecast_max_size_         = 15;
  // 判断正负范围
  int judge_direction_           = 10;
  // 像素点宽度 1080p 1mm 4个像素点
  const int pixel_width_         = 4;
  // 最大突变量
  int abrupt_variable_           = 10;
  // 射击延迟
  float firing_delay_            = 0.3;
  // 实际运动方向
  float filter_direction_        = 0.f;
  // 上一帧运动方向
  float last_direction_          = 0.f;
  // 这一帧运动方向
  float current_direction_       = 0.f;
  // 预测角度
  float forecast_angle_          = 0.f;
  // 哨兵到初始化位置的偏差角度
  float deviation_angle_         = 0.f;
  // 上一次哨兵到初始化位置的偏差角度
  float last_deviation_angle_    = 0.f;
  // 初始化陀螺仪 Yaw 轴位置
  float initial_gyroscope_       = 0.f;
};

}  // namespace basic_armor
