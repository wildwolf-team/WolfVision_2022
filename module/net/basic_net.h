#ifndef DETECTOR_H
#define DETECTOR_H
#include <opencv2/opencv.hpp>
#include <inference_engine.hpp>
#include <iostream>
#include <chrono>
#include <opencv2/dnn/dnn.hpp>
#include <cmath>
#include "utils/utils.hpp"
#include "utils/kalman.h"

using namespace InferenceEngine;
constexpr int S = 2;


namespace basic_net {
// 四点模型结构体
struct alignas(4) bbox_t {
    cv::Point2f pts[4];     // [pt0, pt1, pt2, pt3]
    float       confidence; // 置信度
    int         color_id;   // 0: blue, 1: red, 2: gray 3: ??
    int         tag_id;     // 0: guard, 1-5: number, 6: base
    int         distance_center = 0;
    // bool operator==(const bbox_t&) const = default;
    // bool operator!=(const bbox_t&) const = default;
    // 因为g++版本不够，上述代码不能编译
    // 若在较低版本运行改重载，需要如下 [重写 重载]
    bool operator==(const bbox_t& bbox) const {
        return (pts[0] == bbox.pts[0] || pts[1] == bbox.pts[1] || pts[2] == bbox.pts[2] || pts[3] == bbox.pts[3] 
        || confidence == bbox.confidence || color_id == bbox.color_id || tag_id == bbox.tag_id);
    }
    bool operator!=(const bbox_t& bbox) const {
        return (pts[0] != bbox.pts[0] || pts[1] != bbox.pts[1] || pts[2] != bbox.pts[2] || pts[3] != bbox.pts[3] 
        || confidence != bbox.confidence || color_id != bbox.color_id || tag_id != bbox.tag_id);
    }
};

struct index_sort {
    int     index;
    float   confidence;
};

struct armor_detection {
    std::vector<bbox_t> rst;        // 单张图片的包含的装甲板识别
    float  armor_t;                 // 时间戳
    int    quantity[7];
    int    top_id;
    cv::Point2f top_center;              
};


class Detector
{
public:
    Detector();
    ~Detector();
    //初始化
    bool detection_init(std::string xml_path, std::string device); // ,double cof_threshold,double nms_area_threshold
    //释放资源
    bool uninit();
    //处理图像获取结果
    // std::vector<bbox_t> process_frame(Mat& inframe);
    void process_frame(cv::Mat& inframe, armor_detection& armor);
    bool screen_armor(const RoboInf& _robo_inf, armor_detection& armor, cv::Mat _src_img);
    bool screen_top_armor(const RoboInf& _robo_inf, armor_detection& armor, cv::Mat _src_img);
    void defense_tower(const float depth, const int bullet_velocity, cv::Point2f p[4], cv::Mat src_img);
    void forecastFlagV(float time, double angle, double p_angle);
    /**
     * @brief 装甲板一阶卡尔曼计算
     * 
     * @param depth 
     * @param bullet_velocity 
     * @return std::vector<cv::Point2f> 
     */
    void forecast_armor(const float depth, const int bullet_velocity, cv::Point2f p[4], cv::Mat src_img);
    bool topAutoShoot(const int depth, const int bullet_velocity, cv::Point2f p[4], const cv::RotatedRect top_armor, cv::Mat src_img);
    cv::RotatedRect returnArmorRotatedRect() { return last_top_armor; }
   private:
    /**
     * @brief 计算两点之间距离
     * 
     * @param a       点 A
     * @param b       点 B
     * @return float  两点之间距离
     */
    float getDistance(const cv::Point a, const cv::Point b);
    void  kalman_init();
    std::vector<bbox_t> armor_;
    cv::RotatedRect last_top_armor;
    //存储初始化获得的可执行网络
    ExecutableNetwork   _network;           //
    OutputsDataMap          _outputinfo;        // 
    std::string             _input_name;        // 
    //参数区
    std::string             _xml_path;          //OpenVINO模型xml文件路径
    std::string             _device;            // 加载模型的设备
    // double                  _cof_threshold;       //置信度阈值,计算方法是框置信度乘以物品种类置信度
    // double                  _nms_area_threshold;  //nms最小重叠面积阈值

    static constexpr int    TOPK_NUM = 128;     // std::vector<bbox_t> rst;  rst.reserve(TOPK_NUM);
    static constexpr float  KEEP_THRES = 0.1f;  // if (box_buffer[8] < inv_sigmoid(KEEP_THRES)) break;

    using _Kalman = Kalman<1, S>;
    _Kalman             x_v;
    _Kalman             y_v;
    _Kalman             top_angle;
    _Kalman::Matrix_xxd A;
    _Kalman::Matrix_zxd H;
    _Kalman::Matrix_xxd R;
    _Kalman::Matrix_zzd Q{4};
    _Kalman::Matrix_x1d init{0, 0};
    int num[7];
    double              c_speed = 0.f;
    double              p_speed = 0.f;
};
}
#endif