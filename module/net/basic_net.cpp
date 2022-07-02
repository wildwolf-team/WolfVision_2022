#include "basic_net.h"

namespace basic_net {
Detector::Detector(){}

Detector::~Detector(){}

float Detector::getDistance(const cv::Point a, const cv::Point b) { return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)); }

// 哨兵筛选最优装甲板
int Detector::finaly_armor_num(armor_detection& armor){
  for(int i = 0;i<armor.rst.size();i++){
    if(armor.rst[i].tag_id == 1){
      return i;
    }
  }
  return -1;
}

// 小陀螺自动击打
bool Detector::topAutoShoot(const float depth, const int bullet_velocity, cv::Point2f p[4], const cv::RotatedRect top_armor, cv::Mat src_img) {
  double predict_time  = (float(depth) / (bullet_velocity));
  float T                    = fabs(90 / (c_speed * 180 / CV_PI));
  cv::putText(src_img, std::to_string(T), cv::Point(50, 100), 1, 2, cv::Scalar(0, 255, 0));
  int i = 0;
  while (fabs(predict_time) > fabs(T)) {
    T += fabs(90 / (c_speed * 180 / CV_PI));
    ++i;
    if (i > 4) {
      break;
    }
  }
  float      diff_time = T - fabs(predict_time);
  // cv::putText(src_img, std::to_string(diff_time), cv::Point(50, 100), 1, 2, cv::Scalar(0, 255, 0));
  static int last_last_compensate_w;
  static int last_compensate_w;
  double     s_yaw        = atan2(diff_time * c_speed * float(depth), float(depth));
  if (T > 10) {
    s_yaw = atan2(predict_time * c_speed * float(depth), float(depth));
  }
  int        compensate_w = 0.5 * tan(s_yaw) * 180 / M_PI;
  // compensate_w            = (last_last_compensate_w + last_compensate_w + compensate_w) * 0.333;
  // last_compensate_w       = compensate_w;
  // last_last_compensate_w  = last_compensate_w;
  cv::putText(src_img, std::to_string(compensate_w), cv::Point(50, 50), 1, 2, cv::Scalar(255, 255, 255));
  static cv::Point2f ss           = cv::Point2f(0, 0);
  ss                              = cv::Point2f(compensate_w, 0);
  p[0] += ss;
  p[1] += ss;
  p[2] += ss;
  p[3] += ss;
  cv::Point2f center = (p[2] + p[0]) * 0.5;
  int width = (p[2].y- p[0].y) * 1.2;
  cv::circle(src_img, center, 15, cv::Scalar(0, 255, 255), -1);
  cv::line(src_img, cv::Point(src_img.cols * 0.52 - width, 0), cv::Point(src_img.cols * 0.52 - width, src_img.rows), cv::Scalar(0, 0, 255));
  cv::line(src_img, cv::Point(src_img.cols * 0.52 + width, 0), cv::Point(src_img.cols * 0.52 + width, src_img.rows), cv::Scalar(0, 0, 255));
  if (center.x < src_img.cols * 0.52 + width && center.x > src_img.cols * 0.52 - width) {
    cv::putText(src_img, std::to_string(1), cv::Point(50, 150), 1, 2, cv::Scalar(255, 255, 255));
    return true;
  }
  cv::putText(src_img, std::to_string(0), cv::Point(50, 150), 1, 2, cv::Scalar(255, 255, 255));
  return false;
}

void Detector::forecast_armor(const float depth, const int bullet_velocity, cv::Point2f p[4], cv::Mat src_img) {
  double predict_time = (float(depth) / (bullet_velocity));
  static int last_last_compensate_w = 0.f;
  static int last_compensate_w = 0.f;
  double s_yaw                       = atan2(predict_time * c_speed * float(depth), float(depth));
  int compensate_w                   = 40 * tan(s_yaw) * 180 / M_PI;
  compensate_w                       = (last_last_compensate_w + last_compensate_w + compensate_w) * 0.333;
  last_compensate_w                  = compensate_w;
  last_last_compensate_w             = last_compensate_w;
  
  double s_pitch = atan2(predict_time * c_speed * float(depth), float(depth));  //, depth*0.001
    // std::cout << "s_pitch=" << s_pitch << std::endl;
  static int last_last_compensate_p = 0.f;
  static int last_compensate_p = 0.f;
  int compensate_p           = 10*tan(s_pitch)* 180 / M_PI;
  compensate_p           = (last_last_compensate_p + last_compensate_p + compensate_p) * 0.333;
  last_last_compensate_p = last_compensate_p;
  last_compensate_p = compensate_p ;
  
  cv::putText(src_img, std::to_string(c_speed), cv::Point(50, 50), 3, 6, cv::Scalar(0, 255, 0));
  cv::putText(src_img, std::to_string(compensate_w), cv::Point(50, 200), 3, 6, cv::Scalar(0, 0, 0));
  static cv::Point2f ss              = cv::Point2f(0, 0);
  ss                                 = cv::Point2f(-compensate_w,-compensate_p);
  cv::Point2f center                 = (p[0] + p[2]) * 0.5 + ss;
  cv::circle(src_img, center, 15, cv::Scalar(0, 255, 255), -1);
  p[0] += ss;
  p[1] += ss;
  p[2] += ss; 
  p[3] += ss;
}

void Detector::kalman_init() {
  A       = _Kalman::Matrix_xxd::Identity();
  R       = _Kalman::Matrix_xxd::Identity();
  H(0, 0) = 1;
  R(0, 0) = 0.01;
  for (int i = 1; i < S; i++) {
    R(i, i) = 10;
  }
  x_v = _Kalman(A, H, R, Q, init, 0);
  y_v = _Kalman(A, H, R, Q, init, 0);
  R(0, 0) = 0.1;
  _Kalman::Matrix_zzd Q_{20};
  top_angle = _Kalman(A, H, R, Q_, init, 0);
}

void Detector::forecastFlagV(float time, double angle, double p_angle) {
  static double last_angle = 0.f;
  angle                    = angle * M_PI / 180;
  p_angle                  = p_angle * M_PI / 180;
  if (std::fabs(last_angle - angle) > 5. / 180. * M_PI) {
    x_v.reset(angle, time);
    last_angle = angle;
    std::cout << "reset-yaw" << std::endl;
  } else {
    last_angle = angle;
    Eigen::Matrix<double, 1, 1> ck{angle};
    _Kalman::Matrix_x1d         state = x_v.update(ck, time);
    c_speed                           = state(1, 0);
    // std::cout << "c_speed==========================" << c_speed << std::endl;
  }
  static double p_last_angle = 0.f;
  if (std::fabs(p_last_angle - p_angle) > 5. / 180. * M_PI) {
    y_v.reset(p_angle, time);
    p_last_angle = p_angle;
    std::cout << "reset-pitch" << std::endl;
  } else {
    p_last_angle = p_angle;
    Eigen::Matrix<double, 1, 1> ck{p_angle};
    _Kalman::Matrix_x1d         state = y_v.update(ck, time);
    p_speed                           = state(1, 0);
    // std::cout << "c_speed==========================" << c_speed << std::endl;
  }
}

bool Detector::screen_top_armor(const RoboInf& _robo_inf, armor_detection& armor, cv::Mat _src_img) {
  cv::Point img_center = cv::Point(_src_img.cols * 0.5, _src_img.rows * 0.5);
  if (_robo_inf.robot_color == Color::BLUE) {
    for (int i = 0; i < armor.rst.size(); ++i) {
      if (armor.rst[i].color_id == 1) {
        armor_.push_back(armor.rst[i]);
        armor.quantity[armor.rst[i].tag_id] += 1;
      }
    }
  } else if (_robo_inf.robot_color == Color::RED) {
    for (int i = 0; i < armor.rst.size(); ++i) {
      if (armor.rst[i].color_id == 0) {
        armor_.push_back(armor.rst[i]);
        armor.quantity[armor.rst[i].tag_id] += 1;
      }
    }
  }
  armor.rst = armor_;
  for (int i = 0; i < armor.rst.size(); ++i) {
    cv::Point armor_center       = (armor.rst[i].pts[0] + armor.rst[i].pts[1] + armor.rst[i].pts[2] + armor.rst[i].pts[3]) * 0.25;
    armor.rst[i].distance_center = getDistance(img_center, armor_center);
    num[armor.rst[i].tag_id] += 1;
  }
  std::sort(armor.rst.begin(), armor.rst.end(), [](bbox_t _a, bbox_t _b) { return _a.distance_center < _b.distance_center; });

  armor.top_id = armor.rst[0].tag_id;
  std::cout << "armor.top_id = " << armor.top_id << "\n";
  int height = 0;
  if (armor.quantity[armor.top_id] > 0) {
    for (int i = 0; i < armor.rst.size(); ++i) {
      cv::Point2f center;
      if (armor.rst[i].tag_id == armor.top_id) {
        center = (armor.rst[i].pts[0] + armor.rst[i].pts[1] + armor.rst[i].pts[2] + armor.rst[i].pts[3]) * 0.25;
        height += abs(armor.rst[i].pts[2].y - armor.rst[i].pts[0].y);
        cv::circle(_src_img, center, 10, cv::Scalar(255, 255, 255), -1);
      }
      armor.top_center += center;
    }
    armor.top_center /= (armor.quantity[armor.top_id] + 1);
    height /= (armor.quantity[armor.top_id]);
    std::cout << "armor.top_center = " << armor.top_center << "\n";
  }

  if (armor.rst[0].tag_id == 1 || armor.rst[0].tag_id == 0) {
    last_top_armor = cv::RotatedRect(armor.top_center, cv::Size(height * 3.3, height), 0);
  } else {
    last_top_armor = cv::RotatedRect(armor.top_center, cv::Size(height * 2.2, height), 0);
  }

  cv::rectangle(_src_img, last_top_armor.boundingRect(), cv::Scalar(255, 255, 255), 3, 8);

  const cv::Scalar colors[4] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}, {0, 0, 0}};
  for (const auto& b : armor.rst) {
    cv::line(_src_img, b.pts[0], b.pts[1], colors[2], 2);
    cv::line(_src_img, b.pts[1], b.pts[2], colors[2], 2);
    cv::line(_src_img, b.pts[2], b.pts[3], colors[2], 2);
    cv::line(_src_img, b.pts[3], b.pts[0], colors[2], 2);
    cv::putText(_src_img, std::to_string(b.tag_id), b.pts[0], cv::FONT_HERSHEY_SIMPLEX, 3, colors[b.color_id], 3);
  }
  // std::cout << "armor.top_center = " << armor.top_center << "\n";
  cv::circle(_src_img, armor.top_center, 10, cv::Scalar(255, 0, 255), -1);
  armor_.clear();
  armor_.shrink_to_fit();
  if (armor.rst.size() > 0) {
    return true;
  }
  return false;
}

bool Detector::screen_armor(const RoboInf& _robo_inf, armor_detection& armor, cv::Mat _src_img) {
    cv::Point img_center = cv::Point(_src_img.cols * 0.5, _src_img.rows * 0.5);
    if (_robo_inf.robot_color == Color::BLUE) {
        for (int i = 0; i < armor.rst.size(); ++i) {
            if (armor.rst[i].color_id == 1) {
              armor_.push_back(armor.rst[i]);
            }
        }
    } else if (_robo_inf.robot_color == Color::RED) {
        for (int i = 0; i < armor.rst.size(); ++i) {
          if (armor.rst[i].color_id == 0) {
            armor_.push_back(armor.rst[i]);
          }
        }
    }
    armor.rst = armor_;
    for (int i = 0; i < armor.rst.size(); ++i) {
        cv::Point armor_center       = (armor.rst[i].pts[0] + armor.rst[i].pts[1] + armor.rst[i].pts[2] + armor.rst[i].pts[3]) * 0.25;
        armor.rst[i].distance_center = getDistance(img_center, armor_center);
        num[armor.rst[i].tag_id] += 1;
    }
    std::sort(armor.rst.begin(), armor.rst.end(), [](bbox_t _a, bbox_t _b) { return _a.distance_center < _b.distance_center; });

    const cv::Scalar colors[4] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}, {0, 0, 0}};
    for (const auto& b : armor.rst) {
      cv::line(_src_img, b.pts[0], b.pts[1], colors[2], 2);
      cv::line(_src_img, b.pts[1], b.pts[2], colors[2], 2);
      cv::line(_src_img, b.pts[2], b.pts[3], colors[2], 2);
      cv::line(_src_img, b.pts[3], b.pts[0], colors[2], 2);
      cv::putText(_src_img, std::to_string(b.tag_id), b.pts[0], cv::FONT_HERSHEY_SIMPLEX, 3, colors[b.color_id], 3);
    }
    armor_.clear();
    armor_.shrink_to_fit();
    if (armor.rst.size() > 0) {
        return true;
    }
    return false;
}

//初始化
bool Detector::detection_init(std::string xml_path, std::string device) {  // ,double cof_threshold,double nms_area_threshold
  _xml_path = xml_path;
  // _cof_threshold      = cof_threshold;
  // _nms_area_threshold = nms_area_threshold;
  Core ie;
  auto cnnNetwork = ie.ReadNetwork(_xml_path);
  //输入设置
  InputsDataMap   inputInfo(cnnNetwork.getInputsInfo());
  InputInfo::Ptr& input = inputInfo.begin()->second;
  _input_name           = inputInfo.begin()->first;
  input->setPrecision(Precision::FP32);
  input->getInputData()->setLayout(Layout::NCHW);
  ICNNNetwork::InputShapes inputShapes  = cnnNetwork.getInputShapes();
  SizeVector&              inSizeVector = inputShapes.begin()->second;
  cnnNetwork.reshape(inputShapes);
  //输出设置
  _outputinfo = OutputsDataMap(cnnNetwork.getOutputsInfo());
  for (auto& output : _outputinfo) {
    output.second->setPrecision(Precision::FP32);
  }
  kalman_init();
  //获取可执行网络
  _network = ie.LoadNetwork(cnnNetwork, device);  // CPU or GPU
  return true;
}

//释放资源
bool Detector::uninit(){
    return true;
}

/*
    后处理所需要函数
        is_overlap() : 查看两个装甲板是否重叠
        argmax()     : 查询那个下标为的数据最大, 并返回下标
        inv_sigmoid(): 逆sigmoid
        sigmoid()    : sigmoid
*/
template<class F, class T, class ...Ts>
T reduce(F &&func, T x, Ts... xs) {
    if constexpr (sizeof...(Ts) > 0){
        return func(x, reduce(std::forward<F>(func), xs...));
    } else {
        return x;
    }
}

template<class T, class ...Ts>
T reduce_max(T x, Ts... xs) {
    return reduce([](auto &&a, auto &&b){return std::max(a, b);}, x, xs...);
}

template<class T, class ...Ts>
T reduce_min(T x, Ts... xs) {
    return reduce([](auto &&a, auto &&b){return std::min(a, b);}, x, xs...);
}

static inline bool is_overlap(const float pts1[8], const float pts2[8]) {
    cv::Rect2f bbox1, bbox2;
    bbox1.x = reduce_min(pts1[0], pts1[2], pts1[4], pts1[6]);
    bbox1.y = reduce_min(pts1[1], pts1[3], pts1[5], pts1[7]);
    bbox1.width = reduce_max(pts1[0], pts1[2], pts1[4], pts1[6]) - bbox1.x;
    bbox1.height = reduce_max(pts1[1], pts1[3], pts1[5], pts1[7]) - bbox1.y;
    bbox2.x = reduce_min(pts2[0], pts2[2], pts2[4], pts2[6]);
    bbox2.y = reduce_min(pts2[1], pts2[3], pts2[5], pts2[7]);
    bbox2.width = reduce_max(pts2[0], pts2[2], pts2[4], pts2[6]) - bbox2.x;
    bbox2.height = reduce_max(pts2[1], pts2[3], pts2[5], pts2[7]) - bbox2.y;
    return (bbox1 & bbox2).area() > 0;
}

static inline int argmax(const float *ptr, int len) {
    int max_arg = 0;
    for (int i = 1; i < len; i++) {
        if (ptr[i] > ptr[max_arg]) max_arg = i;
    }
    return max_arg;
}

constexpr float inv_sigmoid(float x) {
    return -std::log(1 / x - 1);
}

constexpr float sigmoid(float x) {
    return 1 / (1 + std::exp(-x));
}

// 降序排序
bool DescSort(index_sort a, index_sort b) {
    return a.confidence > b.confidence;
}

//处理图像获取结果
void Detector::process_frame(cv::Mat& inframe, armor_detection& armor){
    if(inframe.empty()){
        std::cout << "无效图片输入" << std::endl;
        return ;
    }
    // std::cout << "inframe.cols: " << inframe.cols << std::endl;
    // std::cout << "inframe.rows: " << inframe.rows << std::endl;
    
    // 图片预处理
    cv::Mat x;  // 不要使用static, 该Mat在后续需要转换为CV_32F类型, 但传入的图片是CV_U8类型, 使用的static会消耗多余时间用于转换类型
    float fx, fy; // x, y 轴的拉伸比例
    float f = (float)inframe.cols / (float)inframe.rows; //长宽比
    if(f > (640.f / 384.f)) {
      // 如果原图长宽比大于模型输入长宽比，则拉伸长。
      cv::resize(inframe, x, cv::Size(), (double)640 / (double)inframe.cols, (double)640 / (double)inframe.cols);
      fx = (float)inframe.cols / 640.f;
      fy = (float)inframe.cols / 640.f;
      cv::copyMakeBorder(x, x, 0, 384 - x.rows, 0, 0,
                        cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0)); // 填充黑色
    } else {
      // 如果原图长宽比小于模型输入长宽比，则拉伸宽。
      cv::resize(inframe, x, cv::Size(), (double)384 / (double)inframe.rows, (double)384 / (double)inframe.rows);
      fx = (float)inframe.rows / 384.f;
      fy = (float)inframe.rows / 384.f;
      cv::copyMakeBorder(x, x, 0, 0, 0, 640 - x.cols,
                          cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0)); // 填充黑色
    }
    cv::cvtColor(x, x, cv::COLOR_BGR2RGB);

    // 获取输入节点
    InferRequest::Ptr infer_request = _network.CreateInferRequestPtr();
    Blob::Ptr frameBlob = infer_request->GetBlob(_input_name);
    InferenceEngine::LockedMemory<void> blobMapped_input = InferenceEngine::as<InferenceEngine::MemoryBlob>(frameBlob)->wmap();
    float* blob_data = blobMapped_input.as<float*>();

    x.convertTo(x, CV_32F);
    static size_t input_size = 1*384*640*3; 
    memcpy(blob_data, x.data, input_size*sizeof(float));

    // 执行预测
    infer_request->Infer();

    // 四点模型的 _outputinfo 应该只有一个输入节点--"output"
    // 获取 output节点 
    Blob::Ptr blob = infer_request->GetBlob("output");
    LockedMemory<const void> blobMapped_output = InferenceEngine::as<InferenceEngine::MemoryBlob>(blob)->rmap();
    const float *output_blob = blobMapped_output.as<float *>();
    // output_blob[] 1*15120*20

    // 按置信度排序 [下标记录]
    std::vector<index_sort> index_desc;
    index_desc.reserve(15120);
    for (int i=0; i<15120; i++) {
        auto *buffer = output_blob + i*20;

        if (buffer[8] < inv_sigmoid(KEEP_THRES)) continue; // 置信度过滤

        index_desc.emplace_back();
        auto &temp = index_desc.back();
        temp.index = i;
        temp.confidence = buffer[8];
    }
    std::sort(index_desc.begin(), index_desc.end(), DescSort);  // 降序排序

    // 后处理获得最终检测结果
    int index_desc_size = index_desc.size();
    // std::cout << "vector_index_size:  " << index_desc_size << std::endl;
    // std::vector<bbox_t> rst;
    // rst.reserve(TOPK_NUM);
    std::vector<uint8_t> removed(TOPK_NUM);
    for (int i=0; i<TOPK_NUM ; i++) {
        if (i >= index_desc_size) {
            break;
        }
        int index_i = index_desc[i].index;

        auto *box_buffer = output_blob + index_i * 20;  // 20->23
        // if (box_buffer[8] < inv_sigmoid(KEEP_THRES)) break; // 置信度过滤
        if (removed[i]) continue;                           // 与之前高执行度框有重合, 跳过

        // 当前四点框通过了前面的过滤条件, 开始处理
        armor.rst.emplace_back();
        auto &box = armor.rst.back();
        memcpy(&box.pts, box_buffer, 8 * sizeof(float));    // 赋值, 把八个点取出来
        for (auto &pt : box.pts) pt.x *= fx, pt.y *= fy;    // 对应resize图片前后的倍数, 把四点数据映射回原图
        box.confidence  = sigmoid(box_buffer[8]);           // 置信度
        box.color_id    = argmax(box_buffer + 9, 4);        // 颜色 分类  // 0: blue, 1: red, 2: gray       3: ??
        box.tag_id      = argmax(box_buffer + 13, 7);       // 数字图案 分类  // 0: guard, 1-5: number, 6: base

        // 检查之后较低置信度的四点框和当前四点框是否有重合, 若重合, 打个标记
        for (int j = i + 1; j < TOPK_NUM; j++) {
            if (j >= index_desc_size) {
                break;
            }
            int index_j = index_desc[j].index;

            auto *box2_buffer = output_blob + index_j * 20;
            // if (box2_buffer[8] < inv_sigmoid(KEEP_THRES)) break;    // 置信度过滤
            if (removed[j]) continue;
            if (is_overlap(box_buffer, box2_buffer)) removed[j] = true;
        }
    }
}


}

