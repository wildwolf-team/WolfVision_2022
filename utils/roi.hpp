#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

class RoI
{
private:

public:
    /**
     * @brief Get the Roi object
     * 
     * @return cv::Rect 
     */
    cv::Rect getRoi(const cv::RotatedRect _target_rect, const float coefficient = 1.0f) {
      float w = _target_rect.size.width * coefficient;
      float h = _target_rect.size.height * coefficient;
      return cv::Rect(_target_rect.center.x - (w * 0.5), _target_rect.center.y - (h * 0.5), w, h);
    }
    RoI() : tl_(cv::Point2d(0, 0)) {}

    virtual ~RoI() {}
    /**
   * @brief 截取 ROI 图像
   *
   * @param _input_img 需要截取的图像
   * @param _rect      需要截取的位置(外接矩形)
   * @return cv::Mat   返回截取后的图片
   * @author RCX
   */
    cv::Mat cutRoIRect(const cv::Mat& _input_img, const cv::Rect& _rect) {
      tl_ = _rect.tl();
      _input_img(_rect).copyTo(roi_img_);

      return roi_img_;
    }
    /**
     * @brief ROI 安全条件
     * 
     * @param _input_img 
     * @param _r_rect 
     * @return cv::Rect 
     */
    cv::Rect makeRectSafeTailor(const cv::Mat& _input_img, const cv::Rect& _r_rect) {
      int width  = _r_rect.width;
      int height = _r_rect.height;

      cv::Point tl = _r_rect.tl();
      // 限制 ROI 出界条件
      if (tl.x < 0) {
        width -= 0 - tl.x;
        tl.x = 0;
      }
      if (tl.y < 0) {
        height -= 0 - tl.y;
        tl.y = 0;
      }

      if (tl.x + width > _input_img.cols) {
        width -= (tl.x + width - _input_img.cols);
      }
      if (tl.y + height > _input_img.rows) {
        height -= (tl.y + height - _input_img.rows);
      }

      return cv::Rect(tl.x, tl.y, width, height);
    }

protected:
    cv::Mat     roi_img_;
    cv::Point2d tl_;
};