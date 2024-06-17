/************************************
 * 去畸变部分处理
 */
#ifndef __CALIBRATION_HPP
#define __CALIBRATION_HPP

#include <opencv2/opencv.hpp>

extern cv::Mat cap_frame_cli;

void Capture_Calibration(cv::Mat &frame, cv::Mat &res_frame);

#endif