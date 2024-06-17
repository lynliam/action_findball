#include "calibration.hpp"

cv::Mat mtx = (cv::Mat_<double>(3, 3) << 639.33499058, 0.00000000e+00, 401.36467711, 0, 639.11256485, 303.72173882, 0, 0, 1); // 相机内参数矩阵

//cv::Mat mtx = (cv::Mat_<double>(3, 3) << 486.57833874, 0, 593.50714193, 0, 487.00568217, 330.37584242, 0, 0, 1); // 相机内参数矩阵
/*
mtx:
 [[639.33499058   0.         401.36467711]
 [  0.         639.11256485 303.72173882]
 [  0.           0.           1.        ]]
dist畸变值:
 [[ 0.01425356 -0.14932095 -0.00207266  0.00343363  0.15796778]]
*/

// [[ 5.90860112e+00,-1.09602882e+03, -6.78501204e-01, -6.91232373e-02, 3.99310077e+04]]
cv::Mat dist = (cv::Mat_<double>(1, 5) << 0.01425356, -0.14932095, -0.00207266,  0.00343363,  0.15796778); // 畸变系数向量

cv::Mat cap_frame_cli;

/**
 * @brief   摄像头去畸变函数
 */
void Capture_Calibration(cv::Mat &frame, cv::Mat &res_frame)
{
    //res_frame.release();
    int u = frame.rows;
    int v = frame.cols;
    cv::Rect validROI; // 输出的有效ROI区域

    // 使用getOptimalNewCameraMatrix优化内参矩阵
    cv::Mat optimizedCameraMatrix = getOptimalNewCameraMatrix(mtx, dist, cv::Size(u, v), 0, cv::Size(u, v), &validROI);

    // 初始化去畸变映射
    cv::Mat mapx, mapy;
    initUndistortRectifyMap(mtx, dist, cv::Mat(), optimizedCameraMatrix, cv::Size(v, u), CV_32FC1, mapx, mapy);

    // 去畸变
    cv::remap(frame, res_frame, mapx, mapy, cv::INTER_LINEAR);
}
