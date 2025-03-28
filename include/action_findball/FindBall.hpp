#ifndef FINDBALL_HPP
#define FINDBALL_HPP

//   #define ENABLE_THRESHOLD

#include <memory>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>
#include <opencv2/opencv.hpp>
#include "opencv2/ximgproc.hpp"
#include "camera_distribute.hpp"


extern cv::Scalar lower_purple;
extern cv::Scalar upper_purple;
extern cv::Scalar lower_red;
extern cv::Scalar upper_red;
extern cv::Scalar lower_blue;
extern cv::Scalar upper_blue;

enum class Situation : int8_t
{
    Direct = 0,
    Purple_block = 1
};

class FindBallServer
{
    public:
    std::shared_ptr<cv::VideoCapture> cap;

    FindBallServer();
    ~FindBallServer();   // 析构函数

    virtual bool find_ball(int type, std::vector<cv::Vec3d> &ball_result,std::vector<cv::Vec3d> &purple_result);
    virtual void get_color_img(cv::Mat &frame);

    bool usbcamera_init(int camera_up_index_);
    bool usbcamera_deinit();
    virtual bool usbcamera_getImage(cv::Mat &frame);
    bool usbcamera_getImage();
    bool up_decision_making(
    std::vector<cv::Vec3d> &ball_info_, 
    std::vector<cv::Vec3d> &purple_info_, bool is_found_, Situation &situation_,int reset);

    void EDinit(cv::Ptr<cv::ximgproc::EdgeDrawing> &ed,
    std::shared_ptr<cv::ximgproc::EdgeDrawing::Params> &EDParams);

    //bool findball_with_Kalman(int type, cv::Vec3d &data);
    bool main_init(int camera_up_index_);
    void imgshow_DEBUG_INIT();
    void imgshow_DEBUG();

    //std::shared_ptr<cv::KalmanFilter> Kalman;

    cv::Mat color_image;
    cv::Mat combinedImage;
    protected:
    std::vector<cv::Scalar> lower = {lower_red, lower_purple, lower_blue};
    std::vector<cv::Scalar> upper = {upper_red, upper_purple, upper_blue};
    
    cv::Vec3d ball_result;

    // variables
    cv::Mat temp_frame;
    cv::Mat hsv;
    cv::Mat mask;
    cv::Mat mask2;
    cv::Mat red_mask;

    cv::Mat edge_images_1;
    cv::Mat edge_images_2;
    cv::Mat mask_for_target;
    cv::Mat mask_for_purple;
    cv::Mat img_for_target;
    cv::Mat img_for_purple;
    cv::Mat gray_for_target;
    cv::Mat gray_for_purple;
    cv::Mat thre_for_target;
    cv::Mat thre_for_purple;

    std::vector<std::vector<cv::Point>> contours;

    //边缘检测variables
    std::vector<cv::Vec6d> ellipses_target;
    std::vector<cv::Vec6d> ellipses_purple;
    std::vector<cv::Vec3d> target_ellipses;
    std::vector<cv::Vec3d> purple_ellipses;

    cv::Ptr<cv::ximgproc::EdgeDrawing> ed;
    std::shared_ptr<cv::ximgproc::EdgeDrawing::Params> EDParams;

    // Kalman variables
    cv::Vec2f last_measurement;
    cv::Vec2f current_measurement ;
    cv::Vec4f last_prediction ;
    cv::Vec4f current_prediction ;
    int current_radius;
    int last_radius;
    std::shared_ptr<cv::KalmanFilter> Kalman;

    cv::Vec2f last_measurement_purple;
    cv::Vec2f current_measurement_purple ;
    cv::Vec4f last_prediction_purple ;
    cv::Vec4f current_prediction_purple ;
    std::shared_ptr<cv::KalmanFilter> Kalman_purple;

    cv::Point3d tracking_ball;
    cv::Point3d tracking_purple;

    Situation situation;
    
    // 计时开始
    double start_time;
    double current_time;
    int frame_number;
    int frame_number_record;
    int frames_per_second;
    double elapsed_seconds;

    //DEGUB
    // Global variables to hold the threshold values
    #ifdef ENABLE_THRESHOLD
    int lowH, lowS, lowV;
    int highH, highS, highV;
    #endif // ENABLE_THRESHOLD
};

class CameraUPServer : public FindBallServer
{
    public:
    CameraUPServer();
    ~CameraUPServer();
    bool find_ball(int type, std::vector<cv::Vec3d> &ball_result,std::vector<cv::Vec3d> &purple_result);
    void get_color_img(cv::Mat &frame);
    bool usbcamera_getImage(cv::Mat &frame);
    private:
};

class CameraJawServer : public FindBallServer
{
    public:
    CameraJawServer();
    ~CameraJawServer();
    void get_color_img(cv::Mat &frame);
    bool usbcamera_getImage(cv::Mat &frame);
    bool find_ball(int type, std::vector<cv::Vec3d> &ball_result,std::vector<cv::Vec3d> &purple_result);
    private:
};

#endif // FINDBALL_HPP