#include "FindBall.hpp"
#include "camera_distribute.hpp"
#include "ximgproc/find_ellipses.hpp"
#include <iostream>
#include "calibration.hpp"
// #define UP_DEBUG

//#define ENABLE_THRESHOLD
//#define ENABLE_IMSHOW


#ifdef ENABLE_THRESHOLD
// Trackbar callback function
void onTrackbar(int, void*) {
    // Nothing to do here, just for trackbar functionality
}
#endif // ENABLE_THRESHOLD

enum BallType { RED = 0, PURPLE = 1, BLUE = 2 };

cv::Scalar lower_purple = cv::Scalar(132, 95, 0);
cv::Scalar upper_purple = cv::Scalar(165, 230, 255);
// cv::Scalar lower_blue = cv::Scalar(104, 104, 0);
// cv::Scalar upper_blue = cv::Scalar(123, 215, 255);

// cv::Scalar lower_blue = cv::Scalar(97, 127, 0);
// cv::Scalar upper_blue = cv::Scalar(132, 245, 255);

// cv::Scalar lower_blue = cv::Scalar(100, 92, 0);
// cv::Scalar upper_blue = cv::Scalar(132, 184, 255);

// cv::Scalar lower_blue = cv::Scalar(103, 125, 0);
// cv::Scalar upper_blue = cv::Scalar(124, 233, 255);

cv::Scalar lower_blue = cv::Scalar(102, 145, 0);
cv::Scalar upper_blue = cv::Scalar(125, 235, 255);

cv::Scalar lower_red = cv::Scalar(157, 100, 0);
cv::Scalar upper_red = cv::Scalar(190, 251, 255);
cv::Scalar lower_red_1 = cv::Scalar(0, 100, 0);
cv::Scalar upper_red_1 = cv::Scalar(24, 255, 255);

cv::Scalar lower_purple_jaw = cv::Scalar(142, 67, 41);
cv::Scalar upper_purple_jaw = cv::Scalar(175, 162, 255);
cv::Scalar lower_blue_jaw = cv::Scalar(111, 114, 0);
cv::Scalar upper_blue_jaw = cv::Scalar(122, 175, 255);

cv::Scalar lower_red_jaw = cv::Scalar(157, 100, 0);
cv::Scalar upper_red_jaw = cv::Scalar(190, 251, 255);
cv::Scalar lower_red_1_jaw = cv::Scalar(0, 100, 0);
cv::Scalar upper_red_1_jaw = cv::Scalar(24, 255, 255);

FindBallServer::FindBallServer():   start_time(0.0), current_time(0.0), frame_number(-1), frame_number_record(-1), 
                                    frames_per_second(0), elapsed_seconds(0.0)
{
    std::cout << "FindBallServer created" << std::endl;
    camera_distribute();
    camera_index_read();
    combinedImage = cv::Mat(cv::Size(800, 600), CV_8UC3);
    cap = std::make_shared<cv::VideoCapture>();

    //ball_result = cv::Vec3d(0, 0, 0);

    ed = cv::ximgproc::createEdgeDrawing();
    EDParams = std::make_shared<cv::ximgproc::EdgeDrawing::Params>();
    EDinit(ed, EDParams);

    Kalman = std::make_shared<cv::KalmanFilter>(4, 2);
    Kalman->measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);
    Kalman->transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
    Kalman->processNoiseCov = (cv::Mat_<float>(4, 4) << 0.1, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0.1);
    Kalman->measurementNoiseCov = (cv::Mat_<float>(2, 2) << 0.1, 0, 0, 0.1);

    Kalman_purple = std::make_shared<cv::KalmanFilter>(4, 2);
    Kalman_purple->measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);
    Kalman_purple->transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
    Kalman_purple->processNoiseCov = (cv::Mat_<float>(4, 4) << 0.1, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0.1);
    Kalman_purple->measurementNoiseCov = (cv::Mat_<float>(2, 2) << 0.1, 0, 0, 0.1);

    situation = Situation::Direct;

    #ifdef ENABLE_THRESHOLD
    lowH = 184, lowS = 118, lowV = 150;
    highH = 194, highS = 218, highV = 255;
    #endif // ENABLE_THRESHOLD
}

FindBallServer::~FindBallServer()
{
    std::cout << "FindBallServer destroyed" << std::endl;
}

bool FindBallServer::usbcamera_init(int camera_up_index_)
{
    this->cap->open(camera_up_index_);
    this->cap->open(camera_up_index_,cv::CAP_V4L2);
    this->cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap->set(cv::CAP_PROP_FPS, 60);
    // cap->set(cv::CAP_PROP_FRAME_WIDTH, 640);
    // cap->set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap->set(cv::CAP_PROP_FRAME_WIDTH, 800);
    cap->set(cv::CAP_PROP_FRAME_HEIGHT, 600);
    if ((!cap->isOpened()) || (!cap->isOpened())) {
        return false;
    }
    return true;
}

bool FindBallServer::usbcamera_deinit()
{
    this->cap->release();
    return true;
}

bool FindBallServer::usbcamera_getImage(cv::Mat &frame)
{
    this->cap->read(frame);
    if (frame.empty()) {
        return false;
    }
    Capture_Calibration(frame, frame);
    return true;
}

bool FindBallServer::usbcamera_getImage()
{
    this->cap->read(temp_frame);
    if (temp_frame.empty()) {
        return false;
    }
    return true;
}

void FindBallServer::EDinit(cv::Ptr<cv::ximgproc::EdgeDrawing> &ed,
    std::shared_ptr<cv::ximgproc::EdgeDrawing::Params> &EDParams)
{
    EDParams->MinPathLength = 60;
    EDParams->MinLineLength = 20;
    EDParams->PFmode = false;
    EDParams->NFAValidation = true;
    ed->setParams(*EDParams);
}
/*
bool FindBallServer::findball_with_Kalman(int type, cv::Vec3d &data)
{
    frame_number = frame_number + 1;

    cv::Vec3d ref;
    // if(!find_ball(type, ref))
    //     return false;
    last_prediction = current_prediction;
    last_measurement = current_measurement;
    if (ref == cv::Vec3d(0, 0, 0))
        current_measurement = last_measurement;
    else
        current_measurement = cv::Vec2f(ref[0], ref[1]);
    Kalman->correct(cv::Mat(current_measurement));
    current_prediction = Kalman->predict();
    data[0] = current_prediction[0];
    data[1] = current_prediction[1];
    data[2] = ref[2];
    current_radius = ref[2];

    current_time = (double)cv::getTickCount() / cv::getTickFrequency();
    elapsed_seconds = (current_time - start_time);
    if (elapsed_seconds >= 1.0)
    {
        frames_per_second =  frame_number - frame_number_record;
        // 重置计时器
        start_time = current_time;
        frame_number_record = frame_number;
    }
    return true;
}
*/
bool FindBallServer::main_init(int camera_up_index_)
{
    if(!usbcamera_init(camera_up_index_))
        return false;
    for(int i = 0; i < 10; i++)
    {
        usbcamera_getImage();
    }
    start_time = (double)cv::getTickCount() / cv::getTickFrequency();
    return true;
}

void FindBallServer::imgshow_DEBUG_INIT()
{
    #ifdef ENABLE_THRESHOLD
    cv::namedWindow("Threshold Adjustments", cv::WINDOW_NORMAL);
    // Create trackbars
    cv::createTrackbar("Low H", "Threshold Adjustments", &lowH, 255, onTrackbar);
    cv::createTrackbar("High H", "Threshold Adjustments", &highH, 255, onTrackbar);
    cv::createTrackbar("Low S", "Threshold Adjustments", &lowS, 255, onTrackbar);
    cv::createTrackbar("High S", "Threshold Adjustments", &highS, 255, onTrackbar);
    cv::createTrackbar("Low V", "Threshold Adjustments", &lowV, 255, onTrackbar);
    cv::createTrackbar("High V", "Threshold Adjustments", &highV, 255, onTrackbar);
    #endif // ENABLE_THRESHOLD

    #ifdef ENABLE_IMSHOW
    cv::namedWindow("color_image", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("color", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("hsv", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("blendSRaisen", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("thre", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("edge", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("mask",cv::WINDOW_AUTOSIZE);
    cv::namedWindow("prediction",cv::WINDOW_AUTOSIZE);
    #endif // ENABLE_IMSHOW
}

void FindBallServer::imgshow_DEBUG()
{
    #ifdef ENABLE_THRESHOLD
    cv::Mat thresholdImage(100, 600, CV_8UC3, cv::Scalar(0, 0, 0));
    // Draw rectangles for each trackbar
    cv::rectangle(thresholdImage, cv::Rect(0, 0, 100, 100), cv::Scalar(lowH, lowS, lowV), -1);
    cv::rectangle(thresholdImage, cv::Rect(100, 0, 100, 100), cv::Scalar(highH, highS, highV), -1);
    cv::rectangle(thresholdImage, cv::Rect(200, 0, 100, 100), cv::Scalar(lowH, 255, 255), -1);
    cv::rectangle(thresholdImage, cv::Rect(300, 0, 100, 100), cv::Scalar(highH, 255, 255), -1);
    cv::rectangle(thresholdImage, cv::Rect(400, 0, 100, 100), cv::Scalar(255, lowS, 255), -1);
    cv::rectangle(thresholdImage, cv::Rect(500, 0, 100, 100), cv::Scalar(255, highS, 255), -1);
    cv::rectangle(thresholdImage, cv::Rect(600, 0, 100, 100), cv::Scalar(255, 255, lowV), -1);
    cv::rectangle(thresholdImage, cv::Rect(700, 0, 100, 100), cv::Scalar(255, 255, highV), -1);

    lower[1] = cv::Scalar(lowH, lowS, lowV);
    upper[1] = cv::Scalar(highH, highS, highV);
    // Show the image
    cv::imshow("Threshold Adjustments", thresholdImage);

    #endif // ENABLE_THRESHOLD

    #ifdef ENABLE_IMSHOW
    cv::Mat prediction = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::circle(prediction, cv::Point(current_prediction[0], current_prediction[1]), current_radius, cv::Scalar(0, 255, 0), -1);
    cv::imshow("prediction", prediction);

    cv::imshow("hsv", hsv);
    cv::imshow("mask", mask);
    cv::imshow("color", blendSRaisen);
    cv::imshow("thre", thre);
    cv::imshow("edge", edge_image);
    cv::imshow("blendSRaisen", blendSRaisen);
    #endif // ENABLE_IMSHOW
}

bool CameraUPServer::find_ball(int type, std::vector<cv::Vec3d> &ball_result,std::vector<cv::Vec3d> &purple_result)
{
    this->usbcamera_getImage(color_image);
    if (color_image.empty()) {
        return false;
    }

    //double frame_rate = this->cap->get(cv::CAP_PROP_FPS);
    //std::cout << "摄像头的帧率: "<< frame_rate << std::endl;

    hsv.release();
    mask.release();
    mask2.release();
    red_mask.release();
    edge_images_1.release();
    edge_images_2.release();
    img_for_purple.release();
    img_for_target.release();
    gray_for_purple.release();
    gray_for_purple.release();
    thre_for_purple.release();
    thre_for_target.release();
    ellipses_purple.clear();
    ellipses_target.clear();
    target_ellipses.clear();
    purple_ellipses.clear();

    cv::cvtColor(color_image, hsv, cv::COLOR_BGR2HSV);
    cv::medianBlur(hsv, hsv, 5);
    cv::GaussianBlur(hsv, hsv, cv::Size(9, 9), 0);

    cv::inRange(hsv, lower_purple, upper_purple, mask_for_purple);
    cv::inRange(hsv, this->lower[type], this->upper[type], mask);
    
    if(type == 0)
    {
        cv::inRange(hsv, lower_red_1, upper_red_1, mask2);
        cv::bitwise_or(mask, mask2, red_mask);
        cv::bitwise_and(hsv, hsv, img_for_target, red_mask);

    }else{
        cv::bitwise_and(hsv, hsv, img_for_target, mask);
    }
    cv::bitwise_and(color_image, color_image, img_for_purple,mask_for_purple);

    // cv::GaussianBlur(img_for_purple, img_for_purple, cv::Size(7, 7), 0);
    // cv::GaussianBlur(img_for_target, img_for_target, cv::Size(7, 7), 0);

    cv::cvtColor(img_for_purple, gray_for_purple, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_for_target, gray_for_target, cv::COLOR_BGR2GRAY);

    cv::threshold(gray_for_purple, thre_for_purple, 0, 255, cv::THRESH_BINARY);
    cv::threshold(gray_for_target, thre_for_target, 0, 255, cv::THRESH_BINARY);
    
    cv::morphologyEx(thre_for_purple, thre_for_purple, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)), cv::Point(-1, -1), 2);
    cv::morphologyEx(thre_for_target, thre_for_target, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)), cv::Point(-1, -1), 2);

    contours.clear();
    cv::findContours(thre_for_purple, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (auto contour : contours) {
        cv::fillPoly(thre_for_purple, contour, cv::Scalar(255, 255, 255));
    }
    contours.clear();
    cv::findContours(thre_for_target, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (auto contour : contours) {
        cv::fillPoly(thre_for_target, contour, cv::Scalar(255, 255, 255));
    }

    //target
    ed->detectEdges(thre_for_target);
    ed->getEdgeImage(edge_images_1);
    ed->detectEllipses(ellipses_target);

    //purple
    ed->detectEdges(thre_for_purple);
    ed->getEdgeImage(edge_images_2);
    ed->detectEllipses(ellipses_purple);

    for (size_t i=0; i<ellipses_target.size(); i++)
    {
        cv::Point center((int)ellipses_target[i][0], (int)ellipses_target[i][1]);
        cv::Size axes((int)ellipses_target[i][2] + (int)ellipses_target[i][3], (int)ellipses_target[i][2] + (int)ellipses_target[i][4]);
        target_ellipses.push_back(cv::Vec3d(center.x, center.y, axes.width/2.0 + axes.height/2.0));
        //cv::ellipse(color_image, center, axes, ellipses_target[i][5], 0, 360, cv::Scalar(0, 255, 0), 2);
    }
    // 使用 lambda 表达式进行排序
    std::sort(target_ellipses.begin(), target_ellipses.end(), 
            [](const cv::Vec3d& a, const cv::Vec3d& b) {
            return (600 - a[1])*(600 - a[1])+(400 - a[0])*(400 - a[0]) < (600 - b[1])*(600 - b[1])+(400 - b[0])*(400 - b[0]); // 按 center.y 降序排序
            });
    if(target_ellipses.size() == 0)
        return false;
    cv::Point center((int)ellipses_target[0][0], (int)ellipses_target[0][1]);
        cv::Size axes((int)ellipses_target[0][2] + (int)ellipses_target[0][3], (int)ellipses_target[0][2] + (int)ellipses_target[0][4]);
    //cv::ellipse(color_image, center, axes, ellipses_target[0][5], 0, 360, cv::Scalar(0, 255, 0), 2);
    ball_result.resize(5);
    ball_result.assign(target_ellipses.begin(), target_ellipses.end());
    if(target_ellipses.size() == 0)
        return false;
    for (size_t i=0; i<ellipses_purple.size(); i++)
    {
        cv::Point center((int)ellipses_purple[i][0], (int)ellipses_purple[i][1]);
        cv::Size axes((int)ellipses_purple[i][2] + (int)ellipses_purple[i][3], (int)ellipses_purple[i][2] + (int)ellipses_purple[i][4]);
        purple_ellipses.push_back(cv::Vec3d(center.x, center.y, axes.width/2.0 + axes.height/2.0));
        //cv::ellipse(color_image, center, axes, ellipses_purple[i][5], 0, 360, cv::Scalar(0, 0, 255), 2);
        //cv::rectangle(color_image, cv::Point(center.x - (axes.width/2.0 + axes.height/2.0), center.y - (axes.width/2.0 + axes.height/2.0)), cv::Point(center.x + (axes.width/2.0 + axes.height/2.0), center.y + (axes.width/2.0 + axes.height/2.0)), cv::Scalar(0, 0, 255), -1);
    }
    // 使用 lambda 表达式进行排序
    std::sort(purple_ellipses.begin(), purple_ellipses.end(), 
            [](const cv::Vec3d& a, const cv::Vec3d& b) {
            return (600 - a[1])*(600 - a[1])+(400 - a[0])*(400 - a[0]) < (600 - b[1])*(600 - b[1])+(400 - b[0])*(400 - b[0]); // 按 center.y 降序排序
            });
    purple_result.resize(5);
    purple_result.assign(purple_ellipses.begin(), purple_ellipses.end());

#ifdef UP_DEBUG
    up_decision_making(ball_result, purple_result, true, situation,0);
    cv::circle(color_image, cv::Point(tracking_ball.y, tracking_ball.x), tracking_ball.z, cv::Scalar(255, 0, 0), 5);
    if(situation == Situation::Purple_block)
    {
        cv::circle(color_image, cv::Point(tracking_purple.y, tracking_purple.x), tracking_purple.z, cv::Scalar(128, 0, 128), 5);
        double width = fabs(tracking_ball.y - tracking_purple.y) + tracking_ball.z + tracking_purple.z;
        double hight = fabs(tracking_ball.x - tracking_purple.x) + tracking_ball.z + tracking_purple.z;
        double rectCenterX = (tracking_ball.y + tracking_purple.y) / 2.0;
        double rectCenterY = (tracking_ball.x + tracking_purple.x) / 2.0;
        cv::Rect_<float> boundingRect( rectCenterX - width/2.0, rectCenterY - hight/2.0, width, hight);
        cv::rectangle(color_image, boundingRect, cv::Scalar(0, 255, 0), 3);
    }
#endif
    cv::Mat resized_image_color;
    cv::Mat resized_image_mask;
    // 将单通道掩码图像转换为三通道图像
    cv::Mat mask3Channel;
    
    cv::resize(color_image, resized_image_color, cv::Size(400, 300), 0, 0,cv::INTER_AREA);
    cv::resize(mask, resized_image_mask, cv::Size(400, 300), 0, 0,cv::INTER_AREA);
    cv::cvtColor(resized_image_mask, mask3Channel, cv::COLOR_GRAY2BGR);
    // 创建一个新的图像，宽度是两张图片的宽度之和，高度不变
    
    resized_image_color.copyTo(combinedImage(cv::Rect(0, 0, resized_image_color.cols, resized_image_color.rows)));
    mask3Channel.copyTo(combinedImage(cv::Rect(resized_image_color.cols, 0, mask3Channel.cols, mask3Channel.rows)));

    return true;
}

bool FindBallServer::up_decision_making(
    std::vector<cv::Vec3d> &ball_info_, 
    std::vector<cv::Vec3d> &purple_info_, bool is_found_, Situation &situation_,int reset)
{
    cv::Point3d most_near;
    static int count_lost = 0;
    static int init = 0;

    if(is_found_)
    {
        count_lost = 0;
        static int count = 0;
        most_near.x = ball_info_[0][1];
        most_near.y = ball_info_[0][0];
        most_near.z = ball_info_[0][2];

        if(init == 0)
        {
            Kalman->statePost = (cv::Mat_<float>(4, 1) << 0, 0, 0, 0);
            Kalman->errorCovPost = (cv::Mat_<float>(4, 4) <<
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1);
            current_measurement = cv::Vec2f(ball_info_[0][0], ball_info_[0][1]);
            Kalman->correct(cv::Mat(current_measurement));
            current_prediction = Kalman->predict();
            tracking_ball.x = current_prediction[1];
            tracking_ball.y = current_prediction[0];
            tracking_ball.z = ball_info_[0][2];
            if((most_near.x-tracking_ball.x)*(most_near.x-tracking_ball.x) + (most_near.y-tracking_ball.y)*(most_near.y-tracking_ball.y) < 25)
                init = 1;
        }else {
            if((most_near.x-tracking_ball.x)*(most_near.x-tracking_ball.x) + (most_near.y-tracking_ball.y)*(most_near.y-tracking_ball.y) < 900 + count*2)
            {
                current_measurement = cv::Vec2f(ball_info_[0][0], ball_info_[0][1]);
                Kalman->correct(cv::Mat(current_measurement));
                current_prediction = Kalman->predict();
                tracking_ball.x = current_prediction[1];
                tracking_ball.y = current_prediction[0];
                tracking_ball.z = ball_info_[0][2];
                count = 0;
            }else
            {
                if(count > 50)
                {
                std::cout << "No target found!" << std::endl;
                count = 0;
                Kalman->statePost = (cv::Mat_<float>(4, 1) << 0, 0, 0, 0);
                Kalman->errorCovPost = (cv::Mat_<float>(4, 4) <<
                    1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1);
                current_measurement = cv::Vec2f(ball_info_[0][0], ball_info_[0][1]);
                Kalman->correct(cv::Mat(current_measurement));
                current_prediction = Kalman->predict();
                tracking_ball.x = current_prediction[1];
                tracking_ball.y = current_prediction[0];
                tracking_ball.z = ball_info_[0][2];
                }else {
                    current_measurement = last_measurement;
                    Kalman->correct(cv::Mat(current_measurement));
                    current_prediction = Kalman->predict();
                    tracking_ball.x = current_prediction[1];
                    tracking_ball.y = current_prediction[0];
                    tracking_ball.z = ball_info_[0][2];
                }
                count++;
            }
        }

    if(situation_ == Situation::Direct)
    {
        for(auto &purple_ball : purple_info_)
        {
            if(purple_ball[1] > tracking_ball.x && fabs(purple_ball[0] - tracking_ball.y) < tracking_ball.z + purple_ball[2] + 50)
            {
                Kalman_purple->statePost = (cv::Mat_<float>(4, 1) << 0, 0, 0, 0);
                Kalman_purple->errorCovPost = (cv::Mat_<float>(4, 4) <<
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1);
                current_measurement_purple = cv::Vec2f(purple_ball[0], purple_ball[1]);
                Kalman_purple->correct(cv::Mat(current_measurement_purple));
                current_prediction_purple = Kalman_purple->predict();
                if(purple_ball[1] > tracking_ball.x && fabs(purple_ball[0] - tracking_ball.y) < tracking_ball.z + purple_ball[2] - 10)
                {
                    std::cout << "紫球挡住了目标球。。。。。。" << std::endl;
                    situation_ =Situation::Purple_block;
                }
                break;
            }
        }
    }else {
        static int count_purple = 0;
        for(size_t i = 0; i< purple_info_.size();i++)
        {
            if((purple_info_[i][1]-tracking_purple.x)*(purple_info_[i][1]-tracking_purple.x) + (purple_info_[i][0]-tracking_purple.y)*(purple_info_[i][0]-tracking_purple.y) < 800)
            {
                current_measurement_purple = cv::Vec2f(purple_info_[i][0], purple_info_[i][1]);
                Kalman_purple->correct(cv::Mat(current_measurement_purple));
                current_prediction_purple = Kalman_purple->predict();
                tracking_purple.x = current_prediction_purple[1];
                tracking_purple.y = current_prediction_purple[0];
                tracking_purple.z = purple_info_[i][2];
                count_purple = 0;
                break;
            }
            if(i == purple_info_.size()-1)
            {
                current_measurement_purple = last_measurement_purple;
                Kalman_purple->correct(cv::Mat(current_measurement_purple));
                current_prediction_purple = Kalman_purple->predict();
                tracking_purple.x = current_prediction_purple[1];
                tracking_purple.y = current_prediction_purple[0];
                count_purple ++;
            }
            if(count_purple > 20)
            {
                count_purple = 0;
                situation_ = Situation::Direct;
                std::cout << "回到Direct" << std::endl;
            }
        }
        if(fabs(tracking_purple.y - tracking_ball.y) > tracking_ball.z + tracking_purple.z - 10)
        {
            situation_ = Situation::Direct;
        }
    }

    }else {
        std::cout <<  "Ball not found......." << std::endl;
        return false;
    }
    last_prediction = current_prediction;
    last_measurement = current_measurement;
    last_radius = current_radius;

    last_measurement_purple = current_measurement_purple;
    last_prediction_purple = current_prediction_purple;

    std::cout << "target x: " << tracking_ball.x << " y: " << tracking_ball.y << std::endl;
    return true;
}

bool CameraJawServer::find_ball(int type, std::vector<cv::Vec3d> &ball_result,std::vector<cv::Vec3d> &purple_result)
{
    this->usbcamera_getImage(color_image);
    if (color_image.empty()) {
        return false;
    }

    //double frame_rate = this->cap->get(cv::CAP_PROP_FPS);
    //std::cout << "摄像头的帧率: "<< frame_rate << std::endl;

    hsv.release();
    mask.release();
    mask2.release();
    red_mask.release();
    edge_images_1.release();
    edge_images_2.release();
    img_for_purple.release();
    img_for_target.release();
    gray_for_purple.release();
    gray_for_purple.release();
    thre_for_purple.release();
    thre_for_target.release();
    ellipses_purple.clear();
    ellipses_target.clear();
    target_ellipses.clear();
    purple_ellipses.clear();

    cv::cvtColor(color_image, hsv, cv::COLOR_BGR2HSV);
    cv::medianBlur(hsv, hsv, 5);
    cv::GaussianBlur(hsv, hsv, cv::Size(9, 9), 0);

    cv::inRange(hsv, lower_purple, upper_purple, mask_for_purple);
    cv::inRange(hsv, this->lower[type], this->upper[type], mask);
    
    if(type == 0)
    {
        cv::inRange(hsv, lower_red_1, upper_red_1, mask2);
        cv::bitwise_or(mask, mask2, red_mask);
        cv::bitwise_and(hsv, hsv, img_for_target, red_mask);

    }else{
        cv::bitwise_and(hsv, hsv, img_for_target, mask);
    }
    cv::bitwise_and(color_image, color_image, img_for_purple,mask_for_purple);


    // cv::GaussianBlur(img_for_purple, img_for_purple, cv::Size(7, 7), 0);
    // cv::GaussianBlur(img_for_target, img_for_target, cv::Size(7, 7), 0);

    cv::cvtColor(img_for_purple, gray_for_purple, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_for_target, gray_for_target, cv::COLOR_BGR2GRAY);

    cv::threshold(gray_for_purple, thre_for_purple, 0, 255, cv::THRESH_BINARY);
    cv::threshold(gray_for_target, thre_for_target, 0, 255, cv::THRESH_BINARY);
    
    cv::morphologyEx(thre_for_purple, thre_for_purple, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)), cv::Point(-1, -1), 2);
    cv::morphologyEx(thre_for_target, thre_for_target, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)), cv::Point(-1, -1), 2);

    contours.clear();
    cv::findContours(thre_for_purple, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (auto contour : contours) {
        cv::fillPoly(thre_for_purple, contour, cv::Scalar(255, 255, 255));
    }
    contours.clear();
    cv::findContours(thre_for_target, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (auto contour : contours) {
        cv::fillPoly(thre_for_target, contour, cv::Scalar(255, 255, 255));
    }

    //target
    ed->detectEdges(thre_for_target);
    ed->getEdgeImage(edge_images_1);
    ed->detectEllipses(ellipses_target);

    //purple
    ed->detectEdges(thre_for_purple);
    ed->getEdgeImage(edge_images_2);
    ed->detectEllipses(ellipses_purple);

    for (size_t i=0; i<ellipses_target.size(); i++)
    {
        cv::Point center((int)ellipses_target[i][0], (int)ellipses_target[i][1]);
        cv::Size axes((int)ellipses_target[i][2] + (int)ellipses_target[i][3], (int)ellipses_target[i][2] + (int)ellipses_target[i][4]);
        target_ellipses.push_back(cv::Vec3d(center.x, center.y, axes.width/2.0 + axes.height/2.0));
        cv::ellipse(color_image, center, axes, ellipses_target[i][5], 0, 360, cv::Scalar(0, 255, 0), 2);
    }
    // 使用 lambda 表达式进行排序
    std::sort(target_ellipses.begin(), target_ellipses.end(), 
            [](const cv::Vec3d& a, const cv::Vec3d& b) {
            return (600 - a[1])*(600 - a[1])+(400 - a[0])*(400 - a[0]) < (600 - b[1])*(600 - b[1])+(400 - b[0])*(400 - b[0]); // 按 center.y 降序排序
            });
    if(target_ellipses.size() == 0)
        return false;
    cv::Point center((int)ellipses_target[0][0], (int)ellipses_target[0][1]);
        cv::Size axes((int)ellipses_target[0][2] + (int)ellipses_target[0][3], (int)ellipses_target[0][2] + (int)ellipses_target[0][4]);
    cv::ellipse(color_image, center, axes, ellipses_target[0][5], 0, 360, cv::Scalar(0, 255, 0), 2);
    ball_result.resize(5);
    ball_result.assign(target_ellipses.begin(), target_ellipses.end());

    for (size_t i=0; i<ellipses_purple.size(); i++)
    {
        cv::Point center((int)ellipses_purple[i][0], (int)ellipses_purple[i][1]);
        cv::Size axes((int)ellipses_purple[i][2] + (int)ellipses_purple[i][3], (int)ellipses_purple[i][2] + (int)ellipses_purple[i][4]);
        purple_ellipses.push_back(cv::Vec3d(center.x, center.y, axes.width/2.0 + axes.height/2.0));
        //cv::ellipse(color_image, center, axes, ellipses_purple[i][5], 0, 360, cv::Scalar(0, 0, 255), 2);
        //cv::rectangle(color_image, cv::Point(center.x - (axes.width/2.0 + axes.height/2.0), center.y - (axes.width/2.0 + axes.height/2.0)), cv::Point(center.x + (axes.width/2.0 + axes.height/2.0), center.y + (axes.width/2.0 + axes.height/2.0)), cv::Scalar(0, 0, 255), -1);
    }
    // 使用 lambda 表达式进行排序
    std::sort(purple_ellipses.begin(), purple_ellipses.end(), 
            [](const cv::Vec3d& a, const cv::Vec3d& b) {
            return (600 - a[1])*(600 - a[1])+(400 - a[0])*(400 - a[0]) < (600 - b[1])*(600 - b[1])+(400 - b[0])*(400 - b[0]); // 按 center.y 降序排序
            });
    purple_result.resize(5);
    purple_result.assign(purple_ellipses.begin(), purple_ellipses.end());
    return true;
}

CameraUPServer::CameraUPServer(): FindBallServer()
{
    std::cout << "CameraUPServer created" << std::endl;
    main_init(camera_up_index);
}

CameraJawServer::CameraJawServer(): FindBallServer()
{
    std::cout << "CameraJawServer created" << std::endl;
    main_init(camera_jaw_index);
}

CameraUPServer::~CameraUPServer()
{
    std::cout << "CameraUPServer destroyed" << std::endl;
    usbcamera_deinit();
}

CameraJawServer::~CameraJawServer()
{
    std::cout << "CameraJawServer destroyed" << std::endl;
    usbcamera_deinit();
}

bool FindBallServer::find_ball(int type, std::vector<cv::Vec3d> &ball_result,std::vector<cv::Vec3d> &purple_result)
{
    std::cerr << "错误调用！！！！！！" << std::endl;
    return false;
}

void FindBallServer::get_color_img(cv::Mat &frame)
{
    std::cerr << "错误调用！！！！！！" << std::endl;
}

void CameraJawServer::get_color_img(cv::Mat &frame)
{
    frame = this->color_image;
}

void CameraUPServer::get_color_img(cv::Mat &frame)
{
    frame = this -> color_image;
}

bool CameraJawServer::usbcamera_getImage(cv::Mat &frame)
{
    this->cap->read(frame);
    if (frame.empty()) {
        return false;
    }
    return true;
}

bool CameraUPServer::usbcamera_getImage(cv::Mat &frame)
{
    this->cap->read(frame);
    if (frame.empty()) {
        return false;
    }
    Capture_Calibration(frame, frame);
    return true;
}