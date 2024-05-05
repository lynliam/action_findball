#include "FindBall.hpp"
#include "camera_distribute.hpp"
#include "ximgproc/find_ellipses.hpp"
#include <iostream>

//#define ENABLE_THRESHOLD
//#define ENABLE_IMSHOW


#ifdef ENABLE_THRESHOLD
// Trackbar callback function
void onTrackbar(int, void*) {
    // Nothing to do here, just for trackbar functionality
}
#endif // ENABLE_THRESHOLD

enum BallType { RED = 0, PURPLE = 1, BLUE = 2 };

cv::Scalar lower_purple = cv::Scalar(142, 67, 41);
cv::Scalar upper_purple = cv::Scalar(175, 162, 255);
cv::Scalar lower_blue = cv::Scalar(111, 114, 0);
cv::Scalar upper_blue = cv::Scalar(122, 175, 255);

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
    cap = std::make_shared<cv::VideoCapture>();

    //ball_result = cv::Vec3d(0, 0, 0);

    ed = cv::ximgproc::createEdgeDrawing();
    EDParams = std::make_shared<cv::ximgproc::EdgeDrawing::Params>();
    EDinit(ed, EDParams);

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
    usbcamera_getImage(color_image);
    if (color_image.empty()) {
        return false;
    }

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

    cv::inRange(hsv, lower_purple, upper_purple, mask_for_purple);
    cv::inRange(hsv, this->lower[type], this->upper[type], mask);
    
    if(type == 0)
    {
        cv::inRange(hsv, lower_red_1, upper_red_1, mask2);
        cv::bitwise_or(mask, mask2, red_mask);
        cv::bitwise_and(color_image, color_image, img_for_target, red_mask);

    }else{
        cv::bitwise_and(color_image, color_image, img_for_target, mask);
    }
    cv::bitwise_and(color_image, color_image, img_for_purple,mask_for_purple);


    cv::GaussianBlur(img_for_purple, img_for_purple, cv::Size(9, 9), 0);
    cv::GaussianBlur(img_for_target, img_for_target, cv::Size(9, 9), 0);

    cv::cvtColor(img_for_purple, gray_for_purple, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_for_target, gray_for_target, cv::COLOR_BGR2GRAY);

    cv::threshold(gray_for_purple, thre_for_purple, 0, 255, cv::THRESH_BINARY);
    cv::threshold(gray_for_target, thre_for_target, 0, 255, cv::THRESH_BINARY);
    
    cv::morphologyEx(thre_for_purple, thre_for_purple, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)), cv::Point(-1, -1), 2);
    cv::morphologyEx(thre_for_target, thre_for_target, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)), cv::Point(-1, -1), 2);

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
            return a[1] > b[1]; // 按 center.y 降序排序
            });
    if(target_ellipses.size() == 0)
        return false;
    ball_result.resize(5);
    ball_result.assign(target_ellipses.begin(), target_ellipses.end());

    for (size_t i=0; i<ellipses_purple.size(); i++)
    {
        cv::Point center((int)ellipses_purple[i][0], (int)ellipses_purple[i][1]);
        cv::Size axes((int)ellipses_purple[i][2] + (int)ellipses_purple[i][3], (int)ellipses_purple[i][2] + (int)ellipses_purple[i][4]);
        purple_ellipses.push_back(cv::Vec3d(center.x, center.y, axes.width/2.0 + axes.height/2.0));
        cv::ellipse(color_image, center, axes, ellipses_purple[i][5], 0, 360, cv::Scalar(0, 0, 255), 2);
        //cv::rectangle(color_image, cv::Point(center.x - (axes.width/2.0 + axes.height/2.0), center.y - (axes.width/2.0 + axes.height/2.0)), cv::Point(center.x + (axes.width/2.0 + axes.height/2.0), center.y + (axes.width/2.0 + axes.height/2.0)), cv::Scalar(0, 0, 255), -1);
    }
    // 使用 lambda 表达式进行排序
    std::sort(purple_ellipses.begin(), purple_ellipses.end(), 
            [](const cv::Vec3d& a, const cv::Vec3d& b) {
            return a[1] > b[1]; // 按 center.y 降序排序
            });
    purple_result.resize(5);
    purple_result.assign(purple_ellipses.begin(), purple_ellipses.end());
    return true;
}

bool CameraJawServer::find_ball(int type, std::vector<cv::Vec3d> &ball_result,std::vector<cv::Vec3d> &purple_result)
{
    usbcamera_getImage(color_image);
    if (color_image.empty()) {
        return false;
    }

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

    cv::inRange(hsv, lower_purple, upper_purple, mask_for_purple);
    cv::inRange(hsv, this->lower[type], this->upper[type], mask);
    
    if(type == 0)
    {
        cv::inRange(hsv, lower_red_1, upper_red_1, mask2);
        cv::bitwise_or(mask, mask2, red_mask);
        cv::bitwise_and(color_image, color_image, img_for_target, red_mask);

    }else{
        cv::bitwise_and(color_image, color_image, img_for_target, mask);
    }
    cv::bitwise_and(color_image, color_image, img_for_purple,mask_for_purple);


    cv::GaussianBlur(img_for_purple, img_for_purple, cv::Size(9, 9), 0);
    cv::GaussianBlur(img_for_target, img_for_target, cv::Size(9, 9), 0);

    cv::cvtColor(img_for_purple, gray_for_purple, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_for_target, gray_for_target, cv::COLOR_BGR2GRAY);

    cv::threshold(gray_for_purple, thre_for_purple, 0, 255, cv::THRESH_BINARY);
    cv::threshold(gray_for_target, thre_for_target, 0, 255, cv::THRESH_BINARY);
    
    cv::morphologyEx(thre_for_purple, thre_for_purple, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)), cv::Point(-1, -1), 2);
    cv::morphologyEx(thre_for_target, thre_for_target, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)), cv::Point(-1, -1), 2);

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
            return a[1] > b[1]; // 按 center.y 降序排序
            });
    if(target_ellipses.size() == 0)
        return false;
    ball_result.resize(5);
    ball_result.assign(target_ellipses.begin(), target_ellipses.end());

    for (size_t i=0; i<ellipses_purple.size(); i++)
    {
        cv::Point center((int)ellipses_purple[i][0], (int)ellipses_purple[i][1]);
        cv::Size axes((int)ellipses_purple[i][2] + (int)ellipses_purple[i][3], (int)ellipses_purple[i][2] + (int)ellipses_purple[i][4]);
        purple_ellipses.push_back(cv::Vec3d(center.x, center.y, axes.width/2.0 + axes.height/2.0));
        cv::ellipse(color_image, center, axes, ellipses_purple[i][5], 0, 360, cv::Scalar(0, 0, 255), 2);
        //cv::rectangle(color_image, cv::Point(center.x - (axes.width/2.0 + axes.height/2.0), center.y - (axes.width/2.0 + axes.height/2.0)), cv::Point(center.x + (axes.width/2.0 + axes.height/2.0), center.y + (axes.width/2.0 + axes.height/2.0)), cv::Scalar(0, 0, 255), -1);
    }
    // 使用 lambda 表达式进行排序
    std::sort(purple_ellipses.begin(), purple_ellipses.end(), 
            [](const cv::Vec3d& a, const cv::Vec3d& b) {
            return a[1] > b[1]; // 按 center.y 降序排序
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