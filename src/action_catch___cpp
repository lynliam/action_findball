#include <memory>
#include <mutex>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>
#include <std_msgs/msg/detail/u_int32__builder.hpp>

#include <vector>
#include <thread>
#include <functional>
#include <thread>
#include <cmath>
#include <string>
#include <iostream>
#include <filesystem>


#include "action_catch.hpp"
#include "tinyxml2.h"

// 判定球将被抓住的阈值，相对于 image_size_half
const int x_catch = 280;
const int y_catch = 328;
// 判定球进入下一区域的阈值
const int x_next = 340;


// which node to handle
static constexpr char const * lifecycle_node = "findball";
static constexpr char const * node_get_state_topic = "findball/get_state";
static constexpr char const * node_change_state_topic = "findball/change_state";

template<typename FutureT, typename WaitTimeT>
    std::future_status action_findball::wait_for_result( FutureT & future, WaitTimeT time_to_wait)
{
    auto end = std::chrono::steady_clock::now() + time_to_wait;
    std::chrono::milliseconds wait_period(100);
    std::future_status status = std::future_status::timeout;
    do {
        auto now = std::chrono::steady_clock::now();
        auto time_left = end - now;
        if (time_left <= std::chrono::seconds(0)) {break;}
        status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
    } while (rclcpp::ok() && status != std::future_status::ready);
    return status;
}

action_findball::ActionCatchBall::ActionCatchBall(const std::string & node_name,  const rclcpp::NodeOptions & options)
    :Node(node_name, options), 
    PIDController_x(0.0004,0.00001,0.001),             
    PIDController_y(0.0006,0.00001,0.0001),
    PIDController_w(1.3,0.02,0.06),
    PIDController_x_near(0.0005,0.00001,0.001),
    PIDController_y_near(0.0005,0.0,0.0),
    findball_node_state_(false)
{
    RCLCPP_INFO(this->get_logger(), "ActionCatchBall Node has been created");
    
    // lifecycle node init
    client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(node_get_state_topic);
    client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(node_change_state_topic);

    // callback group init
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // action server init
    this->action_server_ = rclcpp_action::create_server<CatchBall>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "catchball",
    std::bind(&ActionCatchBall::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ActionCatchBall::handle_cancel, this, std::placeholders::_1),
    std::bind(&ActionCatchBall::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    callback_group_);

    //findball_node_init();

    ballinfo_sub_= this->create_subscription<rc2024_interfaces::msg::BallInfo>("ball_info",2, std::bind(&ActionCatchBall::ballinfo_callback,this,std::placeholders::_1));

    sub_notification_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
                            "/findball/transition_event", 10, std::bind(&ActionCatchBall::notification_callback, this, std::placeholders::_1));

    chassis_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("car/cmd_vel", 2);
    chassis_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("filtered_odom", 2, std::bind(&::action_findball::ActionCatchBall::get_pose_callback,this,std::placeholders::_1));
    imu_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("car/imu", 2, std::bind(&ActionCatchBall::get_imu_callback,this,std::placeholders::_1));
    up_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/car/up_cmd", 2);
    up_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/car/up_fdbk", 3, std::bind(&ActionCatchBall::get_jointstate_callback,this,std::placeholders::_1));

    // 等待findball节点上线，并初始化findball节点
    //findball_node_state_ = findball_node_init();
    PIDController_w.PID_MaxMin(50, -50);
    PIDController_w.integralMax = 30;
    PIDController_w.integralMin = -30;

    Kalman = std::make_shared<cv::KalmanFilter>(4, 2);
    Kalman->measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);
    Kalman->transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
    Kalman->processNoiseCov = (cv::Mat_<float>(4, 4) << 0.1, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0.1);

    acquire_PID_variable();
    // std_msgs::msg::UInt32 test; 
    // test.data = 3500;
    // up_pub_->publish(test);
}

unsigned int action_findball::ActionCatchBall::get_state(std::chrono::seconds timeout)
{
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    if(!client_get_state_->wait_for_service(timeout))
    {
        if(!client_get_state_->wait_for_service(timeout))
        {
            RCLCPP_ERROR(this->get_logger(), "Service %s not available.", client_get_state_->get_service_name());
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }
    }
    
    // ask for the node to return its state
    auto future_result = client_get_state_->async_send_request(request).future.share();

    auto future_status = wait_for_result(future_result, timeout);

    if (future_status != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "Server time out while getting current state for node %s", lifecycle_node);
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    if(future_result.get()){
        RCLCPP_INFO(this->get_logger(), "Node %s is in current state %s", lifecycle_node, future_result.get()->current_state.label.c_str());
        return future_result.get()->current_state.id;
    }else{
        RCLCPP_ERROR(this->get_logger(), "Failed to get current state for node %s", lifecycle_node);
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
}

bool  action_findball::ActionCatchBall::change_state(std::uint8_t transition, std::chrono::seconds timeout)
{
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if(!client_change_state_->wait_for_service(timeout))
    {
        RCLCPP_INFO(this->get_logger(), "Service %s not available.", client_change_state_->get_service_name());
        return false;
    }

    auto future_result = client_change_state_->async_send_request(request).future.share();
    auto future_state = wait_for_result(future_result, timeout);

    if(future_state != std::future_status::ready)
    {
        RCLCPP_ERROR(this->get_logger(), "Server time out while changing state for node %s", lifecycle_node);
        return false;
    }

    if(future_result.get()->success)
    {
        RCLCPP_INFO(this->get_logger(), "Transition %s was successful for node %d", lifecycle_node, static_cast<int>(transition));
        return true;
    }else{
        RCLCPP_WARN(this->get_logger(), "Transition %s failed for node %u", lifecycle_node, static_cast<int>(transition));
        return false;
    }
}

rclcpp_action::GoalResponse action_findball::ActionCatchBall::handle_goal(
const rclcpp_action::GoalUUID & uuid,
std::shared_ptr<const CatchBall::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request, target color: %d",goal->color);
    RCLCPP_INFO(this->get_logger(), "Received goal request id : %s", reinterpret_cast<const char *>(uuid.data()));
    if(goal->color > 3)
    {
        RCLCPP_INFO(this->get_logger(), "Wrong color index, please choose from 0 to 2");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse action_findball::ActionCatchBall::handle_cancel(
    const std::shared_ptr<GoalHandleCatchBall> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;

    // if(!change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
    // {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to deactive node %s", lifecycle_node);
    // }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void action_findball::ActionCatchBall::handle_accepted(const std::shared_ptr<GoalHandleCatchBall> goal_handle)
{
    using namespace std::placeholders;
    std::thread{std::bind(&ActionCatchBall::test_execute, this, _1), goal_handle}.detach();
}

void action_findball::ActionCatchBall::execute(const std::shared_ptr<GoalHandleCatchBall> goal_handle)
{}
/*
{
    //动作延迟初始化
    rclcpp::Rate loop_rate(20);
    rclcpp::Rate action_rate(2);

    //控制器参数初始化
    int count = 0;
    acquire_PID_variable();

    //球信息初始化
    std::unique_lock<std::mutex> lock(variable_mutex_);
    ball_info.clear();
    purple_info.clear();
    lock.unlock();

    //获取goal && result && feedback
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<CatchBall::Result>();
    auto feedback = std::make_shared<CatchBall::Feedback>();
    auto & feedback_msg = feedback->progress;

    //判断颜色是否正确
    if(!change_state_to_active())
    {
        RCLCPP_INFO(this->get_logger(), "different color index");
        result->time = this->now().seconds() + this->now().nanoseconds() * 1e-9;
        result->state = 1;
        goal_handle->abort(result);
        return;
    }

    action_rate.sleep();

    RCLCPP_INFO(this->get_logger(), "Executing goal, id: %s", std::string(reinterpret_cast<const char *>(goal_handle->get_goal_id().data())).c_str());

    //初始化速度控制msg
    geometry_msgs::msg::Pose2D Data_To_Pub;
    int flag = 0;
    int flag_for_w = 0;

    //初始化机械臂控制msg
    std::shared_ptr< sensor_msgs::msg::JointState> JointControl_to_pub = std::make_shared<sensor_msgs::msg::JointState>(); 
    JointControl_to_pub->header.frame_id="joint_control";
    JointControl_to_pub->name.push_back("Joint1");
    JointControl_to_pub->name.push_back("Joint2");
    JointControl_to_pub->name.push_back("Joint3");
    JointControl_to_pub->name.push_back("Joint4");

    action_rate.sleep();

    // 关节初始化位置代码
    JointControl_to_pub->position.push_back(0.0);
    JointControl_to_pub->position.push_back(-1.553);
    JointControl_to_pub->position.push_back(-1.285);
    JointControl_to_pub->position.push_back(0.405);
    JointControl_to_pub->header.stamp = this->now();
    up_pub_->publish(*JointControl_to_pub);

    std::vector<geometry_msgs::msg::Point32> ball_info__;        
    std::vector<geometry_msgs::msg::Point32> purple_info__;
    bool is_found_ = false;

    //开始执行
    while (rclcpp::ok()) {
        if(goal_handle->is_canceling())
        {
            Data_To_Pub.x = 0;
            Data_To_Pub.y = 0;
            Data_To_Pub.theta = 0;
            chassis_pub_->publish(Data_To_Pub);

            // 关节归位代码
            // ...
            JointControl_to_pub->position.push_back(0.0);
            JointControl_to_pub->position.push_back(-1.553);
            JointControl_to_pub->position.push_back(-1.285);
            JointControl_to_pub->position.push_back(0.405);
            JointControl_to_pub->header.stamp = this->now();
            up_pub_->publish(*JointControl_to_pub);

            result->time = this->now().seconds() + this->now().nanoseconds() * 1e-9;
            result->state = static_cast<int>(CatchBallState::CANCEL);
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }
        ball_info__.clear();
        purple_info__.clear();

        lock.lock();
        ball_info__.assign(ball_info.begin(), ball_info.end());
        purple_info__.assign(purple_info.begin(), purple_info.end());
        is_found_ = is_found;
        lock.unlock();

        up_decision_making(ball_info__,purple_info__,is_found_);

        if(!is_found_)
        {
            count++;
            if(count > 20*5)
            {
                if(!change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to deactive node %s", lifecycle_node);
                }
                RCLCPP_INFO(this->get_logger(), "Ball not found for about 10s");
                result->time = this->now().seconds() + this->now().nanoseconds() * 1e-9;
                result->state = static_cast<int>(CatchBallState::TIMEOUT);
                goal_handle->abort(result);
                return;
            }
            Data_To_Pub.x = 0;
            Data_To_Pub.y = 0;
            Data_To_Pub.theta = 0.0;
            chassis_pub_->publish(Data_To_Pub);

            loop_rate.sleep();
            continue;
        }else if(is_found_ && count > 0){
            count = 0;
        }

        if( flag_for_w == 0)
        {
            Data_To_Pub.x = 0;
            Data_To_Pub.y = 0;
            Data_To_Pub.theta = -PIDController_w.PosePID_Calc(-atan2(y_catch - ballinfo__.y, x_catch - ballinfo__.x));
            RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, theta: %f", Data_To_Pub.x, Data_To_Pub.y, Data_To_Pub.theta);
            if(int(fabs(y_catch - ballinfo__.y)) < 25)
            {
                flag_for_w = 1;
            }
        } 
        else if(flag_for_w == 1){
            if(flag == 0)
            {
                if(ballinfo__.y < 468 && ballinfo__.y > 161 && ballinfo__.x > 333)
                {
                    flag = 1;
                }
                Data_To_Pub.x = -PIDController_x.PID_Calc(x_next - ballinfo__.x);
                Data_To_Pub.y = -PIDController_y.PID_Calc(y_catch - ballinfo__.y);
                Data_To_Pub.theta = 0;
            }else {
                    test.data = 3350;
                    up_pub_->publish(test);
                    
                    Data_To_Pub.x = -PIDController_x_near.PID_Calc(x_catch - ballinfo__.x);
                    Data_To_Pub.y = -PIDController_y_near.PID_Calc(y_catch - ballinfo__.y);
                    Data_To_Pub.theta = 0;
            }
        }

        if(x_catch < ballinfo__.x && std::fabs(ballinfo__.y - y_catch)< 25)
        {
            //RCLCPP_INFO(this->get_logger(), "Goal Succeed!!");
            //RCLCPP_INFO(this->get_logger(), "%f,%f,%f",ballinfo__.x,ballinfo__.y,ballinfo__.z);
            Data_To_Pub.x = 0;
            Data_To_Pub.y = 0;
            Data_To_Pub.theta = 0;
            chassis_pub_->publish(Data_To_Pub);
            break;
        }
        
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, theta: %f", Data_To_Pub.x, Data_To_Pub.y, Data_To_Pub.theta);

        chassis_pub_->publish(Data_To_Pub);
        feedback_msg = 0.5;
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
    }

    if(rclcpp::ok())
    {
        if(!change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to deactive node %s", lifecycle_node);
        }
        action_rate.sleep();
        up_cmd.data = static_cast<int>(UpCmd::CatchBall);
        test.data = 3500;
        up_pub_->publish(test);
        RCLCPP_INFO(this->get_logger(), "UpCmd: %d", up_cmd.data);
        up_pub_->publish(up_cmd);

        action_rate.sleep();
        action_rate.sleep();
        action_rate.sleep();
        action_rate.sleep();
        action_rate.sleep();
        action_rate.sleep();
        
        result->time = this->now().seconds();
        result->state = static_cast<int>(CatchBallState::SUCCEED);

        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
}
*/

void action_findball::ActionCatchBall::test_execute(const std::shared_ptr<GoalHandleCatchBall> goal_handle)
{
    rclcpp::Rate loop_rate(20);
    rclcpp::Rate action_rate(1);
    
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<CatchBall::Result>();

    RCLCPP_INFO(this->get_logger(), "Executing goal, id: %s", std::string(reinterpret_cast<const char *>(goal_handle->get_goal_id().data())).c_str());
    auto feedback = std::make_shared<CatchBall::Feedback>();
    auto & feedback_msg = feedback->progress;

    //初始化机械臂控制msg
    std::shared_ptr< sensor_msgs::msg::JointState> JointControl_to_pub = std::make_shared<sensor_msgs::msg::JointState>(); 
    sensor_msgs::msg::JointState JointState_;
    sensor_msgs::msg::JointState JointState_Last;
    JointState_.position.resize(4);
    JointState_.position[0] = 0.0;
    JointState_.position[1] = 0.0;
    JointState_.position[2] = 0.0;
    JointState_.position[3] = 0.0;
    JointState_Last.position.resize(4);
    JointState_Last.position.assign(JointState_.position.begin(), JointState_.position.end());

    JointControl_to_pub->header.frame_id="joint_control";
    JointControl_to_pub->name.push_back("Joint1");
    JointControl_to_pub->name.push_back("Joint2");
    JointControl_to_pub->name.push_back("Joint3");
    JointControl_to_pub->name.push_back("Joint4");

    // 获取初始位置
    std::unique_lock<std::mutex> lock(variable_mutex__1);
    JointState_.position.clear();
    JointState_.position.resize(4);
    JointState_.position.assign(up_joint_state.position.begin(), up_joint_state.position.end());
    lock.unlock();
    // 关节初始化位置代码
    // JointControl_to_pub->position.push_back(0.0);
    // JointControl_to_pub->position.push_back(-1.553);
    // JointControl_to_pub->position.push_back(-1.285);
    // JointControl_to_pub->position.push_back(0.405);
    // JointControl_to_pub->header.stamp = this->now();
    // up_pub_->publish(*JointControl_to_pub);
    arm_executor(JointControl_to_pub, JointState_, 0.0, 0.009, 0.594, -0.400);
    //action_rate.sleep();
    int state_ = 0;

    while (rclcpp::ok() && state_ <= 5) {
        if(goal_handle->is_canceling())
        {
            arm_executor(JointControl_to_pub, JointState_, 0.0, 0.009, 0.594, -0.400);
            result->time = this->now().seconds() + this->now().nanoseconds() * 1e-9;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }
        
        std::unique_lock<std::mutex> lock(variable_mutex__1);
        JointState_.position.clear();
        JointState_.position.resize(4);
        JointState_.position.assign(up_joint_state.position.begin(), up_joint_state.position.end());
        lock.unlock();
        //RCLCPP_INFO(this->get_logger(), "JointState: %f, %f, %f, %f", JointState_.position[0], JointState_.position[1], JointState_.position[2], JointState_.position[3]);
        RCLCPP_INFO(this->get_logger(),"state: %d", state_);
        switch (state_)
        {
            case 0:
                arm_executor(JointControl_to_pub, JointState_, 0.0, 0.009, 0.594, 0.223);
                {
                    state_ = 1;
                    action_rate.sleep();
                }
                break;
            case 1:
                arm_executor(JointControl_to_pub, JointState_, 0.0, 0.009, 0.594, -0.223);
                {
                    state_ = 2;
                    action_rate.sleep();
                }
                break;
            case 2:
                if(arm_executor(JointControl_to_pub, JointState_, 0.0, -2.9, 1.477, -0.223))
                {
                    state_ = 3;
                    action_rate.sleep();
                }
                break;
            case 3:
                if(arm_executor(JointControl_to_pub, JointState_, 0.0, -2.9, 1.477, 0.223))
                {
                    state_ = 4;
                    action_rate.sleep();
                }
                break;
            case 4:
                arm_executor(JointControl_to_pub, JointState_, 0.0, 0.009, 0.594, -0.400);
                {
                    state_ = 5;
                    action_rate.sleep();
                }
                break;
            case 5:
                state_ = 6;

        }

        JointState_Last.position.assign(JointState_.position.begin(), JointState_.position.end());
        feedback_msg = 0.5;
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
    }

    if(rclcpp::ok())
    {
        action_rate.sleep();
        arm_executor(JointControl_to_pub, JointState_, 0.0, 0.009, 0.594, -0.400);
        result->time = this->now().seconds();
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
}

void action_findball::ActionCatchBall::pid_test_execute(const std::shared_ptr<GoalHandleCatchBall> goal_handle)
{
    acquire_goal();

    //初始化机械臂控制msg
    std::shared_ptr< sensor_msgs::msg::JointState> JointControl_to_pub = std::make_shared<sensor_msgs::msg::JointState>(); 
    sensor_msgs::msg::JointState JointState_;
    sensor_msgs::msg::JointState JointState_Last;
    JointState_.position.resize(4);
    JointState_.position[0] = 0.0;
    JointState_.position[1] = 0.0;
    JointState_.position[2] = 0.0;
    JointState_.position[3] = 0.0;
    JointState_Last.position.resize(4);
    JointState_Last.position.assign(JointState_.position.begin(), JointState_.position.end());

    JointControl_to_pub->header.frame_id="joint_control";
    JointControl_to_pub->name.push_back("Joint1");
    JointControl_to_pub->name.push_back("Joint2");
    JointControl_to_pub->name.push_back("Joint3");
    JointControl_to_pub->name.push_back("Joint4");
    arm_executor(JointControl_to_pub, JointState_, 0.0, -1.553, -1.285, 0.405);

    rclcpp::Rate loop_rate(20);
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<CatchBall::Result>();
    RCLCPP_INFO(this->get_logger(), "Executing goal, id: %s", std::string(reinterpret_cast<const char *>(goal_handle->get_goal_id().data())).c_str());
    auto feedback = std::make_shared<CatchBall::Feedback>();
    auto & feedback_msg = feedback->progress;
    RCLCPP_INFO(this->get_logger(), "test_goal: %f", test_goal);
    geometry_msgs::msg::Pose2D Data_To_Pub;
    while (rclcpp::ok()) {
        if(goal_handle->is_canceling())
        {
            Data_To_Pub.x = 0;
            Data_To_Pub.y = 0;
            Data_To_Pub.theta = 0;
            chassis_pub_->publish(Data_To_Pub);
            result->time = this->now().seconds() + this->now().nanoseconds() * 1e-9;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

            Data_To_Pub.x = 0;
            Data_To_Pub.y = 0;
            std::unique_lock<std::mutex> lock(variable_mutex__);
            Data_To_Pub.theta =  PIDController_w.PosePID_Calc(test_goal-imu_data.data[2]);
            RCLCPP_INFO(this->get_logger(), "cmd: %f", Data_To_Pub.theta);
            lock.unlock();
            chassis_pub_->publish(Data_To_Pub);
            feedback_msg = 0.5;
            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
    }

    if(rclcpp::ok())
    {
        result->time = this->now().seconds();
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
}

bool action_findball::ActionCatchBall::findball_node_init()
{
    rclcpp::Rate loop_rate(2);
    float time_now = this->now().seconds() + this->now().nanoseconds() * 1e-9;
    while(!client_get_state_->wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_INFO(this->get_logger(), "Service %s not available now, continue waitting...", client_get_state_->get_service_name());
        if (this->now().seconds() + this->now().nanoseconds() * 1e-9 - time_now > 15.0)
            return false;
        loop_rate.sleep();
    }
    if(this->get_state() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
    {
            if(!change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure node %s", lifecycle_node);
            return false;
        }
        if (!this->get_state()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get state for node %s", lifecycle_node);
            return  false;
        }
    }
    RCLCPP_INFO(this->get_logger(), "Node returned");
    return true;
}

void action_findball::ActionCatchBall::ballinfo_callback(const rc2024_interfaces::msg::BallInfo::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "Received ball info, x: %f, y: %f, z: %f", msg->balls_info.x, msg->balls_info.y, msg->balls_info.z);
    std::unique_lock<std::mutex> lock(variable_mutex_);
    ball_info.clear();
    purple_info.clear();
    ball_info.assign(msg->balls_info.begin(), msg->balls_info.end());
    purple_info.assign(msg->purple_info.begin(), msg->purple_info.end());
    is_found = msg->is_found;
    if(is_found)
        tracking_ball = msg->balls_info[0];
    lock.unlock();
}

bool action_findball::ActionCatchBall::change_state_to_active()
{
    rclcpp::WallRate time_between_state_changes(10);  // 0.1s

    RCLCPP_INFO(this->get_logger(), "Testing lifecycle for node %s", lifecycle_node);

    if(this->get_state() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
    {
            if(!change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure node %s", lifecycle_node);
            return false;
        }
        if (!this->get_state()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get state for node %s", lifecycle_node);
            return  false;
        }
    }

      // activate
    {
        time_between_state_changes.sleep();
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for state change");
            return false;
        }
        if (!this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to activate node %s", lifecycle_node);
            return false;
        }
        
        if (!this->get_state()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get state for node %s", lifecycle_node);
            return false;
        }
    }

    return true;
}

void action_findball::ActionCatchBall::notification_callback(lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr msg)
{
RCLCPP_INFO(
    get_logger(), "notify callback: Transition from state %s to %s",
    msg->start_state.label.c_str(), msg->goal_state.label.c_str());
}

void action_findball::ActionCatchBall::get_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(variable_mutex_);
    ChassisPa.x = msg->pose.pose.position.x;
    ChassisPa.y = msg->pose.pose.position.y;
    ChassisPa.theta = msg->pose.pose.orientation.w;
    vel_cal.cal_vel(ChassisPa, this->now().seconds(), this->now().nanoseconds(), ChassisPv);
}

void action_findball::ActionCatchBall::get_imu_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(variable_mutex__);
    imu_data.data = msg->data;
}

action_findball::VelCal::VelCal():last_time(0.0)
{
    Pa_last.x = 0;
    Pa_last.y = 0;
    Pa_last.theta = 0;
}

void action_findball::VelCal::cal_vel(geometry_msgs::msg::Pose2D &Pa, float time_s, float time_ns, geometry_msgs::msg::Pose2D &vel)
{
    float time_now = time_s + time_ns * 1e-9;
    float dt = time_now - last_time;
    vel.x = (Pa.x - Pa_last.x) / dt;
    vel.y = (Pa.y - Pa_last.y) / dt;
    vel.theta = (Pa.theta - Pa_last.theta) / dt;
    Pa_last.x = Pa.x;
    Pa_last.y = Pa.y;
    Pa_last.theta = Pa.theta;
    last_time = time_now;
}


int action_findball::ActionCatchBall::acquire_PID_variable()
{
    std::filesystem::path currentPath = std::filesystem::current_path();
    RCLCPP_INFO(this->get_logger(), "currentPath: %s", currentPath.c_str());
    const std::string package_name = "action_findball";
    std::filesystem::path dirPath = currentPath / "install" / package_name / "share" / package_name / "xml" / "variable.xml";// 假设已经构造好基础路径
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(dirPath.c_str()) != tinyxml2::XML_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load file: %s", dirPath.c_str());
        return -1;
    }
    
    tinyxml2::XMLElement* pidElement = doc.FirstChildElement("pid");
    if (pidElement == nullptr) {
        std::cerr << "Failed to find 'pid' element." << std::endl;
        return -1;
    }

    for (tinyxml2::XMLElement* axisElement = pidElement->FirstChildElement(); axisElement; axisElement = axisElement->NextSiblingElement()) {
        const char* axisName = axisElement->Name();

        tinyxml2::XMLElement* kpElement = axisElement->FirstChildElement("kp");
        tinyxml2::XMLElement* kiElement = axisElement->FirstChildElement("ki");
        tinyxml2::XMLElement* kdElement = axisElement->FirstChildElement("kd");
        RCLCPP_INFO(this->get_logger(), "axisName: %s", axisName);
        RCLCPP_INFO(this->get_logger(), "kp: %f, ki: %f, kd: %f", kpElement->FloatText(), kiElement->FloatText(), kdElement->FloatText());

        if (kpElement && kiElement && kdElement) {
            if(axisName == std::string("x"))
            {
                PIDController_x.PID_setParam(kpElement->FloatText(), kiElement->FloatText(), kdElement->FloatText());
            }else if(axisName == std::string("y"))
            {
                PIDController_y.PID_setParam(kpElement->FloatText(), kiElement->FloatText(), kdElement->FloatText());
            }else if(axisName == std::string("w"))
            {
                PIDController_w.PID_setParam(kpElement->FloatText(), kiElement->FloatText(), kdElement->FloatText());
            }
        }
    }
    return 0;
}


int action_findball::ActionCatchBall::acquire_goal()
{
    std::filesystem::path currentPath = std::filesystem::current_path();
    RCLCPP_INFO(this->get_logger(), "currentPath: %s", currentPath.c_str());
    const std::string package_name = "action_findball";
    std::filesystem::path dirPath = currentPath / "install" / package_name / "share" / package_name / "xml" / "goal.xml";// 假设已经构造好基础路径
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(dirPath.c_str()) != tinyxml2::XML_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load file: %s", dirPath.c_str());
        return -1;
    }
    RCLCPP_INFO(this->get_logger(), "load file: %s", dirPath.c_str());
    tinyxml2::XMLElement* pidElement = doc.FirstChildElement("goal");
    if (pidElement == nullptr) {
        std::cerr << "Failed to find 'pid' element." << std::endl;
        return -1;
    }
    tinyxml2::XMLElement* axisElement = pidElement->FirstChildElement();
    const char* axisName = axisElement->Name();
    if (axisName) {
            if(axisName == std::string("test"))
            {
                test_goal = axisElement->FloatText();
                RCLCPP_INFO(this->get_logger(), "test_goal: %f", axisElement->FloatText());
            }
    }
    return 0;
}

bool action_findball::ActionCatchBall::up_decision_making(
    std::vector<geometry_msgs::msg::Point32> &ball_info_, 
    std::vector<geometry_msgs::msg::Point32> &purple_info_, bool is_found_)
{
    if(is_found_)
    {
        for(int i = 0; i< ball_info_.size(); i++)
        {
            current_measurement = cv::Vec2f(ball_info_[i].x, ball_info_[i].y);
            current_radius = ball_info_[i].z;
            if((current_measurement[0]-last_measurement[0])*(current_measurement[0]-last_measurement[0]) + (current_measurement[1]-last_measurement[1])*(current_measurement[1]-last_measurement[1]) < 225)
            {
                Kalman->correct(cv::Mat(current_measurement));
                current_prediction = Kalman->predict();
                tracking_ball.x = current_prediction[0];
                tracking_ball.y = current_prediction[1];
                tracking_ball.z = current_radius;
                //cv::circle(color_image,cv::Point(current_prediction[0],current_prediction[1]),ball_info_[i][2],cv::Scalar(255,0,0),2);
                break;
            }
            if(i == ball_info_.size()-1)
            {
                std::cout << "No target found!" << std::endl;
                current_measurement = cv::Vec2f(ball_info_[0].x, ball_info_[0].y);
                current_radius = ball_info_[0].z;
                tracking_ball.x = current_measurement[0];
                tracking_ball.y = current_measurement[1];
                tracking_ball.z = current_radius;
                //cv::circle(color_image,cv::Point(last_measurement[0],last_measurement[1]),last_radius,cv::Scalar(255,0,0),2);
            }
        }
        last_prediction = current_prediction;
        last_measurement = current_measurement;
        last_radius = current_radius;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Ball not found.......");
        return false;
    }
    return true;
}

bool action_findball::ActionCatchBall::jaw_decision_making()
{
    bool is_found_ = false;
    std::vector<geometry_msgs::msg::Point32> ball_info_;
    std::vector<geometry_msgs::msg::Point32> purple_info_;
    std::unique_lock<std::mutex> lock(variable_mutex_);
    ball_info_.assign(ball_info.begin(), ball_info.end());
    purple_info_.assign(purple_info.begin(), purple_info.end());
    is_found_ = is_found;
    lock.unlock();
}

void action_findball::ActionCatchBall::get_jointstate_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(variable_mutex__1);
    up_joint_state = *msg;
    lock.unlock();
}

bool action_findball::ActionCatchBall::arm_executor(const std::shared_ptr<sensor_msgs::msg::JointState> JointControl_to_pub,
                    const sensor_msgs::msg::JointState &JointState_,
                        double joint1, double joint2, double joint3, double joint4)
{
    rclcpp::Rate loop_rate(20);
    rclcpp::Rate action_rate(1.5);
    int step = (fabs(JointState_.position[1]-joint2)/3.1415926*40.0 >= 1 ? fabs(JointState_.position[1]-joint2)/3.1415926*40:1.0);
    RCLCPP_INFO(this->get_logger(), "step: %d", step);
    RCLCPP_INFO(this->get_logger(), "JointState_current_pos: %f, %f, %f", JointState_.position[0], JointState_.position[1], JointState_.position[2]);
    RCLCPP_INFO(this->get_logger(), "JointState_current_dif: %f, %f, %f", joint1 - JointState_.position[0], joint2 - JointState_.position[1], joint3 - JointState_.position[2]);

    for(int i = 1;i <= step;i++)
    {
        JointControl_to_pub->position.clear();
        JointControl_to_pub->position.push_back(joint1);
        JointControl_to_pub->position.push_back(JointState_.position[1] + (joint2 - JointState_.position[1])/step*i);
        JointControl_to_pub->position.push_back(joint3);
        JointControl_to_pub->position.push_back(joint4);
        JointControl_to_pub->header.stamp = this->now();
        up_pub_->publish(*JointControl_to_pub);
        loop_rate.sleep();
    }
    action_rate.sleep();
    // JointControl_to_pub->position[1] = (joint2);
    // up_pub_->publish(*JointControl_to_pub);
    // action_rate.sleep();

    if(fabs(JointState_.position[0]-joint1)<=0.1 &&
        fabs(JointState_.position[1]-joint2)<=0.3 &&
            fabs(JointState_.position[2]-joint3)<=0.1)
    {
        return true;
    }
    else{
        return false;
    }
}

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto action_catch = std::make_shared<action_findball::ActionCatchBall>("action_catch");
    executor.add_node(action_catch);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}



/*
{
    int count = 0;
    acquire_PID_variable();
    std::unique_lock<std::mutex> lock(variable_mutex_);
    ball_info.clear();
    purple_info.clear();
    lock.unlock();

    std::shared_ptr< sensor_msgs::msg::JointState> msg_to_pub = std::make_shared<sensor_msgs::msg::JointState>(); 
    
    rclcpp::Rate loop_rate(20);
    rclcpp::Rate action_rate(2);
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<CatchBall::Result>();

    if(!change_state_to_active())
    {
        RCLCPP_INFO(this->get_logger(), "different color index");
        result->time = this->now().seconds() + this->now().nanoseconds() * 1e-9;
        result->state = 1;
        goal_handle->abort(result);
        return;
    }
    action_rate.sleep();
    RCLCPP_INFO(this->get_logger(), "Executing goal, id: %s", std::string(reinterpret_cast<const char *>(goal_handle->get_goal_id().data())).c_str());
    auto feedback = std::make_shared<CatchBall::Feedback>();
    auto & feedback_msg = feedback->progress;

    geometry_msgs::msg::Pose2D Data_To_Pub;
    int flag = 0;
    int flag_for_w = 0;

    std_msgs::msg::UInt32 up_cmd;
    up_cmd.data = static_cast<int>(UpCmd::ChaseBall);
    RCLCPP_INFO(this->get_logger(), "UpCmd: %d", up_cmd.data);
    up_pub_->publish(up_cmd);
    action_rate.sleep();

    //开始执行
    while (rclcpp::ok()) {
        if(goal_handle->is_canceling())
        {
            Data_To_Pub.x = 0;
            Data_To_Pub.y = 0;
            Data_To_Pub.theta = 0;
            chassis_pub_->publish(Data_To_Pub);
            test.data = 3500;
            up_pub_->publish(test);
            result->time = this->now().seconds() + this->now().nanoseconds() * 1e-9;
            result->state = static_cast<int>(CatchBallState::CANCEL);
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }
        lock.lock();
        std::vector<geometry_msgs::msg::Point32> ballinfo__ =ball_info;        
        std::vector<geometry_msgs::msg::Point32> purpleinfo__ =purple_info;
        bool is_found_ = is_found;
        lock.unlock();

        if(!is_found_)
        {
            count++;
            
            if(count > 20*3)
            {
                if(!change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to deactive node %s", lifecycle_node);
                }
                RCLCPP_INFO(this->get_logger(), "Ball not found for about 10s");
                result->time = this->now().seconds() + this->now().nanoseconds() * 1e-9;
                result->state = static_cast<int>(CatchBallState::TIMEOUT);
                goal_handle->abort(result);
                return;
            }
            Data_To_Pub.x = 0;
            Data_To_Pub.y = 0;
            Data_To_Pub.theta = 0.0;
            chassis_pub_->publish(Data_To_Pub);
            loop_rate.sleep();
            continue;
        }else if(is_found_ && count > 0){
            count = 0;
        }

        if( flag_for_w == 0)
        {
            Data_To_Pub.x = 0;
            Data_To_Pub.y = 0;
            Data_To_Pub.theta = -PIDController_w.PosePID_Calc(-atan2(y_catch - ballinfo__.y, x_catch - ballinfo__.x));
            RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, theta: %f", Data_To_Pub.x, Data_To_Pub.y, Data_To_Pub.theta);
            if(int(fabs(y_catch - ballinfo__.y)) < 25)
            {
                flag_for_w = 1;
            }
        } 
        else if(flag_for_w == 1){
            if(flag == 0)
            {
                if(ballinfo__.y < 468 && ballinfo__.y > 161 && ballinfo__.x > 333)
                {
                    flag = 1;
                }
                Data_To_Pub.x = -PIDController_x.PID_Calc(x_next - ballinfo__.x);
                Data_To_Pub.y = -PIDController_y.PID_Calc(y_catch - ballinfo__.y);
                Data_To_Pub.theta = 0;
            }else {
                    test.data = 3350;
                    up_pub_->publish(test);
                    
                    Data_To_Pub.x = -PIDController_x_near.PID_Calc(x_catch - ballinfo__.x);
                    Data_To_Pub.y = -PIDController_y_near.PID_Calc(y_catch - ballinfo__.y);
                    Data_To_Pub.theta = 0;
            }
        }

        if(x_catch < ballinfo__.x && std::fabs(ballinfo__.y - y_catch)< 25)
        {
            //RCLCPP_INFO(this->get_logger(), "Goal Succeed!!");
            //RCLCPP_INFO(this->get_logger(), "%f,%f,%f",ballinfo__.x,ballinfo__.y,ballinfo__.z);
            Data_To_Pub.x = 0;
            Data_To_Pub.y = 0;
            Data_To_Pub.theta = 0;
            chassis_pub_->publish(Data_To_Pub);
            break;
        }
        
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, theta: %f", Data_To_Pub.x, Data_To_Pub.y, Data_To_Pub.theta);

        chassis_pub_->publish(Data_To_Pub);
        feedback_msg = 0.5;
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
    }

    if(rclcpp::ok())
    {
        if(!change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to deactive node %s", lifecycle_node);
        }
        action_rate.sleep();
        up_cmd.data = static_cast<int>(UpCmd::CatchBall);
        test.data = 3500;
        up_pub_->publish(test);
        RCLCPP_INFO(this->get_logger(), "UpCmd: %d", up_cmd.data);
        up_pub_->publish(up_cmd);

        action_rate.sleep();
        action_rate.sleep();
        action_rate.sleep();
        action_rate.sleep();
        action_rate.sleep();
        action_rate.sleep();
        
        result->time = this->now().seconds();
        result->state = static_cast<int>(CatchBallState::SUCCEED);

        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
}
*/