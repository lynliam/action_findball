#include <cstddef>
#include <memory>
#include <mutex>

#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <vector>
#include <thread>
#include <functional>
#include <thread>
#include <cmath>
#include <string>
#include <iostream>
#include <filesystem>

#include "approaching_ball.hpp"
#include "State.hpp"
#include "lifecycle_manager_client.hpp"
#include "pid_controller.hpp"

// which node to handle
static constexpr char const * lifecycle_node = "findball";
static constexpr char const * node_get_state_topic = "findball/get_state";
static constexpr char const * node_change_state_topic = "findball/change_state";

// 判定接近球的阈值
const int x_approach = 260;
const int y_approach = 320;
// 判定球将被抓住的阈值，相对于 image_size_half
const int x_catch = 280;
const int y_catch = 328;
// 判定球进入下一区域的阈值
const int x_next = 340;

//
const int distance = 70;
const int img_center = 320;

action_findball::ApproachingBall::ApproachingBall(const std::string & node_name,  const rclcpp::NodeOptions & options)
    :LifecycleManagerClient(node_name, options),
    findball_node_state_(false),
    PIDController_PTZ(0, 0, 0),
    PIDController_x(0.0004,0.00001,0.001),             
    PIDController_y(0.0006,0.00001,0.0001),
    PIDController_w(1.3,0.02,0.06)
{
    RCLCPP_INFO(this->get_logger(), "ApproachingBall node init");
    spin_to_func = std::make_shared<SpinTo>();
    // 回调组
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // Action
    action_server_ = rclcpp_action::create_server<EmptyGoal>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "approachingball",
        std::bind(&ApproachingBall::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ApproachingBall::handle_cancel, this, std::placeholders::_1),
        std::bind(&ApproachingBall::handle_accepted, this, std::placeholders::_1),
        rcl_action_server_get_default_options(),
        callback_group_);

        // 创建一个QoS配置对象，设置可靠性为BestEffort
    qos_profile = std::make_shared<rclcpp::QoS>(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos_profile->reliability(rclcpp::ReliabilityPolicy::BestEffort);
    
    ballinfo_sub_= this->create_subscription<rc2024_interfaces::msg::BallInfo>("ball_info",*qos_profile, std::bind(&ApproachingBall::ballinfo_callback,this,std::placeholders::_1));
    chassis_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/car/cmd_vel", 2);
    chassis_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 2, std::bind(&ApproachingBall::get_pose_callback,this,std::placeholders::_1));
    up_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/car/up_cmd", 2);
    up_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/car/up_fdbk", 3, std::bind(&ApproachingBall::get_jointstate_callback,this,std::placeholders::_1));
    camera_switch_pub_= this->create_publisher<rc2024_interfaces::msg::CameraSwitch>("camera_switch", 2);

    Kalman = std::make_shared<cv::KalmanFilter>(4, 2);
    Kalman->measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);
    Kalman->transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
    Kalman->processNoiseCov = (cv::Mat_<float>(4, 4) << 0.3, 0, 0, 0, 0, 0.3, 0, 0, 0, 0, 0.3, 0, 0, 0, 0, 0.3);

    PIDController_PTZ.acquire_PID_variable("PTZ");
    PIDController_x.acquire_PID_variable("x");
    PIDController_y.acquire_PID_variable("y");
    PIDController_w.acquire_PID_variable("w");

    //findball_node_init();
}

rclcpp_action::GoalResponse action_findball::ApproachingBall::handle_goal(
const rclcpp_action::GoalUUID & uuid,
std::shared_ptr<const EmptyGoal::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request id : %s", reinterpret_cast<const char *>(uuid.data()));
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse action_findball::ApproachingBall::handle_cancel(
    const std::shared_ptr<GoalHandleEmptyGoal> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;

    if(!change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to deactive node %s", lifecycle_node);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void action_findball::ApproachingBall::handle_accepted(const std::shared_ptr<GoalHandleEmptyGoal> goal_handle)
{
    using namespace std::placeholders;
    std::thread{std::bind(&action_findball::ApproachingBall::execute, this, _1), goal_handle}.detach();
}

void action_findball::ApproachingBall::execute(const std::shared_ptr<GoalHandleEmptyGoal> goal_handle)
{
    PIDController_PTZ.acquire_PID_variable("PTZ");
    rclcpp::Rate loop_rate(50);
    rclcpp::Rate action_rate(1);
    change_state_to_active();

    // 切换摄像头
    rc2024_interfaces::msg::CameraSwitch camera_switch;
    camera_switch.index = 0;
    camera_switch_pub_->publish(camera_switch);

    //互斥锁初始化
    std::unique_lock<std::mutex> lock_joint(variable_mutex__1, std::defer_lock);
    std::unique_lock<std::mutex> lock_ball(variable_mutex_, std::defer_lock);
    std::unique_lock<std::mutex> lock(variable_mutex__, std::defer_lock);
    
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<EmptyGoal::Result>();

    RCLCPP_INFO(this->get_logger(), "Executing goal, id: %s", std::string(reinterpret_cast<const char *>(goal_handle->get_goal_id().data())).c_str());
    auto feedback = std::make_shared<EmptyGoal::Feedback>();

    //ball_info
    std::vector<geometry_msgs::msg::Point32> ball_info_;
    std::vector<geometry_msgs::msg::Point32> purple_info_;
    bool is_found_;
    std_msgs::msg::Header ball_info_header_;

    nav_msgs::msg::Odometry current_pose;

    APPROACHINGBALL state_ = APPROACHINGBALL::IDLE;

    //初始化速度控制msg
    geometry_msgs::msg::Pose2D Data_To_Pub;

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

    JointControl_to_pub->velocity.resize(4);
    JointControl_to_pub->velocity[0] = 0.0;
    JointControl_to_pub->velocity[1] = 0.0;
    JointControl_to_pub->velocity[2] = 0.0;
    JointControl_to_pub->velocity[3] = 0.0;

    // 获取初始位置
    lock_joint.lock();
    JointState_.position.clear();
    JointState_.position.resize(4);
    JointState_.position.assign(up_joint_state.position.begin(), up_joint_state.position.end());
    lock_joint.unlock();

    // 关节初始化位置代码

    JointControl_to_pub->velocity[0] = 0.0;
    arm_executor(JointControl_to_pub, JointState_, 0.0, 0.009, 0.594, -0.400);

    action_rate.sleep();

    while (rclcpp::ok()) {
        if(goal_handle->is_canceling())
        {
            if(!change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to deactive node %s", lifecycle_node);
            }
            JointControl_to_pub->velocity[0] = 0.0;
            arm_executor(JointControl_to_pub, JointState_, 0.0, 0.009, 0.594, -0.400);

            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

        JointState_.position.clear();
        ball_info_.clear();
        purple_info_.clear();

        lock_joint.lock();
        JointState_.position.resize(4);
        JointState_.position.assign(up_joint_state.position.begin(), up_joint_state.position.end());
        lock_joint.unlock();

        lock.lock();
        current_pose = ChassisPa;
        lock.unlock();

        lock_ball.lock();
        ball_info_.assign(ball_info.begin(), ball_info.end());
        purple_info_.assign(purple_info.begin(), purple_info.end());
        ball_info_header_ = ball_info_header;
        is_found_ = is_found;
        lock_ball.unlock();
        
        // 上层摄像头决策球
        up_decision_making(ball_info_, purple_info_, is_found_);
        RCLCPP_INFO(this->get_logger(), "frame time: %f", ball_info_header_.stamp.sec + ball_info_header_.stamp.nanosec * 1e-9);
        //RCLCPP_INFO(this->get_logger(), "now time: %f", this->now().seconds()+float(this->now().nanoseconds()*1e-9));
        //RCLCPP_INFO(this->get_logger(), "The handle time: %f", (ball_info_header_.stamp.sec + float(ball_info_header_.stamp.nanosec) * 1e-9 - this->now().seconds()-float(this->now().nanoseconds()*1e-9)));

        switch (state_) {
            case (APPROACHINGBALL::IDLE):
                state_ = (APPROACHINGBALL::LOOKING);
                break;
            case (APPROACHINGBALL::LOOKING):
            {
                if(is_found_)
                {
                    static int calmdown = 0;
                    // 云台控制
                    JointControl_to_pub->position[0] = PIDController_PTZ.PID_Calc( img_center - tracking_ball.y);
                    if(fabs(tracking_ball.y - img_center) < 5) JointControl_to_pub->position[0] = 0;
                    RCLCPP_INFO(this->get_logger(), "PTZ: %f", JointControl_to_pub->position[0]);

                    // 底盘控制
                    Data_To_Pub.x = 0;
                    Data_To_Pub.y = 0; 
                    Data_To_Pub.theta = 0;

                    // if(fabs(tracking_ball.y - img_center) < 20) calmdown++;
                    // if(calmdown > 25)
                    // {
                    //     RCLCPP_INFO(this->get_logger(), "Ball in the center");
                    //     state_ = (APPROACHINGBALL::TOFORWARD);
                    //     calmdown = 0;
                    // }
                }
                // else {
                //     state_ = (APPROACHINGBALL::LOST);
                // }
                
                chassis_pub_->publish(Data_To_Pub);
                up_pub_->publish(*JointControl_to_pub);
                break;
            }
            
            case (APPROACHINGBALL::TOFORWARD):
            {
                static int state_inner = 0;
                static Status spin_return;
                static int calmdown = 0;
                static float target_anlgle;
                RCLCPP_INFO(this->get_logger(), "target_anlgle: %f", target_anlgle);
                static rclcpp::Duration time_allowance(8, 0);
                //云台控制：
                JointControl_to_pub->position[0] = 0;
                
                // 底盘控制
                if(state_inner == 0)
                {
                    target_anlgle = JointState_.position[0];
                    spin_to_func->onRun(target_anlgle, time_allowance, current_pose);
                    RCLCPP_INFO(this->get_logger(), "Forwarding");
                    state_inner = 1;
                }else if(state_inner == 1){
                    spin_return = spin_to_func->onCycleUpdate(current_pose);
                    if(spin_return == Status::SUCCEEDED)  state_inner = 2;
                }

                if(spin_return == Status::SUCCEEDED) 
                {
                    calmdown++;
                }

                if(calmdown > 25)
                {
                    RCLCPP_INFO(this->get_logger(), "Forward Down!");
                    state_ = (APPROACHINGBALL::APPROACHING);
                    spin_return = Status::FAILED;
                    calmdown = 0;
                    state_inner = 0;
                }
                up_pub_->publish(*JointControl_to_pub);
                break;
            }

            case (APPROACHINGBALL::APPROACHING):
            {
                //JointControl_to_pub->position[0] = PIDController_PTZ.PID_Calc(tracking_ball.x-img_center);
                arm_executor(JointControl_to_pub, JointState_, 0.0, 0.009, 0.594, -0.223);
                
                Data_To_Pub.x = -PIDController_x.PID_Calc(x_approach - tracking_ball.x);
                Data_To_Pub.y = PIDController_y.PID_Calc(y_approach- tracking_ball.y);
                Data_To_Pub.theta = 0;
                RCLCPP_INFO(this->get_logger(), "Data_To_Pub: %f, %f", Data_To_Pub.x, Data_To_Pub.y);

                // Data_To_Pub.x = -PIDController_x.PID_Calc((x_approach - tracking_ball.x)*cos(JointState_.position[0]));
                // Data_To_Pub.y = -PIDController_y.PID_Calc((y_approach - tracking_ball.y)*sin(JointState_.position[0]));
                // Data_To_Pub.theta = 0;
                //JointControl_to_pub->position[0] = PIDController_PTZ.PID_Calc(tracking_ball.x-img_center);

                if(fabs(tracking_ball.x - x_approach) < 5 && fabs(tracking_ball.y - y_approach) < 5)
                {
                    Data_To_Pub.x = 0;
                    Data_To_Pub.y = 0;
                    Data_To_Pub.theta = 0;
                    RCLCPP_INFO(this->get_logger(), "Ball approached");
                    camera_switch.index = 0;
                    camera_switch_pub_->publish(camera_switch);
                    state_ = (APPROACHINGBALL::CATCHING);
                }
                chassis_pub_->publish(Data_To_Pub);
                up_pub_->publish(*JointControl_to_pub);
                break;
            }
/*
            case (APPROACHINGBALL::CATCHING):
            {
                JointControl_to_pub->position[0] = PIDController_PTZ.PID_Calc(tracking_ball.x-img_center);

                Data_To_Pub.x = -PIDController_x.PID_Calc((x_catch - tracking_ball.x)*cos(JointState_.position[0]));
                Data_To_Pub.y = -PIDController_y.PID_Calc((y_catch - tracking_ball.y)*sin(JointState_.position[0]));
                Data_To_Pub.theta = 0;

                if(fabs(tracking_ball.x - x_catch) < 5 && fabs(tracking_ball.y - y_catch) < 5 && tracking_ball.z > 500)
                {
                    RCLCPP_INFO(this->get_logger(), "Catching");
                    camera_switch.index = 1;
                    camera_switch_pub_->publish(camera_switch);
                    state_ = (APPROACHINGBALL::SUCCEED);
                }

                chassis_pub_->publish(Data_To_Pub);
                up_pub_->publish(*JointControl_to_pub);
                break;
            }


            case (APPROACHINGBALL::LOST):
            {
                RCLCPP_INFO(this->get_logger(), "Ball lost");
                static int stay_calm = 0;
                if(stay_calm > 5)
                {
                    //如果在这期间都没找到
                    // 旋转云台来找
                    // 进入Finding状态
                    state_ = (APPROACHINGBALL::FINDING);
                    stay_calm = 0;
                }
                stay_calm ++;
                break;
            }

            case (APPROACHINGBALL::FINDING):
            {
                static int state_angle = 0;
                //向左旋转云台 90 度
                if(state_angle == 0)
                {
                    JointControl_to_pub->position[0] = PIDController_PTZ.PID_Calc(tracking_ball.x-img_center);
                    up_pub_->publish(*JointControl_to_pub);
                }
                //向右旋转云台 90 度

                //再不行根据自己的位置，向后退
                break;
            }
            */
        }

        if(state_ == (APPROACHINGBALL::SUCCEED))
        {
            break;
        }
        
        JointState_Last.position.assign(JointState_.position.begin(), JointState_.position.end());
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
    }

    if(rclcpp::ok())
    {
        action_rate.sleep();
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
}

void action_findball::ApproachingBall::catch_ball_execute(const std::shared_ptr<GoalHandleEmptyGoal> goal_handle)
{
    rclcpp::Rate loop_rate(20);
    rclcpp::Rate action_rate(1);

    //互斥锁初始化
    std::unique_lock<std::mutex> lock_joint(variable_mutex__1, std::defer_lock);
    std::unique_lock<std::mutex> lock_ball(variable_mutex_, std::defer_lock);
    std::unique_lock<std::mutex> lock(variable_mutex__, std::defer_lock);
    
    //action goal / result / feedback 初始化
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<EmptyGoal::Result>();
    RCLCPP_INFO(this->get_logger(), "Executing goal, id: %s", std::string(reinterpret_cast<const char *>(goal_handle->get_goal_id().data())).c_str());
    auto feedback = std::make_shared<EmptyGoal::Feedback>();

    //ball_info
    std::vector<geometry_msgs::msg::Point32> ball_info_;
    std::vector<geometry_msgs::msg::Point32> purple_info_;
    bool is_found_;

    //底盘位置
    nav_msgs::msg::Odometry current_pose;

    //状态机
    CATCHBALL state_ = CATCHBALL::IDLE;

    //初始化速度控制msg
    geometry_msgs::msg::Pose2D Data_To_Pub;
    Data_To_Pub.x = 0;
    Data_To_Pub.y = 0;
    Data_To_Pub.theta = 0;

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

    // 关节初始化位置代码
    //调整至指定位置
    JointControl_to_pub->position.push_back(0.0);
    JointControl_to_pub->position.push_back(-1.553);
    JointControl_to_pub->position.push_back(-1.285);
    JointControl_to_pub->position.push_back(0.405);
    JointControl_to_pub->header.stamp = this->now();
    up_pub_->publish(*JointControl_to_pub);
    chassis_pub_->publish(Data_To_Pub);

    action_rate.sleep();
    action_rate.sleep();

    while (rclcpp::ok()) {
        if(goal_handle->is_canceling())
        {
            JointControl_to_pub->position.clear();
            JointControl_to_pub->position.push_back(0.0);
            JointControl_to_pub->position.push_back(-1.553);
            JointControl_to_pub->position.push_back(-1.285);
            JointControl_to_pub->position.push_back(0.405);

            JointControl_to_pub->header.stamp = this->now();
            up_pub_->publish(*JointControl_to_pub);
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

        // 这一堆变量初始化很烦
        //清空变量
        JointState_.position.clear();
        ball_info_.clear();
        purple_info_.clear();

        lock_joint.lock();
        JointState_.position.resize(4);
        JointState_.position.assign(up_joint_state.position.begin(), up_joint_state.position.end());
        lock_joint.unlock();

        lock.lock();
        current_pose = ChassisPa;
        lock.unlock();

        lock_ball.lock();
        ball_info_.assign(ball_info.begin(), ball_info.end());
        purple_info_.assign(purple_info.begin(), purple_info.end());
        is_found_ = is_found;
        lock_ball.unlock();

        // 爪子摄像头追踪球
        jaw_decision_making(ball_info_, purple_info_, is_found_, state_);

        switch (state_) {
            case CATCHBALL::IDLE:
                break;
            case (CATCHBALL::BOUNCING):
            {
                //下压
                //张爪子
                //完成后抬爪子

                chassis_pub_->publish(Data_To_Pub);
                up_pub_->publish(*JointControl_to_pub);
                break;
            }

            case (CATCHBALL::CATCHING):
            {
                static int state_inner = 0;
                Status spin_return;
                static int calmdown = 0;
                static float target_anlgle = JointState_.position[0];
                static rclcpp::Duration time_allowance(3, 0);

                if(state_inner == 0)
                {
                    //云台控制：
                    JointControl_to_pub->position[0] = PIDController_PTZ.PID_Calc(tracking_ball.x-img_center);
                    Data_To_Pub.x = -PIDController_x.PID_Calc((x_next - tracking_ball.x)*cos(JointState_.position[0]));
                    Data_To_Pub.y = -PIDController_y.PID_Calc((y_catch - tracking_ball.y)*sin(JointState_.position[0]));
                    Data_To_Pub.theta = 0;
                    if(fabs(tracking_ball.x - x_catch) < 5 && fabs(tracking_ball.y - y_catch) < 5)
                    {
                        state_inner = 1;
                    }
                }else{
                    // 执行抓球动作
                    // 抓球成功后，抬起机械臂
                    // 进入下一状态
                    state_ = CATCHBALL::SUCCEED;
                }

                chassis_pub_->publish(Data_To_Pub);
                up_pub_->publish(*JointControl_to_pub);
                break;
            }

        }

        if(state_ == (CATCHBALL::SUCCEED))
        {
            break;
        }
        
        JointState_Last.position.assign(JointState_.position.begin(), JointState_.position.end());
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
    }

    if(rclcpp::ok())
    {
        action_rate.sleep();
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
}

void action_findball::ApproachingBall::put_ball_execute(const std::shared_ptr<GoalHandleEmptyGoal> goal_handle)
{

}


void action_findball::ApproachingBall::ballinfo_callback(const rc2024_interfaces::msg::BallInfo::SharedPtr msg)
{
    ball_info.clear();
    purple_info.clear();
    std::unique_lock<std::mutex> lock(variable_mutex_);
    ball_info.assign(msg->balls_info.begin(), msg->balls_info.end());
    purple_info.assign(msg->purple_info.begin(), msg->purple_info.end());
    ball_info_header = msg->header;
    is_found = msg->is_found;
    if(is_found)
        tracking_ball = msg->balls_info[0];
    lock.unlock();
}

void action_findball::ApproachingBall::get_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(variable_mutex__);
    ChassisPa= *msg;
}

bool action_findball::ApproachingBall::up_decision_making(
    std::vector<geometry_msgs::msg::Point32> &ball_info_, 
    std::vector<geometry_msgs::msg::Point32> &purple_info_, bool is_found_)
{
    if(is_found_)
    {
        for(size_t i = 0; i < ball_info_.size(); i++)
        {
            current_measurement = cv::Vec2f(ball_info_[i].x, ball_info_[i].y);
            current_radius = ball_info_[i].z;
            if((current_measurement[0]-last_measurement[0])*(current_measurement[0]-last_measurement[0]) + (current_measurement[1]-last_measurement[1])*(current_measurement[1]-last_measurement[1]) < 225)
            {
                Kalman->correct(cv::Mat(current_measurement));
                current_prediction = Kalman->predict();
                // 这里进行了一次x，y的交换
                tracking_ball.x = current_prediction[1];
                tracking_ball.y = current_prediction[0];
                tracking_ball.z = current_radius;
                //cv::circle(color_image,cv::Point(current_prediction[0],current_prediction[1]),ball_info_[i][2],cv::Scalar(255,0,0),2);
                break;
            }
            if(i == ball_info_.size()-1)
            {
                std::cout << "No target found!" << std::endl;
                current_measurement = cv::Vec2f(ball_info_[0].x, ball_info_[0].y);
                current_radius = ball_info_[0].z;
                // 这里进行了一次x，y的交换
                tracking_ball.x = current_measurement[1];
                tracking_ball.y = current_measurement[0];
                tracking_ball.z = current_radius;
                //RCLCPP_INFO(this->get_logger(), "target x: %f, y: %f", tracking_ball.x, tracking_ball.y);
                //cv::circle(color_image,cv::Point(last_measurement[0],last_measurement[1]),last_radius,cv::Scalar(255,0,0),2);
            }
        }
        RCLCPP_INFO(this->get_logger(), "target x: %f, y: %f", tracking_ball.x, tracking_ball.y);
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

bool action_findball::ApproachingBall::jaw_decision_making(
            std::vector<geometry_msgs::msg::Point32> &ball_info_, 
            std::vector<geometry_msgs::msg::Point32> &purple_info_, bool is_found_, CATCHBALL &catchball_state)
{
    if(is_found_)
    {
        for(size_t i = 0; i < ball_info_.size(); i++)
        {
            current_measurement = cv::Vec2f(ball_info_[i].x, ball_info_[i].y);
            current_radius = ball_info_[i].z;
            if((current_measurement[0]-last_measurement[0])*(current_measurement[0]-last_measurement[0]) + (current_measurement[1]-last_measurement[1])*(current_measurement[1]-last_measurement[1]) < 225)
            {
                Kalman->correct(cv::Mat(current_measurement));
                current_prediction = Kalman->predict();
                tracking_ball.x = current_prediction[1];
                tracking_ball.y = current_prediction[0];
                tracking_ball.z = current_radius;
                //cv::circle(color_image,cv::Point(current_prediction[0],current_prediction[1]),ball_info_[i][2],cv::Scalar(255,0,0),2);
                break;
            }
            if(i == ball_info_.size()-1)
            {
                std::cout << "No target found!" << std::endl;
                current_measurement = cv::Vec2f(ball_info_[0].x, ball_info_[0].y);
                current_radius = ball_info_[0].z;
                tracking_ball.x = current_measurement[1];
                tracking_ball.y = current_measurement[0];
                tracking_ball.z = current_radius;
                //cv::circle(color_image,cv::Point(last_measurement[0],last_measurement[1]),last_radius,cv::Scalar(255,0,0),2);
            }
        }
        last_prediction = current_prediction;
        last_measurement = current_measurement;
        last_radius = current_radius;

        

        if(catchball_state == CATCHBALL::IDLE || catchball_state == CATCHBALL::LOST)
        {
            for(size_t i = 0; i < purple_info_.size(); i++)
            {
                // 如果存在紫球 并且在目标球与底盘之间
                if(purple_info_[i].y > tracking_ball.x && fabs(purple_info_[i].x - img_center) < distance)
                {
                    catchball_state = CATCHBALL::BOUNCING;
                    return true;
                }
            }
        }

        if(catchball_state != CATCHBALL::BOUNCING)
            catchball_state = CATCHBALL::CATCHING;
        return true;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Ball not found.......");
        catchball_state = CATCHBALL::LOST;
        return false;
    }
}

void action_findball::ApproachingBall::get_jointstate_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(variable_mutex__1);
    up_joint_state = *msg;
    lock.unlock();
}

bool action_findball::ApproachingBall::arm_executor(const std::shared_ptr<sensor_msgs::msg::JointState> JointControl_to_pub,
                    const sensor_msgs::msg::JointState &JointState_,
                        double joint1, double joint2, double joint3, double joint4)
{
    rclcpp::Rate loop_rate(20);
    rclcpp::Rate action_rate(2);
    int step = (fabs(JointState_.position[1]-joint2)/3.1415926*50.0 >= 1 ? fabs(JointState_.position[1]-joint2)/3.1415926*50:1.0);
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
    //action_rate.sleep();
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

bool action_findball::ApproachingBall::findball_node_init()
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

int main(int argc, char * argv[])
{
    //setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto approaching_ball = std::make_shared<action_findball::ApproachingBall>("approaching_ball1");
    executor.add_node(approaching_ball);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
