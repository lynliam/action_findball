#include <cstddef>
#include <geometry_msgs/msg/detail/pose2_d__struct.hpp>
#include <memory>
#include <mutex>

#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/types.h>
#include <vector>
#include <thread>
#include <functional>
#include <thread>
#include <cmath>
#include <string>
#include <iostream>
#include <filesystem>
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

#include "approaching_ball.hpp"
#include "State.hpp"
#include "lifecycle_manager_client.hpp"
#include "pid_controller.hpp"
#include "PTZ_angle.hpp"
#include "camera_distribute.hpp"

// which node to handle
static constexpr char const * lifecycle_node = "findball";
static constexpr char const * node_get_state_topic = "findball/get_state";
static constexpr char const * node_change_state_topic = "findball/change_state";

// 判定接近球的阈值
const int x_approach = 230;
const int y_approach = 310;
// 判定球将被抓住的阈值，相对于 image_size_half
const int x_catch = 350;
const int y_catch = 310;
// 判定球进入下一区域的阈值
const int x_next = 300;

//
const int distance = 70;
const int img_center = 310;

action_findball::ApproachingBall::ApproachingBall(const std::string & node_name,  const rclcpp::NodeOptions & options)
    :LifecycleManagerClient(node_name, options),
    findball_node_state_(false),
    PIDController_PTZ(0, 0, 0),
    PIDController_x(0.0004,0.00001,0.001),             
    PIDController_x_catch(0.002,0.0001,0.01),
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
    chassis_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 2);
    chassis_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 2, std::bind(&ApproachingBall::get_pose_callback,this,std::placeholders::_1));
    up_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/car/up_cmd", 2);
    up_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/car/up_fdbk", *qos_profile, std::bind(&ApproachingBall::get_jointstate_callback,this,std::placeholders::_1));
    camera_switch_pub_= this->create_publisher<rc2024_interfaces::msg::CameraSwitch>("camera_switch", 2);

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

    // PIDController_PTZ.acquire_PID_variable("PTZ");
    PIDController_x.acquire_PID_variable("x");
    PIDController_y.acquire_PID_variable("y");
    PIDController_w.acquire_PID_variable("w");

    PIDController_x.PID_MaxMin(0.6, -0.6);
    PIDController_x_catch.PID_MaxMin(0.3, -0.3);
    PIDController_y.PID_MaxMin(0.5, -0.5);
    PIDController_w.PID_MaxMin(0.3, -0.3);
    PIDController_x.PID_InteMaxMin(5, -5);
    PIDController_x_catch.PID_InteMaxMin(3, -3);
    PIDController_y.PID_InteMaxMin(4, -4);
    PIDController_y.PID_InteMaxMin(3, -3);
    //findball_node_init();

    tf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);
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

    // if(!change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
    // {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to deactive node %s", lifecycle_node);
    // }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void action_findball::ApproachingBall::handle_accepted(const std::shared_ptr<GoalHandleEmptyGoal> goal_handle)
{
    using namespace std::placeholders;
    std::thread{std::bind(&action_findball::ApproachingBall::execute, this, _1), goal_handle}.detach();
}

void action_findball::ApproachingBall::execute(const std::shared_ptr<GoalHandleEmptyGoal> goal_handle)
{
    // 唤醒球位置发布节点
    change_state_to_active();
    // 初始化PID参数
    PIDController_PTZ.acquire_PID_variable("PTZ");
    PIDController_x.acquire_PID_variable("x");
    PIDController_y.acquire_PID_variable("y");
    PIDController_w.acquire_PID_variable("w");

    // 清空PID控制器积分量和历史误差值
    PIDController_x.clear();
    PIDController_y.clear();
    PIDController_w.clear();

    // 动作延时
    rclcpp::Rate loop_rate(120);
    rclcpp::Rate action_rate(1);

    //互斥锁初始化
    std::unique_lock<std::mutex> lock_joint(variable_mutex__1, std::defer_lock);
    std::unique_lock<std::mutex> lock_ball(variable_mutex_, std::defer_lock);
    std::unique_lock<std::mutex> lock(variable_mutex__, std::defer_lock);
    
    // 初始化action 变量
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<EmptyGoal::Result>();
    auto feedback = std::make_shared<EmptyGoal::Feedback>();

    //ball_info
    std::vector<geometry_msgs::msg::Point32> ball_info_;
    std::vector<geometry_msgs::msg::Point32> purple_info_;
    bool is_found_;
    std_msgs::msg::Header ball_info_header_;
    geometry_msgs::msg::Point32 tracking_ball_last;

    // chassis_state
    nav_msgs::msg::Odometry current_pose;

    //初始化状态机
    APPROACHINGBALL state_ = APPROACHINGBALL::IDLE;

    //初始化速度控制msg
    geometry_msgs::msg::Twist Data_To_Pub;

    //初始化机械臂控制msg
    std::shared_ptr< sensor_msgs::msg::JointState> JointControl_to_pub = std::make_shared<sensor_msgs::msg::JointState>(); 

    JointControl_to_pub->velocity.resize(4);
    JointControl_to_pub->velocity[0] = 0.0;
    JointControl_to_pub->velocity[1] = 0.0;
    JointControl_to_pub->velocity[2] = 0.0;
    JointControl_to_pub->velocity[3] = 0.0;
    JointControl_to_pub->effort.resize(4);

    sensor_msgs::msg::JointState JointState_;
    sensor_msgs::msg::JointState JointState_Last;
    JointState_.position.resize(4);
    JointState_.position[0] = 0.0;
    JointState_.position[1] = 0.0;
    JointState_.position[2] = 0.0;
    JointState_.position[3] = 0.0;

    JointState_Last.position.resize(4);
    JointState_Last.position.assign(JointState_.position.begin(), JointState_.position.end());

    // 获取关节初始位置
    JointState_.position.clear();
    JointState_.position.resize(4);
    lock_joint.lock();
    JointState_.position.assign(up_joint_state.position.begin(), up_joint_state.position.end());
    lock_joint.unlock();

    // 获取底盘初始位置
    if (!nav2_util::getCurrentPose(tf_current_pose, *tf_, "map", "base_link", 0.1))
        RCLCPP_ERROR(this->get_logger(),
                        "Current robot pose is not available.\n\n\n");
    
    // 关节初始化位置代码
    JointControl_to_pub->velocity[0] = 0.0;
    JointControl_to_pub->effort[0] = 0.0;

    // 初始化机械臂位置
    // arm_executor(JointControl_to_pub, JointState_, 0.0, -1.511, -1.308, 0.1);
    arm_executor(JointControl_to_pub, JointState_, 0.0, 0.119, 0.527, -0.040);
    action_rate.sleep();
    action_rate.sleep();

    RCLCPP_INFO(this->get_logger(), "Executing goal, id: %s", goal_handle->get_goal_id().data());


    while (rclcpp::ok()) {
        if(goal_handle->is_canceling())
        {
            JointControl_to_pub->velocity[0] = 0.0;
            JointControl_to_pub->effort[0] = 0.0;
            arm_executor(JointControl_to_pub, JointState_, 0.0, 0.119, 0.527, -0.400);
            Data_To_Pub.linear.x = 0;
            Data_To_Pub.linear.y = 0;
            Data_To_Pub.angular.z = 0;
            chassis_pub_->publish(Data_To_Pub);
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

        if (!nav2_util::getCurrentPose(tf_current_pose, *tf_, "map", "base_link", 0.1))
        RCLCPP_ERROR(this->get_logger(),
        "Current robot pose is not available.\n\n\n");

        JointState_.position.clear();
        JointState_.position.resize(4);
        ball_info_.clear();
        purple_info_.clear();

        lock_joint.lock();
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
        if(state_ == APPROACHINGBALL::IDLE)
            up_decision_making(ball_info_, purple_info_, is_found_, situation,0);
        else
            up_decision_making(ball_info_, purple_info_, is_found_, situation,1);
        RCLCPP_INFO(this->get_logger(), "Situation: %hhd",static_cast<int8_t>(situation));

        global_supervisor(tf_current_pose, ChassisPa);

        switch (state_) {
            case (APPROACHINGBALL::IDLE):
                state_ = (APPROACHINGBALL::LOOKING);
                break;
            case (APPROACHINGBALL::LOOKING):
            {
                static int stay_calm = 0;
                if(is_found_)
                {
                    stay_calm = 0;
                    JointControl_to_pub->effort[0] = 0.0;
                    
                    arm_executor(JointControl_to_pub, JointState_, 0.0, 0.119, 0.527, -0.4);
                    state_ = (APPROACHINGBALL::APPROACHING1);
                    // if(situation == Situation::Direct)
                    // {
                    //     /*-------*/
                    //     arm_executor(JointControl_to_pub, JointState_, 0.0, 0.119, 0.527, -0.4);
                    //     state_ = (APPROACHINGBALL::APPROACHING2);
                    // }else{
                    //     arm_executor(JointControl_to_pub, JointState_, 0.0, 0.119, 0.527, -0.4);
                    //     state_ = (APPROACHINGBALL::APPROACHING1);
                    // }
                }
                else {
                    Data_To_Pub.linear.x = 0;
                    Data_To_Pub.linear.y = 0;
                    Data_To_Pub.angular.z = 0;
                    if(stay_calm > 30)
                    {
                        //如果在这期间都没找到
                        // 旋转云台来找
                        // 进入Finding状态
                        state_ = (APPROACHINGBALL::FINDING);
                        stay_calm = 0;
                    }
                    stay_calm ++;
                }
                chassis_pub_->publish(Data_To_Pub);
                up_pub_->publish(*JointControl_to_pub);
                break;
            }

            case (APPROACHINGBALL::APPROACHING1):
            {
                JointControl_to_pub->effort[0] = 2.0;
                JointControl_to_pub->velocity[0] = - PIDController_PTZ.PosePID_Calc( tracking_ball.y - y_approach);
                Data_To_Pub.linear.x = - PIDController_x.PosePID_Calc(float(x_approach + 5 - tracking_ball.x))*std::cos(JointState_.position[0]);
                Data_To_Pub.linear.y = - PIDController_x.PosePID_Calc(float(x_approach + 5 - tracking_ball.x))*std::sin(JointState_.position[0]);
                Data_To_Pub.angular.z = 0;
                
                // 限制角度，如果角度大于114度，速度为0, 并回正
                if(fabs(JointState_.position[0]) >= 2.0)
                {
                    JointControl_to_pub->velocity[0] = 0;
                    state_ = (APPROACHINGBALL::TOFORWARD);
                }else
                {
                    Data_To_Pub.angular.z = 0;
                }

                RCLCPP_INFO(this->get_logger(), "Data_To_Pub: %f, %f", Data_To_Pub.linear.x, Data_To_Pub.linear.y);
                
                /*-------------------------------------------------*/
                if(SupervisorState == action_findball::Status::CRASHED)
                {
                    state_ = (APPROACHINGBALL::BACKWARD);
                }
                /*-------------------------------------------------*/
                
                if(tracking_ball.x - x_approach > 0 && fabs(tracking_ball.y - y_approach) < 50)
                {
                    static int count_brake = 0;
                    static double data_x_temp;
                    static double data_y_temp;
                    if(count_brake <= 25)
                    {
                        if(count_brake == 0)
                        {
                            data_x_temp = 0;
                            data_y_temp = 0;
                        }

                        JointControl_to_pub->velocity[0] = 0;
                        Data_To_Pub.linear.x = data_x_temp;
                        Data_To_Pub.linear.y = data_y_temp;
                        Data_To_Pub.angular.z = 0.0;
                        count_brake ++;
                    }
                    else {
                        count_brake = 0;
                        Data_To_Pub.linear.x = 0.0;
                        Data_To_Pub.linear.y = 0.0;
                        Data_To_Pub.angular.z = 0.0;
                        
                        arm_executor(JointControl_to_pub, JointState_, 0.0, 0.119, 0.527, 0.1);
                        //state_ = (APPROACHINGBALL::CATCHING);
                        if(situation == action_findball::Situation::Direct)
                            state_ = (APPROACHINGBALL::CATCHING);
                        else if(situation == action_findball::Situation::Purple_block)
                            state_ = (APPROACHINGBALL::CATCHING_PURPLE);
                        RCLCPP_INFO(this->get_logger(), "目标球");
                    }
                    RCLCPP_INFO(this->get_logger(), "Ball approached");
                }

                chassis_pub_->publish(Data_To_Pub);
                up_pub_->publish(*JointControl_to_pub);
                break;
            }

            case (APPROACHINGBALL::APPROACHING2):
            {
                JointControl_to_pub->effort[0] = 2.0;
                JointControl_to_pub->velocity[0] = - PIDController_PTZ.PosePID_Calc( tracking_ball.y - y_approach);
                Data_To_Pub.linear.x = - PIDController_x.PosePID_Calc(float(x_approach + 5 - tracking_ball.x))*std::cos(JointState_.position[0]);
                Data_To_Pub.linear.y = - PIDController_x.PosePID_Calc(float(x_approach + 5 - tracking_ball.x))*std::sin(JointState_.position[0]);
                Data_To_Pub.angular.z = 0;
                
                // 限制角度，如果角度大于114度，速度为0, 并回正
                if(fabs(JointState_.position[0]) >= 2.0)
                {
                    JointControl_to_pub->velocity[0] = 0;
                    state_ = (APPROACHINGBALL::TOFORWARD);
                }else
                {
                    Data_To_Pub.angular.z = 0;
                }

                RCLCPP_INFO(this->get_logger(), "Data_To_Pub: %f, %f", Data_To_Pub.linear.x, Data_To_Pub.linear.y);
                
                if(tracking_ball.x - x_approach > 0 && fabs(tracking_ball.y - y_approach) < 50)
                {
                    static int count_brake = 0;
                    static double data_x_temp;
                    static double data_y_temp;
                    if(count_brake <= 25)
                    {
                        if(count_brake == 0)
                        {
                            data_x_temp = 0;
                            data_y_temp = 0;
                        }

                        JointControl_to_pub->velocity[0] = 0;
                        Data_To_Pub.linear.x = data_x_temp;
                        Data_To_Pub.linear.y = data_y_temp;
                        Data_To_Pub.angular.z = 0.0;
                        count_brake ++;
                    }
                    else {
                        count_brake = 0;
                        Data_To_Pub.linear.x = 0.0;
                        Data_To_Pub.linear.y = 0.0;
                        Data_To_Pub.angular.z = 0.0;
                        /*-----*/
                        
                        arm_executor(JointControl_to_pub, JointState_, 0.0, -1.138, -1.308, 0.1);
                        state_ = (APPROACHINGBALL::CATCHING_3);
                        
                        RCLCPP_INFO(this->get_logger(), "目标球");
                    }
                    RCLCPP_INFO(this->get_logger(), "Ball approached");
                }
                chassis_pub_->publish(Data_To_Pub);
                up_pub_->publish(*JointControl_to_pub);
                break;
            }

            case (APPROACHINGBALL::TOFORWARD):
            {
                geometry_msgs::msg::Point32 tracking_ball__;
                static double angle_count = 0;
                static int angle_sign_count = 0;
                static double angle_sign = 0;
                if(situation == Situation::Direct)
                    tracking_ball__ = tracking_ball;
                else if(situation == Situation::Purple_block)
                    tracking_ball__ = tracking_purple;
                
                
                if(angle_sign_count == 0)
                    angle_sign = JointState_.position[0]>0 ? -1.0: 1.0;
                angle_sign_count ++;
                JointControl_to_pub->effort[0] = 2.0;
                JointControl_to_pub->velocity[0] = angle_sign * 0.2;
                if(fabs(JointState_.position[0])<1.57) JointControl_to_pub->velocity[0] = 0;
                Data_To_Pub.linear.x = 0;
                Data_To_Pub.linear.y = 0;
                Data_To_Pub.angular.z =  - PIDController_w.PosePID_Calc(tracking_ball__.y - y_approach);

                if(fabs(tracking_ball__.y - y_approach) < 15)
                {
                    angle_count ++;
                }else {
                    angle_count = 0;
                }
                if(angle_count > 20)
                {
                    angle_count = 0;
                    angle_sign_count = 0;
                    angle_sign = 0;
                    state_ = (APPROACHINGBALL::APPROACHING1);
                }

                chassis_pub_->publish(Data_To_Pub);
                up_pub_->publish(*JointControl_to_pub);
                break;
            }
    
            case (APPROACHINGBALL::CATCHING):
            {
                JointControl_to_pub->effort[0] = 2.0;
                JointControl_to_pub->velocity[0] = - PIDController_PTZ.PosePID_Calc( tracking_ball.y - y_approach);

                Data_To_Pub.linear.x = - PIDController_x_catch.PosePID_Calc(float(x_catch +30 - tracking_ball.x))*std::cos(JointState_.position[0]);
                Data_To_Pub.linear.y = - PIDController_x_catch.PosePID_Calc(float(x_catch +30 - tracking_ball.x))*std::sin(JointState_.position[0]);
                Data_To_Pub.angular.z = 0;

                // 限制角度，如果角度大于114度，速度为0
                if(fabs(JointState_.position[0]) >= 2.0)
                {
                    JointControl_to_pub->velocity[0] = 0;
                    state_ = (APPROACHINGBALL::TOFORWARD);
                }else
                {
                    Data_To_Pub.angular.z = 0;
                }

                if((tracking_ball.x - (x_catch +20) > 0 && fabs(tracking_ball.y - y_approach) < 30) || JointState_.position[3] >= 5.0)
                //if(JointState_.position[3] >= 5.0 )
                {
                    RCLCPP_INFO(this->get_logger(), "Catching");
                    JointControl_to_pub->position[3] = -0.3;
                    up_pub_->publish(*JointControl_to_pub);
                    arm_executor(JointControl_to_pub, JointState_, 0.0, 0.119, 0.527, -0.2);
                    JointControl_to_pub->effort[0] = 2.0;
                    JointControl_to_pub->velocity[0] = 0;
                    Data_To_Pub.linear.x = 0;
                    Data_To_Pub.linear.y = 0;
                    Data_To_Pub.angular.z = 0;
                    
                    state_ = (APPROACHINGBALL::CATCHING_2);
                }

                chassis_pub_->publish(Data_To_Pub);
                up_pub_->publish(*JointControl_to_pub);
                break;
            }

            case (APPROACHINGBALL::CATCHING_PURPLE):
            {
                
                static int action_step = -1;
                if(action_step == -1)
                {
                    JointControl_to_pub->effort[0] = 2.0;
                    JointControl_to_pub->velocity[0] = - PIDController_PTZ.PosePID_Calc( tracking_purple.y - y_approach);

                    Data_To_Pub.linear.x = - PIDController_x_catch.PosePID_Calc(float(x_catch +30 - tracking_purple.x))*std::cos(JointState_.position[0]);
                    Data_To_Pub.linear.y = - PIDController_x_catch.PosePID_Calc(float(x_catch +30 - tracking_purple.x))*std::sin(JointState_.position[0]);
                    Data_To_Pub.angular.z = 0;

                    // 限制角度，如果角度大于114度，速度为0
                    if(fabs(JointState_.position[0]) >= 2.0)
                    {
                        JointControl_to_pub->velocity[0] = 0;
                        state_ = (APPROACHINGBALL::TOFORWARD);
                    }else
                    {
                        Data_To_Pub.angular.z = 0;
                    }

                    if(tracking_purple.x - (x_catch +25) > 0 && fabs(tracking_purple.y - y_approach) < 20)
                    {
                        RCLCPP_INFO(this->get_logger(), "Catching");
                        JointControl_to_pub->position[3] = -0.3;
                        up_pub_->publish(*JointControl_to_pub);
                        arm_executor(JointControl_to_pub, JointState_, 0.0, 0.119, 0.527, -0.2);
                        JointControl_to_pub->effort[0] = 2.0;
                        JointControl_to_pub->velocity[0] = 0;
                        Data_To_Pub.linear.x = 0;
                        Data_To_Pub.linear.y = 0;
                        Data_To_Pub.angular.z = 0;
                        action_step = 0;
                    }

                    chassis_pub_->publish(Data_To_Pub);
                    up_pub_->publish(*JointControl_to_pub);
                }

                
                switch (action_step) {
                    case 0:
                    {
                        arm_executor(JointControl_to_pub, JointState_, JointState_.position[0], 0.119, 0.527, -0.25);
                            action_rate.sleep();
                            action_step = 1;
                        break;
                    }
                    case 1:
                    {
                        arm_executor(JointControl_to_pub, JointState_, JointState_.position[0], -0.188, -0.96, -0.25);
                            action_rate.sleep();
                            action_step = 2;
                        break;
                    }
                    case 2:
                    {
                        arm_executor(JointControl_to_pub, JointState_, JointState_.position[0], -0.5, 0.4, -0.25);
                        action_rate.sleep();
                        JointControl_to_pub->effort[0] = 0.0;
                        PTZ_executor(JointControl_to_pub, JointState_, 0.0);
                        action_rate.sleep();
                        action_step = 3;
                        break;
                    }
                    case 3:
                    {
                        arm_executor(JointControl_to_pub, JointState_, 0.0, -2.6, 1.477, -0.2);
                        action_rate.sleep();
                        action_step = 4;
                        break;
                    }
                    case 4:
                    {
                        arm_executor(JointControl_to_pub, JointState_, 0.0, -2.6, 1.477, 0.223);
                        action_step = 5;
                        action_rate.sleep();
                        break;
                    }
                    case 5:
                    {
                        arm_executor(JointControl_to_pub, JointState_, 0.0, 0.119, 0.527, -0.400);
                        action_step = 6;
                        break;
                    }
                    case 6:
                    {
                        state_ = (APPROACHINGBALL::APPROACHING1);
                        action_step = -1;
                        break;
                    }
                }
                break;
            }

            case(APPROACHINGBALL::CATCHING_2):
            {
                static int action_step = 0;
                switch (action_step) {
                    case 0:
                    {
                        arm_executor(JointControl_to_pub, JointState_, JointState_.position[0], 0.119, 0.527, -0.3);
                        action_step = 1;
                        break;
                    }
                    case 1:
                    {
                        arm_executor(JointControl_to_pub, JointState_, JointState_.position[0], -0.05, -0.96, -0.3);
                        action_rate.sleep();
                        action_step = 2;
                        break;
                    }
                    case 2:
                    {
                        arm_executor(JointControl_to_pub, JointState_, JointState_.position[0], -0.3, 0.4, -0.3);
                        action_rate.sleep();
                        JointControl_to_pub->effort[0] = 0.0;
                        PTZ_executor(JointControl_to_pub, JointState_, 0.0);
                        action_rate.sleep();
                        action_step = 3;
                        break;
                    }

                    /*------  结束 -------- */
                    // case 3:
                    // {
                    //     arm_executor(JointControl_to_pub, JointState_, 0.0, -2.6, 1.477, -0.25);
                    //         action_rate.sleep();
                    //         action_step = 4;
                    //     break;
                    // }
                    // case 4:
                    // {
                    //     arm_executor(JointControl_to_pub, JointState_, 0.0, -2.6, 1.477, 0.15);
                    //     action_step = 5;
                    //     action_rate.sleep();
                    //     break;
                    // }
                    // case 5:
                    // {
                    //     arm_executor(JointControl_to_pub, JointState_, 0.0, 0.119, 0.527, -0.400);
                    //     action_step = 6;
                    //     break;
                    // }
                    case 3:
                    {
                        state_ = (APPROACHINGBALL::SUCCEED);
                        action_step = 0;
                        break;
                    }
                }
                break;
            }

            case (APPROACHINGBALL::CATCHING_3):
            {
                
                static int action_step = -1;
                if(action_step == -1)
                {
                    JointControl_to_pub->effort[0] = 2.0;
                    JointControl_to_pub->velocity[0] = - PIDController_PTZ.PosePID_Calc( tracking_ball.y - y_approach);

                    Data_To_Pub.linear.x = - PIDController_x_catch.PosePID_Calc(float(x_catch +45 - tracking_ball.x))*std::cos(JointState_.position[0]);
                    Data_To_Pub.linear.y = - PIDController_x_catch.PosePID_Calc(float(x_catch +45 - tracking_ball.x))*std::sin(JointState_.position[0]);
                    Data_To_Pub.angular.z = 0;

                    // 限制角度，如果角度大于114度，速度为0
                    if(fabs(JointState_.position[0]) >= 2.0)
                    {
                        JointControl_to_pub->velocity[0] = 0;
                        state_ = (APPROACHINGBALL::TOFORWARD);
                    }else
                    {
                        Data_To_Pub.angular.z = 0;
                    }

                    if(tracking_ball.x - (x_catch +30) > 0 && fabs(tracking_ball.y - y_approach) < 20)
                    {
                        RCLCPP_INFO(this->get_logger(), "Catching");
                        JointControl_to_pub->position[3] = -0.3;
                        up_pub_->publish(*JointControl_to_pub);
                        arm_executor(JointControl_to_pub, JointState_, 0.0, 0.08, -0.457, 0.1);
                        JointControl_to_pub->effort[0] = 2.0;
                        JointControl_to_pub->velocity[0] = 0;
                        Data_To_Pub.linear.x = 0;
                        Data_To_Pub.linear.y = 0;
                        Data_To_Pub.angular.z = 0;
                        action_step = 0;
                    }

                    chassis_pub_->publish(Data_To_Pub);
                    up_pub_->publish(*JointControl_to_pub);
                }

                
                switch (action_step) {
                    case 0:
                    {
                        action_rate.sleep();
                        action_rate.sleep();
                        arm_executor(JointControl_to_pub, JointState_, JointState_.position[0], -0.188, -0.96, 0.1);
                        // -0.25
                        action_rate.sleep();
                        action_step = 1;
                        break;
                    }
                    case 1:
                    {
                        action_rate.sleep();
                        arm_executor(JointControl_to_pub, JointState_, JointState_.position[0], -0.188, -0.96, -0.25);
                        action_rate.sleep();
                        action_rate.sleep();
                        action_step = 2;
                        break;
                    }

                    case 2:
                    {
                        arm_executor(JointControl_to_pub, JointState_, JointState_.position[0], -0.5, 0.4, -0.25);
                        action_rate.sleep();
                        JointControl_to_pub->effort[0] = 0.0;
                        PTZ_executor(JointControl_to_pub, JointState_, 0.0);
                        action_rate.sleep();
                        action_step = 3;
                        break;
                    }
                    case 3:
                    {
                        state_ = (APPROACHINGBALL::SUCCEED);
                        action_step = -1;
                        break;
                    }
                }
                break;
            }

            case(APPROACHINGBALL::BUMP_AWAY):
            {
                static int count_bump = 0;
                static geometry_msgs::msg::PoseStamped goal_pose_bump;

                if(count_bump == 0)
                {
                    goal_pose_bump = tf_current_pose;
                    arm_executor(JointControl_to_pub, JointState_, 0.0, 0.119, 0.527, -0.400);
                }
                
                count_bump ++;
                // Data_To_Pub.x = ;
                // Data_To_Pub.linear.y = ;
                // Data_To_Pub.angular.z = 0;

                break;
            }

            case(APPROACHINGBALL::NO_BALL):
            {
                break;
            }

            case(APPROACHINGBALL::BACKWARD):
            {
                static geometry_msgs::msg::Twist Chassis_Cmd;
                static geometry_msgs::msg::PoseStamped Chassis_Odom;
                static int count_backward = 0;
                if(count_backward == 0)
                {
                    Chassis_Cmd.linear.x = -Data_To_Pub.linear.x;
                    Chassis_Cmd.linear.y = -Data_To_Pub.linear.y;
                    Chassis_Cmd.angular.z = -Data_To_Pub.angular.z;
                    Chassis_Odom = tf_current_pose;
                    count_backward ++;
                }
                Data_To_Pub = Chassis_Cmd;

                if((Chassis_Odom.pose.position.x - tf_current_pose.pose.position.x)*(Chassis_Odom.pose.position.x - tf_current_pose.pose.position.x) + (Chassis_Odom.pose.position.y - tf_current_pose.pose.position.y)*(Chassis_Odom.pose.position.y - tf_current_pose.pose.position.y) > 0.5*0.5)
                {
                    count_backward = 0;
                    Data_To_Pub.linear.x = 0;
                    Data_To_Pub.linear.y = 0;
                    state_ = (APPROACHINGBALL::APPROACHING1);
                }
                chassis_pub_->publish(Data_To_Pub);
                break;
            }

            case (APPROACHINGBALL::FINDING):
            {
                static int state_angle_direction = 0;
                static rclcpp::Time time_execute;
                rclcpp::Duration command_time_allowance_{0, 0};
                //向左旋转云台 45 度
                if(state_angle_direction == 0)
                {
                    time_execute = this->now();
                    if(is_found_)  state_ = (APPROACHINGBALL::APPROACHING1);
                    command_time_allowance_ = this->now() - time_execute;
                    if(command_time_allowance_.seconds() > 2)  state_angle_direction = 1;
                    JointControl_to_pub->effort[0] = 0.0;
                    JointControl_to_pub->position[0] = -0.7;
                    Data_To_Pub.linear.x = 0.0;
                    Data_To_Pub.linear.y = 0.0;
                    Data_To_Pub.angular.z = 0.0;
                    chassis_pub_->publish(Data_To_Pub);
                    up_pub_->publish(*JointControl_to_pub);
                }
                //向右旋转云台 45 度
                else if(state_angle_direction == 1)
                {
                    time_execute = this->now();
                    if(is_found_)  state_ = (APPROACHINGBALL::APPROACHING1);
                    command_time_allowance_ = this->now() - time_execute;
                    if(command_time_allowance_.seconds() > 2)  state_angle_direction = -1;
                    JointControl_to_pub->effort[0] = 0.0;
                    JointControl_to_pub->position[0] = 0.7;
                    Data_To_Pub.linear.x = 0.0;
                    Data_To_Pub.linear.y = 0.0;
                    Data_To_Pub.angular.z = 0.0;
                    chassis_pub_->publish(Data_To_Pub);
                    up_pub_->publish(*JointControl_to_pub);
                }

                //再不行转为NO_BALL
                else if(state_angle_direction == -1)
                {
                    JointControl_to_pub->effort[0] = 0.0;
                    JointControl_to_pub->position[0] = 0.0;
                    Data_To_Pub.linear.x = 0.0;
                    Data_To_Pub.linear.y = 0.0;
                    Data_To_Pub.angular.z = 0.0;
                    chassis_pub_->publish(Data_To_Pub);
                    up_pub_->publish(*JointControl_to_pub);
                    state_ =  APPROACHINGBALL::NO_BALL;
                }
                break;
            }

            case (APPROACHINGBALL::BUCKLE):
            {
                break;
            }
        }
        
        // JointControl_to_pub->header.stamp = this->now();
        // JointControl_to_pub->effort[0] = 2.0;
        // JointControl_to_pub->velocity[0] = - PIDController_PTZ.PosePID_Calc( tracking_ball.y - 400);
        // RCLCPP_INFO(this->get_logger(), "PTZ: %f", JointControl_to_pub->velocity[0]);
        // up_pub_->publish(*JointControl_to_pub);

        // JointControl_to_pub->effort[0] = 2.0;
        // JointControl_to_pub->velocity[0] = - PIDController_PTZ.PosePID_Calc( tracking_ball.y - 400);
        // Data_To_Pub.x = - PIDController_x.PosePID_Calc(float(260 - tracking_ball.x))*std::cos(JointState_.position[0]);
        // Data_To_Pub.y = - PIDController_x.PosePID_Calc(float(260 - tracking_ball.x))*std::sin(JointState_.position[0]);
        // // Data_To_Pub.x = - PIDController_x.PosePID_Calc(float(260 - tracking_ball.x));
        // // Data_To_Pub.y = - PIDController_y.PosePID_Calc(float(y_approach - tracking_ball.y));
        // Data_To_Pub.angular.z = 0;
        // RCLCPP_INFO(this->get_logger(), "position[0]: ,%f", JointState_.position[0]);
        // RCLCPP_INFO(this->get_logger(), "Data_To_Pub: %f, %f", Data_To_Pub.x, Data_To_Pub.y);

        // JointControl_to_pub->effort[0] = 2.0;
        // JointControl_to_pub->velocity[0] = 0;
        // Data_To_Pub.x = 0;
        // Data_To_Pub.y = 0;
        // Data_To_Pub.angular.z =  - PIDController_w.PosePID_Calc(tracking_ball.y - y_approach);

        if(state_ == (APPROACHINGBALL::SUCCEED))
        {
            JointControl_to_pub->velocity[0] = 0.0;
            JointControl_to_pub->effort[0] = 0.0;
            arm_executor(JointControl_to_pub, JointState_, 0.0, 0.119, 0.527, -0.400);
            Data_To_Pub.linear.x = 0;
            Data_To_Pub.linear.y = 0;
            Data_To_Pub.angular.z = 0;
            chassis_pub_->publish(Data_To_Pub);
            state_ = (APPROACHINGBALL::IDLE);
            break;
        }
        RCLCPP_INFO(this->get_logger(), "state: %d", state_);
        tracking_ball_last = tracking_ball;
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
    geometry_msgs::msg::Twist Data_To_Pub;
    Data_To_Pub.linear.x = 0;
    Data_To_Pub.linear.y = 0;
    Data_To_Pub.angular.z = 0;

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
                    Data_To_Pub.linear.x = -PIDController_x.PID_Calc((x_next - tracking_ball.x)*cos(JointState_.position[0]));
                    Data_To_Pub.linear.y = -PIDController_y.PID_Calc((y_catch - tracking_ball.y)*sin(JointState_.position[0]));
                    Data_To_Pub.angular.z = 0;
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
    std::vector<geometry_msgs::msg::Point32> &purple_info_, bool is_found_, Situation &situation_,int reset)
{
    cv::Point3d most_near;
    static int count_lost = 0;
    static int init = 0;
    init = reset;

    std::vector<geometry_msgs::msg::Point32> unblocked_ball;
    unblocked_ball.clear();

    if(is_found_)
    {
        for(auto &target_ball: ball_info_)
        {
            for(auto &purple_ball : purple_info_)
            {
                if(purple_ball.y < target_ball.y + 5 || fabs(purple_ball.x - target_ball.x) > target_ball.z + purple_ball.z - 5)
                    unblocked_ball.push_back(target_ball);
            }
        }
    }

    if(is_found_)
    {
        count_lost = 0;
        static int count = 0;

        if(unblocked_ball.size() > 0)
        {
            most_near.x = unblocked_ball[0].y;
            most_near.y = unblocked_ball[0].x;
            most_near.z = unblocked_ball[0].z;
        }else {
            most_near.x = ball_info_[0].y;
            most_near.y = ball_info_[0].x;
            most_near.z = ball_info_[0].z;
        }

        if(init == 0)
        {
            Kalman->statePost = (cv::Mat_<float>(4, 1) << 0, 0, 0, 0);
            Kalman->errorCovPost = (cv::Mat_<float>(4, 4) <<
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1);
            current_measurement = cv::Vec2f(most_near.y, most_near.x);
            Kalman->correct(cv::Mat(current_measurement));
            current_prediction = Kalman->predict();
            tracking_ball.x = current_prediction[1];
            tracking_ball.y = current_prediction[0];
            tracking_ball.z = most_near.z;
            if((most_near.x-tracking_ball.x)*(most_near.x-tracking_ball.x) + (most_near.y-tracking_ball.y)*(most_near.y-tracking_ball.y) < 25)
                init = 1;
        }else {
            if((most_near.x-tracking_ball.x)*(most_near.x-tracking_ball.x) + (most_near.y-tracking_ball.y)*(most_near.y-tracking_ball.y) < 900 + count*2)
            {
                current_measurement = cv::Vec2f(most_near.y, most_near.x);
                Kalman->correct(cv::Mat(current_measurement));
                current_prediction = Kalman->predict();
                tracking_ball.x = current_prediction[1];
                tracking_ball.y = current_prediction[0];
                tracking_ball.z = most_near.z;
                count = 0;
            }else
            {
                if(count > 40)
                {
                std::cout << "No target found!" << std::endl;
                count = 0;
                Kalman->statePost = (cv::Mat_<float>(4, 1) << 0, 0, 0, 0);
                Kalman->errorCovPost = (cv::Mat_<float>(4, 4) <<
                    1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1);
                current_measurement = cv::Vec2f(most_near.y, most_near.x);
                Kalman->correct(cv::Mat(current_measurement));
                current_prediction = Kalman->predict();
                tracking_ball.x = current_prediction[1];
                tracking_ball.y = current_prediction[0];
                tracking_ball.z = most_near.z;
                }else {
                    for(size_t i = 0; auto &target_ball: ball_info_)
                    {
                        if((target_ball.x-tracking_ball.y)*(target_ball.x-tracking_ball.y) + (target_ball.y-tracking_ball.x)*(target_ball.y-tracking_ball.x) < 1000)
                        {
                            current_measurement = cv::Vec2f(target_ball.x, target_ball.y);
                            Kalman->correct(cv::Mat(current_measurement));
                            current_prediction = Kalman->predict();
                            tracking_ball.x = current_prediction[1];
                            tracking_ball.y = current_prediction[0];
                            tracking_ball.z = target_ball.z;
                            count = 0;
                            break;
                        }
                        if(i == ball_info_.size()-1)
                        {
                            current_measurement = last_measurement;
                            Kalman->correct(cv::Mat(current_measurement));
                            current_prediction = Kalman->predict();
                            tracking_ball.x = current_prediction[1];
                            tracking_ball.y = current_prediction[0];
                            tracking_ball.z = most_near.z;
                        }
                        i++;
                    }
                }
                count++;
            }
        }

    if(situation_ == Situation::Direct)
    {
        for(auto &purple_ball : purple_info_)
        {
            if(purple_ball.y > tracking_ball.x && fabs(purple_ball.x - tracking_ball.y) < tracking_ball.z + purple_ball.z + 50)
            {
                Kalman_purple->statePost = (cv::Mat_<float>(4, 1) << 0, 0, 0, 0);
                Kalman_purple->errorCovPost = (cv::Mat_<float>(4, 4) <<
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1);
                current_measurement_purple = cv::Vec2f(purple_ball.x, purple_ball.y);
                Kalman_purple->correct(cv::Mat(current_measurement_purple));
                current_prediction_purple = Kalman_purple->predict();
                if(purple_ball.y > tracking_ball.x && fabs(purple_ball.x - tracking_ball.y) < tracking_ball.z + purple_ball.z - 10)
                {
                    RCLCPP_INFO(this->get_logger(), "紫球挡住了目标球。。。。。。");
                    situation_ =Situation::Purple_block;
                }
                break;
            }
        }
    }else {
        static int count_purple = 0;
        for(size_t i = 0; i< purple_info_.size();i++)
        {
            if((purple_info_[i].y-tracking_purple.x)*(purple_info_[i].y-tracking_purple.x) + (purple_info_[i].x-tracking_purple.y)*(purple_info_[i].x-tracking_purple.y) < 800)
            {
                current_measurement_purple = cv::Vec2f(purple_info_[i].x, purple_info_[i].y);
                Kalman_purple->correct(cv::Mat(current_measurement_purple));
                current_prediction_purple = Kalman_purple->predict();
                tracking_purple.x = current_prediction_purple[1];
                tracking_purple.y = current_prediction_purple[0];
                tracking_purple.z = purple_info_[i].z;
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
                RCLCPP_INFO(this->get_logger(), "回到Direct");
            }
        }
        if(fabs(tracking_purple.y - tracking_ball.y) > tracking_ball.z + tracking_purple.z - 10)
        {
            situation_ = Situation::Direct;
        }
    }

    }else {
        RCLCPP_INFO(this->get_logger(), "Ball not found.......");
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
// {
//     static bool last_found = false;
//     if(is_found_)
//     {
//         for(size_t i = 0; i < ball_info_.size(); i++)
//         {
//             current_measurement = cv::Vec2f(ball_info_[i].x, ball_info_[i].y);
//             current_radius = ball_info_[i].z;
//             if((current_measurement[0]-last_measurement[0])*(current_measurement[0]-last_measurement[0]) + (current_measurement[1]-last_measurement[1])*(current_measurement[1]-last_measurement[1]) < 225)
//             {
//                 Kalman->correct(cv::Mat(current_measurement));
//                 current_prediction = Kalman->predict();
//                 // 这里进行了一次x，y的交换
//                 tracking_ball.x = current_prediction[1];
//                 tracking_ball.y = current_prediction[0];
//                 tracking_ball.z = current_radius;
//                 color_ = 0;
//                 //cv::circle(color_image,cv::Point(current_prediction[0],current_prediction[1]),ball_info_[i][2],cv::Scalar(255,0,0),2);
//                 break;
//             }
//             if(i == ball_info_.size()-1)
//             {
//                 static int count = 0;
//                 std::cout << "No target found!" << std::endl;
//                 if(last_found) count++; else count = 0;
                
//                 if(count >= 10)
//                 {
//                     std::cout << "目标球丢失，已经超时切换" << std::endl;
//                     current_measurement = cv::Vec2f(ball_info_[0].x, ball_info_[0].y);
//                     current_radius = ball_info_[0].z;
//                     Kalman->statePost = (cv::Mat_<float>(4, 1) << 0, 0, 0, 0);
//                     Kalman->errorCovPost = (cv::Mat_<float>(4, 4) <<
//                         1, 0, 0, 0,
//                         0, 1, 0, 0,
//                         0, 0, 1, 0,
//                         0, 0, 0, 1);
                    
//                     // 这里进行了一次x，y的交换
//                     tracking_ball.x = current_measurement[1];
//                     tracking_ball.y = current_measurement[0];
//                     tracking_ball.z = current_radius;
//                     color_ = 0;
//                 }
//                 //RCLCPP_INFO(this->get_logger(), "target x: %f, y: %f", tracking_ball.x, tracking_ball.y);
//                 //cv::circle(color_image,cv::Point(last_measurement[0],last_measurement[1]),last_radius,cv::Scalar(255,0,0),2);
//             }
//         }
//         int purple_count = 0;
//         for(auto &purple_ball : purple_info_)
//         {
//             if(purple_ball.y > tracking_ball.x && fabs(purple_ball.x - img_center) < tracking_ball.z + purple_ball.z)
//             {
//                 RCLCPP_INFO(this->get_logger(), "紫球挡住了目标球。。。。。。");
//                 tracking_purple.x = purple_ball.y;
//                 tracking_purple.y = purple_ball.x;
//                 tracking_purple.z = purple_ball.z;
//                 color_ = 1;
//             }
//         }

//         RCLCPP_INFO(this->get_logger(), "target x: %f, y: %f", tracking_ball.x, tracking_ball.y);
//         last_prediction = current_prediction;
//         last_measurement = current_measurement;
//         last_radius = current_radius;
//     }
//     else
//     {
//         tracking_ball.x = current_measurement[1];
//         tracking_ball.y = current_measurement[0];
//         tracking_ball.z = current_radius;
//         color_ = 0;
//         std::cout << "asuming target x: " << tracking_ball.x << " y: " << tracking_ball.y << std::endl;
//         RCLCPP_INFO(this->get_logger(), "Ball not found.......");
//         return false;
//     }
//     last_found = is_found_;
//     return true;
// }


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

float action_findball::ApproachingBall::PTZ_ANGLE_decision_making(geometry_msgs::msg::Point32 &tracking_ball_)
{
    int index = round(atan2(480 - tracking_ball_.x, 320 - tracking_ball_.y) * 180.0 / CV_PI);

    auto it = PTZ_2_camera.find(index);
    if(it != PTZ_2_camera.end())
        return it->second;
    else
    {
        if(index < 20) return 0.60;
        if(index > 160) return  -0.60;
    }
    return 0.0;
}

void action_findball::ApproachingBall::get_jointstate_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(variable_mutex__1);
    up_joint_state = *msg;
    lock.unlock();
}

bool action_findball::ApproachingBall::PTZ_executor(const std::shared_ptr<sensor_msgs::msg::JointState> JointControl_to_pub,
                    const sensor_msgs::msg::JointState &JointState_,
                    double joint1)
{
    rclcpp::Rate loop_rate(8);
    rclcpp::Rate action_rate(2);
    JointControl_to_pub->effort[0] = 0.0;
    int step = (fabs(JointState_.position[0]-joint1)/3.1415926*20.0 >= 1 ? fabs(JointState_.position[0]-joint1)/3.1415926*20:1.0);
    RCLCPP_INFO(this->get_logger(), "step: %d", step);
    for(int i = 1;i <= step;i++)
    {
        JointControl_to_pub->position[0] = (JointState_.position[0] + (joint1 - JointState_.position[0])/step*i);
        JointControl_to_pub->header.stamp = this->now();
        up_pub_->publish(*JointControl_to_pub);
        loop_rate.sleep();
    }

    if(fabs(JointState_.position[0]-joint1)<=0.1)
    {
        return true;
    }
    else{
        return false;
    }
}

bool action_findball::ApproachingBall::arm_executor(const std::shared_ptr<sensor_msgs::msg::JointState> JointControl_to_pub,
                    const sensor_msgs::msg::JointState &JointState_,
                        double joint1, double joint2, double joint3, double joint4)
{
    rclcpp::Rate loop_rate(20);
    rclcpp::Rate action_rate(2);
    int step = (fabs(JointState_.position[1]-joint2)/3.1415926*20.0 >= 1 ? fabs(JointState_.position[1]-joint2)/3.1415926*20:1.0);
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
        fabs(JointState_.position[1]-joint2)<=0.3)
    {
        return true;
    }
    else{
        return false;
    }
}

void action_findball::ApproachingBall::global_supervisor(const geometry_msgs::msg::PoseStamped &tf_current_pose_,const nav_msgs::msg::Odometry &ChassisPa_)
{
    // 电子围栏判断
    if(tf_current_pose_.pose.position.x + car_length < area3.left_up.x && tf_current_pose_.pose.position.x + car_length > area3.right_down.x &&
        tf_current_pose_.pose.position.y + car_length < area3.left_up.y && tf_current_pose_.pose.position.y + car_length > area3.right_down.y)
    {
        std::cout << "In area3" << std::endl;
        SupervisorState = action_findball::Status::RUNNING;
    }else {
        std::cerr << "Out of area3" << std::endl;
        SupervisorState = action_findball::Status::CRASHED;
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

/*
    case (APPROACHINGBALL::TOFORWARD):
    {
        static int state_inner = 0;
        static Status spin_return;
        static int calmdown = 0;
        static float target_anlgle;
        
        static rclcpp::Duration time_allowance(8, 0);
        
        // 底盘控制
        if(state_inner == 0)
        {
            JointControl_to_pub->effort[0] = 0.0;
            target_anlgle = JointState_.position[0];
            RCLCPP_INFO(this->get_logger(), "target_anlgle: %f", target_anlgle);
            spin_to_func->onRun(target_anlgle, time_allowance, current_pose);
            RCLCPP_INFO(this->get_logger(), "Forwarding");
            state_inner = 1;
        }else if(state_inner == 1){
            spin_return = spin_to_func->onCycleUpdate(current_pose);
            if(spin_return == Status::SUCCEEDED)  state_inner = 2;
        }
        JointControl_to_pub->effort[0] = 0.0;
        JointControl_to_pub->position[0] = 0;

        if(spin_return == Status::SUCCEEDED) 
        {
            calmdown++;
        }

        if(calmdown > 5)
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

*/

                    // case 3:
                    // {
                    //     arm_executor(JointControl_to_pub, JointState_, 0.0, -2.6, 1.477, -0.2);
                    //     action_rate.sleep();
                    //     action_step = 2;
                    //     break;
                    // }
                    // case 4:
                    // {
                    //     arm_executor(JointControl_to_pub, JointState_, 0.0, -2.6, 1.477, 0.15);
                    //     action_step = 5;
                    //     action_rate.sleep();
                    //     break;
                    // }
                    // case 5:
                    // {
                    //     arm_executor(JointControl_to_pub, JointState_, 0.0, 0.119, 0.527, -0.400);
                    //     action_step = 6;
                    //     break;
                    // }