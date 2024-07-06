#include <cstddef>
#include <geometry_msgs/msg/detail/pose2_d__struct.hpp>
#include <memory>
#include <mutex>

#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rcutils/error_handling.h>
#include <rmw/types.h>
#include <tf2/impl/utils.h>
#include <tf2/utils.h>
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
const int x_approach = 260;
const int y_approach = 310;
// 判定球将被抓住的阈值，相对于 image_size_half
const int x_catch = 430;
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
    PIDController_x_catch(0.0016,0.0008,0.01),
    PIDController_y(0.0006,0.00001,0.0001),
    PIDController_w(1.3,0.02,0.06),

    left_ball_count(0), right_ball_count(0), toward(TOWARD::FRONT)
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
    chassis_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/filtered_odom", 2, std::bind(&ApproachingBall::get_pose_callback,this,std::placeholders::_1));
    up_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/car/up_cmd", 2);
    up_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/car/up_fdbk", *qos_profile, std::bind(&ApproachingBall::get_jointstate_callback,this,std::placeholders::_1));
    goal_update_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_update", 5);
    //camera_switch_pub_= this->create_publisher<rc2024_interfaces::msg::CameraSwitch>("camera_switch", 2);

    Kalman = std::make_shared<cv::KalmanFilter>(4, 2);
    Kalman->measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);
    Kalman->transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
    Kalman->processNoiseCov = (cv::Mat_<float>(4, 4) << 0.1, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0.1);
    Kalman->measurementNoiseCov = (cv::Mat_<float>(2, 2) << 0.1, 0, 0, 0.1);

    Kalman_purple = std::make_shared<cv::KalmanFilter>(4, 2);
    Kalman_purple->measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);
    Kalman_purple->transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
    Kalman_purple->processNoiseCov = (cv::Mat_<float>(4, 4) << 0.1, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0.1);
    Kalman_purple->measurementNoiseCov = (cv::Mat_<float>(2, 2) << 1, 0, 0, 1);

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

    tf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);
    this->declare_parameter("start_side", "UNKNOWN");
    this->get_parameter("start_side", start_side);
    RCLCPP_INFO(this->get_logger(), "start_side: %s", start_side.c_str());
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
    rclcpp::Rate half_rate(2);

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
    int reset = 0;

    // chassis_state
    nav_msgs::msg::Odometry current_pose;

    //初始化状态机
    APPROACHINGBALL state_ = APPROACHINGBALL::IDLE;
    APPROACHINGBALL state_last = APPROACHINGBALL::IDLE;
    SupervisorState = action_findball::Status::RUNNING;

    //初始化速度控制msg
    geometry_msgs::msg::Twist Data_To_Pub;

    //初始化机械臂控制msg
    std::shared_ptr< sensor_msgs::msg::JointState> JointControl_to_pub = std::make_shared<sensor_msgs::msg::JointState>(); 
    JointControl_to_pub->velocity.resize(1);
    JointControl_to_pub->velocity[0] = 0.0;
    JointControl_to_pub->effort.resize(4);
    JointControl_to_pub->effort[0] = 1.0;
    JointControl_to_pub->effort[1] = 1.0;
    JointControl_to_pub->effort[2] = 1.0;
    JointControl_to_pub->effort[3] = 1.0;
    JointControl_to_pub->position.resize(4);

    //初始化关节状态
    sensor_msgs::msg::JointState JointState_;
    JointState_.position.clear();
    JointState_.velocity.clear();
    JointState_.position.resize(4);
    JointState_.velocity.resize(4);
    lock_joint.lock();
    JointState_.position.assign(up_joint_state.position.begin(), up_joint_state.position.end());
    JointState_.velocity.assign(up_joint_state.velocity.begin(), up_joint_state.velocity.end());
    lock_joint.unlock();

    // 获取底盘初始位置JointControl_to_pub->position[0]
    if (!nav2_util::getCurrentPose(tf_current_pose, *tf_, "map", "base_link", 0.1))
        RCLCPP_ERROR(this->get_logger(),
        "Current robot pose is not available.\n\n\n");
    
    // 关节初始化位置代码
    // arm_executor(JointControl_to_pub, JointState_, 0.0, -1.511, -1.308, 0.1);
    JointControl_to_pub->effort[0] = 1.0;
    JointControl_to_pub->effort[1] = 1.0;
    JointControl_to_pub->effort[2] = 1.0;
    JointControl_to_pub->effort[3] = 1.0;
    JointControl_to_pub->position[0] = 0.0;
    JointControl_to_pub->position[1] = -1.511;
    JointControl_to_pub->position[2] = -1.308;
    JointControl_to_pub->position[3] = 0.1;
    up_pub_->publish(*JointControl_to_pub);
    action_rate.sleep();
    RCLCPP_INFO(this->get_logger(), "Executing goal, id: %s", goal_handle->get_goal_id().data());

    while (rclcpp::ok()) {
        if(goal_handle->is_canceling())
        {
            JointControl_to_pub->velocity[0] = 0.0;
            JointControl_to_pub->effort[0] = 10.0;
            // arm_executor(JointControl_to_pub, JointState_, 0.0, 0.119, 0.527, -0.400);
            arm_executor(JointControl_to_pub, JointState_, 0.0, -1.511, -1.308, 0.1);
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
        JointState_.velocity.assign(up_joint_state.velocity.begin(), up_joint_state.velocity.end());
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
        {
            reset = 1;
            up_decision_making(ball_info_, purple_info_, is_found_, situation,0);
        }
        else
        {
            up_decision_making(ball_info_, purple_info_, is_found_, situation,reset);
            reset = 1;
        }
        RCLCPP_DEBUG(this->get_logger(), "Situation: %hhd",static_cast<int8_t>(situation));

        global_supervisor(tf_current_pose, ChassisPa);

        switch (state_) {
            case (APPROACHINGBALL::IDLE):
                state_ = (APPROACHINGBALL::LOOKING);
                break;
            case (APPROACHINGBALL::LOOKING):
            {
                static int stay_calm = 0;
                static rclcpp::Time state_entry_time;
                //  Enter State
                if(stay_calm == 0)
                {
                    state_entry_time = this->now();
                    // arm_executor(JointControl_to_pub, JointState_, 0.0, -1.511, -1.308, 0.1);
                    JointControl_to_pub->effort[0] = 1.0;
                    JointControl_to_pub->effort[1] = 1.0;
                    JointControl_to_pub->effort[2] = 1.0;
                    JointControl_to_pub->effort[3] = 1.0;
                    JointControl_to_pub->position[0] = 0.0;
                    JointControl_to_pub->position[1] = -1.511;
                    JointControl_to_pub->position[2] = -1.308;
                    JointControl_to_pub->position[3] = 0.12;
                    Data_To_Pub.linear.x = 0;
                    Data_To_Pub.linear.y = 0;
                    Data_To_Pub.angular.z = 0;
                    chassis_pub_->publish(Data_To_Pub);
                    up_pub_->publish(*JointControl_to_pub);
                    stay_calm = 1;
                    break;
                }
                //  Exit State
                if(is_found_)
                {
                    JointControl_to_pub->effort[0] = 1.0;
                    JointControl_to_pub->effort[1] = 1.0;
                    JointControl_to_pub->effort[2] = 1.0;
                    JointControl_to_pub->effort[3] = 1.0;
                    JointControl_to_pub->position[0] = 0.0;
                    JointControl_to_pub->position[1] = -1.511;
                    JointControl_to_pub->position[2] = -1.308;
                    JointControl_to_pub->position[3] = 0.12;
                    state_ = (APPROACHINGBALL::APPROACHING1);
                    stay_calm = 0;
                    break;
                }
                
                // Exit State
                if(this->now() - state_entry_time > rclcpp::Duration::from_seconds(0.5))
                {
                    stay_calm = 0;
                    reset = 0;
                    state_ = (APPROACHINGBALL::FINDING);
                    break;
                }
                
                // State Execution
                Data_To_Pub.linear.x = 0;
                Data_To_Pub.linear.y = 0;
                Data_To_Pub.angular.z = 0;
                chassis_pub_->publish(Data_To_Pub);
                break;
            }

            case (APPROACHINGBALL::APPROACHING1):
            {
                static int stay_calm = 0;
                static rclcpp::Time state_entry_time;
                static int not_found_ = 0;
                static int crash_time = 0;
                
                //  Enter State
                if(stay_calm == 0)
                {
                    state_entry_time = this->now();
                    stay_calm = 1;
                    crash_time = 0;
                }

                // Exit State
                if(tracking_ball.x  > x_approach)
                {
                    stay_calm = 0;
                    state_ = (APPROACHINGBALL::CATCHING);
                    crash_time = 0;
                }

                JointControl_to_pub->effort[0] = 10.0;
                JointControl_to_pub->effort[1] = 1.0;
                JointControl_to_pub->effort[2] = 1.0;
                JointControl_to_pub->effort[3] = 1.0;
                JointControl_to_pub->velocity[0] = - PIDController_PTZ.PosePID_Calc( tracking_ball.y - y_approach);
                Data_To_Pub.linear.x = - PIDController_x.PosePID_Calc(float(x_approach + 5 - tracking_ball.x))*std::cos(JointState_.position[0]);
                Data_To_Pub.linear.y = - PIDController_x.PosePID_Calc(float(x_approach + 5 - tracking_ball.x))*std::sin(JointState_.position[0]);
                Data_To_Pub.angular.z = 0;
                
                RCLCPP_DEBUG(this->get_logger(), "Data_To_Pub: %f, %f", Data_To_Pub.linear.x, Data_To_Pub.linear.y);

                // Eixt State
                if(fabs(JointState_.position[0]) >= 2.0)
                {
                    stay_calm = 0;
                    crash_time = 0;
                    JointControl_to_pub->effort[0] = 10.0;
                    JointControl_to_pub->velocity[0] = 0;
                    state_ = (APPROACHINGBALL::TOFORWARD);
                    chassis_pub_->publish(Data_To_Pub);
                    up_pub_->publish(*JointControl_to_pub);
                    break;
                }
                // Eixt State
                if(is_found_ == false)
                {
                    not_found_ ++;
                    if(not_found_ >= 20)
                    {
                        stay_calm = 0;
                        crash_time = 0; 
                        JointControl_to_pub->effort[0] = 10.0;
                        JointControl_to_pub->velocity[0] = 0;
                        Data_To_Pub.linear.x = 0;
                        Data_To_Pub.linear.y = 0;
                        Data_To_Pub.angular.z = 0;
                        state_ = (APPROACHINGBALL::LOOKING);
                        chassis_pub_->publish(Data_To_Pub);
                        up_pub_->publish(*JointControl_to_pub);
                        break;
                    }
                }else{
                    not_found_ = 0;
                }
                
                
                // Eixt State
                if(SupervisorState == action_findball::Status::CRASHED)
                {
                    stay_calm = 0;
                    JointControl_to_pub->effort[0] = 10.0;
                    JointControl_to_pub->velocity[0] = 0;
                    Data_To_Pub.linear.x = 0;
                    Data_To_Pub.linear.y = 0;
                    Data_To_Pub.angular.z = 0;
                    crash_time ++;
                    if(crash_time <=2)
                    {
                        state_ = APPROACHINGBALL::CRASH;
                    }
                    else{
                        state_ = (APPROACHINGBALL::FAIL);
                        crash_time = 0;
                    }
                    break;
                }

                // Exit State
                if(this->now() - state_entry_time > rclcpp::Duration::from_seconds(10))
                {
                    stay_calm = 0;
                    crash_time = 0;
                    JointControl_to_pub->effort[0] = 10.0;
                    JointControl_to_pub->velocity[0] = 0;
                    Data_To_Pub.linear.x = 0;
                    Data_To_Pub.linear.y = 0;
                    Data_To_Pub.angular.z = 0;
                    state_ = (APPROACHINGBALL::FAIL);
                    chassis_pub_->publish(Data_To_Pub);
                    up_pub_->publish(*JointControl_to_pub);
                    break;
                }
                
                // Exit State
                if((tracking_ball.x - x_approach) > 0 && fabs(tracking_ball.y - y_approach) < 40)
                {
                    double arm_position = JointState_.position[0];
                    static int count_brake = 0;
                    static double data_x_temp;
                    static double data_y_temp;
                    if(count_brake <= 2)
                    {
                        if(count_brake == 0)
                        {
                            // data_x_temp = 1.0*std::cos(arm_position);
                            // data_y_temp = -1.0*std::sin(arm_position);
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
                        JointControl_to_pub->velocity[0] = 0.0;
                        JointControl_to_pub->position[1] = -1.138;
                        JointControl_to_pub->position[2] = -1.138;
                        JointControl_to_pub->position[3] = 0.15;
                        //arm_executor(JointControl_to_pub, JointState_, 0.0, -1.138, -1.308, 0.1);
                        stay_calm = 0;
                        crash_time = 0;
                        state_ = (APPROACHINGBALL::CATCHING);

                        RCLCPP_DEBUG(this->get_logger(), "目标球");
                    }
                    RCLCPP_DEBUG(this->get_logger(), "Ball approached");
                }

                chassis_pub_->publish(Data_To_Pub);
                up_pub_->publish(*JointControl_to_pub);
                break;
            }

            case (APPROACHINGBALL::CRASH):
            {
                static int stay_calm = 0;
                static rclcpp::Time state_entry_time;
                static double yaw;
                rclcpp::Duration time_allowance = rclcpp::Duration::from_seconds(10);
                //  Enter State
                if(stay_calm == 0)
                {
                    state_entry_time = this->now();
                    yaw = tf2::getYaw(tf_current_pose.pose.orientation);
                    if(start_side == "left")  spin_to_func->onRun(-yaw - 1.57, time_allowance, tf_current_pose);
                    if(start_side == "right")  spin_to_func->onRun(1.57-yaw, time_allowance, tf_current_pose);
                    stay_calm = 1;
                }
                if(spin_to_func->onCycleUpdate(tf_current_pose) == Status::SUCCEEDED)
                {
                    stay_calm = 0;
                    state_ = (APPROACHINGBALL::CATCHING);
                }
                if(this->now() - state_entry_time > rclcpp::Duration::from_seconds(11))
                {
                    stay_calm = 0;
                    state_ = (APPROACHINGBALL::FAIL);
                }

                break;
            }

            case (APPROACHINGBALL::TOFORWARD):
            {
                geometry_msgs::msg::Point32 tracking_ball__;
                static double angle_sign = 0;
                static double angle_diff = 0;
                static int stay_calm = 0;
                static rclcpp::Time state_entry_time;  
                static Status spin_return;
                static int skip_count = 0;
                rclcpp::Duration time_allowance = rclcpp::Duration::from_seconds(10);
                //  Enter State
                if(stay_calm == 0)
                {
                    state_entry_time = this->now();
                    angle_sign = JointState_.position[0]>0 ? 1.0: -1.0;
                    stay_calm = 1;
                    angle_diff = JointState_.position[0];
                    spin_to_func->onRun(angle_diff, time_allowance, tf_current_pose);
                }

                tracking_ball__ = tracking_ball;
                if(spin_return != Status::SUCCEEDED)
                {
                    RCLCPP_DEBUG(this->get_logger(), "onCycleUpdate");
                    spin_return = spin_to_func->onCycleUpdate(tf_current_pose);
                }

                // Eixt State
                if(spin_return == Status::SUCCEEDED)
                {
                    RCLCPP_DEBUG(this->get_logger(), "spin_return == Status::SUCCEEDED");
                    if(fabs(JointState_.position[0]) >= 1.4)
                    {
                        // test  
                        JointControl_to_pub->effort[0] = 10.0;
                        JointControl_to_pub->velocity[0] = (JointState_.position[0] > 0 ? -1.0 : 1.0) * 8;

                    }
                    else{
                        JointControl_to_pub->effort[0] = 1.0;
                        JointControl_to_pub->position[0] = 0;
                        JointControl_to_pub->velocity[0] = 0;
                    }
                    Data_To_Pub.linear.x = 0;
                    Data_To_Pub.linear.y = 0;
                    Data_To_Pub.angular.z = 0;
                    chassis_pub_->publish(Data_To_Pub);
                    up_pub_->publish(*JointControl_to_pub);
                    RCLCPP_DEBUG(this->get_logger(), "State:2, %f", JointState_.position[0]);
                    if(fabs(JointState_.position[0]) < 0.05)
                    {
                        if(skip_count == 0)
                        {
                            reset = 0;
                        }
                        if(skip_count >= 6)
                        {
                            spin_return = Status::IDLE;
                            stay_calm = 0;
                            angle_sign = 0;
                            skip_count = 0;
                            state_ = (APPROACHINGBALL::APPROACHING1);
                            break;
                        }
                        skip_count ++;
                        break;
                    }
                }else if(spin_return == Status::FAILED)
                {
                    RCLCPP_DEBUG(this->get_logger(), "spin_return == Status::FAILED");   
                    JointControl_to_pub->position[0] = 10.0;
                    JointControl_to_pub->position[0] = 0;
                    JointControl_to_pub->velocity[0] = 0;
                    Data_To_Pub.linear.x = 0;
                    Data_To_Pub.linear.y = 0;
                    Data_To_Pub.angular.z = 0;
                    stay_calm = 0;
                    angle_sign = 0;
                    spin_return = Status::IDLE;
                    reset = 0;
                    chassis_pub_->publish(Data_To_Pub);
                    up_pub_->publish(*JointControl_to_pub);
                    state_ = (APPROACHINGBALL::FAIL);
                    break;
                }

                // Exit State
                if(this->now() - state_entry_time > rclcpp::Duration::from_seconds(12))
                {
                    stay_calm = 0;
                    angle_sign = 0;
                    JointControl_to_pub->effort[0] = 10.0;
                    JointControl_to_pub->velocity[0] = 0;
                    Data_To_Pub.linear.x = 0;
                    Data_To_Pub.linear.y = 0;
                    Data_To_Pub.angular.z = 0;
                    spin_return = Status::IDLE;
                    state_ = (APPROACHINGBALL::FAIL);
                    chassis_pub_->publish(Data_To_Pub);
                    up_pub_->publish(*JointControl_to_pub);
                    break;
                }
                break;
            }
    
            case (APPROACHINGBALL::CATCHING):
            {
                static int stay_calm = 0;
                static rclcpp::Time state_entry_time;  
                static int crash_time = 0;
                static int count__ = 0;
                //  Enter State
                if(stay_calm == 0)
                {
                    state_entry_time = this->now();
                    PIDController_x_catch.clear();
                    stay_calm = 1;
                }
                //RCLCPP_DEBUG(this->get_logger(), "checkpoint0");

                JointControl_to_pub->effort[0] = 10.0;
                JointControl_to_pub->effort[1] = 1.0;
                JointControl_to_pub->effort[2] = 1.0;
                JointControl_to_pub->effort[3] = 1.0;
                JointControl_to_pub->velocity[0] = - PIDController_PTZ.PosePID_Calc( tracking_ball.y - y_catch);
                // RCLCPP_DEBUG(this->get_logger(), "checkpoint1");
                Data_To_Pub.linear.x = - PIDController_x_catch.PosePID_Calc(float(x_catch - tracking_ball.x))*std::cos(JointState_.position[0]);
                Data_To_Pub.linear.y = - PIDController_x_catch.PosePID_Calc(float(x_catch - tracking_ball.x))*std::sin(JointState_.position[0]);
                Data_To_Pub.angular.z = 0;
                //RCLCPP_DEBUG(this->get_logger(), "checkpoint2");
                // 限制角度，如果角度大于114度，速度为0
                // Exit State
                if(fabs(JointState_.position[0]) >= 2.0)
                {
                    stay_calm = 0;
                    crash_time= 0;
                    count__ = 0;
                    JointControl_to_pub->velocity[0] = 0;
                    state_ = (APPROACHINGBALL::TOFORWARD);
                }
                //RCLCPP_DEBUG(this->get_logger(), "checkpoint3");
                // Exit State
                if(this->now() - state_entry_time > rclcpp::Duration::from_seconds(10))
                {
                    stay_calm = 0;
                    crash_time = 0;
                    count__ = 0;
                    JointControl_to_pub->effort[0] = 10.0;
                    JointControl_to_pub->velocity[0] = 0;
                    Data_To_Pub.linear.x = 0;
                    Data_To_Pub.linear.y = 0;
                    Data_To_Pub.angular.z = 0;
                    state_ = (APPROACHINGBALL::FAIL);
                }
                //RCLCPP_DEBUG(this->get_logger(), "checkpoint4");
                // Exit State
                if((fabs(Data_To_Pub.linear.x)>0.2 || fabs(Data_To_Pub.linear.y)>0.2)&& (fabs(current_pose.twist.twist.linear.x) < 0.2 || fabs(current_pose.twist.twist.linear.x) < 0.2))
                {
                    static rclcpp::Time state_entry_time__;
                    if(count__ == 0)
                    {
                        state_entry_time__ = this->now();
                        count__ = 0;
                    }
                    if(this->now() - state_entry_time__ > rclcpp::Duration::from_seconds(2))
                    {
                        stay_calm = 0;
                        crash_time = 0;
                        JointControl_to_pub->effort[0] = 10.0;
                        JointControl_to_pub->velocity[0] = 0;
                        Data_To_Pub.linear.x = 0;
                        Data_To_Pub.linear.y = 0;
                        Data_To_Pub.angular.z = 0;
                        state_ = APPROACHINGBALL::CRASH;
                        count__ = 0;
                    }
                }

                // Eixt State
                if(SupervisorState == action_findball::Status::CRASHED)
                {
                    stay_calm = 0;
                    JointControl_to_pub->effort[0] = 10.0;
                    JointControl_to_pub->velocity[0] = 0;
                    Data_To_Pub.linear.x = 0;
                    Data_To_Pub.linear.y = 0;
                    Data_To_Pub.angular.z = 0;
                    crash_time ++;
                    if(crash_time <=2)
                    {
                        count__ = 0;
                        state_ = APPROACHINGBALL::CRASH;
                    }
                    else{
                        count__ = 0;
                        state_ = (APPROACHINGBALL::FAIL);
                        crash_time = 0;
                    }
                    break;
                }

                if(!is_found_)
                {
                    stay_calm = 0;
                    crash_time = 0;
                    count__ = 0;
                    JointControl_to_pub->effort[0] = 10.0;
                    JointControl_to_pub->velocity[0] = 0;
                    Data_To_Pub.linear.x = 0;
                    Data_To_Pub.linear.y = 0;
                    Data_To_Pub.angular.z = 0;
                    state_ = (APPROACHINGBALL::LOOKING);
                }

                // Eixt State
                static int count = 0;
                if((fabs(tracking_ball.x - x_catch) < 30 || tracking_ball.x -x_catch > 0) && fabs(tracking_ball.y - y_catch) < 30)
                {
                    count ++;
                    if(count >= 5)
                    {
                        stay_calm = 0;
                        crash_time = 0;
                        count = 0;
                        count__ = 0;
                        JointControl_to_pub->effort[0] = 10.0;
                        JointControl_to_pub->velocity[0] = 0;
                        /***----- ----- ***/
                        // arm_executor(JointControl_to_pub, JointState_, JointState_.position[0], -0.05, -0.391, 0.1);
                        arm_executor(JointControl_to_pub, JointState_, JointState_.position[0], -0.2, 0.0, 0.05);
                        Data_To_Pub.linear.x = 0;
                        Data_To_Pub.linear.y = 0;
                        Data_To_Pub.angular.z = 0;
                        chassis_pub_->publish(Data_To_Pub);
                        //up_pub_->publish(*JointControl_to_pub);
                        state_ = (APPROACHINGBALL::CATCHING_2);
                    }
                }else{
                    count = 0;
                }
                RCLCPP_DEBUG(this->get_logger(), "x_diff: %f, y_diff: %f",fabs(tracking_ball.x - x_catch),fabs(tracking_ball.y - y_catch));
                //RCLCPP_DEBUG(this->get_logger(), "checkpoint6");
                chassis_pub_->publish(Data_To_Pub);
                up_pub_->publish(*JointControl_to_pub);
                //RCLCPP_DEBUG(this->get_logger(), "checkpoint7");
                break;
            }

            case(APPROACHINGBALL::CATCHING_2):
            {
                static int action_step = 0;
                switch (action_step) {
                    // case 0:
                    // {
                    //     // -0.2, 0.12, 0.05
                    //     JointControl_to_pub->effort[0] = 10.0;
                    //     JointControl_to_pub->effort[1] = 1.0;
                    //     JointControl_to_pub->effort[2] = 1.0;
                    //     JointControl_to_pub->effort[3] = 1.0;
                    //     JointControl_to_pub->velocity[0] = 0.0;
                    //     JointControl_to_pub->position[1] = -0.1;
                    //     JointControl_to_pub->position[2] = 0.2;
                    //     JointControl_to_pub->position[3] = 0.12;
                    //     up_pub_->publish(*JointControl_to_pub);
                    //     if(fabs(JointState_.position[1] + 0.2) <= 0.1)
                    //         action_step = 1;
                    //     if(JointState_.velocity[0] >=5)
                    //     {
                    //         loop_rate.sleep();
                    //         JointControl_to_pub->position[3] = -0.2;
                    //         up_pub_->publish(*JointControl_to_pub);
                    //         action_step = 3;
                    //         action_rate.sleep();
                    //         action_rate.sleep();
                    //     }
                    //     break;
                    // }

                    case 0:
                    {
                        action_rate.sleep();
                        JointControl_to_pub->effort[0] = 10.0;
                        JointControl_to_pub->effort[1] = 1.0;
                        JointControl_to_pub->effort[2] = 1.0;
                        JointControl_to_pub->effort[3] = 1.0;
                        JointControl_to_pub->velocity[0] = 0.0;
                        JointControl_to_pub->position[1] = -0.15;
                        JointControl_to_pub->position[2] = -0.8;
                        JointControl_to_pub->position[3] = 0.1;
                        up_pub_->publish(*JointControl_to_pub);
                        action_rate.sleep();
                        action_step = 1;
                        break;
                    }
                    case 1:
                    {
                        JointControl_to_pub->effort[0] = 10.0;
                        JointControl_to_pub->effort[1] = 1.0;
                        JointControl_to_pub->effort[2] = 1.0;
                        JointControl_to_pub->effort[3] = 1.0;
                        JointControl_to_pub->velocity[0] = 0.0;
                        JointControl_to_pub->position[3] = -0.2;
                        up_pub_->publish(*JointControl_to_pub);
                        action_step = 2;
                        break;
                    }
                    case 2:
                    {
                        static int count_for_this_state = 0;
                        static int count_for_result = 0;
                        if(count_for_result < 3)
                        {
                            loop_rate.sleep();
                            if(JointState_.velocity[0] >= 5.0)
                            {
                                JointControl_to_pub->position[3] = -0.2;
                                up_pub_->publish(*JointControl_to_pub);
                                loop_rate.sleep();
                                JointControl_to_pub->position[3] = -0.2;
                                up_pub_->publish(*JointControl_to_pub);
                                count_for_result = 0;
                                action_step = 0;
                                state_ = (APPROACHINGBALL::SUCCEED);
                                break;
                            }
                            JointControl_to_pub->effort[0] = 10.0;
                            JointControl_to_pub->effort[1] = 1.0;
                            JointControl_to_pub->effort[2] = 1.0;
                            JointControl_to_pub->effort[3] = 1.0;
                            JointControl_to_pub->velocity[0] = 0.0;
                            JointControl_to_pub->position[1] = -0.05;
                            JointControl_to_pub->position[2] = -1.0;
                            JointControl_to_pub->position[3] = 0.12;
                            up_pub_->publish(*JointControl_to_pub);
                            RCLCPP_DEBUG(this->get_logger(),"count :%d", count_for_result);
                        }
                        else if(count_for_result >= 3)
                        {
                            count_for_this_state ++;
                            count_for_result = 0;
                            action_step = 0;
                            reset = 0;
                            JointControl_to_pub->effort[0] = 10.0;
                            JointControl_to_pub->effort[1] = 1.0;
                            JointControl_to_pub->effort[2] = 1.0;
                            JointControl_to_pub->effort[3] = 1.0;
                            JointControl_to_pub->velocity[0] = 0.0;
                            JointControl_to_pub->position[1] = -1.138;
                            JointControl_to_pub->position[2] = -1.138;
                            JointControl_to_pub->position[3] = 0.15;
                            up_pub_->publish(*JointControl_to_pub);
                            state_ = APPROACHINGBALL::CATCHING;
                            // reset = 0;
                            // action_rate.sleep();
                            if(count_for_this_state >=3)
                            {
                                count_for_this_state = 0;
                                action_rate.sleep();
                            }
                            RCLCPP_DEBUG(this->get_logger(),"else count :%d", count_for_result);
                        }
                        count_for_result ++;
                        break;
                    }
                }
                break;
            }
            case (APPROACHINGBALL::CATCHING_3):
            {
                static int action_step = 0;
                switch (action_step) {
                    case 0:
                    {
                        PTZ_executor(JointControl_to_pub, JointState_, 1.57);
                        action_rate.sleep();
                        action_step = 1;
                        break;
                    }
                    case 1:
                    {
                        JointControl_to_pub->position[3] = 0.1;
                        up_pub_->publish(*JointControl_to_pub);
                        action_rate.sleep();
                        action_step = 2;
                        break;
                    }
                    case 2:
                    {
                        JointControl_to_pub->effort[0] = 1.0;
                        JointControl_to_pub->effort[1] = 1.0;
                        JointControl_to_pub->effort[2] = 1.0;
                        JointControl_to_pub->effort[3] = 1.0;
                        JointControl_to_pub->position[0] = 0.0;
                        JointControl_to_pub->position[1] = -1.138;
                        JointControl_to_pub->position[2] = -1.138;
                        JointControl_to_pub->position[3] = 0.05;
                        Data_To_Pub.linear.x = 0;
                        Data_To_Pub.linear.y = 0;
                        Data_To_Pub.angular.z =  0;
                        action_rate.sleep();
                        action_rate.sleep();
                        reset = 0;
                        up_pub_->publish(*JointControl_to_pub);
                        chassis_pub_->publish(Data_To_Pub);
                        action_step = 0;
                        state_ = (APPROACHINGBALL::CATCHING);
                        break;
                    }

                }
                break;
            }

            case(APPROACHINGBALL::NO_BALL):
            {
                state_ = (APPROACHINGBALL::FAIL);
                break;
            }

            case(APPROACHINGBALL::BACKWARD):
            {
                JointControl_to_pub->effort[0] = 10.0;
                JointControl_to_pub->effort[1] = 1.0;
                JointControl_to_pub->effort[2] = 1.0;
                JointControl_to_pub->effort[3] = 1.0;
                JointControl_to_pub->velocity[0] = - PIDController_PTZ.PosePID_Calc( tracking_ball.y - y_approach);
                Data_To_Pub.linear.x = 0;
                Data_To_Pub.linear.y = 0;
                Data_To_Pub.angular.z = 0;
                RCLCPP_DEBUG(this->get_logger(), "Waiting..... for global planner to complete the path");
                break;
            }

            case (APPROACHINGBALL::FINDING):
            {
                static int state_angle_direction = 0;
                static rclcpp::Time time_execute;
                double y_position;
                rclcpp::Duration command_time_allowance_{0, 0};
                
                if(fabs(tf_current_pose.pose.position.y) < 3.5) y_position = 0.7; else y_position = 1.52;

                if(toward == TOWARD::FRONT)
                {
                    if(left_ball_count != 0 && left_ball_count >= right_ball_count)
                    {
                        state_angle_direction = 0;
                    }else if(right_ball_count != 0 && left_ball_count < right_ball_count)
                    {
                        state_angle_direction = 2;
                    }
                }
                //向左旋转云台 45/90 度
                if(state_angle_direction == 0)
                {
                    left_ball_count = ball_info_.size();
                    JointControl_to_pub->effort[0] = 1.0;
                    JointControl_to_pub->position[0] = -y_position;
                    Data_To_Pub.linear.x = 0.0;
                    Data_To_Pub.linear.y = 0.0;
                    Data_To_Pub.angular.z = 0.0;
                    chassis_pub_->publish(Data_To_Pub);
                    up_pub_->publish(*JointControl_to_pub);
                    RCLCPP_DEBUG(this->get_logger(), "state:Finding, position[0]:%f", JointControl_to_pub->position[0]);
                    static int count_ = 0;
                    if(count_ == 0)
                    {
                        action_rate.sleep();
                        time_execute = this->now();
                        reset = 0;
                        count_ = 1;
                    }
                    
                    if(is_found_){
                        left_ball_count -- ;
                        if(left_ball_count < 0) left_ball_count = 0;
                        //state_ = (APPROACHINGBALL::TOFORWARD);
                        state_ = (APPROACHINGBALL::APPROACHING1);
                        state_angle_direction = 0;
                        count_ = 0;
                    }
                    command_time_allowance_ = this->now() - time_execute;
                    if(command_time_allowance_.seconds() > 2){
                        state_angle_direction = 1;
                        left_ball_count = 0;
                        count_ = 0;
                    }
                }else if(state_angle_direction == 1)
                {
                    RCLCPP_DEBUG(this->get_logger(), "hui zhong");
                    JointControl_to_pub->effort[0] = 1.0;
                    JointControl_to_pub->position[0] = 0.0;
                    Data_To_Pub.linear.x = 0.0;
                    Data_To_Pub.linear.y = 0.0;
                    Data_To_Pub.angular.z = 0.0;
                    chassis_pub_->publish(Data_To_Pub);
                    up_pub_->publish(*JointControl_to_pub);
                    if(fabs(JointState_.position[0]) <0.2)
                        state_angle_direction = 2;
                }

                //向右旋转云台 45/90 度
                else if(state_angle_direction == 2)
                {
                    if(toward == TOWARD::FRONT)
                        right_ball_count = ball_info_.size();
                    static int count_ = 0;
                    JointControl_to_pub->effort[0] = 1.0;
                    JointControl_to_pub->position[0] = y_position;
                    Data_To_Pub.linear.x = 0.0;
                    Data_To_Pub.linear.y = 0.0;
                    Data_To_Pub.angular.z = 0.0;
                    chassis_pub_->publish(Data_To_Pub);
                    up_pub_->publish(*JointControl_to_pub);
                    RCLCPP_DEBUG(this->get_logger(), "state:Finding, position[0]:%f", JointControl_to_pub->position[0]);

                    if(count_ == 0)
                    {
                        action_rate.sleep();
                        time_execute = this->now();
                        reset = 0;
                        count_ = 1;
                    }

                    if(is_found_) {
                        count_ = 0;
                        state_angle_direction = 0;
                        if(toward == TOWARD::FRONT)
                        {
                            right_ball_count --;
                            if(right_ball_count < 0) right_ball_count = 0;
                        }
                        // state_ = (APPROACHINGBALL::TOFORWARD);
                        state_ = (APPROACHINGBALL::APPROACHING1);
                    }
                    command_time_allowance_ = this->now() - time_execute;
                    if(command_time_allowance_.seconds() > 2){
                        if(toward == TOWARD::FRONT) right_ball_count = 0;
                        state_angle_direction = -1;
                        count_ = 0;
                    }
                }
                //再不行转为NO_BALL
                else if(state_angle_direction == -1)
                {
                    Data_To_Pub.linear.x = 0.0;
                    Data_To_Pub.linear.y = 0.0;
                    Data_To_Pub.angular.z = 0.0;
                    chassis_pub_->publish(Data_To_Pub);
                    up_pub_->publish(*JointControl_to_pub);
                    state_angle_direction = 0;
                    state_ =  APPROACHINGBALL::FAIL;
                }
                
                RCLCPP_DEBUG(this->get_logger(), "state:Finding, %d", state_angle_direction);
                break;
            }
        }

        if(state_ == (APPROACHINGBALL::SUCCEED))
        {
            JointControl_to_pub->velocity[0] = 0.0;
            //JointControl_to_pub->effort[0] = 10.0;
            //arm_executor(JointControl_to_pub, JointState_, 0.0, 0.119, 0.527, -0.400);
            Data_To_Pub.linear.x = 0;
            Data_To_Pub.linear.y = 0;
            Data_To_Pub.angular.z = 0;
            chassis_pub_->publish(Data_To_Pub);
            up_pub_->publish(*JointControl_to_pub);
            state_ = (APPROACHINGBALL::IDLE);
            break;
        }
        if(state_ == (APPROACHINGBALL::FAIL))
        {
            JointControl_to_pub->velocity[0] = 0.0;
            //JointControl_to_pub->effort[0] = 1.0;
            //arm_executor(JointControl_to_pub, JointState_, 0.0, 0.119, 0.527, -0.400);
            Data_To_Pub.linear.x = 0;
            Data_To_Pub.linear.y = 0;
            Data_To_Pub.angular.z = 0;
            geometry_msgs::msg::PoseStamped data;
            data.pose.position.x = 10.05;
            //******* 待增加对面场次的判断 ********
            if(start_side == "left")
            {
                data.pose.position.y = 3.9;
                data.pose.orientation.z = -0.707;
                data.pose.orientation.w = 0.707;
            }
            else if(start_side == "right")
            {
                data.pose.position.y = -3.9;
                data.pose.orientation.z = 0.707;
                data.pose.orientation.w = 0.707;
            }
            //******* ----------------- ********
            data.header.frame_id = "map";
            data.header.stamp = this->now();
            data.pose.position.z = 0.0;
            data.pose.orientation.x = 0.0;
            data.pose.orientation.y = 0.0;

            goal_update_->publish(data);
            toward = TOWARD::FRONT;
            chassis_pub_->publish(Data_To_Pub);
            state_ = (APPROACHINGBALL::IDLE);
            result->state = -1;
            goal_handle->abort(result);
            return;
        }
        RCLCPP_DEBUG(this->get_logger(), "state: %d", static_cast<int>(state_));
        RCLCPP_DEBUG(this->get_logger(), "left: %d,right: %d", left_ball_count, right_ball_count);
        state_last = state_;
        tracking_ball_last = tracking_ball;
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
    }

    if(rclcpp::ok())
    {
        result->state = 0;
        /*
        if(left_ball_count != 0 && left_ball_count >= right_ball_count)
        {
            geometry_msgs::msg::PoseStamped data;
            data.pose.position.x = 10.05;
            //******* 待增加对面场次的判断 *********
            if(start_side == "left")
            {
                data.header.frame_id = "map";
                data.header.stamp = this->now();
                data.pose.position.y = 3.85;
                data.pose.position.z = 0.0;
                data.pose.orientation.x = 0.0;
                data.pose.orientation.y = 0.0;
                data.pose.orientation.z = 0.0;
                data.pose.orientation.w = 1.0;
            }
            else if(start_side == "right")
            {
                data.header.frame_id = "map";
                data.header.stamp = this->now();
                data.pose.position.y = -3.85;
                data.pose.position.z = 0.0;
                data.pose.orientation.x = 0.0;
                data.pose.orientation.y = 0.0;
                data.pose.orientation.z = -1.0;
                data.pose.orientation.w = 0.0;
            }
            //******* ----------------- *********
            
            goal_update_->publish(data);
            RCLCPP_INFO(this->get_logger(), "Toward: Left");
            toward = TOWARD::LEFT;
            left_ball_count --;
            if(left_ball_count < 0) left_ball_count = 0;
        }else if(right_ball_count != 0 && left_ball_count < right_ball_count)
        {
            geometry_msgs::msg::PoseStamped data;
            data.pose.position.x = 10.05;
            //******* 待增加对面场次的判断 *********
            if(start_side == "left")
            {
                data.header.frame_id = "map";
                data.header.stamp = this->now();
                data.pose.position.y = 3.85;
                data.pose.position.z = 0.0;
                data.pose.orientation.x = 0.0;
                data.pose.orientation.y = 0.0;
                data.pose.orientation.z = -1.0;
                data.pose.orientation.w = 0.0;
            }
            else if(start_side == "right")
            {
                data.header.frame_id = "map";
                data.header.stamp = this->now();
                data.pose.position.y = -3.85;
                data.pose.position.z = 0.0;
                data.pose.orientation.x = 0.0;
                data.pose.orientation.y = 0.0;
                data.pose.orientation.z = 0.0;
                data.pose.orientation.w = 1.0;
            }
            //******* ----------------- *********
            
            goal_update_->publish(data);
            right_ball_count --;
            if(right_ball_count <0 ) right_ball_count = 0;
            toward = TOWARD::RIGHT;
            RCLCPP_INFO(this->get_logger(), "Toward: Right");
        }else {
            geometry_msgs::msg::PoseStamped data;
            data.pose.position.x = 10.05;
            //******* 待增加对面场次的判断 ********
            if(start_side == "left")
                data.pose.position.y = 2.1;
            else if(start_side == "right")
                data.pose.position.y = -2.1;
            //******* ----------------- ********
            data.header.frame_id = "map";
            data.header.stamp = this->now();
            data.pose.position.z = 0.0;
            data.pose.orientation.x = 0.0;
            data.pose.orientation.y = 0.0;
            data.pose.orientation.z = -0.707;
            data.pose.orientation.w = 0.707;
            goal_update_->publish(data);
            toward = TOWARD::FRONT;
            RCLCPP_INFO(this->get_logger(), "Toward: Front");
        }
        */
        geometry_msgs::msg::PoseStamped data;
        data.pose.position.x = 10.05;
        //******* 待增加对面场次的判断 ********
        if(start_side == "left")
        {
            data.pose.position.y = 3.85;
            data.pose.orientation.z = -0.707;
            data.pose.orientation.w = 0.707;
        }
        else if(start_side == "right")
        {
            data.pose.position.y = 3.85;
            data.pose.orientation.z = 0.707;
            data.pose.orientation.w = 0.707;
        }
        //******* ----------------- ********
        data.header.frame_id = "map";
        data.header.stamp = this->now();
        data.pose.position.z = 0.0;
        data.pose.orientation.x = 0.0;
        data.pose.orientation.y = 0.0;

        goal_update_->publish(data);
        toward = TOWARD::FRONT;
        RCLCPP_INFO(this->get_logger(), "Toward: Front");
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        // if(!change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to deactive node %s", lifecycle_node);
        // }
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
            if((most_near.x-tracking_ball.x)*(most_near.x-tracking_ball.x) + (most_near.y-tracking_ball.y)*(most_near.y-tracking_ball.y) < 100)
                init = 1;
        }else {
            if((most_near.x-tracking_ball.x)*(most_near.x-tracking_ball.x) + (most_near.y-tracking_ball.y)*(most_near.y-tracking_ball.y) < 1600 + count*2)
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
                        if((target_ball.x-tracking_ball.y)*(target_ball.x-tracking_ball.y) + (target_ball.y-tracking_ball.x)*(target_ball.y-tracking_ball.x) < 1600)
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

        if(tracking_ball.x > 600 || tracking_ball.y >800)   init = 0;
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

     // std::cout << "target x: " << tracking_ball.x << " y: " << tracking_ball.y << std::endl;
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
    JointControl_to_pub->effort[0] = 1.0;
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
    rclcpp::Rate loop_rate(10);
    rclcpp::Rate action_rate(2);
    
    int step = (fabs(JointState_.position[1]-joint2)/3.1415926*30.0 >= 1 ? fabs(JointState_.position[1]-joint2)/3.1415926*20:1.0);
    RCLCPP_INFO(this->get_logger(), "step: %d", step);
    RCLCPP_INFO(this->get_logger(), "JointState_current_pos: %f, %f, %f", JointState_.position[0], JointState_.position[1], JointState_.position[2]);
    RCLCPP_INFO(this->get_logger(), "JointState_current_dif: %f, %f, %f", joint1 - JointState_.position[0], joint2 - JointState_.position[1], joint3 - JointState_.position[2]);
    if(JointControl_to_pub->effort[0] >5.0) step =1;
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
    return true;
}

void action_findball::ApproachingBall::global_supervisor(const geometry_msgs::msg::PoseStamped &tf_current_pose_,const nav_msgs::msg::Odometry &ChassisPa_)
{
    // 电子围栏判断
    if(tf_current_pose_.pose.position.x + car_length  < area3.lleft_up.x && tf_current_pose_.pose.position.x - car_length > area3.lright_down.x &&
        fabs(tf_current_pose_.pose.position.y) + car_length < area3.lleft_up.y)
    {
        //std::cout << "In area3" << std::endl;
        SupervisorState = action_findball::Status::RUNNING;
    }else {
        std::cerr << "Out of area3" << std::endl;
        SupervisorState = action_findball::Status::CRASHED;
    }
}

bool action_findball::ApproachingBall::check_position(const geometry_msgs::msg::PoseStamped &tf_current_pose_,float x,float y, float w)
{
    if(fabs(tf_current_pose_.pose.position.x -x) < 0.6 &&
        fabs(tf_current_pose_.pose.position.y -y) < 0.6 &&
        fabs(tf2::getYaw(tf_current_pose_.pose.orientation) - w) < 0.2)
    {
        return true;
    }else {
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


/*

static rclcpp::Time state_end_time;  
                static double arm_angle = 0;
                static geometry_msgs::msg::PoseStamped goal_pose_temp;
                static double chassis_yaw;
                static int bstep = -1;
                // static int bdirection = 0;
                static rclcpp::Duration time_allowance(8, 0);
                
                //  Enter State
                if(stay_calm == 0)
                {
                    state_end_time = this->now()+std::chrono::seconds(15);
                    arm_angle = JointState_.position[0];
                    goal_pose_temp = tf_current_pose;
                    chassis_yaw = tf2::getYaw(goal_pose_temp.pose.orientation);
                    // if(M_PI + chassis_yaw + arm_angle < M_PI/2.0) bdirection = 1;  else bdirection = -1;
                    JointControl_to_pub->effort[0] = 10.0;
                    JointControl_to_pub->velocity[0] = 0;
                    Data_To_Pub.linear.x = 0;
                    Data_To_Pub.linear.y = 0;
                    Data_To_Pub.angular.z = 0;
                    up_pub_->publish(*JointControl_to_pub);
                    chassis_pub_->publish(Data_To_Pub);
                    stay_calm = 1;
                }

                // Exit State
                if(fabs(JointState_.position[0]) >= 2.0)
                {
                    stay_calm = 0;
                    bstep = -1;
                    JointControl_to_pub->velocity[0] = 0;
                    state_ = (APPROACHINGBALL::CATCHING);
                }
                
                // Exit State
                if(this->now()>state_end_time)
                {
                    stay_calm = 0;
                    bstep = -1;
                    JointControl_to_pub->effort[0] = 10.0;
                    JointControl_to_pub->velocity[0] = 0;
                    Data_To_Pub.linear.x = 0;
                    Data_To_Pub.linear.y = 0;
                    Data_To_Pub.angular.z = 0;
                    up_pub_->publish(*JointControl_to_pub);
                    chassis_pub_->publish(Data_To_Pub);
                    state_ = (APPROACHINGBALL::FAIL);
                }

                if(bstep == -1)
                {
                    JointControl_to_pub->effort[0] = 10.0;
                    JointControl_to_pub->velocity[0] = 0;
                    Data_To_Pub.linear.x = 1.0*std::cos(arm_angle);
                    Data_To_Pub.linear.y = -1.0*std::sin(arm_angle);
                    Data_To_Pub.angular.z = 0;
                    chassis_pub_->publish(Data_To_Pub);
                    up_pub_->publish(*JointControl_to_pub);
                    
                    if(this->now() - (state_end_time - std::chrono::seconds(15)) > _back_time_allowance)
                    {
                        bstep = 0;
                        Data_To_Pub.linear.x = 0;
                        Data_To_Pub.linear.y = 0;
                        Data_To_Pub.angular.z = 0;
                        chassis_pub_->publish(Data_To_Pub);
                        up_pub_->publish(*JointControl_to_pub);
                    }
                    // _back_time_allowance
                    break;
                }

                if(bstep == 0)
                {
                    JointControl_to_pub->velocity[0] = - PIDController_PTZ.PosePID_Calc( tracking_ball.y - y_approach);
                    up_pub_->publish(*JointControl_to_pub);
                    RCLCPP_INFO(this->get_logger(), "start_side: %s", start_side.c_str());
                    if(start_side == "left") spin_to_func->onRun(-1.57, time_allowance, tf_current_pose);
                    if(start_side == "right") spin_to_func->onRun(1.57, time_allowance, tf_current_pose);
                    spin_to_func->onRun(-1.57, time_allowance, tf_current_pose);
                    bstep = 1;
                }else if(bstep == 1){
                    JointControl_to_pub->velocity[0] = - PIDController_PTZ.PosePID_Calc( tracking_ball.y - y_approach);
                    up_pub_->publish(*JointControl_to_pub);
                    action_findball::Status spin_return = spin_to_func->onCycleUpdate(tf_current_pose);
                    if(spin_return == Status::SUCCEEDED)  bstep = 2;
                }else if(bstep == 2)
                {
                    JointControl_to_pub->velocity[0] = - PIDController_PTZ.PosePID_Calc( tracking_ball.y - y_approach);
                    Data_To_Pub.linear.x = 0;
                    Data_To_Pub.linear.y = - PIDController_y.PosePID_Calc(y_approach - tracking_ball.y);
                    Data_To_Pub.angular.z = 0;
                    if(fabs(y_approach - tracking_ball.y) < 10)  bstep = 3;

                    up_pub_->publish(*JointControl_to_pub);
                    chassis_pub_->publish(Data_To_Pub);
                }else if(bstep == 3)
                {
                    stay_calm = 0;
                    bstep = -1;
                    JointControl_to_pub->effort[0] = 10.0;
                    JointControl_to_pub->velocity[0] = 0;
                    Data_To_Pub.linear.x = 0;
                    Data_To_Pub.linear.y = 0;
                    Data_To_Pub.angular.z = 0;
                    up_pub_->publish(*JointControl_to_pub);
                    chassis_pub_->publish(Data_To_Pub);
                    state_ = APPROACHINGBALL::CATCHING;
                }


*/