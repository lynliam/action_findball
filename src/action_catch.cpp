#include <memory>
#include <vector>
#include <thread>
#include <functional>
#include <thread>
#include <cmath>
#include <string>
#include <iostream>


#include "action_catch.hpp"


// which node to handle
static constexpr char const * lifecycle_node = "findball";
static constexpr char const * node_get_state_topic = "findball/get_state";
static constexpr char const * node_change_state_topic = "findball/change_state";

template<typename FutureT, typename WaitTimeT>
    std::future_status action_catch_ball::wait_for_result( FutureT & future, WaitTimeT time_to_wait)
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

action_catch_ball::ActionCatchBall::ActionCatchBall(const std::string & node_name,  const rclcpp::NodeOptions & options)
    :Node(node_name, options), findball_node_state_(false)
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
    "catch_ball",
    std::bind(&ActionCatchBall::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ActionCatchBall::handle_cancel, this, std::placeholders::_1),
    std::bind(&ActionCatchBall::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    callback_group_);

    //findball_node_init();

    ballinfo_sub_= this->create_subscription<rc2024_interfaces::msg::BallInfo>("ball_info",10, std::bind(&ActionCatchBall::ballinfo_callback,this,std::placeholders::_1));

    sub_notification_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
                            "/findball/transition_event", 10, std::bind(&ActionCatchBall::notification_callback, this, std::placeholders::_1));

    chassis_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("car/cmd_vel", 2);
    chassis_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>("car/ops", 2, std::bind(&ActionCatchBall::get_pose_speed_callback,this,std::placeholders::_1));

    // 等待findball节点上线，并初始化findball节点
    findball_node_state_ = findball_node_init();
}

unsigned int action_catch_ball::ActionCatchBall::get_state(std::chrono::seconds timeout)
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

bool  action_catch_ball::ActionCatchBall::change_state(std::uint8_t transition, std::chrono::seconds timeout)
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

rclcpp_action::GoalResponse action_catch_ball::ActionCatchBall::handle_goal(
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

rclcpp_action::CancelResponse action_catch_ball::ActionCatchBall::handle_cancel(
    const std::shared_ptr<GoalHandleCatchBall> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;

    if(!change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to deactive node %s", lifecycle_node);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void action_catch_ball::ActionCatchBall::handle_accepted(const std::shared_ptr<GoalHandleCatchBall> goal_handle)
{
    using namespace std::placeholders;
    std::thread{std::bind(&ActionCatchBall::execute, this, _1), goal_handle}.detach();
}

void action_catch_ball::ActionCatchBall::execute(const std::shared_ptr<GoalHandleCatchBall> goal_handle)
{
    rclcpp::Rate loop_rate(1);
    test_lifeycle();
    loop_rate.sleep();

    RCLCPP_INFO(this->get_logger(), "Executing goal, id: %s", reinterpret_cast<const char *>(goal_handle->get_goal_id().data()));
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<CatchBall::Feedback>();
    auto & feedback_msg = feedback->progress;
    auto result = std::make_shared<CatchBall::Result>();

    while (rclcpp::ok()) {
        if(goal_handle->is_canceling())
        {
            result->time = this->now().seconds() + this->now().nanoseconds() * 1e-9;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

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

bool action_catch_ball::ActionCatchBall::findball_node_init()
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
            return false;
        }
    }
    return true;
}

void action_catch_ball::ActionCatchBall::ballinfo_callback(const rc2024_interfaces::msg::BallInfo::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received ball info, x: %f, y: %f, z: %f", msg->balls_info.x, msg->balls_info.y, msg->balls_info.z);
}

void action_catch_ball::ActionCatchBall::test_lifeycle()
{
    rclcpp::WallRate time_between_state_changes(10);  // 10s

    RCLCPP_INFO(this->get_logger(), "Testing lifecycle for node %s", lifecycle_node);

    if(this->get_state() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
    {
            if(!change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure node %s", lifecycle_node);
            return;
        }
        if (!this->get_state()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get state for node %s", lifecycle_node);
            return;
        }
    }

      // activate
    {
        time_between_state_changes.sleep();
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for state change");
        return;
        }
        if (!this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to activate node %s", lifecycle_node);
            return;
        }
        return;
        }
        if (!this->get_state()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get state for node %s", lifecycle_node);
            return;
    }
}

void action_catch_ball::ActionCatchBall::notification_callback(lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr msg)
{
RCLCPP_INFO(
    get_logger(), "notify callback: Transition from state %s to %s",
    msg->start_state.label.c_str(), msg->goal_state.label.c_str());
}

void action_catch_ball::ActionCatchBall::get_pose_speed_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(variable_mutex_);
    ChassisPa.x = msg->x;
    ChassisPa.y = msg->y;
    ChassisPa.theta = msg->theta;
    vel_cal.cal_vel(ChassisPa, this->now().seconds(), this->now().nanoseconds(), ChassisPv);
}

action_catch_ball::VelCal::VelCal():last_time(0.0)
{
    Pa_last.x = 0;
    Pa_last.y = 0;
    Pa_last.theta = 0;
}

void action_catch_ball::VelCal::cal_vel(geometry_msgs::msg::Pose2D &Pa, float time_s, float time_ns, geometry_msgs::msg::Pose2D &vel)
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

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto action_catch = std::make_shared<action_catch_ball::ActionCatchBall>("action_catch");
    executor.add_node(action_catch);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
