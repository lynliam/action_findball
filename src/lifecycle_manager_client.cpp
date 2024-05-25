#include "lifecycle_manager_client.hpp"

// which node to handle
static constexpr char const * lifecycle_node = "findball";
static constexpr char const * node_get_state_topic = "findball/get_state";
static constexpr char const * node_change_state_topic = "findball/change_state";

template<typename FutureT, typename WaitTimeT> std::future_status wait_for_result(
    FutureT & future, WaitTimeT time_to_wait)
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

LifecycleManagerClient::LifecycleManagerClient(const std::string & node_name,  const rclcpp::NodeOptions & options)
    :Node(node_name, options)
{
    RCLCPP_INFO(this->get_logger(), "lifecycle manager for %s has been created",node_name.c_str());
    // lifecycle node init
    client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(node_get_state_topic);
    client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(node_change_state_topic);
    sub_notification_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
                        "/findball/transition_event", 10, std::bind(&LifecycleManagerClient::notification_callback, this, std::placeholders::_1));
}


unsigned int LifecycleManagerClient::get_state(std::chrono::seconds timeout)
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

bool  LifecycleManagerClient::change_state(std::uint8_t transition, std::chrono::seconds timeout)
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

bool LifecycleManagerClient::change_state_to_active()
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

void LifecycleManagerClient::notification_callback(lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr msg)
{
RCLCPP_INFO(
    get_logger(), "notify callback: Transition from state %s to %s",
    msg->start_state.label.c_str(), msg->goal_state.label.c_str());
}