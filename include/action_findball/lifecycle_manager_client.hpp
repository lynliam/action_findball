#ifndef LIFECYCLE_MANAGER_CLIENT_HPP
#define LIFECYCLE_MANAGER_CLIENT_HPP

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;

class LifecycleManagerClient : public rclcpp::Node
{
    public:
        explicit LifecycleManagerClient(const std::string & node_name,  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        unsigned int get_state(std::chrono::seconds timeout = 5s);
        bool  change_state(std::uint8_t transition, std::chrono::seconds timeout = 5s);
        bool change_state_to_active();
        std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
        std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
        std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>> sub_notification_;
    private:
        void notification_callback(lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr msg);
};

#endif