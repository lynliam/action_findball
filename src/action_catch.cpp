#include <memory>
#include <vector>
#include <thread>
#include <functional>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rc2024_interfaces/action/catch_ball.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

namespace action_catch_ball {

    using namespace std::chrono_literals;

    // which node to handle
    static constexpr char const * lifecycle_node = "findball";
    static constexpr char const * node_get_state_topic = "findball/get_state";
    static constexpr char const * node_change_state_topic = "lc_talker/change_state";

    template<typename FutureT, typename WaitTimeT>
        std::future_status wait_for_result( FutureT & future, WaitTimeT time_to_wait)
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

    class ActionCatchBall : public rclcpp::Node
    {
        public:
            using CatchBall = rc2024_interfaces::action::CatchBall;
            using GoalHandleCatchBall = rclcpp_action::ClientGoalHandle<CatchBall>;

            explicit ActionCatchBall(const std::string & node_name,  const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
            :Node(node_name, options)
            {
                RCLCPP_INFO(this->get_logger(), "ActionCatchBall Node has been created");
                
                // lifecycle node init
                client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(node_get_state_topic);
                client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(node_change_state_topic);
            }

            unsigned int get_state(std::chrono::seconds timeout = 1s)
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

                }

            }


        private:
            std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
            std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
    };
}// namespace action_catch_ball

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto action_catch = std::make_shared<action_catch_ball::ActionCatchBall>("action_catch");
    rclcpp::spin(action_catch);
    rclcpp::shutdown();
    return 0;
}
