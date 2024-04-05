#ifndef ACTION_CATCH_BALL_HPP
#define ACTION_CATCH_BALL_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rc2024_interfaces/action/catch_ball.hpp"
#include "rc2024_interfaces/msg/ball_info.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

// ros2 action send_goal /catch_ball rc2024_interfaces/action/CatchBall "{color: 1}"

namespace action_catch_ball {

    using namespace std::chrono_literals;

    template<typename FutureT, typename WaitTimeT>
        std::future_status wait_for_result( FutureT & future, WaitTimeT time_to_wait);

    class ActionCatchBall : public rclcpp::Node
    {
        public:
            using CatchBall = rc2024_interfaces::action::CatchBall;
            using GoalHandleCatchBall = rclcpp_action::ServerGoalHandle<CatchBall>;

            explicit ActionCatchBall(const std::string & node_name,  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
            unsigned int get_state(std::chrono::seconds timeout = 5s);
            bool  change_state(std::uint8_t transition, std::chrono::seconds timeout = 5s);
            void init();

            void test_lifeycle();

        private:
            std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
            std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
            std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>> sub_notification_;
            void notification_callback(lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr msg);
            rclcpp::CallbackGroup::SharedPtr callback_group_;

            rclcpp_action::Server<CatchBall>::SharedPtr action_server_;

            rclcpp_action::GoalResponse handle_goal(
                                                    const rclcpp_action::GoalUUID & uuid,
                                                    std::shared_ptr<const CatchBall::Goal> goal);
            rclcpp_action::CancelResponse handle_cancel(
                                                    const std::shared_ptr<GoalHandleCatchBall> goal_handle);
            void handle_accepted(const std::shared_ptr<GoalHandleCatchBall> goal_handle);
            void execute(const std::shared_ptr<GoalHandleCatchBall> goal_handle);

            rclcpp::Subscription<rc2024_interfaces::msg::BallInfo>::SharedPtr ballinfo_sub_;
            void ballinfo_callback(const rc2024_interfaces::msg::BallInfo::SharedPtr msg);
            
            bool findball_node_init();
    };


}// namespace action_catch_ball

#endif // ACTION_CATCH_BALL_HPP

