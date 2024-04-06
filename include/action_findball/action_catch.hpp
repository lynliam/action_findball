#ifndef ACTION_CATCH_BALL_HPP
#define ACTION_CATCH_BALL_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rc2024_interfaces/action/catch_ball.hpp"
#include "rc2024_interfaces/msg/ball_info.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

#include <mutex>
#include "chassis_related.hpp"

// ros2 action send_goal /catch_ball rc2024_interfaces/action/CatchBall "{color: 1}"

namespace action_catch_ball {

    using namespace std::chrono_literals;

    template<typename FutureT, typename WaitTimeT>
        std::future_status wait_for_result( FutureT & future, WaitTimeT time_to_wait);

    class VelCal 
    {
        public:
            VelCal();
            void cal_vel(geometry_msgs::msg::Pose2D &Pa, float time_s, float time_ns, geometry_msgs::msg::Pose2D &vel);
        private:
            geometry_msgs::msg::Pose2D Pa_last;
            float last_time;
    };

    class ActionCatchBall : public rclcpp::Node
    {
        public:
            using CatchBall = rc2024_interfaces::action::CatchBall;
            using GoalHandleCatchBall = rclcpp_action::ServerGoalHandle<CatchBall>;

            explicit ActionCatchBall(const std::string & node_name,  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
            unsigned int get_state(std::chrono::seconds timeout = 5s);
            bool  change_state(std::uint8_t transition, std::chrono::seconds timeout = 5s);

            void test_lifeycle();

        private:
            std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
            std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
            std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>> sub_notification_;
            void notification_callback(lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr msg);

            // 回调组
            rclcpp::CallbackGroup::SharedPtr callback_group_;
            // Action
            rclcpp_action::Server<CatchBall>::SharedPtr action_server_;
            rclcpp_action::GoalResponse handle_goal(
                                                    const rclcpp_action::GoalUUID & uuid,
                                                    std::shared_ptr<const CatchBall::Goal> goal);
            rclcpp_action::CancelResponse handle_cancel(
                                                    const std::shared_ptr<GoalHandleCatchBall> goal_handle);
            void handle_accepted(const std::shared_ptr<GoalHandleCatchBall> goal_handle);
            void execute(const std::shared_ptr<GoalHandleCatchBall> goal_handle);

            // BallInfo subscriber
            rclcpp::Subscription<rc2024_interfaces::msg::BallInfo>::SharedPtr ballinfo_sub_;
            void ballinfo_callback(const rc2024_interfaces::msg::BallInfo::SharedPtr msg);

            // 底盘
            rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr chassis_pub_;
            rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr chassis_sub_;
            void get_pose_speed_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
            // 底盘 variable
            geometry_msgs::msg::Pose2D ChassisPa;
            geometry_msgs::msg::Pose2D ChassisPv;

            // variable_mutex
            std::mutex variable_mutex_;
            bool findball_node_init();

            // 速度计算
            VelCal vel_cal;

            //findball_node state
            bool findball_node_state_;
    };

}// namespace action_catch_ball

#endif // ACTION_CATCH_BALL_HPP

