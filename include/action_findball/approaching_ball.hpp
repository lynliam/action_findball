#ifndef APPROACHING_BALL_HPP
#define APPROACHING_BALL_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rc2024_interfaces/action/catch_ball.hpp"
#include "rc2024_interfaces/msg/ball_info.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rc2024_interfaces/action/put_ball.hpp"
#include "rc2024_interfaces/msg/camera_switch.hpp"

#include "lifecycle_manager_client.hpp"
#include "pid_controller.hpp"
#include "spin_to_func.hpp"
#include "State.hpp"

#include <memory>
#include <opencv2/opencv.hpp>

namespace action_findball {
    using EmptyGoal = rc2024_interfaces::action::PutBall;
    using GoalHandleEmptyGoal = rclcpp_action::ServerGoalHandle<EmptyGoal>;

    class ApproachingBall : public LifecycleManagerClient
    {
        public:
            explicit ApproachingBall(const std::string & node_name,  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
            
        private:
        // 回调组
            rclcpp::CallbackGroup::SharedPtr callback_group_;
            // Action
            rclcpp_action::Server<EmptyGoal>::SharedPtr action_server_;
            rclcpp_action::GoalResponse handle_goal(
                                                    const rclcpp_action::GoalUUID & uuid,
                                                    std::shared_ptr<const EmptyGoal::Goal> goal);
            rclcpp_action::CancelResponse handle_cancel(
                                                    const std::shared_ptr<GoalHandleEmptyGoal> goal_handle);
            void handle_accepted(const std::shared_ptr<GoalHandleEmptyGoal> goal_handle);
            void execute(const std::shared_ptr<GoalHandleEmptyGoal> goal_handle);
            bool arm_executor(const std::shared_ptr<sensor_msgs::msg::JointState> JointControl_to_pub,
                                const sensor_msgs::msg::JointState &JointState_,
                                double joint1, double joint2, double joint3, double joint4);
            void catch_ball_execute(const std::shared_ptr<GoalHandleEmptyGoal> goal_handle);
            void put_ball_execute(const std::shared_ptr<GoalHandleEmptyGoal> goal_handle);
            
            bool up_decision_making(
            std::vector<geometry_msgs::msg::Point32> &ball_info_, 
            std::vector<geometry_msgs::msg::Point32> &purple_info_, bool is_found_);

            bool jaw_decision_making(
            std::vector<geometry_msgs::msg::Point32> &ball_info_, 
            std::vector<geometry_msgs::msg::Point32> &purple_info_, bool is_found_, CATCHBALL &catchball_state);


            // 底盘
            rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr chassis_pub_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr chassis_sub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr up_pub_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr up_sub_;

            void get_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
            void get_imu_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
            void get_jointstate_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

            // BallInfo subscriber
            rclcpp::Subscription<rc2024_interfaces::msg::BallInfo>::SharedPtr ballinfo_sub_;
            void ballinfo_callback(const rc2024_interfaces::msg::BallInfo::SharedPtr msg);

            bool findball_node_init();

            // CameraSwitch publisher
            rclcpp::Publisher<rc2024_interfaces::msg::CameraSwitch>::SharedPtr camera_switch_pub_;

            //spin_to_func
            std::shared_ptr<SpinTo> spin_to_func;

            //BallInfo variable
            std::vector<geometry_msgs::msg::Point32> ball_info;
            std::vector<geometry_msgs::msg::Point32> purple_info;
            bool is_found;

            // 底盘 variable
            nav_msgs::msg::Odometry ChassisPa;

            // 上肢 variable
            sensor_msgs::msg::JointState up_joint_state;

            // variable_mutex
            std::mutex variable_mutex_;
            std::mutex variable_mutex__;
            std::mutex variable_mutex__1;

            //findball_node state
            bool findball_node_state_;

            //kalman
            // Kalman variables
            cv::Vec2f last_measurement;
            cv::Vec2f current_measurement ;
            cv::Vec4f last_prediction ;
            cv::Vec4f current_prediction ;
            int current_radius;
            int last_radius;
            std::shared_ptr<cv::KalmanFilter> Kalman;
            geometry_msgs::msg::Point32 tracking_ball;

            //PID 控制器
            PIDController PIDController_PTZ;
            PIDController PIDController_x;
            PIDController PIDController_y;
            PIDController PIDController_w;

    };

} // namespace action_findball

#endif // APPROACHING_BALL_HPP