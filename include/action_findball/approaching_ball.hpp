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

#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace action_findball {
    using EmptyGoal = rc2024_interfaces::action::CatchBall;
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
            bool PTZ_executor(const std::shared_ptr<sensor_msgs::msg::JointState> JointControl_to_pub,
                                const sensor_msgs::msg::JointState &JointState_,
                                double joint1);
            void global_supervisor(const geometry_msgs::msg::PoseStamped &tf_current_pose_,const nav_msgs::msg::Odometry &ChassisPa_);
            bool check_position(const geometry_msgs::msg::PoseStamped &tf_current_pose_, float x,float y, float w);
            
            void catch_ball_execute(const std::shared_ptr<GoalHandleEmptyGoal> goal_handle);
            void put_ball_execute(const std::shared_ptr<GoalHandleEmptyGoal> goal_handle);
            
            bool up_decision_making(
            std::vector<geometry_msgs::msg::Point32> &ball_info_, 
            std::vector<geometry_msgs::msg::Point32> &purple_info_, bool is_found_,Situation &situation_,int reset);

            float PTZ_ANGLE_decision_making(geometry_msgs::msg::Point32 &tracking_ball_);

            bool jaw_decision_making(
            std::vector<geometry_msgs::msg::Point32> &ball_info_, 
            std::vector<geometry_msgs::msg::Point32> &purple_info_, bool is_found_, CATCHBALL &catchball_state);

            std::shared_ptr<rclcpp::QoS> qos_profile;

            // 底盘
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr chassis_pub_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr chassis_sub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr up_pub_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr up_sub_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_update_;

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
            std_msgs::msg::Header ball_info_header;
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
            Situation situation;

            //kalman
            // Kalman variables
            cv::Vec2f last_measurement;
            cv::Vec2f current_measurement ;
            cv::Vec4f last_prediction ;
            cv::Vec4f current_prediction ;
            int current_radius;
            int last_radius;
            std::shared_ptr<cv::KalmanFilter> Kalman;

            cv::Vec2f last_measurement_purple;
            cv::Vec2f current_measurement_purple ;
            cv::Vec4f last_prediction_purple ;
            cv::Vec4f current_prediction_purple ;
            std::shared_ptr<cv::KalmanFilter> Kalman_purple;

            geometry_msgs::msg::Point32 tracking_ball;
            geometry_msgs::msg::Point32 tracking_purple;

            //PID 控制器
            PIDController PIDController_PTZ;
            PIDController PIDController_x;
            PIDController PIDController_x_catch;
            PIDController PIDController_y;
            PIDController PIDController_w;

            // tf Global Position
            std::shared_ptr<tf2_ros::Buffer> tf_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
            geometry_msgs::msg::PoseStamped tf_current_pose;

            action_findball::Status SupervisorState;

            std::string start_side;
            

            int left_ball_count;
            int right_ball_count;

            action_findball::TOWARD toward;

    };

} // namespace action_findball

#endif // APPROACHING_BALL_HPP