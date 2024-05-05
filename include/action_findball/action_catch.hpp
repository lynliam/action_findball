#ifndef ACTION_CATCH_BALL_HPP
#define ACTION_CATCH_BALL_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rc2024_interfaces/action/catch_ball.hpp"
#include "rc2024_interfaces/msg/ball_info.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

#include <mutex>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>
#include <vector>
#include "chassis_related.hpp"
#include <opencv2/opencv.hpp>

// ros2 action send_goal /catch_ball rc2024_interfaces/action/CatchBall "{color: 1}"

namespace action_catch_ball {

    using namespace std::chrono_literals;

    template<typename FutureT, typename WaitTimeT>
        std::future_status wait_for_result( FutureT & future, WaitTimeT time_to_wait);

    enum class UpCmd {
        Reset = 1,
        ChaseBall = 2,
        CatchBall = 4,
        PutBall = 8,
        ChangeAngle = 16
    };

    enum class CatchBallState{
        SUCCEED = 1,
        CANCEL = 2,
        TIMEOUT = 3
    };
    class VelCal 
    {
        public:
            VelCal();
            void cal_vel(geometry_msgs::msg::Pose2D &Pa, float time_s, float time_ns, geometry_msgs::msg::Pose2D &vel);
        private:
            geometry_msgs::msg::Pose2D Pa_last;
            float last_time;
    };
    class PIDController
    {
        public:
            PIDController(float kp, float ki, float kd);
            float PID_Calc(float cur_error_);
            float PosePID_Calc(float cur_error_);
            void PID_MaxMin(float max, float min);
            void PID_setParam(float kp, float ki, float kd);
            float integralMax;  // 积分上限
            float integralMin;  // 积分下限 用于积分饱和
        private:
            float KP;        // PID参数P
            float KI;        // PID参数I
            float KD;        // PID参数D
            float cur_error; // 当前误差
            float error[2];  // 前两次误差
            float integral;  // 积分
            float output;    // 输出值
            float outputMax; // 最大输出值的绝对值
            float outputMin; // 最小输出值的绝对值用于防抖
    };

    class ActionCatchBall : public rclcpp::Node
    {
        public:
            using CatchBall = rc2024_interfaces::action::CatchBall;
            using GoalHandleCatchBall = rclcpp_action::ServerGoalHandle<CatchBall>;

            explicit ActionCatchBall(const std::string & node_name,  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
            unsigned int get_state(std::chrono::seconds timeout = 5s);
            bool  change_state(std::uint8_t transition, std::chrono::seconds timeout = 5s);

            bool change_state_to_active();

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
            void test_execute(const std::shared_ptr<GoalHandleCatchBall> goal_handle);
            void pid_test_execute(const std::shared_ptr<GoalHandleCatchBall> goal_handle);

            // BallInfo subscriber
            rclcpp::Subscription<rc2024_interfaces::msg::BallInfo>::SharedPtr ballinfo_sub_;
            void ballinfo_callback(const rc2024_interfaces::msg::BallInfo::SharedPtr msg);

            // 底盘
            rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr chassis_pub_;
            rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr chassis_sub_;
            rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr up_pub_;
            rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr imu_sub_;
            void get_pose_speed_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
            void get_imu_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
            // 底盘 variable
            geometry_msgs::msg::Pose2D ChassisPa;
            geometry_msgs::msg::Pose2D ChassisPv;
            std_msgs::msg::Float32MultiArray imu_data;

            // variable_mutex
            std::mutex variable_mutex_;
            std::mutex variable_mutex__;
            bool findball_node_init();

            // 速度计算
            VelCal vel_cal;
            std::vector<geometry_msgs::msg::Point32> ball_info;
            std::vector<geometry_msgs::msg::Point32> purple_info;
            bool is_found;

            //PID Controller
            PIDController PIDController_x;
            PIDController PIDController_y;
            PIDController PIDController_w;

            PIDController PIDController_x_near;
            PIDController PIDController_y_near;

            //findball_node state
            bool findball_node_state_;

            int acquire_PID_variable();
            int acquire_goal();
            float test_goal;

            bool up_decision_making();
            bool jaw_decision_making();

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

    };

}// namespace action_catch_ball

#endif // ACTION_CATCH_BALL_HPP

