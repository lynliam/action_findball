#include <cmath>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "State.hpp"


namespace action_findball {

class SpinTo
{
    public:
        SpinTo();

        Status onRun(float command,rclcpp::Duration time_allowance, geometry_msgs::msg::PoseStamped current_pose);
        void onConfigure();
        Status onCycleUpdate(geometry_msgs::msg::PoseStamped current_pose);
    private:
        rclcpp::Node::SharedPtr time_node;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        void stopRobot();
        double min_rotational_vel_;
        double max_rotational_vel_;
        double rotational_acc_lim_;
        double cmd_yaw_;
        double prev_yaw_;
        
        double relative_yaw_;
        double simulate_ahead_time_;
        rclcpp::Duration command_time_allowance_{0, 0};
        rclcpp::Time end_time_;
}; // class SpinTo
} // namespace action_findball

