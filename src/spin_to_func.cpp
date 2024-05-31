#include "spin_to_func.hpp"
#include <geometry_msgs/msg/detail/pose2_d__struct.hpp>
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_util/node_utils.hpp"

// Stop the robot with a commanded velocity
void action_findball::SpinTo::stopRobot()
{
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Pose2D>();
    cmd_vel->x = 0.0;
    cmd_vel->y = 0.0;
    cmd_vel->theta = 0.0;
    vel_pub_->publish(std::move(cmd_vel));
}

action_findball::SpinTo::SpinTo()
:   min_rotational_vel_(0.0),
    max_rotational_vel_(0.0),
    rotational_acc_lim_(0.0),
    cmd_yaw_(0.0),
    prev_yaw_(0.0),
    relative_yaw_(0.0),
    simulate_ahead_time_(0.0)
{
    time_node = rclcpp::Node::make_shared("spin_to_time_node");
    vel_pub_ = time_node->create_publisher<geometry_msgs::msg::Pose2D>("/car/cmd_vel", 5);
    onConfigure();
}

void action_findball::SpinTo::onConfigure()
{
    simulate_ahead_time_ = 1.0;
    max_rotational_vel_ = 1.2;
    min_rotational_vel_ = 0.1;
    rotational_acc_lim_ = 0.3;
}

action_findball::Status action_findball::SpinTo::onRun(float command,rclcpp::Duration time_allowance,nav_msgs::msg::Odometry current_pose)
{
    prev_yaw_ = tf2::getYaw(current_pose.pose.pose.orientation);
    relative_yaw_ = 0.0;
    cmd_yaw_ = command;
    while(fabs(cmd_yaw_)>=M_PI)
    {
        cmd_yaw_ = -(copysign(2 * M_PI , cmd_yaw_)-cmd_yaw_);
        RCLCPP_INFO(time_node->get_logger(), "cmd_yaw_ %2f out of limit,change it to %2f ",command,cmd_yaw_);
    }
      RCLCPP_INFO(
    time_node->get_logger(), "Turning %0.2f for spin behavior.starting from %0.2f",
    cmd_yaw_, prev_yaw_);

    command_time_allowance_ = time_allowance;
    end_time_ = time_node->get_clock()->now() + command_time_allowance_;
    return Status::RUNNING;
}

action_findball::Status action_findball::SpinTo::onCycleUpdate(nav_msgs::msg::Odometry current_pose)
{
    rclcpp::Duration time_remaining = end_time_ - time_node->get_clock()->now();
    RCLCPP_INFO(time_node->get_logger(), "time_remaining %f",time_remaining.seconds());
    if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
        stopRobot();
        RCLCPP_WARN(
      time_node->get_logger(),
      "Exceeded time allowance before reaching the SpinTo goal - Exiting SpinTo");
        return Status::FAILED;
    }

    const double current_yaw = tf2::getYaw(current_pose.pose.pose.orientation);
    RCLCPP_INFO(time_node->get_logger(), "current_yaw %f",current_yaw);

    double delta_yaw = current_yaw - prev_yaw_; // 上周期转过的角度
    if (abs(delta_yaw) > M_PI) {
        delta_yaw = copysign(2 * M_PI - abs(delta_yaw), prev_yaw_);
    }

    relative_yaw_ += delta_yaw; // 总共转过的角度
    prev_yaw_ = current_yaw; 

    double remaining_yaw = cmd_yaw_ - current_yaw; // 剩余需要转的角度
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Pose2D>();
    cmd_vel->x = 0.0;
    cmd_vel->y = 0.0;
    RCLCPP_INFO(time_node->get_logger(), "remaining_yaw %2f",remaining_yaw);
    if (abs(remaining_yaw) < 0.05) {
        //stopRobot();
        //RCLCPP_INFO(logger_, "Stopped spinning ");
        if(abs(delta_yaw)<1e-3)
        {
            RCLCPP_INFO(time_node->get_logger(), "Reached goal on %.2f.",current_yaw);
            stopRobot();
            vel_pub_->publish(std::move(cmd_vel));  
            return Status::SUCCEEDED;
        }
        else
        {
            // cmd_vel->angular.z = -delta_yaw*cycle_frequency_;
            stopRobot();
            vel_pub_->publish(std::move(cmd_vel));   
            return Status::RUNNING;
        }
    }
    double vel = sqrt(2 * rotational_acc_lim_ * abs(remaining_yaw));
    vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

    cmd_vel->theta = copysign(vel, remaining_yaw);

    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = current_pose.pose.pose.position.x;
    pose2d.y = current_pose.pose.pose.position.y;
    pose2d.theta = tf2::getYaw(current_pose.pose.pose.orientation);
    vel_pub_->publish(std::move(cmd_vel));

    return Status::RUNNING;
}