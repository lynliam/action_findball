#include <iostream>
#include <memory>
#include <vector>
#include <thread>
#include <functional>
#include <condition_variable>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs//msg/string.hpp"

#include "FindBall.hpp"
#include "rc2024_interfaces/msg/ball_info.hpp"

// rclcpp生命周期节点
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "lifecycle_msgs/msg/transition.hpp"

namespace PublisherFindballCPP {
    class PublisherFindball : public rclcpp_lifecycle::LifecycleNode
    {
        public:
            explicit PublisherFindball(const std::string & node_name, bool intra_process_comms = false)
            :rclcpp_lifecycle::LifecycleNode(node_name,rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)),
            signal(false),stop(false)
            {
                RCLCPP_INFO(this->get_logger(), "PublisherFindball Node has been created");
                this->declare_parameter<int>("balltype", 1);
                //publisher_ = this->create_publisher<rc2024_interfaces::msg::BallInfo>("ball_info", 10);
                findball_server_handler = std::make_shared<FindBallServer>();
                findball_server_handler->main_init();
            }
            ~PublisherFindball()
            {
                RCLCPP_INFO(this->get_logger(), "PublisherFindball Node has been destroyed");
            }

            void executor()
            {
                rclcpp::Rate rate(50);
                RCLCPP_INFO(this->get_logger(), "PublisherFindball thread was created");
                //findball_server_handler->imgshow_DEBUG_INIT();
                while(rclcpp::ok())
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    condition_.wait(lock, [this]{return signal;});
                    if(stop)
                        break;
                    lock.unlock();
                    auto ball_info = rc2024_interfaces::msg::BallInfo();
                    auto type = this->get_parameter("balltype").as_int();
                    cv::Vec3d ball_result;
                    if(findball_server_handler->findball_with_Kalman(type, ball_result))
                    {
                        ball_info.balls_info.x = ball_result[0];
                        ball_info.balls_info.y = ball_result[1];
                        ball_info.balls_info.z = ball_result[2];
                        ball_info.type = type;
                        RCLCPP_INFO(this->get_logger(), "Ball found at x: %f, y: %f, z: %f", ball_result[0], ball_result[1], ball_result[2]);
                        publisher_->publish(ball_info);
                    }
                    else {
                        RCLCPP_INFO(this->get_logger(), "Ball not found");
                    }
                    //findball_server_handler->imgshow_DEBUG();
                    rate.sleep();
                }
            }

            /* lifecycle config */

            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_configure(const rclcpp_lifecycle::State &)
            {
                publisher_ = this->create_publisher<rc2024_interfaces::msg::BallInfo>("ball_info", 10);
                RCLCPP_INFO(get_logger(), "on_configure() is called.");

                // We return a success and hence invoke the transition to the next
                // step: "inactive".
                // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
                // would stay in the "unconfigured" state.
                // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
                // this callback, the state machine transitions to state "errorprocessing".
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
            }
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_activate(const rclcpp_lifecycle::State & state)
            {
                LifecycleNode::on_activate(state);
                std::unique_lock<std::mutex> lock(mutex_);
                signal = true;
                condition_.notify_all();
                RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

                // We return a success and hence invoke the transition to the next
                // step: "active".
                // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
                // would stay in the "inactive" state.
                // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
                // this callback, the state machine transitions to state "errorprocessing".
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
            }

            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_deactivate(const rclcpp_lifecycle::State & state)
            {
                LifecycleNode::on_deactivate(state);
                RCUTILS_LOG_INFO_NAMED(get_name(), "acquiring lock ...");
                std::unique_lock<std::mutex> lock(mutex_);
                findball_server_handler->Kalman->init(4,2);
                findball_server_handler->Kalman->measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);
                findball_server_handler->Kalman->transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
                findball_server_handler->Kalman->processNoiseCov = (cv::Mat_<float>(4, 4) << 0.01, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0.01);
                signal = false;
                condition_.notify_all();
                RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

                // We return a success and hence invoke the transition to the next
                // step: "inactive".
                // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
                // would stay in the "active" state.
                // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
                // this callback, the state machine transitions to state "errorprocessing".
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
            }
            
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_cleanup(const rclcpp_lifecycle::State &)
            {
                std::unique_lock<std::mutex> lock(mutex_);
                signal = false;
                stop = true;
                condition_.notify_all();
                publisher_.reset();
                RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
                // We return a success and hence invoke the transition to the next
                // step: "unconfigured".
                // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
                // would stay in the "inactive" state.
                // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
                // this callback, the state machine transitions to state "errorprocessing".
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
            }


            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_shutdown(const rclcpp_lifecycle::State & state)
            {
                publisher_.reset();

                RCUTILS_LOG_INFO_NAMED(
                get_name(),
                "on shutdown is called from state %s.",
                state.label().c_str());

                // We return a success and hence invoke the transition to the next
                // step: "finalized".
                // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
                // would stay in the current state.
                // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
                // this callback, the state machine transitions to state "errorprocessing".
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
            }

        private:
            std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<rc2024_interfaces::msg::BallInfo>> publisher_;
            std::shared_ptr<FindBallServer> findball_server_handler;

            std::mutex mutex_;
            std::condition_variable condition_;
            bool signal;
            bool stop;
            
    };
} // namespace action_findball_cpp

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exe;
    
    auto node = std::make_shared<PublisherFindballCPP::PublisherFindball>("findball");
    exe.add_node(node->get_node_base_interface());
    std::thread threadOBJ(&PublisherFindballCPP::PublisherFindball::executor, &(*node));
    exe.spin();
    rclcpp::shutdown();
    return 0;
}
