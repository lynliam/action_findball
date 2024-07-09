#include <cstddef>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <rc2024_interfaces/msg/detail/camera_switch__struct.hpp>
#include <rclcpp/qos.hpp>
#include <vector>
#include <thread>
#include <functional>
#include <condition_variable>
#include <thread>

#include <curl/curl.h>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs//msg/string.hpp"

#include "FindBall.hpp"
#include "rc2024_interfaces/msg/ball_info.hpp"
#include "rc2024_interfaces/msg/camera_switch.hpp"
#include "std_msgs/msg/u_int32.hpp"

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
            signal(false),stop(false),photo_count(0)
            {
                RCLCPP_INFO(this->get_logger(), "PublisherFindball Node has been created");
                declare_parameter<int>("balltype", 2);
                //publisher_ = this->create_publisher<rc2024_interfaces::msg::BallInfo>("ball_info", 10);
                curl = curl_easy_init();
                findball_server_handler_up = std::make_shared<CameraUPServer>();
                //findball_server_handler_jaw = std::make_shared<CameraJawServer>();
                findball_server_handler = findball_server_handler_up;
                qos_profile = std::make_shared<rclcpp::QoS>(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
                qos_profile->reliability(rclcpp::ReliabilityPolicy::BestEffort);
                this->create_subscription<rc2024_interfaces::msg::CameraSwitch>("camera_switch", 2, std::bind(&PublisherFindball::up_cmd_callback, this, std::placeholders::_1));
            }
            ~PublisherFindball()
            {
                RCLCPP_INFO(this->get_logger(), "PublisherFindball Node has been destroyed");
            }

            void executor()
            {
                rclcpp::Rate rate(1000);
                RCLCPP_INFO(this->get_logger(), "PublisherFindball thread was created");
                std::unique_lock<std::mutex> lock_flag(mutex_flag, std::defer_lock);
                if(curl) curl_easy_setopt(curl, CURLOPT_URL, "http://0.0.0.0:8000/upload");  // 设置URL
                cv::Mat color_img;
                while(rclcpp::ok())
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    condition_.wait(lock, [this]{return signal;});
                    if(stop)
                        break;
                    lock.unlock();

                    auto ball_info = std::make_shared<rc2024_interfaces::msg::BallInfo>();
                    std::vector<cv::Vec3d> ball_result;
                    std::vector<cv::Vec3d> purple_result;
                    // lock_flag.lock();
                    // int camera_flag_ = 0;
                    // lock_flag.unlock();
                    findball_server_handler = findball_server_handler_up;
                    // if(camera_flag_ == 0)
                        
                    // else if(camera_flag_ == 1)
                    //     findball_server_handler = findball_server_handler_jaw;
                    // type = 2;
                    // RCLCPP_INFO(this->get_logger(), "colcor : %d", type);
                    if(findball_server_handler->find_ball(type, ball_result, purple_result))
                    {
                        ball_info->balls_info.resize(ball_result.size());
                        ball_info->purple_info.resize(purple_result.size());
                        for(size_t i = 0;auto &ball : ball_result)
                        {
                            ball_info->balls_info[i].x = ball[0];
                            ball_info->balls_info[i].y = ball[1];
                            ball_info->balls_info[i].z = ball[2];
                            i++;
                        }
                        for(size_t i = 0;auto &purple : purple_result)
                        {
                            ball_info->purple_info[i].x = purple[0];
                            ball_info->purple_info[i].y = purple[1];
                            ball_info->purple_info[i].z = purple[2];
                            i++;
                        }
                        //findball_server_handler->imgshow_DEBUG();
                        ball_info->header.stamp = this->now();
                        ball_info->type = type;
                        ball_info->is_found = true;
                        publisher_->publish(*ball_info);
                    }
                    else {
                        // RCLCPP_INFO(this->get_logger(), "Ball not found");

                        // for(auto &purple : purple_result)
                        // {
                        //     ball_info->purple_info[0].x = purple[0];
                        //     ball_info->purple_info[0].y = purple[1];
                        //     ball_info->purple_info[0].z = purple[2];
                        // }
                        // ball_info->type = 10;
                        ball_info->is_found = false;
                        publisher_->publish(*ball_info);
                    }
                    //findball_server_handler->get_color_img(color_img);
                    color_img = findball_server_handler->combinedImage;
                    if(!color_img.empty())
                    {
                        if(photo_count <= 20)
                        {
                            // 获取当前时间
                            time_t rawtime;
                            struct tm * timeinfo;
                            char buffer [80];
                            time (&rawtime);
                            timeinfo = localtime(&rawtime);
                            // 将时间格式化为字符串
                            strftime(buffer,80,"%Y-%m-%d_%H-%M-%S",timeinfo);
                            std::string filename = std::string(buffer) + std::to_string(photo_count) + ".jpg";
                            RCLCPP_INFO(this->get_logger(), "Save photo %s", filename.c_str());
                            cv::imwrite(filename, findball_server_handler->color_image);
                            photo_count ++;
                        }

                        cv::imencode(".jpg", color_img, buf);
                        std::string img_data(buf.begin(), buf.end());
                        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, img_data.c_str());
                        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, img_data.size());
                        // 设置超时时间
                        curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 50L);

                        // 执行请求
                        res = curl_easy_perform(curl);
                        // 检查错误码
                        if(res != CURLE_OK) {
                            if(res == CURLE_OPERATION_TIMEDOUT) {
                                //std::cerr << "操作超时，丢帧处理" << std::endl;
                            } else if(res == CURLE_COULDNT_CONNECT) {
                                //std::cerr << "无法连接到服务器，服务端是不是还没开起来?" << std::endl;
                            } else {
                                //std::cerr << "请求失败: " << curl_easy_strerror(res) << std::endl;
                            }
                        } else {
                            // 检查HTTP响应码
                            long response_code;
                            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
                            //std::cout << "HTTP响应码: " << response_code << std::endl;
                            if(response_code != 200) {
                                //std::cerr << "接口名字是不是写错了" << std::endl;
                            }
                        }
                    }
                    
                    //findball_server_handler->imgshow_DEBUG();
                    rate.sleep();
                }
            }

            void up_cmd_callback(const rc2024_interfaces::msg::CameraSwitch msg)
            {
                std::unique_lock<std::mutex> lock_flag(mutex_flag);
                if(msg.index == 0)
                    camera_flag = 0;
                else if(msg.index == 1)
                    camera_flag =1;
                lock_flag.unlock();
            }

            /* lifecycle config */

            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_configure(const rclcpp_lifecycle::State &)
            {
                // 创建一个QoS配置对象，设置可靠性为BestEffort

                publisher_ = this->create_publisher<rc2024_interfaces::msg::BallInfo>("ball_info",*qos_profile);
                get_parameter("balltype",type);                
                RCLCPP_INFO(get_logger(), "on_configure() is called.balltype: %d", type);


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
                //findball_server_handler->Kalman->init(4,2);
                //findball_server_handler->Kalman->measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);
                //findball_server_handler->Kalman->transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
                //findball_server_handler->Kalman->processNoiseCov = (cv::Mat_<float>(4, 4) << 0.01, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0.01);
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
            std::shared_ptr<FindBallServer> findball_server_handler_up;
            std::shared_ptr<FindBallServer> findball_server_handler_jaw;

            std::mutex mutex_;
            std::mutex mutex_flag;
            std::condition_variable condition_;
            bool signal;
            bool stop;
            int type ;

            std::vector<uchar> buf;

            std::shared_ptr<rclcpp::QoS> qos_profile;

            int camera_flag;
            int photo_count;

            CURL* curl;
            CURLcode res;
            
    };
} // namespace action_findball_cpp

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exe;
    
    auto node = std::make_shared<PublisherFindballCPP::PublisherFindball>("publisher_findball");
    exe.add_node(node->get_node_base_interface());
    std::thread threadOBJ(&PublisherFindballCPP::PublisherFindball::executor, &(*node));
    exe.spin();
    rclcpp::shutdown();
    return 0;
}
