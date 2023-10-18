/*!*******************************************************************************************
 *  \file       ros2_cpp_template.hpp
 *  \brief      Class definition
 *  \authors    Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef ROS2_CPP_TEMPLATE_HPP
#define ROS2_CPP_TEMPLATE_HPP

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ros2_cpp_template/msg/my_template_msg.hpp"
#include "ros2_cpp_template/srv/my_template_srv.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace ros2_cpp_template {

/**
 * @brief Class Ros2CppTemplate
 */
class Ros2CppTemplate : public rclcpp::Node {
public:
  /**
   * @brief Construct a new Ros 2 Cpp Template object
   *
   * @param namespace_ std::string Namespace of the node
   */
  Ros2CppTemplate(const std::string &namespace_ = "ros2_cpp_template");

  /**
   * @brief Destroy the Ros 2 Cpp Template object
   */
  ~Ros2CppTemplate();

private:
  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Subscription<ros2_cpp_template::msg::MyTemplateMsg>::SharedPtr custom_subscription_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<ros2_cpp_template::msg::MyTemplateMsg>::SharedPtr custom_publisher_;

  // Timers
  rclcpp::TimerBase::SharedPtr timer_;

  // Services
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::Service<ros2_cpp_template::srv::MyTemplateSrv>::SharedPtr custom_service_;

  // Service clients
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr service_client_;
  std_srvs::srv::SetBool::Request::SharedPtr service_request_;
  std_srvs::srv::SetBool::Response::SharedPtr service_response_;

  rclcpp::Client<ros2_cpp_template::srv::MyTemplateSrv>::SharedPtr custom_service_client_;
  ros2_cpp_template::srv::MyTemplateSrv::Request::SharedPtr custom_service_request_;
  ros2_cpp_template::srv::MyTemplateSrv::Response::SharedPtr custom_service_response_;

private:
  // Callbacks Subscribers

  /**
   * @brief Subscription callback
   *
   * @param msg std_msgs::msg::String::SharedPtr Message received
   */
  void subscription_callback(const std_msgs::msg::String::SharedPtr msg);

  /**
   * @brief Subscription callback
   *
   * @param msg ros2_cpp_template::msg::MyTemplateMsg::SharedPtr Message received
   */
  void custom_subscription_callback(const ros2_cpp_template::msg::MyTemplateMsg::SharedPtr msg);

  // Callbacks Timers

  /**
   * @brief Timer callback
   */
  void timer_callback();

  // Callbacks Services

  /**
   * @brief Service callback
   *
   * @param request std::shared_ptr<std_srvs::srv::SetBool::Request> Request received
   * @param response std::shared_ptr<std_srvs::srv::SetBool::Response> Response to send
   */
  void service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        const std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /**
   * @brief Service callback
   *
   * @param request std::shared_ptr<ros2_cpp_template::srv::MyTemplateSrv::Request> Request received
   * @param response std::shared_ptr<ros2_cpp_template::srv::MyTemplateSrv::Response> Response to
   * send
   */
  void custom_service_callback(
      const std::shared_ptr<ros2_cpp_template::srv::MyTemplateSrv::Request> request,
      const std::shared_ptr<ros2_cpp_template::srv::MyTemplateSrv::Response> response);

  // Call Service clients

  /**
   * @brief Call service
   */
  void call_service();

  /**
   * @brief Call service
   */
  void call_custom_service();
};
}  // namespace ros2_cpp_template

#endif  // ROS2_CPP_TEMPLATE_HPP