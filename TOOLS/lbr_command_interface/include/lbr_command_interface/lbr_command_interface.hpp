// Copyright (c) 2024 Tohoku Univ. Space Robotics Lab.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LBR_COMMAND_INTERFACE__LBR_COMMAND_INTERFACE_HPP_
#define LBR_COMMAND_INTERFACE__LBR_COMMAND_INTERFACE_HPP_

#include <functional>
#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "lbr_msgs/msg/motion_task.hpp"

class CommandInterface : public rclcpp::Node
{
public:
  CommandInterface();

private:
  void showCommandList();
  void execInputCommand(const std::string command);
  /**
   * @brief Publish base motion task based on joy input.
   *
   * @param joy
   */
  void joyBaseMotionTask(const sensor_msgs::msg::Joy & joy);
  /**
   * @brief This callback function runs when this node subscribes /joy topic.
   *
   * @param joy_cmd
   */
  void joyCallback(const sensor_msgs::msg::Joy & joy_cmd);
  /**
   * @brief Publish limb motion task based on joy input.
   *
   * @param limb_id
   * @param joy
   */
  void joyLimbMotionTask(const int & limb_id, const sensor_msgs::msg::Joy & joy);
  /**
   * @brief This function publishes gripper open/close command.
   *
   * @param limb_id
   * @param close_gripper
   */
  void publishGripperCommand(const int & limb_id, const bool & close_gripper);
  void stringCallback(const std_msgs::msg::String & string);
  /**
   * @brief This function runs in certain rate.
   */
  void timerCallback();

  // Publisher
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_state_pub_LF_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_state_pub_LH_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_state_pub_RH_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_state_pub_RF_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_to_LLC_pub_;
  // pub used by HLC to send high level motion request (like Grieel Tranisiton procedure or others)
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_to_HLC_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_to_state_estimator_pub_;
  rclcpp::Publisher<lbr_msgs::msg::BaseMotionTask>::SharedPtr base_motion_task_pub_;
  rclcpp::Publisher<lbr_msgs::msg::LimbMotionTask>::SharedPtr limb_motion_task_pub_;

  // Subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Variables
  sensor_msgs::msg::Joy input_joy_cmd;
  int timer_loop_rate;  // [ms]
};

#endif  // LBR_COMMAND_INTERFACE__LBR_COMMAND_INTERFACE_HPP_