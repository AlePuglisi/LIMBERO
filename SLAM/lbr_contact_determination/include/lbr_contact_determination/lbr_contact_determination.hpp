// Copyright (c) 2023 Tohoku Univ. Space Robotics Lab.
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

#ifndef LBR_CONTACT_DETERMINATION__LBR_CONTACT_DETERMINATION_HPP_
#define LBR_CONTACT_DETERMINATION__LBR_CONTACT_DETERMINATION_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <lbr_parameter.hpp>
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

class ContactDetermination : public rclcpp::Node
{
public:
  ContactDetermination();

private:
  /**
   * @brief This callback function runs when Contact determination subscribes wrench (filtered force and torque) from State Estimator.
   *  This callback function does contact determination by force values.
   *
   * @param force_torque
   */
  void leptrinoCallback(const geometry_msgs::msg::WrenchStamped force_torque);
  void publishContactStates(
    const std_msgs::msg::Bool contact_states,
    const geometry_msgs::msg::WrenchStamped filtered_force_torque);

  // Publisher
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr contact_state_pub_LF_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr contact_state_pub_LH_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr contact_state_pub_RH_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr contact_state_pub_RF_;

  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_torque_sub_LF_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_torque_sub_LH_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_torque_sub_RH_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_torque_sub_RF_;

  std::vector<geometry_msgs::msg::WrenchStamped> filtered_force_torque;
  std_msgs::msg::Bool contact_state;
  const int max_count_num;
  int count;
  bool option;
  double norm2;
  std::vector<double> norm;
  double sum_norm;
  double av_norm;
};

#endif  // LBR_CONTACT_DETERMINATION__LBR_CONTACT_DETERMINATION_HPP_
