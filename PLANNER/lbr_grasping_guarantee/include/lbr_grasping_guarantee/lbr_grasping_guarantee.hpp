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

#ifndef LBR_GRASPING_GUARANTEE__LBR_GRASPING_GUARANTEE_HPP_
#define LBR_GRASPING_GUARANTEE__LBR_GRASPING_GUARANTEE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

class GraspingGuarantee : public rclcpp::Node
{
public:
  GraspingGuarantee();

private:
  /**
   * @brief This callback function runs when Grasping Guarantee subscribes bool from Low Level Controller.
   *
   * @param grasp_detect
   */
  void graspCallback(const std_msgs::msg::Bool grasp_detect);
  /**
   * @brief This callback function runs when Grasping Guarantee subscribes bool from Low Level Controller.
   *
   * @param grasp_detect_pitch
   */
  void graspPitchCallback(const std_msgs::msg::Bool grasp_detect_pitch);
  /**
   * @brief This callback function runs when Grasping Guarantee subscribes bool from Low Level Controller.
   *
   * @param grasp_trace
   */
  void traceCallback(const std_msgs::msg::Bool grasp_trace);
  /**
   * @brief This callback function runs when Contact determination subscribes wrench (filtered force and torque) from State Estimator.
   *
   * @param force_torque
   */
  void leptrinoCallback(const geometry_msgs::msg::WrenchStamped force_torque);
  /**
   * @brief This function detect if tracing is successful
   *
   * @param ft_data
   */
  void tracingGrasp(const geometry_msgs::msg::WrenchStamped ft_data);
  /**
   * @brief This grasping determination of z if tracing is successful
   *
   * @param ft_data
   */
  void graspingDetermination(const geometry_msgs::msg::WrenchStamped ft_data);
  /**
   * @brief This grasping determination of pitch if tracing is successful
   *
   * @param ft_data
   */
  void graspingPitchDetermination(const geometry_msgs::msg::WrenchStamped ft_data);

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr msg_pub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grasping_detect_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grasping_detect_pitch_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grasping_trace_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_torque_sub_LF_;

  geometry_msgs::msg::WrenchStamped filtered_force_torque;
  std_msgs::msg::String motion_msg;
  bool grasp_detect_start;
  bool grasp_detect_pitch_start;
  bool grasp_trace_start;
  bool trace_direction_is_x;
  double norm_xy;
  double norm_xy2;
  int count;
  int guarantee_option;
};

#endif  // LBR_GRASPING_GUARANTEE__LBR_GRASPING_GUARANTEE_HPP_
