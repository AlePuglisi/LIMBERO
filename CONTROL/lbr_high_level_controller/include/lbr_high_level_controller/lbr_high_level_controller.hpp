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

#ifndef LBR_HIGH_LEVEL_CONTROLLER__LBR_HIGH_LEVEL_CONTROLLER_HPP_
#define LBR_HIGH_LEVEL_CONTROLLER__LBR_HIGH_LEVEL_CONTROLLER_HPP_

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "Eigen/StdVector"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int64.hpp"
#include "lbr_msgs/msg/base_motion_task.hpp"
#include "lbr_msgs/msg/end_effector_pose_four_dof.hpp"
#include "lbr_msgs/msg/limb_motion_task.hpp"
#include "lbr_msgs/msg/motion_package.hpp"
#include "lbr_msgs/msg/motion_task.hpp"
#include "lbr_msgs/msg/end_effector_contact_state.hpp"
#include "lbr_msgs/msg/grieel_mode_change.hpp"
#include <lbr_parameter.hpp>

constexpr double LIMB_TRANSFORM_HEIGHT = 0.2;
constexpr double LIMB_TRANSFORM_DOWN_GRIPPER = 0.2;
constexpr double LIMB_TRANSFORM_DOWN_WHEEL = 0.15;

class HighLevelController : public rclcpp::Node
{
public:
  HighLevelController();

private:
  /**
   * @brief This callback function runs when HLC subscribes user command by ros2 topic pub.
   *
   * @details ros2 topic pub --once /user_command std_msgs/msg/String '{data: test}'
   *
   * @param msg
   */
  void userCommandCallback(const std_msgs::msg::String & msg);
    /**
   * @brief This callback function runs when HLC subscirbes to interface command from lbr_command_interface.
   *        Accordigly to received msg it starts an high level task.
   *
   * @param msg
   */
  void interfaceCommandCallback(const std_msgs::msg::String & msg);
  /**
   * @brief This function generate motion package for leg <limb_id> grieel mode transformation.
   *  It create base motion to stable position (using current support polygon), limb raise and grieel transition request
   *
   * @param
   */
  void singleLegGrieelTransformation(
    const lbr_msgs::msg::EndEffectorContactState & supporting_leg_polygon,
    int limb_id);
  /**
   * @brief This function generate motion package for all grieel mode transformation.
   *
   * @param
   */
  void grieelTransformation();

  /**
   * @brief This function generates motion package for crawl gait and publish it to LLC.
   *
   * @param move_direction
   */
  void generateCrawlGait(const geometry_msgs::msg::Twist & move_direction);
  /**
   * @brief This function generates motion package for trot gait and publish it to LLC.
   *
   * @param move_direction
   */
  void generateTrotGait(const geometry_msgs::msg::Twist & move_direction);
  /**
   * @brief This callback subscribes finish signal of executing motion package from LLC.
   *
   * @param signal
   */
  void motionFinishSignalCallback(const std_msgs::msg::String & signal);
  /**
   * @brief Determine if the base_link is contained within the triangle,
   *        and if not, move the position of the base_link to the center of gravity of the triangle.
   *
   * @param supporting_leg_polygon
   */
  void stabilizeBaseInSupportingLegsTriangle(
    const lbr_msgs::msg::EndEffectorContactState
    & supporting_leg_polygon);
  /**
   * @brief Determine if all 4 foot are in contact with the ground,
   *        and if yes, move the position of the base_link to the center of gravity of the polygon.
   *        Usefull after a transition to go back to a stable kinematic configuration
   * @param supporting_leg_polygon
   */
  void centerBase(
    const lbr_msgs::msg::EndEffectorContactState
    & supporting_leg_polygon);
  /**
   * @brief This callback function runs when HLC subscribes the contact state of the end effector.
   *
   * @param msg
   */
  void endEffectorContactStateCallback(const lbr_msgs::msg::EndEffectorContactState & msg);
  /**
   * @brief Determine if it is contained within a triangle.
   *
   * @param base_link
   * @param p1
   * @param p2
   * @param p3
   * @return true if base_link is inside the triangle.
   */
  bool isInsideTriangle(
    const Eigen::Vector3d & base_link, const Eigen::Vector3d & p1,
    const Eigen::Vector3d & p2, const Eigen::Vector3d & p3);
  /**
   * @brief This callback function runs when HLC subscribes to dynamixel when grieel set 0 velocity.
   * ( meaning that the transition is finished after the required transformation time)
   *
   * @param grieel_finish_string
   */
  void grieelFinishCallback(const std_msgs::msg::String & grieel_finish_string);

  // Publisher
  rclcpp::Publisher<lbr_msgs::msg::BaseMotionTask>::SharedPtr base_motion_pub_;
  rclcpp::Publisher<lbr_msgs::msg::LimbMotionTask>::SharedPtr limb_motion_pub_;
  rclcpp::Publisher<lbr_msgs::msg::MotionPackage>::SharedPtr motion_package_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr grasping_detect_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr grasping_trace_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr grasping_detect_pitch_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr msg_pub_;
  // Pub to send request for Grieel transition (Sub by low_level_controller)
  rclcpp::Publisher<lbr_msgs::msg::GrieelModeChange>::SharedPtr grieel_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr grieel_driving_configuration_pub_;

  // rclcpp::Publisher<lbr_msgs::msg::EndEffectorContactState>::SharedPtr fake_end_effector_contact_state_pub_;

  // Subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr user_command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_finish_signal_sub_;
  rclcpp::Subscription<lbr_msgs::msg::EndEffectorContactState>::SharedPtr
    supporting_leg_triangle_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_interface_sub_;

  // Sub to know when Grieel Transition has finished
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grieel_transform_end_sub_LF_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grieel_transform_end_sub_LH_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grieel_transform_end_sub_RH_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grieel_transform_end_sub_RF_;

  // Variables
  bool trot_gait_is_executing;
  bool motion_package_end;
  bool grieel_transformation_;
  bool single_transform_end_;
  std::string grieel_state_;
  lbr_msgs::msg::EndEffectorContactState supporting_leg_polygon;

  // lbr_msgs::msg::EndEffectorContactState fake_contact;
};

#endif  // LBR_HIGH_LEVEL_CONTROLLER__LBR_HIGH_LEVEL_CONTROLLER_HPP_