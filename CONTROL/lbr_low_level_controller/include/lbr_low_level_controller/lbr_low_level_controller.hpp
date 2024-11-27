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

#ifndef LBR_LOW_LEVEL_CONTROLLER__LBR_LOW_LEVEL_CONTROLLER_HPP_
#define LBR_LOW_LEVEL_CONTROLLER__LBR_LOW_LEVEL_CONTROLLER_HPP_

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include "Eigen/Dense"
#include "Eigen/StdVector"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "lbr_msgs/msg/base_motion_task.hpp"
#include "lbr_msgs/msg/end_effector_pose_four_dof.hpp"
#include "lbr_msgs/msg/limb_motion_task.hpp"
#include "lbr_msgs/msg/motion_package.hpp"
#include "lbr_msgs/msg/motion_task.hpp"
#include "lbr_msgs/msg/grieel_mode_change.hpp"
#include <lbr_parameter.hpp>

class LowLevelController : public rclcpp::Node
{
public:
  LowLevelController();

private:
  void commandCallback(const std_msgs::msg::String & command);
  /**
   * @brief This callback function runs when LLC subscribes EE pose from LC.
   *
   * @param EE_pose
   */
  void endEffectorPoseCallback(const lbr_msgs::msg::EndEffectorPoseFourDof & EE_pose);
  /**
   * @brief This function checks error between current EE pose and next EE pose, and update EE pose if the error is small.
   *
   * @param next_EE_pose
   */
  void updateEEPose(const lbr_msgs::msg::EndEffectorPoseFourDof & next_EE_pose);
  /**
   * @brief This callback function runs when LLC subscribes limb motion displacement from HLC.
   *
   * @param limb_motion
   */
  void limbMotionCallback(const lbr_msgs::msg::LimbMotionTask & limb_motion);
  /**
   * @brief This function publishes target EE pose to LC based on limb motion task.
   *
   * @param limb_motion
   */
  void execLimbMotion(const lbr_msgs::msg::LimbMotionTask & limb_motion);
  /**
   * @brief This function publishes target EE pose to LC based on limb motion task array.
   *
   * @param limb_motion_task_array
   */
  void execLimbMotionArray(
    const std::vector<lbr_msgs::msg::LimbMotionTask> & limb_motion_task_array);
  /**
   * @brief This callback function runs when LLC subscribes base motion task from HLC.
   *
   * @param base_motion
   */
  void baseMotionCallback(const lbr_msgs::msg::BaseMotionTask & base_motion);
  /**
   * @brief This function publishes target EE pose to LC based on base motion task.
   *
   * @param base_motion
   */
  void execBaseMotion(const lbr_msgs::msg::BaseMotionTask & base_motion);
  /**
   * @brief This function publishes grieel command to LC based on current mode.
   *
   * @param grieel_mode
   */
  void GrieelModeCallback(const lbr_msgs::msg::GrieelModeChange & grieel_mode);
  /**
   * @brief This function publishes grieel driving configuration to LC.
   *
   * @param driving_configuration
   */
  void grieelDrivingConfigurationCallback(const std_msgs::msg::Int64 & driving_configuration);
  /**
   * @brief This function publishes desired EE pose that current_EE_pose + EE_displacement.
   *
   * @param EE_displacement
   */
  void publishNextEndEffectorPose(const lbr_msgs::msg::EndEffectorPoseFourDof & EE_displacement);
  /**
   * @brief This function calculates rotation transform from Base to Limb coordinate.
   *
   * @param limb_id
   * @param vector_in_base_coordinate
   * @return vector_in_limb_coordinate
   */
  Eigen::Vector3d transformRotationBaseToLimb(
    const int & limb_id,
    const Eigen::Vector3d vector_in_base_coordinate);
  /**
   * @brief This function publishes gripper open/close command to LC.
   *
   * @param limb_id
   * @param close_gripper
   */
  void publishGripperCommand(const int & limb_id, const bool & close_gripper);
  /**
   * @brief Set all EE poses as the Default pose.
   *
   * @param default_EE_pose
   */
  void setDefaultEndEffectorPose(
    const std::vector<lbr_msgs::msg::EndEffectorPoseFourDof> & default_EE_pose);
  /**
   * @brief Convert base rotation (roll, pitch, yaw) to EE motion (x, y, z) in base coordinate.
   *
   * @param limb_id
   * @param base_rotation
   * @return EE_motion
   */
  Eigen::Vector3d baseMotionToEndEffectorMotion(
    const int & limb_id, const Eigen::Vector3d & base_rotation);
  /**
   * @brief Calculate coefficient of 7th order spline
   *
   * @details Please ask [MI]
   *
   * @param initial_position
   * @param final_position
   * @param mid_position
   * @param swing_duration
   * @return spline_coefficient
   */
  Eigen::Vector<double, 8> calculate7orderSplineTrajectory(
    const double & final_position,
    const double & mid_position,
    const int & swing_duration);
  /**
   * @brief This callback function runs when LLC subscribes motion package from HLC.
   *
   * @param motion_package
   */
  void motionPackageCallback(const lbr_msgs::msg::MotionPackage & motion_package);

  rclcpp::TimerBase::SharedPtr timer_;

  // Publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motion_finish_signal_pub_;

  rclcpp::Publisher<lbr_msgs::msg::EndEffectorPoseFourDof>::SharedPtr EE_pose_pub_LF_;
  rclcpp::Publisher<lbr_msgs::msg::EndEffectorPoseFourDof>::SharedPtr EE_pose_pub_LH_;
  rclcpp::Publisher<lbr_msgs::msg::EndEffectorPoseFourDof>::SharedPtr EE_pose_pub_RH_;
  rclcpp::Publisher<lbr_msgs::msg::EndEffectorPoseFourDof>::SharedPtr EE_pose_pub_RF_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_state_pub_LF_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_state_pub_LH_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_state_pub_RH_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_state_pub_RF_;
  // Pub new Grieel mode, sub by LC and dynamixelC
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr grieel_state_pub_LF_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr grieel_state_pub_LH_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr grieel_state_pub_RH_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr grieel_state_pub_RF_;

  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr grieel_driving_mode_pub_LF_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr grieel_driving_mode_pub_LH_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr grieel_driving_mode_pub_RH_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr grieel_driving_mode_pub_RF_;


  // Subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;

  rclcpp::Subscription<lbr_msgs::msg::EndEffectorPoseFourDof>::SharedPtr EE_pose_sub_LF_;
  rclcpp::Subscription<lbr_msgs::msg::EndEffectorPoseFourDof>::SharedPtr EE_pose_sub_LH_;
  rclcpp::Subscription<lbr_msgs::msg::EndEffectorPoseFourDof>::SharedPtr EE_pose_sub_RH_;
  rclcpp::Subscription<lbr_msgs::msg::EndEffectorPoseFourDof>::SharedPtr EE_pose_sub_RF_;

  rclcpp::Subscription<lbr_msgs::msg::BaseMotionTask>::SharedPtr base_motion_sub_;
  rclcpp::Subscription<lbr_msgs::msg::BaseMotionTask>::SharedPtr base_motion_command_sub_;
  rclcpp::Subscription<lbr_msgs::msg::LimbMotionTask>::SharedPtr limb_motion_sub_;
  rclcpp::Subscription<lbr_msgs::msg::LimbMotionTask>::SharedPtr limb_motion_command_sub_;
  rclcpp::Subscription<lbr_msgs::msg::MotionPackage>::SharedPtr motion_package_sub_;
  // Sub from HLC about Grieel transformation request
  rclcpp::Subscription<lbr_msgs::msg::GrieelModeChange>::SharedPtr grieel_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr grieel_driving_configuration_sub_;


  // EE position and pitch angle in Limb coordinate.
  std::vector<lbr_msgs::msg::EndEffectorPoseFourDof> current_EE_pose;
  std::vector<lbr_msgs::msg::EndEffectorPoseFourDof> initial_EE_pose;
  std::vector<lbr_msgs::msg::EndEffectorPoseFourDof> folding_EE_pose;
};

#endif  // LBR_LOW_LEVEL_CONTROLLER__LBR_LOW_LEVEL_CONTROLLER_HPP_
