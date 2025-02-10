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

#ifndef LBR_LIMB_CONTROLLER__LBR_LIMB_CONTROLLER_HPP_
#define LBR_LIMB_CONTROLLER__LBR_LIMB_CONTROLLER_HPP_

#include <chrono>
#include <thread>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "lbr_msgs/msg/end_effector_pose_four_dof.hpp"
#include <lbr_parameter.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include "control_msgs/action/gripper_command.hpp"

class LimbController : public rclcpp::Node
{
public:
  LimbController();

private:
  void timerCallback();
  /**
   * @brief This callback function runs when LC subscribes target EE pose from LLC.
   *
   * @param EE_pose_msg
   */
  void endEffectorPoseCallback(
    const lbr_msgs::msg::EndEffectorPoseFourDof & EE_pose_msg);
  void encoderJointStateCallback(
    const sensor_msgs::msg::JointState & encoder_joint_state);
  /**
   * @brief This function checks error between current joint state and next joint state, and update joint state if the error is small.
   *
   * @param next_joint_state
   * @return true
   * @return false
   */
  bool updateJointState(const sensor_msgs::msg::JointState & next_joint_state);
  /**
   * @brief This callback function runs when LC subscribes gripper command from LLC.
   *
   * @param execute_grasping
   */
  void gripperCommandCallback(const std_msgs::msg::Bool & execute_grasping);
    /**
   * @brief This callback function runs when LC subscribes grieel change command from LLC.
   *
   * @param new_grieel_mode
   */
  void grieelChangeCallback(const std_msgs::msg::String & new_grieel_mode);
  /**
   * @brief This callback function runs when LC subscribes grieel ristH driving mode message from LLC
   *
   * @param driving_mode
   */
  void drivingModeCallback(const std_msgs::msg::Int64 & driving_mode);
  /**
   * @brief This function calculates EE pose from given joint angles.
   *
   * @param joint_state
   * @return EE_pose
   */
  lbr_msgs::msg::EndEffectorPoseFourDof forwardKinematics(
    const sensor_msgs::msg::JointState & joint_state);
  /**
   * @brief This function calculates joint angles from given EE pose.
   *
   * @param EE_pose
   * @return joint_state
   */
  sensor_msgs::msg::JointState inverseKinematics(
    const lbr_msgs::msg::EndEffectorPoseFourDof & EE_pose);
  /**
   * @brief This function returns true if joint_state does not have NaN.
   *
   * @param joint_state
   * @return true
   * @return false
   */
  bool checkInverseKinematicsSolution(const sensor_msgs::msg::JointState & joint_state);
  bool checkJointLimit(const sensor_msgs::msg::JointState & joint_state);
  void publishGripperCommand(const std_msgs::msg::Bool execute_grasping);
  void publishEndEffectorTrajectory();
  std::array<float, 6> computeFifthOrderTraj(float q0, float Dq, float T);
    
  void sendGripperCommand(double position, double max_effort);
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_action_client_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr rviz_joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripping_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr gripper_state_text_pub_;
  rclcpp::Publisher<lbr_msgs::msg::EndEffectorPoseFourDof>::SharedPtr EE_pose_pub_;
  rclcpp::Publisher<lbr_msgs::msg::EndEffectorPoseFourDof>::SharedPtr estimated_EE_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    target_position_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    trajectory_array_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr grieel_joint_finish_pub_;

  // Subscriber
  rclcpp::Subscription<lbr_msgs::msg::EndEffectorPoseFourDof>::SharedPtr EE_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_state_sub_;
  //Sub to new Grieel mode request from LLC
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grieel_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr encoder_joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr driving_mode_sub_;

  // Variables
  std::vector<double> LOWER_LIMIT;
  std::vector<double> UPPER_LIMIT;
  sensor_msgs::msg::JointState current_joint_state;
  lbr_msgs::msg::EndEffectorPoseFourDof current_EE_pose;
  geometry_msgs::msg::WrenchStamped leptrino_force_torque;
  visualization_msgs::msg::Marker target_EE_position;
  visualization_msgs::msg::MarkerArray EE_trajectory_array;
  int LIMB_ID;
  std::string joint_prefix;
  std::string grieel_mode; // store current grieel mode (needed in inv kin to keep correct Grieel joints)
  bool driving;
  bool stop;
  int direction; 
  bool enable_rviz_debug;


};

#endif  // LBR_LIMB_CONTROLLER__LBR_LIMB_CONTROLLER_HPP_
