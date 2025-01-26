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

#ifndef LBR_STATE_ESTIMATOR__LBR_STATE_ESTIMATOR_HPP_
#define LBR_STATE_ESTIMATOR__LBR_STATE_ESTIMATOR_HPP_

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include "Eigen/Dense"
#include "Eigen/StdVector"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "lbr_msgs/msg/end_effector_pose_four_dof.hpp"
#include <lbr_parameter.hpp>
#include "lbr_msgs/msg/end_effector_contact_state.hpp"

// #include "tf2_ros/transform_broadcaster.h"
// #include "geometry_msgs/msg/transform_stamped.hpp"
// #include "nav_msgs/msg/odometry.hpp"


class StateEstimator : public rclcpp::Node
{
public:
  StateEstimator();

private:
  void baseImuCallback(const sensor_msgs::msg::Imu & imu_raw_data);
  void commandCallback(const std_msgs::msg::String & command);
  void encoderJointStateCallback(const sensor_msgs::msg::JointState & encoder_joint_state);
  void grieelSimRuntimeUpdateJointStateCallback(const sensor_msgs::msg::JointState & grieel_sim_runtime_update_joint_state);
  void dynamixelTemperatureCallback(const std_msgs::msg::Int64MultiArray & temperature);
  /**
   * @brief This callback function runs when State Estimator subscribes wrench (force and torque) from leptrino.
   *
   * @param force_torque
   */
  void leptrinoForceTorqueCallback(const geometry_msgs::msg::WrenchStamped & force_torque);
  /**
   * @brief This function is used to calculated the offset when open or close, any timing.
   *
   * @param raw_force_torque
   * @param filter_number
   */
  void calculateOffsetForceTorque(
    const geometry_msgs::msg::WrenchStamped raw_force_torque,
    const int filter_number);
  /**
   * @brief This function is used that the force values are zero for desired seconds when opening and closing.
   *
   * @param force_torque
   * @param time
   */
  void offsetForSomeSeconds(const geometry_msgs::msg::WrenchStamped force_torque, int time);
  void publishForceTorque(
    const geometry_msgs::msg::WrenchStamped raw_force_torque,
    const Eigen::Vector3d average_force);
  void endEffectorContactStateCallback(
    const lbr_msgs::msg::EndEffectorContactState & contact_state);
  lbr_msgs::msg::EndEffectorPoseFourDof forwardKinematics(
    const std::vector<sensor_msgs::msg::JointState> & joint_state, const int & limb_id);
  /**
   * @brief Calculate supporting leg polygon based on contact information.
   */
  void calculateSupportingLegPolygon();
  void markerInitialization();
  // void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Publisher
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr imu_accel_pub_;  // Publish to RViz
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    supporting_leg_triangle_pub_;  // Publish to RViz

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_torque_pub_LF_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_torque_pub_LH_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_torque_pub_RH_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_torque_pub_RF_;
  rclcpp::Publisher<lbr_msgs::msg::EndEffectorContactState>::SharedPtr
    supporting_leg_triangle_points_pub_;

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr base_imu_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr encoder_joint_state_sub_LF_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr encoder_joint_state_sub_LH_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr encoder_joint_state_sub_RH_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr encoder_joint_state_sub_RF_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gazebo_encoder_joint_state_sub_LF_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gazebo_encoder_joint_state_sub_LH_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gazebo_encoder_joint_state_sub_RH_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gazebo_encoder_joint_state_sub_RF_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr rviz_joint_state_sub_LF_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr rviz_joint_state_sub_LH_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr rviz_joint_state_sub_RH_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr rviz_joint_state_sub_RF_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr grieel_sim_runtime_update_joint_state_sub_;

  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr dynamixel_temperature_sub_LF_;
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr dynamixel_temperature_sub_LH_;
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr dynamixel_temperature_sub_RH_;
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr dynamixel_temperature_sub_RF_;

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr leptrino_force_torque_sub_LF_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr leptrino_force_torque_sub_LH_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr leptrino_force_torque_sub_RH_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr leptrino_force_torque_sub_RF_;

  rclcpp::Subscription<lbr_msgs::msg::EndEffectorContactState>::SharedPtr
    end_effector_contact_state_sub_;

  // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  odom_subscription_;

  std::vector<sensor_msgs::msg::JointState> lbr_joint_state;
  std::vector<lbr_msgs::msg::EndEffectorPoseFourDof> lbr_EE_pose;
  std::vector<geometry_msgs::msg::WrenchStamped> gripper_force_torque;
  std::vector<bool> lbr_gripper_is_open;  // true -> open, false -> close

  sensor_msgs::msg::Imu base_imu_raw_data;
  sensor_msgs::msg::JointState all_joint_state;
  std_msgs::msg::Int64MultiArray dynamixel_temperature;
  visualization_msgs::msg::Marker imu_accel_arrow;
  lbr_msgs::msg::EndEffectorContactState lbr_end_effector_contact_state;
  lbr_msgs::msg::EndEffectorContactState supporting_leg_triangle_points;
  lbr_msgs::msg::EndEffectorPoseFourDof contact_EE_pose_in_limb_coordinate;

  geometry_msgs::msg::WrenchStamped filtered_force_torque;
  geometry_msgs::msg::WrenchStamped filtered_open_force_torque;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  // std::shared_ptr<tf2_ros::TransformBroadcaster>tf_broadcaster_;

  bool close_option;
  bool open_state;
  bool close_state;
  bool open_option;
  bool any_option;
  int count;
  const int max_count_num;
  int option;
  int offset_condition;

  // bool base_link_found; 

  std::vector<Eigen::Vector3d,
  Eigen::aligned_allocator<Eigen::Vector3d>> force;
  Eigen::Vector3d sum_force;
  Eigen::Vector3d average_force;
  Eigen::Vector3d average_open_force;
  Eigen::Vector3d average_close_force;
  visualization_msgs::msg::Marker supporting_leg_triangle;
};

#endif  // LBR_STATE_ESTIMATOR__LBR_STATE_ESTIMATOR_HPP_
