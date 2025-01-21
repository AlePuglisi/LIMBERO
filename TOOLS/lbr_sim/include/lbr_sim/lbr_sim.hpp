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

#ifndef LBR_SIM__LBR_SIM_HPP_
#define LBR_SIM__LBR_SIM_HPP_

#include <vector>
#include <array>
#include <cmath>
#include <cstdlib> // For std::system

//#include "gazebo_msgs/msg/contacts_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "lbr_msgs/msg/end_effector_contact_state.hpp"
#include "lbr_msgs/msg/grieel_joint_task.hpp"
#include "lbr_msgs/msg/single_end_effector_contact_state.hpp"
#include <lbr_parameter.hpp>

// class LowPassFilter{
// public:
//   LowPassFilter() : alpha_(0), output_(0), initialized_(false) {}
//   LowPassFilter(double cutoff_freq, double sample_freq);
//   double filter(double input);
//   void reset();

// private:
//     double alpha_;
//     double output_;
//     bool initialized_;
// };

// class ButterworthFilter{
// public:
//     ButterworthFilter() : b_{0,0,0}, a_{0,0,0}, x_{0,0,0}, y_{0,0,0} {}
//     ButterworthFilter(double cutoff_freq, double sample_freq);
//     double filter(double input);

// private:
//     std::array<double, 3> b_;
//     std::array<double, 3> a_;
//     std::array<double, 3> x_;
//     std::array<double, 3> y_;
// };

// class SlewRateLimiter {
// public:
//     SlewRateLimiter() : max_rate_(0), last_output_(0) {}
//     SlewRateLimiter(double max_rate);

//     double limit(double input, double dt);

// private:
//     double max_rate_;
//     double last_output_;
// };


class LimberoSim : public rclcpp::Node
{
public:
  LimberoSim();

private:
  void jointStateCallback(const sensor_msgs::msg::JointState & joint_state);
  void targetJointStateCallback(const sensor_msgs::msg::JointState & target_joint_state);
  void endEffectorContactStateCallback_LF(const std_msgs::msg::Bool & contact_state);
  void endEffectorContactStateCallback_LH(const std_msgs::msg::Bool & contact_state);
  void endEffectorContactStateCallback_RH(const std_msgs::msg::Bool & contact_state);
  void endEffectorContactStateCallback_RF(const std_msgs::msg::Bool & contact_state);
  void dynamixelContactStateCallback(const lbr_msgs::msg::SingleEndEffectorContactState & contact_state);
  void checkingContact_LF();
  void checkingContact_LH();
  void checkingContact_RH();
  void checkingContact_RF();
  // void timerCallback();
  // void fakeContactStateCallback(const lbr_msgs::msg::EndEffectorContactState &whole_contact_state);

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr encoder_joint_state_pub_LF_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr encoder_joint_state_pub_LH_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr encoder_joint_state_pub_RH_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr encoder_joint_state_pub_RF_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
  rclcpp::Publisher<lbr_msgs::msg::EndEffectorContactState>::SharedPtr
    whole_end_effector_contact_state_pub_;

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_LF_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_LH_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_RH_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_RF_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_joint_state_sub_LF_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_joint_state_sub_LH_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_joint_state_sub_RH_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_joint_state_sub_RF_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    end_effector_contact_state_sub_LF_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    end_effector_contact_state_sub_LH_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    end_effector_contact_state_sub_RH_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    end_effector_contact_state_sub_RF_;

  rclcpp::Subscription<lbr_msgs::msg::SingleEndEffectorContactState>::SharedPtr
    dynamixel_contact_state_sub_LF_;
  rclcpp::Subscription<lbr_msgs::msg::SingleEndEffectorContactState>::SharedPtr
    dynamixel_contact_state_sub_LH_;
  rclcpp::Subscription<lbr_msgs::msg::SingleEndEffectorContactState>::SharedPtr
    dynamixel_contact_state_sub_RH_;
  rclcpp::Subscription<lbr_msgs::msg::SingleEndEffectorContactState>::SharedPtr
    dynamixel_contact_state_sub_RF_;

  // rclcpp::Subscription<lbr_msgs::msg::EndEffectorContactState>::SharedPtr
  //   fake_contact_state_sub_;

  // Variable
  std::vector<sensor_msgs::msg::JointState> encoder_joint_state;
  trajectory_msgs::msg::JointTrajectory lbr_joint_trajectory;
  trajectory_msgs::msg::JointTrajectoryPoint lbr_joint_trajectory_point;
  lbr_msgs::msg::EndEffectorContactState whole_end_effector_contact_state;
  lbr_msgs::msg::SingleEndEffectorContactState end_effector_contact_state;
  //std::array<std::array<LowPassFilter, JOINT_NUM>, LIMB_NUM> joint_filters_;
  //std::array<std::array<SlewRateLimiter, JOINT_NUM>, LIMB_NUM> joint_limiters_;
  //double dt; // signal interval
  // rclcpp::TimerBase::SharedPtr timer_;

  bool checking_contact_LF; 
  int count_contact_LF; 
  bool checking_contact_LH; 
  int count_contact_LH; 
  bool checking_contact_RH; 
  int count_contact_RH; 
  bool checking_contact_RF; 
  int count_contact_RF; 

};

#endif  // LBR_SIM__LBR_SIM_HPP_