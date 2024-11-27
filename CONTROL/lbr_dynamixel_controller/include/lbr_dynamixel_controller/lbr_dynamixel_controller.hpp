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

#ifndef LBR_DYNAMIXEL_CONTROLLER__LBR_DYNAMIXEL_CONTROLLER_HPP_
#define LBR_DYNAMIXEL_CONTROLLER__LBR_DYNAMIXEL_CONTROLLER_HPP_

#include <chrono>
#include <thread>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "lbr_msgs/msg/grieel_mode_change.hpp"
#include "lbr_msgs/msg/single_end_effector_contact_state.hpp"
#include "std_msgs/msg/string.hpp"
#include <lbr_parameter.hpp>


// Control table address (Dynamixel XM430-W350)
#define ADDR_TORQUE_ENABLE        64
#define ADDR_LED                  65
#define ADDR_POSITION_D_GAIN      80
#define ADDR_POSITION_I_GAIN      82
#define ADDR_POSITION_P_GAIN      84
#define ADDR_GOAL_CURRENT        102
#define ADDR_PROFILE_VELOCITY    112
#define ADDR_GOAL_POSITION       116
#define ADDR_GOAL_VELOCITY       104
#define ADDR_PRESENT_CURRENT     126
#define ADDR_PRESENT_VELOCITY    128
#define ADDR_PRESENT_POSITION    132
#define ADDR_PRESENT_TEMPERATURE 146
// Data Byte Length
#define LEN_PRESENT_TEMPERATURE    1
#define LEN_PRESENT_CURRENT        2
// Protocol version
#define PROTOCOL_VERSION         2.0  // Protocol version of DYNAMIXEL X series.
// Baudrate setting
#define BAUDRATE             1000000  // Default Baudrate of DYNAMIXEL X series is 57600 bps.

struct PIDGains {
  std::string joint;
  int kp;
  int ki;
  int kd;
};

class DynamixelController : public rclcpp::Node
{
public:
  DynamixelController();
  ~DynamixelController();

private:
  void setPositionCallback(const sensor_msgs::msg::JointState & input_joint_state);
  void setVelocityCallback(const std_msgs::msg::String & grieel_mode_change);
  bool setGoalVelocity(int goal_velocity_input, int id);
  void grippingCallback(const std_msgs::msg::Bool & execute_grasping);
  void timerCallback();
  void secondTimerCallback();
  void contactCallback();
  void enableTorque(bool enable_torque);
  void enableLED(bool enable_LED);
  void setPIDGains();
  void closePort();
  /**
   * @brief This function checks angles and if one of them exceeds its limit, stop the node.
   *
   * @param joint_state
   */
  void checkAngleLimit(const sensor_msgs::msg::JointState & joint_state);
  int angleToDynamixelPositionValue(float angle, int joint_id);
  /**
   * @brief convert Dynamixel position value into radian value.
   *
   * @param joint_id
   * @param dynamixel_position
   * @return angle
   */
  double dynamixelPositionToAngle(const int & joint_id, const int & dynamixel_position);
  /**
   * @brief convert Dynamixel velocity value into [rad/s] value.
   *
   * @param joint_id
   * @param dynamixel_velocity
   * @return angular_velocity
   */
  double dynamixelVelocityToAngularVelocity(const int & joint_id, const int & dynamixel_velocity);
  /**
   * @brief convert Dynamixel current value into current value in [mA].
   *
   * @param joint_id
   * @param dynamixel_current
   * @return current
   */
  double convertDynamixelCurrent(const int & joint_id, const int & dynamixel_current);
  /**
   * @brief Set the Velocity Limit of Dynamixel. Setting zero value means no velocity limit.
   *
   * @param velocity_limit_input
   */
  void setVelocityLimit(int velocity_limit_input);
  /**
   * @brief Read Dynamixel encoder positions and return true when finish the process.
   *
   * @param dynamixel_position
   * @return true
   * @return false
   */
  bool readPosition(std::vector<int> & dynamixel_position);
  /**
   * @brief Read Dynamixel encoder velocities and return true when finish the process.
   *
   * @param dynamixel_velocity
   * @return true
   * @return false
   */
  bool readVelocity(std::vector<int> & dynamixel_velocity);
  /**
   * @brief Read Dynamixel current and current value value and return true when finish the process.
   *
   * @param dynamixel_current, current
   * @return true
   * @return false
   */
  bool readCurrent(std::vector<int> & dynamixel_current, std::vector<int> & current);
  /**
   * @brief Read Dynamixel temperature and return true when finish the process.
   *
   * @param dynamixel_temperature
   * @return true
   * @return false
   */
  bool readTemperature(std_msgs::msg::Int64MultiArray & dynamixel_temperature);

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr encoder_joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr dynamixel_temperature_pub_;
  // Pub to have feedback on Grieel mode transition end, feedback sub by HLC
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr grieel_transform_end_pub_;
  // Pub to simulate a conact sensor using T2E joint current, sent to lbr_sim for contact state
  rclcpp::Publisher<lbr_msgs::msg::SingleEndEffectorContactState>::SharedPtr contact_state_pub_;


  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripping_sub_;
  // Sub to grieel transform request from LLC (simultaneusly to Grieel joints mode transition for now..)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grieel_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr second_timer_;
  rclcpp::TimerBase::SharedPtr contact_timer_;

  // Variables
  std::string device_name;
  std::vector<int> dynamixel_position;
  std::vector<int> dynamixel_velocity;
  std::vector<int> dynamixel_current;
  std::vector<int> current;

  // DYNAMIXEL_OFFSET_F2T/T2G are confirmed on SolidWorks
  // for initial position when F2T-T2G is 200 mm.
  const double DYNAMIXEL_OFFSET_F2T;  // [rad]
  const double DYNAMIXEL_OFFSET_T2G;
  /**
   * @brief Assigned ID for the first Dynamixel of each limb.
   */
  int INITIAL_DYNAMIXEL_ID;
  int limb_id;

  sensor_msgs::msg::JointState encoder_joint_state;
  std_msgs::msg::Int64MultiArray dynamixel_temperature;
  std::string limb_namespace;

  dynamixel::PortHandler * portHandler;
  dynamixel::PacketHandler * packetHandler;
};

#endif  // LBR_DYNAMIXEL_CONTROLLER__LBR_DYNAMIXEL_CONTROLLER_HPP_
