#ifndef LBR_INDIPENDENT_JOINT_CONTROLLER__LBR_INDIPENDENT_JOINT_CONTROLLER_HPP_
#define LBR_INDIPENDENT_JOINT_CONTROLLER__LBR_INDIPENDENT_JOINT_CONTROLLER_HPP_

#include <chrono>
#include <cstdio>
#include <string>
#include <math.h>

#include <iostream>
#include <fstream>
#include <array>
#include <memory>
#include <functional>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "rclcpp/rclcpp.hpp"
#include <lbr_parameter.hpp>
#include "lbr_indipendent_joint_controller/lbr_control_parameter.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "gazebo_msgs/msg/contacts_state.hpp"
#include "lbr_msgs/msg/end_effector_contact_state.hpp"


class IndipendentJointController : public rclcpp::Node
{
public:
  IndipendentJointController();

  std::fstream file_out;

private:

  /**
   * @brief This function runs periodically when called by the control_loop_timer_.
   *        It publish the torque control action to the effort_controller.
   */
  void controlLoop();
  /**
   * @brief This function is used to compute gravutational torque, used for feed forward gravity compensation in the control loop
   *
   */
  std::array<float, JOINT_NUM> GravityCompensation();
  /**
   * @brief This callback function runs when IJC subscribes current joint state from Gazebo.
   *
   * @param EE_pose_msg
   */
  void currentJointStateCallback(
    const sensor_msgs::msg::JointState & joint_state);
  /**
   * @brief This callback function runs when IJC subscribes target joint state from LC.
   *
   * @param EE_pose_msg
   */
  void referenceJointStateCallback(
    const sensor_msgs::msg::JointState & joint_state);

  /**
   * @brief This callback function runs when Bumber sensor publish contact state.
   *
   * @param EE_contact_state
   */
  void endEffectorContactStateCallback(const gazebo_msgs::msg::ContactsState & contact_state);

  /**
   * @brief This callback function runs when lbr_sim publish contact state of all limbs.
   *
   * @param EE_contact_state
   */
  void allEndEffectorContactStateCallback(const lbr_msgs::msg::EndEffectorContactState & contact_state);

  // Timer
  rclcpp::TimerBase::SharedPtr control_loop_timer_; // sets digital controller sampling time Ts

  //  Publisher
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_control_pub_; // computed control output torque

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_reference_sub_; // received LC set points
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr current_joint_state_sub_; // received joint state from Gazebo node
  rclcpp::Subscription<lbr_msgs::msg::EndEffectorContactState>::SharedPtr whole_contact_state_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr end_effector_contact_state_sub_;

  // attributes
  int LIMB_ID; // LF=0, LH=1, RH=2, RF=3, identify IJC related limb
  int Ts; // controller period (fs = 1/Ts) [ms]

  float Kpp[JOINT_NUM]; // position controller P gain
  float Kpv[JOINT_NUM]; // velocity controller P gain
  float Tiv[JOINT_NUM]; // velocity controller D gain
  float torque_limit[JOINT_NUM]; // Saturation limit of joint motors

  float N[JOINT_NUM]; // reduction ratio

  float joint_velocity_feed_forward[JOINT_NUM]; // feed forward velocity, to speed up control
  float previous_reference_joint_position[JOINT_NUM]; // store previous reference for numerical derivation
  float previous_reference_joint_position_filtered[JOINT_NUM]; // previous filtered set point
  float previous_velocity_error[JOINT_NUM]; // previous error in velocity tracking
  float previous_saturated_torque[JOINT_NUM]; // previous value of torque after saturation
  float previous_u1[JOINT_NUM]; // additional variable for desaturation anti-windup
  float previous_u2[JOINT_NUM]; // additional variable for desaturation anti-windup
  float reference_joint_position_filtered[JOINT_NUM]; // filtered set point to avoid abrupt controller action

  float integral[JOINT_NUM]; //keep track of cumulative integrator
  float anti_wind_up[JOINT_NUM]; // additional anti-wind up action

  float Tc;  // anti-wind up feedback gain, for compensation
  float Tf;  // pre-filter time constant
  float Tdf; // derivative high frequency pole, for LP filtering (relaizability)

  int limb_in_contact;  // number of contact limb
  int swing_counter;    // counter to avoid false non contact detection
  bool in_contact;      // check if leg in contact
  bool start_control;   // flag variabl related to robot spawning in Gazebo environment

  // gravity compensation related variable
  std::array<float, JOINT_NUM> gravitational_torque;
  int gravity_compensation;

  double initial_time; // initial time data, for data saving on csv file

  std::string joint_prefix;


  // messages updated by controller
  sensor_msgs::msg::JointState current_joint_state; // store current joint state from gazebo
  sensor_msgs::msg::JointState reference_joint_state; // store current joint reference from LC
  std_msgs::msg::Float64MultiArray torque_control_output; // store torque control values
};

#endif  // LBR_INDIPENDENT_JOINT_CONTROLLER__LBR_INDIPENDENT_JOINT_CONTROLLER_HPP_