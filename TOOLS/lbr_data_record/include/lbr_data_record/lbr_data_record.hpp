#ifndef LBR_DATA_RECORD__LBR_DATA_RECORD_HPP_
#define LBR_DATA_RECORD__LBR_DATA_RECORD_HPP_

#include <chrono>
#include <iostream>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <lbr_parameter.hpp>

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"

#include "rclcpp/rclcpp.hpp"

class DataRecord : public rclcpp::Node
{
public: 
    DataRecord();
    std::fstream file_out_LF;
    std::fstream file_out_LH;
    std::fstream file_out_RH;
    std::fstream file_out_RF;

    std::fstream* file_out_list[LIMB_NUM];

private:
  /**
   * @brief This callback function runs when data_record subscribes to "custom_joint_trajectory_controller/controller_state".
   *
   * @param JointState
   */
  void controllerStateCallback(
    const control_msgs::msg::JointTrajectoryControllerState & controller_state);

  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr controller_state_sub_; // received LC set points

};

#endif