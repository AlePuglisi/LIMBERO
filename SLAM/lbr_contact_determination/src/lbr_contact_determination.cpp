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

#include "lbr_contact_determination/lbr_contact_determination.hpp"

#define DEBUG_ENABLED false

ContactDetermination::ContactDetermination()
: Node("lbr_contact_determination"), max_count_num(50)
{
  std::cout << "ContactDetermination class is established." << std::endl;

  contact_state_pub_LF_ = this->create_publisher<std_msgs::msg::Bool>(
    "/LF/lbr_contact_determination/contact_state", 1);
  contact_state_pub_LH_ = this->create_publisher<std_msgs::msg::Bool>(
    "/LH/lbr_contact_determination/contact_state", 1);
  contact_state_pub_RH_ = this->create_publisher<std_msgs::msg::Bool>(
    "/RH/lbr_contact_determination/contact_state", 1);
  contact_state_pub_RF_ = this->create_publisher<std_msgs::msg::Bool>(
    "/RF/lbr_contact_determination/contact_state", 1);

  force_torque_sub_LF_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/LF/filtered_force_torque", 10,
    std::bind(&ContactDetermination::leptrinoCallback, this, std::placeholders::_1));
  force_torque_sub_LH_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/LH/filtered_force_torque", 10,
    std::bind(&ContactDetermination::leptrinoCallback, this, std::placeholders::_1));
  force_torque_sub_RH_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/RH/filtered_force_torque", 10,
    std::bind(&ContactDetermination::leptrinoCallback, this, std::placeholders::_1));
  force_torque_sub_RF_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/RF/filtered_force_torque", 10,
    std::bind(&ContactDetermination::leptrinoCallback, this, std::placeholders::_1));

  filtered_force_torque.resize(LIMB_NUM);
  norm.resize(max_count_num);
}

void ContactDetermination::leptrinoCallback(const geometry_msgs::msg::WrenchStamped force_torque)
{
  int limb_id;
  if (force_torque.header.frame_id == "/LF/leptrino") {
    limb_id = 0;
  } else if (force_torque.header.frame_id == "/LH/leptrino") {
    limb_id = 1;
  } else if (force_torque.header.frame_id == "/RH/leptrino") {
    limb_id = 2;
  } else if (force_torque.header.frame_id == "/RF/leptrino") {
    limb_id = 3;
  } else {
    std::cerr << "ERROR: limb_id is not defined." << std::endl;
    std::abort();
  }

  filtered_force_torque.at(limb_id) = force_torque;  // Force [N], Torque [Nm]

  // Synthetic force norm squared
  norm2 =
    pow(filtered_force_torque.at(limb_id).wrench.force.x, 2) +
    pow(filtered_force_torque.at(limb_id).wrench.force.y, 2) +
    pow(filtered_force_torque.at(limb_id).wrench.force.z, 2);

  // 50 FT data averaged
  if (option == true) {
    norm.at(count) = sqrt(norm2);  //  Synthetic force norm [N]
    count++;

    if (count >= max_count_num) {
      for (int i = 0; i < max_count_num; i++) {
        sum_norm += norm.at(i);
      }
      av_norm = sum_norm / static_cast<double>(max_count_num);
#if DEBUG_ENABLED
      std::cout << "av_norm = " << av_norm << std::endl;
#endif  // DEBUG_ENABLED

      // detect contact
      if (av_norm > 2.5) {  // [N], contact determination
#if DEBUG_ENABLED
        std::cout << "Contacted." << std::endl;
#endif  // DEBUG_ENABLED
        contact_state.data = true;
      } else {
#if DEBUG_ENABLED
        std::cout << "Not contacted." << std::endl;
#endif  // DEBUG_ENABLED
        contact_state.data = false;
      }

      count = 0;
      sum_norm = 0.0;

      publishContactStates(contact_state, filtered_force_torque.at(limb_id));
    }
  } else if (option == false) {  // initialize count
    option = true;
    count = 0;
  }
}

void ContactDetermination::publishContactStates(
  const std_msgs::msg::Bool contact_states,
  const geometry_msgs::msg::WrenchStamped filtered_force_torque)
{
  if (filtered_force_torque.header.frame_id == "/LF/leptrino") {
    contact_state_pub_LF_->publish(contact_states);
  } else if (filtered_force_torque.header.frame_id == "/LH/leptrino") {
    contact_state_pub_LH_->publish(contact_states);
  } else if (filtered_force_torque.header.frame_id == "/RH/leptrino") {
    contact_state_pub_RH_->publish(contact_states);
  } else if (filtered_force_torque.header.frame_id == "/RF/leptrino") {
    contact_state_pub_RF_->publish(contact_states);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ContactDetermination>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
