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

#include "lbr_grasping_guarantee/lbr_grasping_guarantee.hpp"

GraspingGuarantee::GraspingGuarantee()
: Node("lbr_grasping_guarantee")
{
  std::cout << "GraspingGuarantee class is established." << std::endl;

  msg_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/user_command", 1);

  grasping_detect_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/lbr_low_level_controller/grasping_detect", 1,
    std::bind(&GraspingGuarantee::graspCallback, this, std::placeholders::_1));
  grasping_detect_pitch_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/lbr_low_level_controller/grasping_detect_pitch", 1,
    std::bind(&GraspingGuarantee::graspPitchCallback, this, std::placeholders::_1));
  grasping_trace_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/lbr_low_level_controller/grasping_trace", 1,
    std::bind(&GraspingGuarantee::traceCallback, this, std::placeholders::_1));
  // TODO(KT): use topic prefix instead of LF for the other limbs.
  force_torque_sub_LF_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/LF/filtered_force_torque", 10,
    std::bind(&GraspingGuarantee::leptrinoCallback, this, std::placeholders::_1));

  guarantee_option = 0;
  count = 0;
}

// After motion for grasping determination
void GraspingGuarantee::graspCallback(const std_msgs::msg::Bool grasp_detect)
{
  if (grasp_detect.data == true) {
    guarantee_option = 1;
  }
}

// After motion for grasping determination of pitch
void GraspingGuarantee::graspPitchCallback(const std_msgs::msg::Bool grasp_detect_pitch)
{
  if (grasp_detect_pitch.data == true) {
    guarantee_option = 2;
  }
}

// After motion for tracing grasp
void GraspingGuarantee::traceCallback(const std_msgs::msg::Bool grasp_trace)
{
  if (grasp_trace.data == true) {
    guarantee_option = 3;
  }
}

void GraspingGuarantee::leptrinoCallback(
  const geometry_msgs::msg::WrenchStamped force_torque)
{
  filtered_force_torque = force_torque;  // Force [N], Torque [Nm]

  switch (guarantee_option) {
    case 1:  // grasping guarantee of z
      graspingDetermination(filtered_force_torque);
      break;
    case 2:  // grasping guarantee of pitch
      graspingPitchDetermination(filtered_force_torque);
      break;
    case 3:  // tracing grasp
      tracingGrasp(filtered_force_torque);
      break;
    default:
      std::cout << "guarantee_option = " << guarantee_option << std::endl;
      break;
  }

  guarantee_option = 4;
}

// Detect tracing grasp
void GraspingGuarantee::tracingGrasp(const geometry_msgs::msg::WrenchStamped ft_data)
{
  // Synthetic force norm squared
  norm_xy2 = pow(ft_data.wrench.force.x, 2) + pow(ft_data.wrench.force.y, 2);
  norm_xy = sqrt(norm_xy2);  // Synthetic force norm [N]

  if (count > 3) {  // Fifth tracing fail
    std::cout << "Tracing fail." << std::endl;
  } else {
    const double norm_xy_threshold = 1.0;  // [N]
    if (norm_xy > norm_xy_threshold) {
      std::cout << "Tracing success." << std::endl;
      motion_msg.data = "grasp_detect";
      msg_pub_->publish(motion_msg);  // to High Level Controller
    } else {
      if (trace_direction_is_x == false) {
        if (ft_data.wrench.force.y < 0.0) {
          std::cout << "Tracing now." << std::endl;
          motion_msg.data = "tracing_grasp_plus_y";
          msg_pub_->publish(motion_msg);  // to High Level Controller
          count++;
          trace_direction_is_x = true;
        } else {
          std::cout << "Tracing now." << std::endl;
          motion_msg.data = "tracing_grasp_minus_y";
          msg_pub_->publish(motion_msg);  // to High Level Controller
          count++;
          trace_direction_is_x = true;
        }
      } else if (trace_direction_is_x == true) {
        if (ft_data.wrench.force.x < 0.0) {
          std::cout << "Tracing now." << std::endl;
          motion_msg.data = "tracing_grasp_plus_x";
          msg_pub_->publish(motion_msg);  // to High Level Controller
          count++;
          trace_direction_is_x = false;
        } else {
          std::cout << "Tracing now." << std::endl;
          motion_msg.data = "tracing_grasp_minus_x";
          msg_pub_->publish(motion_msg);  // to High Level Controller
          count++;
          trace_direction_is_x = false;
        }
      }
    }
  }
}

// Detect grasping of z
void GraspingGuarantee::graspingDetermination(
  const geometry_msgs::msg::WrenchStamped ft_data)
{
  const double force_threshold = 1.6;  // [N]
  if (ft_data.wrench.force.z > force_threshold) {  // Use norm of z force [N]
    std::cout << "Grasping success." << std::endl;
    motion_msg.data = "grasp_detect_return";
    msg_pub_->publish(motion_msg);  // to High Level Controller
  } else {
    std::cout << "Grasping fail." << std::endl;
  }
}

// Detect grasping of pitch
void GraspingGuarantee::graspingPitchDetermination(
  const geometry_msgs::msg::WrenchStamped ft_data)
{
  const double torque_threshold = 0.01;  // [Nm]
  if (ft_data.wrench.torque.y > torque_threshold) {  // Use pitch torque (around y axis) [Nm]
    std::cout << "Grasping pitch success." << std::endl;
  } else {
    std::cout << "Grasping pitch fail." << std::endl;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GraspingGuarantee>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
