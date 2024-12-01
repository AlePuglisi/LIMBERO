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

#include "lbr_sim/lbr_sim.hpp"

#define DEBUG_ENABLED false
#define USE_TARGET_JOINT_STATE_IN_RVIZ false

// Supplementary class for signal filtering
// LowPassFilter::LowPassFilter(double cutoff_freq, double sample_freq)
//     : alpha_(1.0 - std::exp(-2.0 * M_PI * cutoff_freq/sample_freq)),
//     initialized_(false) {}

// double LowPassFilter::filter(double input) {
//     if(!initialized_){
//       output_ = input;
//       initialized_ = true;
//     } else {
//       output_ = alpha_ * input + (1.0 - alpha_) * output_;
//     }
//     return output_;
// }

// void LowPassFilter::reset() {
//     initialized_ = false;
// }

// ButterworthFilter::ButterworthFilter(double cutoff_freq, double sample_freq)
//   : b_{}, a_{}, x_{}, y_{} {
//     double w0 = 2.0 * M_PI * cutoff_freq / sample_freq;
//     double alpha = sin(w0) / sqrt(2.0);

//     b_[0] = (1.0 - cos(w0)) / 2.0;
//     b_[1] = 1.0 - cos(w0);
//     b_[2] = b_[0];

//     a_[0] = 1.0 + alpha;
//     a_[1] = -2.0 * cos(w0);
//     a_[2] = 1.0 - alpha;

//     for (int i = 0; i < 3; ++i) {
//         b_[i] /= a_[0];
//         a_[i] /= a_[0];
//     }
//   }

// double ButterworthFilter::filter(double input) {
//     x_[2] = x_[1];
//     x_[1] = x_[0];
//     x_[0] = input;

//     double output = b_[0] * x_[0] + b_[1] * x_[1] + b_[2] * x_[2]
//                     - a_[1] * y_[1] - a_[2] * y_[2];

//     y_[2] = y_[1];
//     y_[1] = output;

//     return output;
// }

// SlewRateLimiter::SlewRateLimiter(double max_rate)
//   : max_rate_(max_rate), last_output_(0){}

// double SlewRateLimiter::limit(double input, double dt) {
//   double rate = (input - last_output_)/dt;
//   rate = std::clamp(rate, -max_rate_, max_rate_);
//   double output = last_output_ + rate*dt;
//   last_output_ = output;
//   return output;
// }




LimberoSim::LimberoSim()
: Node("lbr_sim")
{
  std::cout << "LimberoSim class is established." << std::endl;

  std::string topic_prefix = std::string(this->get_name());
#if DEBUG_ENABLED
  std::cout << "topic_prefix = " << topic_prefix << std::endl;
#endif  // DEBUG_ENABLED

  // timer_ = this->create_wall_timer(
  //   std::chrono::milliseconds(100), std::bind(&LimberoSim::timerCallback, this));

  // Publisher
  encoder_joint_state_pub_LF_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/" + topic_prefix + "/LF/encoder_joint_state", 10);
  encoder_joint_state_pub_LH_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/" + topic_prefix + "/LH/encoder_joint_state", 10);
  encoder_joint_state_pub_RH_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/" + topic_prefix + "/RH/encoder_joint_state", 10);
  encoder_joint_state_pub_RF_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/" + topic_prefix + "/RF/encoder_joint_state", 10);

  joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/joint_trajectory_controller/joint_trajectory", 1);
  whole_end_effector_contact_state_pub_ =
    this->create_publisher<lbr_msgs::msg::EndEffectorContactState>(
    "/" + topic_prefix + "/contact_state", 10);

  // Subscriber
  joint_state_sub_LF_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/LF/joint_states", 10,
    std::bind(&LimberoSim::jointStateCallback, this, std::placeholders::_1));
  joint_state_sub_LH_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/LH/joint_states", 10,
    std::bind(&LimberoSim::jointStateCallback, this, std::placeholders::_1));
  joint_state_sub_RH_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/RH/joint_states", 10,
    std::bind(&LimberoSim::jointStateCallback, this, std::placeholders::_1));
  joint_state_sub_RF_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/RF/joint_states", 10,
    std::bind(&LimberoSim::jointStateCallback, this, std::placeholders::_1));

  target_joint_state_sub_LF_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/LF/lbr_limb_controller/joint_state", 100,
    std::bind(&LimberoSim::targetJointStateCallback, this, std::placeholders::_1));
  target_joint_state_sub_LH_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/LH/lbr_limb_controller/joint_state", 100,
    std::bind(&LimberoSim::targetJointStateCallback, this, std::placeholders::_1));
  target_joint_state_sub_RH_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/RH/lbr_limb_controller/joint_state", 100,
    std::bind(&LimberoSim::targetJointStateCallback, this, std::placeholders::_1));
  target_joint_state_sub_RF_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/RF/lbr_limb_controller/joint_state", 100,
    std::bind(&LimberoSim::targetJointStateCallback, this, std::placeholders::_1));

  end_effector_contact_state_sub_LF_ = this->create_subscription<gazebo_msgs::msg::ContactsState>(
    "/LF/bumper_states", 10,
    std::bind(&LimberoSim::endEffectorContactStateCallback, this, std::placeholders::_1));
  end_effector_contact_state_sub_LH_ = this->create_subscription<gazebo_msgs::msg::ContactsState>(
    "/LH/bumper_states", 10,
    std::bind(&LimberoSim::endEffectorContactStateCallback, this, std::placeholders::_1));
  end_effector_contact_state_sub_RH_ = this->create_subscription<gazebo_msgs::msg::ContactsState>(
    "/RH/bumper_states", 10,
    std::bind(&LimberoSim::endEffectorContactStateCallback, this, std::placeholders::_1));
  end_effector_contact_state_sub_RF_ = this->create_subscription<gazebo_msgs::msg::ContactsState>(
    "/RF/bumper_states", 10,
    std::bind(&LimberoSim::endEffectorContactStateCallback, this, std::placeholders::_1));

  dynamixel_contact_state_sub_LF_ = this->create_subscription<lbr_msgs::msg::SingleEndEffectorContactState>(
    "/LF/lbr_dynamixel_controller/contact_state", 10,
    std::bind(&LimberoSim::dynamixelContactStateCallback, this, std::placeholders::_1));
  dynamixel_contact_state_sub_LH_ = this->create_subscription<lbr_msgs::msg::SingleEndEffectorContactState>(
    "/LH/lbr_dynamixel_controller/contact_state", 10,
    std::bind(&LimberoSim::dynamixelContactStateCallback, this, std::placeholders::_1));
  dynamixel_contact_state_sub_RH_ = this->create_subscription<lbr_msgs::msg::SingleEndEffectorContactState>(
    "/RH/lbr_dynamixel_controller/contact_state", 10,
    std::bind(&LimberoSim::dynamixelContactStateCallback, this, std::placeholders::_1));
  dynamixel_contact_state_sub_RF_ = this->create_subscription<lbr_msgs::msg::SingleEndEffectorContactState>(
    "/RF/lbr_dynamixel_controller/contact_state", 10,
    std::bind(&LimberoSim::dynamixelContactStateCallback, this, std::placeholders::_1));

  // fake_contact_state_sub_ = this->create_subscription<lbr_msgs::msg::EndEffectorContactState>(
  //   "/lbr_high_level_controller/contact_state", 10,
  //   std::bind(&LimberoSim::fakeContactStateCallback, this, std::placeholders::_1));

  encoder_joint_state.resize(LIMB_NUM);
  for (int i = 0; i < LIMB_NUM; i++) {
    encoder_joint_state.at(i).position.resize(JOINT_NUM);
    encoder_joint_state.at(i).velocity.resize(JOINT_NUM);
    encoder_joint_state.at(i).name.resize(JOINT_NUM);
  }
  // LF
  encoder_joint_state.at(0).header.frame_id = "/LF";
  encoder_joint_state.at(0).name.at(0) = "LF_B2C";
  encoder_joint_state.at(0).name.at(1) = "LF_C2F";
  encoder_joint_state.at(0).name.at(2) = "LF_F2T";
  encoder_joint_state.at(0).name.at(3) = "LF_T2E";
  encoder_joint_state.at(0).name.at(4) = "LF_wristH";
  encoder_joint_state.at(0).name.at(5) = "LF_wristV";
  encoder_joint_state.at(0).name.at(6) = "LF_driving";
  // LH
  encoder_joint_state.at(1).header.frame_id = "/LH";
  encoder_joint_state.at(1).name.at(0) = "LH_B2C";
  encoder_joint_state.at(1).name.at(1) = "LH_C2F";
  encoder_joint_state.at(1).name.at(2) = "LH_F2T";
  encoder_joint_state.at(1).name.at(3) = "LH_T2E";
  encoder_joint_state.at(1).name.at(4) = "LH_wristH";
  encoder_joint_state.at(1).name.at(5) = "LH_wristV";
  encoder_joint_state.at(1).name.at(6) = "LH_driving";
  // RH
  encoder_joint_state.at(2).header.frame_id = "/RH";
  encoder_joint_state.at(2).name.at(0) = "RH_B2C";
  encoder_joint_state.at(2).name.at(1) = "RH_C2F";
  encoder_joint_state.at(2).name.at(2) = "RH_F2T";
  encoder_joint_state.at(2).name.at(3) = "RH_T2E";
  encoder_joint_state.at(2).name.at(4) = "RH_wristH";
  encoder_joint_state.at(2).name.at(5) = "RH_wristV";
  encoder_joint_state.at(2).name.at(6) = "RH_driving";
  // RF
  encoder_joint_state.at(3).header.frame_id = "/RF";
  encoder_joint_state.at(3).name.at(0) = "RF_B2C";
  encoder_joint_state.at(3).name.at(1) = "RF_C2F";
  encoder_joint_state.at(3).name.at(2) = "RF_F2T";
  encoder_joint_state.at(3).name.at(3) = "RF_T2E";
  encoder_joint_state.at(3).name.at(4) = "RF_wristH";
  encoder_joint_state.at(3).name.at(5) = "RF_wristV";
  encoder_joint_state.at(3).name.at(6) = "RF_driving";

  lbr_joint_trajectory.points.resize(1);
  lbr_joint_trajectory.joint_names = {
    "LF_B2C", "LF_C2F", "LF_F2T", "LF_T2E", "LF_wristH", "LF_wristV", "LF_driving",
    "LH_B2C", "LH_C2F", "LH_F2T", "LH_T2E", "LH_wristH", "LH_wristV", "LH_driving",
    "RH_B2C", "RH_C2F", "RH_F2T", "RH_T2E", "RH_wristH", "RH_wristV", "RH_driving",
    "RF_B2C", "RF_C2F", "RF_F2T", "RF_T2E", "RF_wristH", "RF_wristV", "RF_driving"};

  lbr_joint_trajectory_point.positions.resize(LIMB_NUM * JOINT_NUM);
  lbr_joint_trajectory_point.velocities.resize(LIMB_NUM * JOINT_NUM);
  lbr_joint_trajectory_point.accelerations.resize(LIMB_NUM * JOINT_NUM);

  whole_end_effector_contact_state.is_contact.resize(LIMB_NUM);

  //double cutoff_freq = 20.0;
  //double sample_freq = 1000.0;
  // double max_rate = 1.0;
  // for (int i=0; i < LIMB_NUM; i++) {
  //   for(int j=0; j < JOINT_NUM; j++) {
  //     joint_limiters_[i][j] = SlewRateLimiter(max_rate);
  //   }
  // }

  // dt = 0.03;
}

void LimberoSim::jointStateCallback(const sensor_msgs::msg::JointState & joint_state)
{
  int limb_id;
  if (joint_state.name.at(0).find("LF") != std::string::npos) {
    limb_id = 0;
  } else if (joint_state.name.at(0).find("LH") != std::string::npos) {
    limb_id = 1;
  } else if (joint_state.name.at(0).find("RH") != std::string::npos) {
    limb_id = 2;
  } else if (joint_state.name.at(0).find("RF") != std::string::npos) {
    limb_id = 3;
  }

  for (int i = 0; i < JOINT_NUM; i++) {
    encoder_joint_state.at(limb_id).position.at(i) = joint_state.position.at(i);
    encoder_joint_state.at(limb_id).velocity.at(i) = joint_state.velocity.at(i);
  }

  encoder_joint_state.at(limb_id).header.stamp = this->now();
#if !USE_TARGET_JOINT_STATE_IN_RVIZ
  if (limb_id == 0) {
    encoder_joint_state_pub_LF_->publish(encoder_joint_state.at(limb_id));
  } else if (limb_id == 1) {
    encoder_joint_state_pub_LH_->publish(encoder_joint_state.at(limb_id));
  } else if (limb_id == 2) {
    encoder_joint_state_pub_RH_->publish(encoder_joint_state.at(limb_id));
  } else if (limb_id == 3) {
    encoder_joint_state_pub_RF_->publish(encoder_joint_state.at(limb_id));
  }
#endif
}

void LimberoSim::targetJointStateCallback(
  const sensor_msgs::msg::JointState & target_joint_state)
{
  int limb_id;
  if (target_joint_state.header.frame_id == "/LF") {
    limb_id = 0;
  } else if (target_joint_state.header.frame_id == "/LH") {
    limb_id = 1;
  } else if (target_joint_state.header.frame_id == "/RH") {
    limb_id = 2;
  } else if (target_joint_state.header.frame_id == "/RF") {
    limb_id = 3;
  }

  lbr_joint_trajectory_point.positions.at(limb_id * JOINT_NUM) = target_joint_state.position.at(0);
  lbr_joint_trajectory_point.positions.at(limb_id * JOINT_NUM + 1) = target_joint_state.position.at(1);
  lbr_joint_trajectory_point.positions.at(limb_id * JOINT_NUM + 2) = target_joint_state.position.at(2);
  lbr_joint_trajectory_point.positions.at(limb_id * JOINT_NUM + 3) = target_joint_state.position.at(3);
  lbr_joint_trajectory_point.positions.at(limb_id * JOINT_NUM + 4) = target_joint_state.position.at(4);
  lbr_joint_trajectory_point.positions.at(limb_id * JOINT_NUM + 5) = target_joint_state.position.at(5);
  lbr_joint_trajectory_point.positions.at(limb_id * JOINT_NUM + 6) = 0.0;

  lbr_joint_trajectory_point.velocities.at(limb_id * JOINT_NUM) = 0.0;
  lbr_joint_trajectory_point.velocities.at(limb_id * JOINT_NUM + 1) = 0.0;
  lbr_joint_trajectory_point.velocities.at(limb_id * JOINT_NUM + 2) = 0.0;
  lbr_joint_trajectory_point.velocities.at(limb_id * JOINT_NUM + 3) = 0.0;
  lbr_joint_trajectory_point.velocities.at(limb_id * JOINT_NUM + 4) = 0.0;
  lbr_joint_trajectory_point.velocities.at(limb_id * JOINT_NUM + 5) = 0.0;
  lbr_joint_trajectory_point.velocities.at(limb_id * JOINT_NUM + 6) = target_joint_state.velocity.at(6);

  lbr_joint_trajectory_point.accelerations.at(limb_id * JOINT_NUM) = 0.0;
  lbr_joint_trajectory_point.accelerations.at(limb_id * JOINT_NUM + 1) = 0.0;
  lbr_joint_trajectory_point.accelerations.at(limb_id * JOINT_NUM + 2) = 0.0;
  lbr_joint_trajectory_point.accelerations.at(limb_id * JOINT_NUM + 3) = 0.0;
  lbr_joint_trajectory_point.accelerations.at(limb_id * JOINT_NUM + 4) = 0.0;
  lbr_joint_trajectory_point.accelerations.at(limb_id * JOINT_NUM + 5) = 0.0;
  lbr_joint_trajectory_point.accelerations.at(limb_id * JOINT_NUM + 6) = 0.0;

  lbr_joint_trajectory_point.time_from_start.sec = 0;
  lbr_joint_trajectory_point.time_from_start.nanosec = 1000000;  // 0.001 s

  lbr_joint_trajectory.points.at(0) = lbr_joint_trajectory_point;

  lbr_joint_trajectory.header.stamp.sec = 0;
  lbr_joint_trajectory.header.stamp.nanosec = 0;

  // low pass filtering
  // for (int i=0; i < JOINT_NUM; i++) {
  //   double filtered_position = joint_filters_[limb_id][i].filter(target_joint_state.position.at(i));
  //   lbr_joint_trajectory_point.positions.at(limb_id * JOINT_NUM + i) = filtered_position;
  // }
  // lbr_joint_trajectory.points.at(0) = lbr_joint_trajectory_point;

  // joint signal rate limiter
  // for (int i=0; i < JOINT_NUM; i++) {
  //   double limited_position = joint_limiters_[limb_id][i].limit(target_joint_state.position.at(i), dt);
  //   lbr_joint_trajectory_point.positions.at(limb_id * JOINT_NUM + i) = limited_position;
  // }
  // lbr_joint_trajectory.points.at(0) = lbr_joint_trajectory_point;

  joint_trajectory_pub_->publish(lbr_joint_trajectory);

#if USE_TARGET_JOINT_STATE_IN_RVIZ
  for (int i = 0; i < JOINT_NUM; i++) {
    encoder_joint_state.at(limb_id).position.at(i) = target_joint_state.position.at(i);
  }

  encoder_joint_state.at(limb_id).header.stamp = this->now();
  if (limb_id == 0) {
    encoder_joint_state_pub_LF_->publish(encoder_joint_state.at(limb_id));
  } else if (limb_id == 1) {
    encoder_joint_state_pub_LH_->publish(encoder_joint_state.at(limb_id));
  } else if (limb_id == 2) {
    encoder_joint_state_pub_RH_->publish(encoder_joint_state.at(limb_id));
  } else if (limb_id == 3) {
    encoder_joint_state_pub_RF_->publish(encoder_joint_state.at(limb_id));
  }
#endif
}

// IF NOT IN SIMULATION WE NEED TO USE THE CURRENT FEEDBACK FROM LBR_DYNAMIXEL CONTROLLER
void LimberoSim::endEffectorContactStateCallback(
  const gazebo_msgs::msg::ContactsState & contact_state)
{
  int limb_id;
  if (contact_state.header.frame_id.find("LF") != std::string::npos) {
    limb_id = 0;
    whole_end_effector_contact_state.header.frame_id = "LF";
  } else if (contact_state.header.frame_id.find("LH") != std::string::npos) {
    limb_id = 1;
    whole_end_effector_contact_state.header.frame_id = "LH";
  } else if (contact_state.header.frame_id.find("RH") != std::string::npos) {
    limb_id = 2;
    whole_end_effector_contact_state.header.frame_id = "RH";
  } else if (contact_state.header.frame_id.find("RF") != std::string::npos) {
    limb_id = 3;
    whole_end_effector_contact_state.header.frame_id = "RF";
  }

  if (contact_state.states.empty() == true) {
    whole_end_effector_contact_state.is_contact.at(limb_id) = false;
  } else if (contact_state.states.empty() == false) {
    whole_end_effector_contact_state.is_contact.at(limb_id) = true;
  }
#if DEBUG_ENABLED
  for (int i = 0; i < LIMB_NUM; i++) {
    std::cout << "whole_end_effector_contact_state.is_contact.at(" << i << ") = " <<
      whole_end_effector_contact_state.is_contact.at(i) << std::endl;
  }
#endif  // DEBUG_ENABLED
  whole_end_effector_contact_state.header.stamp = this->now();
  whole_end_effector_contact_state_pub_->publish(whole_end_effector_contact_state);
}

void LimberoSim::dynamixelContactStateCallback(
  const lbr_msgs::msg::SingleEndEffectorContactState & contact_state)
{
  int limb_id;
  if (contact_state.header.frame_id == "LF") {
    limb_id = 0;
    whole_end_effector_contact_state.header.frame_id = "LF";
  } else if (contact_state.header.frame_id == "LH") {
    limb_id = 1;
    whole_end_effector_contact_state.header.frame_id = "LH";
  } else if (contact_state.header.frame_id == "RH") {
    limb_id = 2;
    whole_end_effector_contact_state.header.frame_id = "RH";
  } else if (contact_state.header.frame_id == "RF") {
    limb_id = 3;
    whole_end_effector_contact_state.header.frame_id = "RF";
  }

  if (contact_state.is_contact == true) {
    whole_end_effector_contact_state.is_contact.at(limb_id) = true;
  } else if (contact_state.is_contact == false) {
    whole_end_effector_contact_state.is_contact.at(limb_id) = false;
  }
#if DEBUG_ENABLED
  for (int i = 0; i < LIMB_NUM; i++) {
    std::cout << "whole_end_effector_contact_state.is_contact.at(" << i << ") = " <<
      whole_end_effector_contact_state.is_contact.at(i) << std::endl;
  }
#endif  // DEBUG_ENABLED
  whole_end_effector_contact_state.header.stamp = this->now();
  whole_end_effector_contact_state_pub_->publish(whole_end_effector_contact_state);
}

// void LimberoSim::fakeContactStateCallback(const lbr_msgs::msg::EndEffectorContactState &whole_contact_state){
//   whole_end_effector_contact_state = whole_contact_state;
// }

// void LimberoSim::timerCallback(){
//   if(whole_end_effector_contact_state.is_contact.size() == 0){
//     whole_end_effector_contact_state.is_contact.at(0) = true;
//     whole_end_effector_contact_state.is_contact.at(1) = true;
//     whole_end_effector_contact_state.is_contact.at(2) = true;
//     whole_end_effector_contact_state.is_contact.at(3) = true;
//   }
//   whole_end_effector_contact_state.header.stamp = this->now();
//   whole_end_effector_contact_state_pub_->publish(whole_end_effector_contact_state);
// }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LimberoSim>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
