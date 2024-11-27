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

#include "lbr_state_estimator/lbr_state_estimator.hpp"

#define DEBUG_ENABLED false

// TODO(Limb)
// IMU
// TODO(RN): LiDAR
// TODO(ME): Contact Estimation
// GIA

StateEstimator::StateEstimator()
: Node("lbr_state_estimator"), max_count_num(50)
{
  std::cout << "StateEstimator class is established." << std::endl;

  // Get namespace and node name.
  std::string topic_prefix = "/" + std::string(this->get_name());

  // Publisher
  imu_accel_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    topic_prefix + "/imu_accel", 1);
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    topic_prefix + "/joint_state", 1);
  supporting_leg_triangle_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    topic_prefix + "/supporting_leg_triangle", 1);

  supporting_leg_triangle_points_pub_ = this->create_publisher
    <lbr_msgs::msg::EndEffectorContactState>(
    topic_prefix + "/supporting_leg_triangle_points", 1);

  force_torque_pub_LF_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "/LF/filtered_force_torque", 10);
  force_torque_pub_LH_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "/LH/filtered_force_torque", 10);
  force_torque_pub_RH_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "/RH/filtered_force_torque", 10);
  force_torque_pub_RF_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "/RF/filtered_force_torque", 10);

  // Subscriber
  base_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data_raw", 1,
    std::bind(&StateEstimator::baseImuCallback, this, std::placeholders::_1));
  command_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/lbr_command_interface/command/state_estimator", 3,
    std::bind(&StateEstimator::commandCallback, this, std::placeholders::_1));

  encoder_joint_state_sub_LF_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/LF/lbr_dynamixel_controller/encoder_joint_state", 1,
    std::bind(&StateEstimator::encoderJointStateCallback, this, std::placeholders::_1));
  encoder_joint_state_sub_LH_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/LH/lbr_dynamixel_controller/encoder_joint_state", 1,
    std::bind(&StateEstimator::encoderJointStateCallback, this, std::placeholders::_1));
  encoder_joint_state_sub_RH_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/RH/lbr_dynamixel_controller/encoder_joint_state", 1,
    std::bind(&StateEstimator::encoderJointStateCallback, this, std::placeholders::_1));
  encoder_joint_state_sub_RF_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/RF/lbr_dynamixel_controller/encoder_joint_state", 1,
    std::bind(&StateEstimator::encoderJointStateCallback, this, std::placeholders::_1));

  // For Gazebo simulation
  gazebo_encoder_joint_state_sub_LF_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/lbr_sim/LF/encoder_joint_state", 1,
    std::bind(&StateEstimator::encoderJointStateCallback, this, std::placeholders::_1));
  gazebo_encoder_joint_state_sub_LH_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/lbr_sim/LH/encoder_joint_state", 1,
    std::bind(&StateEstimator::encoderJointStateCallback, this, std::placeholders::_1));
  gazebo_encoder_joint_state_sub_RH_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/lbr_sim/RH/encoder_joint_state", 1,
    std::bind(&StateEstimator::encoderJointStateCallback, this, std::placeholders::_1));
  gazebo_encoder_joint_state_sub_RF_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/lbr_sim/RF/encoder_joint_state", 1,
    std::bind(&StateEstimator::encoderJointStateCallback, this, std::placeholders::_1));

  // For RViz debug
  rviz_joint_state_sub_LF_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/LF/lbr_limb_controller/rviz_joint_state", 1,
    std::bind(&StateEstimator::encoderJointStateCallback, this, std::placeholders::_1));
  rviz_joint_state_sub_LH_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/LH/lbr_limb_controller/rviz_joint_state", 1,
    std::bind(&StateEstimator::encoderJointStateCallback, this, std::placeholders::_1));
  rviz_joint_state_sub_RH_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/RH/lbr_limb_controller/rviz_joint_state", 1,
    std::bind(&StateEstimator::encoderJointStateCallback, this, std::placeholders::_1));
  rviz_joint_state_sub_RF_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/RF/lbr_limb_controller/rviz_joint_state", 1,
    std::bind(&StateEstimator::encoderJointStateCallback, this, std::placeholders::_1));

  grieel_sim_runtime_update_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/grieel_sim_runtime_update/joint_state", 1,
    std::bind(&StateEstimator::grieelSimRuntimeUpdateJointStateCallback, this, std::placeholders::_1));

  dynamixel_temperature_sub_LF_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
    "/LF/lbr_dynamixel_controller/temperature", 1,
    std::bind(&StateEstimator::dynamixelTemperatureCallback, this, std::placeholders::_1));
  dynamixel_temperature_sub_LH_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
    "/LH/lbr_dynamixel_controller/temperature", 1,
    std::bind(&StateEstimator::dynamixelTemperatureCallback, this, std::placeholders::_1));
  dynamixel_temperature_sub_RH_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
    "/RH/lbr_dynamixel_controller/temperature", 1,
    std::bind(&StateEstimator::dynamixelTemperatureCallback, this, std::placeholders::_1));
  dynamixel_temperature_sub_RF_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
    "/RF/lbr_dynamixel_controller/temperature", 1,
    std::bind(&StateEstimator::dynamixelTemperatureCallback, this, std::placeholders::_1));

  leptrino_force_torque_sub_LF_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/LF/force_torque", 1,
    std::bind(&StateEstimator::leptrinoForceTorqueCallback, this, std::placeholders::_1));
  leptrino_force_torque_sub_LH_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/LH/force_torque", 1,
    std::bind(&StateEstimator::leptrinoForceTorqueCallback, this, std::placeholders::_1));
  leptrino_force_torque_sub_RH_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/RH/force_torque", 1,
    std::bind(&StateEstimator::leptrinoForceTorqueCallback, this, std::placeholders::_1));
  leptrino_force_torque_sub_RF_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/RF/force_torque", 1,
    std::bind(&StateEstimator::leptrinoForceTorqueCallback, this, std::placeholders::_1));

  end_effector_contact_state_sub_ =
    this->create_subscription<lbr_msgs::msg::EndEffectorContactState>(
    "/lbr_sim/contact_state", 1,
    std::bind(&StateEstimator::endEffectorContactStateCallback, this, std::placeholders::_1));

  lbr_joint_state.resize(LIMB_NUM);
  for (int i = 0; i < LIMB_NUM; i++) {
    lbr_joint_state.at(i).position.resize(JOINT_NUM);
  }
  lbr_EE_pose.resize(LIMB_NUM);
  gripper_force_torque.resize(LIMB_NUM);
  lbr_gripper_is_open.resize(LIMB_NUM);
  dynamixel_temperature.data.resize(LIMB_NUM * (JOINT_NUM * GRIPPER_MOTOR_NUM));

  all_joint_state.position.resize(LIMB_NUM * JOINT_NUM);
  all_joint_state.velocity.resize(LIMB_NUM * JOINT_NUM);
  all_joint_state.name.resize(LIMB_NUM * JOINT_NUM);
  all_joint_state.name.at(0) = "LF_B2C";
  all_joint_state.name.at(1) = "LF_C2F";
  all_joint_state.name.at(2) = "LF_F2T";
  all_joint_state.name.at(3) = "LF_T2E";
  all_joint_state.name.at(4) = "LF_wristH";
  all_joint_state.name.at(5) = "LF_wristV";
  all_joint_state.name.at(6) = "LF_driving";
  all_joint_state.name.at(7) = "LH_B2C";
  all_joint_state.name.at(8) = "LH_C2F";
  all_joint_state.name.at(9) = "LH_F2T";
  all_joint_state.name.at(10) = "LH_T2E";
  all_joint_state.name.at(11) = "LH_wristH";
  all_joint_state.name.at(12) = "LH_wristV";
  all_joint_state.name.at(13) = "LH_driving";
  all_joint_state.name.at(14) = "RH_B2C";
  all_joint_state.name.at(15) = "RH_C2F";
  all_joint_state.name.at(16) = "RH_F2T";
  all_joint_state.name.at(17) = "RH_T2E";
  all_joint_state.name.at(18) = "RH_wristH";
  all_joint_state.name.at(19) = "RH_wristV";
  all_joint_state.name.at(20) = "RH_driving";
  all_joint_state.name.at(21) = "RF_B2C";
  all_joint_state.name.at(22) = "RF_C2F";
  all_joint_state.name.at(23) = "RF_F2T";
  all_joint_state.name.at(24) = "RF_T2E";
  all_joint_state.name.at(25) = "RF_wristH";
  all_joint_state.name.at(26) = "RF_wristV";
  all_joint_state.name.at(27) = "RF_driving";

  lbr_end_effector_contact_state.is_contact.resize(LIMB_NUM);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  markerInitialization();

  force.resize(max_count_num);
}

void StateEstimator::markerInitialization()
{
  imu_accel_arrow.header.frame_id = "base_link";
  imu_accel_arrow.id = 0;
  imu_accel_arrow.pose.position.x = 0.0;
  imu_accel_arrow.pose.position.y = 0.0;
  imu_accel_arrow.pose.position.z = 0.0;
  imu_accel_arrow.scale.x = 0.5;
  imu_accel_arrow.scale.y = imu_accel_arrow.scale.z = 0.01;
  imu_accel_arrow.color.r = 1.0;
  imu_accel_arrow.color.g = imu_accel_arrow.color.b = 0.0;
  imu_accel_arrow.color.a = 0.5;
  imu_accel_arrow.type = visualization_msgs::msg::Marker::ARROW;
  imu_accel_arrow.action = visualization_msgs::msg::Marker::ADD;
  imu_accel_arrow.lifetime = rclcpp::Duration::from_seconds(0.0);

  supporting_leg_triangle.header.frame_id = "base_link";
  supporting_leg_triangle.ns = "supporting_leg_triangle_list";
  supporting_leg_triangle.id = 0;
  supporting_leg_triangle.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  supporting_leg_triangle.pose.position.x = 0.0;
  supporting_leg_triangle.pose.position.y = 0.0;
  supporting_leg_triangle.pose.position.z = 0.0;
  supporting_leg_triangle.scale.x = supporting_leg_triangle.scale.y =
    supporting_leg_triangle.scale.z = 1.0;
  supporting_leg_triangle.color.r = 0.0;
  supporting_leg_triangle.color.g = 0.5;
  supporting_leg_triangle.color.b = 0.5;
  supporting_leg_triangle.color.a = 0.5;
  supporting_leg_triangle.frame_locked = true;
}

// TODO(RN): implement filter such as EKF.
void StateEstimator::baseImuCallback(const sensor_msgs::msg::Imu & imu_raw_data)
{
  base_imu_raw_data = imu_raw_data;

  // TODO(KT): This is tentative offset value for debug.
  double temp_offset_x = -0.5;
  double temp_offset_y = 0.85;
  double temp_offset_z = -9.5;

  base_imu_raw_data.linear_acceleration.x -= temp_offset_x;
  base_imu_raw_data.linear_acceleration.y -= temp_offset_y;
  base_imu_raw_data.linear_acceleration.z -= temp_offset_z;

  // Use tf2 class to obtain orientation.
  tf2::Vector3 imu_accel_vec;
  imu_accel_vec.setX(base_imu_raw_data.linear_acceleration.x);
  imu_accel_vec.setY(base_imu_raw_data.linear_acceleration.y);
  imu_accel_vec.setZ(base_imu_raw_data.linear_acceleration.z);

  double scale = 0.1;
  imu_accel_arrow.scale.x = scale * sqrt(
    pow(base_imu_raw_data.linear_acceleration.x, 2) +
    pow(base_imu_raw_data.linear_acceleration.y, 2) +
    pow(base_imu_raw_data.linear_acceleration.z, 2));

  imu_accel_arrow.pose.orientation.w = imu_accel_vec.w();
  imu_accel_arrow.pose.orientation.x = imu_accel_vec.x();
  imu_accel_arrow.pose.orientation.y = imu_accel_vec.y();
  imu_accel_arrow.pose.orientation.z = imu_accel_vec.z();
  imu_accel_pub_->publish(imu_accel_arrow);
}

void StateEstimator::encoderJointStateCallback(
  const sensor_msgs::msg::JointState & encoder_joint_state)
{
#if DEBUG_ENABLED
  std::cout << "encoder_joint_state.header.frame_id = " <<
    encoder_joint_state.header.frame_id << std::endl;
#endif  // DEBUG_ENABLED
  int limb_id;
  if (encoder_joint_state.header.frame_id == "/LF") {
    limb_id = 0;
  } else if (encoder_joint_state.header.frame_id == "/LH") {
    limb_id = 1;
  } else if (encoder_joint_state.header.frame_id == "/RH") {
    limb_id = 2;
  } else if (encoder_joint_state.header.frame_id == "/RF") {
    limb_id = 3;
  }
  lbr_joint_state.at(limb_id) = encoder_joint_state;

  for (int i = 0; i < JOINT_NUM; i++) {
    all_joint_state.position.at(JOINT_NUM * limb_id + i) = encoder_joint_state.position.at(i);
  }
  all_joint_state.velocity.at(JOINT_NUM*limb_id + 6) = encoder_joint_state.velocity.at(6);
  joint_state_pub_->publish(all_joint_state);

  // TODO(KT): Implement joint current margin.

  lbr_EE_pose.at(limb_id) = forwardKinematics(lbr_joint_state, limb_id);
}

void StateEstimator::grieelSimRuntimeUpdateJointStateCallback(
  const sensor_msgs::msg::JointState & grieel_sim_runtime_update_joint_state)
{
  all_joint_state.position = grieel_sim_runtime_update_joint_state.position;
  joint_state_pub_->publish(all_joint_state);
}

void StateEstimator::dynamixelTemperatureCallback(
  const std_msgs::msg::Int64MultiArray & temperature)
{
  // TODO(KT): Distinguish each limb
  dynamixel_temperature = temperature;
}

void StateEstimator::leptrinoForceTorqueCallback(
  const geometry_msgs::msg::WrenchStamped & force_torque)
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
  }
  gripper_force_torque.at(limb_id) = force_torque;  // Force[N], Torque[Nm]

  switch (offset_condition) {
    case 0:  // filtered FT value when open
      // TODO(KT): FT sensor place is changed so do not need consider open/close condition.
      calculateOffsetForceTorque(gripper_force_torque.at(limb_id), 1);
      break;
    case 1:  // filtered FT value when any motion
      calculateOffsetForceTorque(gripper_force_torque.at(limb_id), 2);
      break;
    case 2:
      offsetForSomeSeconds(gripper_force_torque.at(limb_id), 2);
      option = 0;
      offset_condition = 5;
      break;
    case 3:
      offsetForSomeSeconds(gripper_force_torque.at(limb_id), 3);
      option = 1;
      offset_condition = 5;
      break;
    case 4:  // filtered FT value when close
      calculateOffsetForceTorque(gripper_force_torque.at(limb_id), 0);
      break;
    default:
      break;
  }

  if (option == 0) {
    // publish filtered FT value when close
    publishForceTorque(
      gripper_force_torque.at(limb_id), average_close_force);
  } else if (option == 1) {
    // publish filtered FT value when open
    publishForceTorque(
      gripper_force_torque.at(limb_id), average_open_force);
  } else if (option == 2) {
    // publish filtered FT value when any motion
    publishForceTorque(gripper_force_torque.at(limb_id), average_force);
  } else {
    publishForceTorque(
      gripper_force_torque.at(limb_id), Eigen::Vector3d::Zero());  // before filter
    count = 0;
  }
#if DEBUG_ENABLED
  std::cout << "leptrinoForceTorqueCallback" << std::endl;
  std::cout << "force_torque.header.frame_id = " << force_torque.header.frame_id << std::endl;
  std::cout << "limb_id = " << limb_id << std::endl;
#endif  // DEBUG_ENABLED
}

void StateEstimator::calculateOffsetForceTorque(
  const geometry_msgs::msg::WrenchStamped raw_force_torque,
  const int filter_number)
{
  force.at(count)(0) = raw_force_torque.wrench.force.x;
  force.at(count)(1) = raw_force_torque.wrench.force.y;
  force.at(count)(2) = raw_force_torque.wrench.force.z;
  count++;

  if (count >= max_count_num) {
    for (int i = 0; i < max_count_num; i++) {
      sum_force += force.at(i);
    }
    offset_condition = 5;

    if (filter_number == 0) {
      average_close_force = sum_force / static_cast<double>(max_count_num);
      option = 0;
    } else if (filter_number == 1) {
      average_open_force = sum_force / static_cast<double>(max_count_num);
      option = 1;
    } else if (filter_number == 2) {
      average_force = sum_force / static_cast<double>(max_count_num);
      option = 2;
    }

    count = 0;
    sum_force.setZero();
  }
}

void StateEstimator::offsetForSomeSeconds(
  const geometry_msgs::msg::WrenchStamped force_torque,
  int time)  // Force values are zero for some seconds
{
  auto start_time = std::chrono::steady_clock::now();
  while (true) {
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_time =
      std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);  // [sec]
    if (elapsed_time.count() >= time) {
      break;
    }
    filtered_force_torque = force_torque;

    filtered_force_torque.wrench.force.x = 0.0;
    filtered_force_torque.wrench.force.y = 0.0;
    filtered_force_torque.wrench.force.z = 0.0;

    if (force_torque.header.frame_id == "/LF/leptrino") {
      force_torque_pub_LF_->publish(filtered_force_torque);
    } else if (force_torque.header.frame_id == "/LH/leptrino") {
      force_torque_pub_LH_->publish(filtered_force_torque);
    } else if (force_torque.header.frame_id == "/RH/leptrino") {
      force_torque_pub_RH_->publish(filtered_force_torque);
    } else if (force_torque.header.frame_id == "/RF/leptrino") {
      force_torque_pub_RF_->publish(filtered_force_torque);
    }
  }
}

void StateEstimator::publishForceTorque(
  const geometry_msgs::msg::WrenchStamped raw_force_torque,
  const Eigen::Vector3d average_force)
{
  filtered_force_torque = raw_force_torque;

  filtered_force_torque.wrench.force.x = raw_force_torque.wrench.force.x - average_force(0);  // [N]
  filtered_force_torque.wrench.force.y = raw_force_torque.wrench.force.y - average_force(1);  // [N]
  filtered_force_torque.wrench.force.z = raw_force_torque.wrench.force.z - average_force(2);  // [N]

  if (raw_force_torque.header.frame_id == "/LF/leptrino") {
    force_torque_pub_LF_->publish(filtered_force_torque);
  } else if (raw_force_torque.header.frame_id == "/LH/leptrino") {
    force_torque_pub_LH_->publish(filtered_force_torque);
  } else if (raw_force_torque.header.frame_id == "/RH/leptrino") {
    force_torque_pub_RH_->publish(filtered_force_torque);
  } else if (raw_force_torque.header.frame_id == "/RF/leptrino") {
    force_torque_pub_RF_->publish(filtered_force_torque);
  }
}

// filter F/T value by joy controller
void StateEstimator::commandCallback(const std_msgs::msg::String & command)
{
  if (command.data == "offset_condition_4") {
    offset_condition = 4;
  } else if (command.data == "offset_condition_2") {
    offset_condition = 2;
  } else if (command.data == "offset_condition_3") {
    offset_condition = 3;
  } else if (command.data == "offset_condition_0") {
    offset_condition = 0;
  }
}

// TODO(KT): Merge FK function in LC.
lbr_msgs::msg::EndEffectorPoseFourDof StateEstimator::forwardKinematics(
  const std::vector<sensor_msgs::msg::JointState> & joint_state, const int & limb_id)
{
  const double theta1 = joint_state.at(limb_id).position.at(0);
  const double theta2 = joint_state.at(limb_id).position.at(1);
  const double theta3 = joint_state.at(limb_id).position.at(2);
  const double theta4 = joint_state.at(limb_id).position.at(3);

  // Forward Kinematics (using [rad] and [m])
  double pitch = theta2 + theta3 + theta4;
  double x = cos(theta1) * (L1 + L2 * cos(theta2) + L3 * cos(theta2 + theta3) + L4 * cos(pitch));
  double y = sin(theta1) * (L1 + L2 * cos(theta2) + L3 * cos(theta2 + theta3) + L4 * cos(pitch));
  double z = -L2 * sin(theta2) - L3 * sin(theta2 + theta3) - L4 * sin(pitch);

  lbr_msgs::msg::EndEffectorPoseFourDof EE_pose;
  EE_pose.end_effector_position.x = x;
  EE_pose.end_effector_position.y = y;
  EE_pose.end_effector_position.z = z;
  EE_pose.end_effector_pitch_angle = pitch;

#if DEBUG_ENABLED
  std::cout << "EE_pose.end_effector_position.x = " << EE_pose.end_effector_position.x << std::endl;
  std::cout << "EE_pose.end_effector_position.y = " << EE_pose.end_effector_position.y << std::endl;
  std::cout << "EE_pose.end_effector_position.z = " << EE_pose.end_effector_position.z << std::endl;
  std::cout << "EE_pose.end_effector_pitch_angle = " <<
    EE_pose.end_effector_pitch_angle << std::endl;
#endif  // DEBUG_ENABLED

  return EE_pose;
}

void StateEstimator::endEffectorContactStateCallback(
  const lbr_msgs::msg::EndEffectorContactState & contact_state)
{
  lbr_end_effector_contact_state = contact_state;

  calculateSupportingLegPolygon();
}

void StateEstimator::calculateSupportingLegPolygon()
{
  Eigen::VectorXi supporting_limb_numbers(3, 1);
  // Limb names forming a supporting leg triangle
  std::vector<std::string> supporting_limb_names;
  std::vector<int> supporting_limb_id;

  // Conditions for contact state
  bool all_limbs_are_contacted = false;
  bool LF_is_swing_limb = false;
  bool LH_is_swing_limb = false;
  bool RH_is_swing_limb = false;
  bool RF_is_swing_limb = false;

  int contact_limb_num = 0;
  for (int limb_id = 0; limb_id < LIMB_NUM; limb_id++) {
    if (lbr_end_effector_contact_state.is_contact.at(limb_id) == true) {
      contact_limb_num++;
    }
  }

  if (contact_limb_num == LIMB_NUM) {
    all_limbs_are_contacted = true;
  } else if (contact_limb_num == 3) {
    if (lbr_end_effector_contact_state.is_contact.at(0) == false) {
      LF_is_swing_limb = true;
    } else if (lbr_end_effector_contact_state.is_contact.at(1) == false) {
      LH_is_swing_limb = true;
    } else if (lbr_end_effector_contact_state.is_contact.at(2) == false) {
      RH_is_swing_limb = true;
    } else if (lbr_end_effector_contact_state.is_contact.at(3) == false) {
      RF_is_swing_limb = true;
    }
  }
#if DEBUG_ENABLED
  std::cout << "LF_is_swing_limb = " << LF_is_swing_limb << std::endl;
  std::cout << "LH_is_swing_limb = " << LH_is_swing_limb << std::endl;
  std::cout << "RH_is_swing_limb = " << RH_is_swing_limb << std::endl;
  std::cout << "RF_is_swing_limb = " << RF_is_swing_limb << std::endl;
#endif  // DEBUG_ENABLED

  bool supporting_leg_triangle_is_formed = false;
  if (all_limbs_are_contacted == true) {
    supporting_limb_numbers.resize(4, 1);
    supporting_limb_numbers << 0, 1, 2, 3;
    supporting_limb_names = {"LF", "LH", "RH", "RF"};
    supporting_leg_triangle_is_formed = true;
  } else if (LF_is_swing_limb == true) {
    supporting_limb_numbers << 1, 3, 2;
    supporting_limb_id = {1, 3, 2};
    supporting_limb_names = {"LH", "RF", "RH"};
    supporting_leg_triangle_is_formed = true;
  } else if (LH_is_swing_limb == true) {
    supporting_limb_numbers << 0, 2, 3;
    supporting_limb_id = {0, 2, 3};
    supporting_limb_names = {"LF", "RH", "RF"};
    supporting_leg_triangle_is_formed = true;
  } else if (RH_is_swing_limb == true) {
    supporting_limb_numbers << 1, 3, 0;
    supporting_limb_id = {1, 3, 0};
    supporting_limb_names = {"LH", "RF", "LF"};
    supporting_leg_triangle_is_formed = true;
  } else if (RF_is_swing_limb == true) {
    supporting_limb_numbers << 0, 2, 1;
    supporting_limb_names = {"LF", "RH", "LH"};
    supporting_limb_id = {0, 2, 1};
    supporting_leg_triangle_is_formed = true;
  } else {
    supporting_leg_triangle_is_formed = false;
  }
#if DEBUG_ENABLED
  std::cout << "supporting_limb_names = {" << supporting_limb_names.at(0) << ", "
            << supporting_limb_names.at(1) << ", " << supporting_limb_names.at(2) << "}" <<
    std::endl;
#endif  // DEBUG_ENABLED

  // Calculate supporting leg triangle if more than three legs are contacted.
  if (supporting_leg_triangle_is_formed == true) {
    Eigen::MatrixXd vertices_of_supporting_leg_triangle = Eigen::MatrixXd::Zero(
      3, supporting_limb_numbers.size());
    if (supporting_limb_numbers.size() == 4) {
      supporting_leg_triangle.points.resize(supporting_limb_numbers.size() * 3);
      supporting_leg_triangle_points.end_effector_position.resize(
        supporting_limb_numbers.size() * 3);
    } else {
      supporting_leg_triangle.points.resize(supporting_limb_numbers.size());
      supporting_leg_triangle_points.end_effector_position.resize(supporting_limb_numbers.size());
    }
    std::string target_frame = "base_link";
    for (int i = 0; i < supporting_limb_numbers.size(); i++) {
      contact_EE_pose_in_limb_coordinate = lbr_EE_pose.at(supporting_limb_numbers(i, 0));
      std::string source_frame = supporting_limb_names.at(i) + "_limb_root";
      geometry_msgs::msg::TransformStamped tf_msg_base_to_limb_root;
      tf_msg_base_to_limb_root = tf_buffer_->lookupTransform(
        target_frame, source_frame, tf2::TimePointZero);

      tf2::Transform tf_base_to_limb_root;
      tf2::convert(tf_msg_base_to_limb_root.transform, tf_base_to_limb_root);

      // Rotation matrix from base frame to limb root frame
      tf2::Matrix3x3 rotation_matrix;
      rotation_matrix = tf_base_to_limb_root.getBasis();
      // Translation vector from base frame to limb root frame
      tf2::Vector3 translation_vector;
      translation_vector = tf_base_to_limb_root.getOrigin();
      // Homogeneous transformation matrix from base frame to limb root frame
      Eigen::MatrixXd homogeneous_transformation_matrix(4, 4);
      homogeneous_transformation_matrix <<
        rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], translation_vector.x(),
        rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], translation_vector.y(),
        rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2], translation_vector.z(),
        0.0, 0.0, 0.0, 1.0;
#if DEBUG_ENABLED
      std::cout << "rotation_matrix = " << std::endl;
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          std::cout << rotation_matrix[i][j] << " ";
        }
        std::cout << std::endl;
      }
      std::cout << "translation_vector = (" << translation_vector.x() << ", "
                << translation_vector.y() << ", " << translation_vector.z() << ")'" << std::endl;
      std::cout << "homogeneous_transformation_matrix = " << homogeneous_transformation_matrix <<
        std::endl;
#endif  // DEBUG_ENABLED

      Eigen::Vector4d homogeneous_vector_contact_EE_position_in_limb_coordinate;
      homogeneous_vector_contact_EE_position_in_limb_coordinate <<
        contact_EE_pose_in_limb_coordinate.end_effector_position.x,
        contact_EE_pose_in_limb_coordinate.end_effector_position.y,
        contact_EE_pose_in_limb_coordinate.end_effector_position.z,
        1.0;
      Eigen::Vector4d homogeneous_vector_contact_EE_pose_in_base_coordinate;
      homogeneous_vector_contact_EE_pose_in_base_coordinate =
        homogeneous_transformation_matrix *
        homogeneous_vector_contact_EE_position_in_limb_coordinate;

      vertices_of_supporting_leg_triangle.col(i) <<
        homogeneous_vector_contact_EE_pose_in_base_coordinate(0, 0),
        homogeneous_vector_contact_EE_pose_in_base_coordinate(1, 0),
        homogeneous_vector_contact_EE_pose_in_base_coordinate(2, 0);

      if (supporting_limb_numbers.size() == LIMB_NUM) {
        supporting_leg_triangle.points.at(i).x = vertices_of_supporting_leg_triangle(0, i);
        supporting_leg_triangle.points.at(i).y = vertices_of_supporting_leg_triangle(1, i);
        supporting_leg_triangle.points.at(i).z = vertices_of_supporting_leg_triangle(2, i);

        supporting_leg_triangle_points.end_effector_position.at(i).x =
          vertices_of_supporting_leg_triangle(0, i);
        supporting_leg_triangle_points.end_effector_position.at(i).y =
          vertices_of_supporting_leg_triangle(1, i);
        supporting_leg_triangle_points.end_effector_position.at(i).z =
          vertices_of_supporting_leg_triangle(2, i);

        supporting_leg_triangle.points.at(i + LIMB_NUM).x =
          vertices_of_supporting_leg_triangle(0, i);
        supporting_leg_triangle.points.at(i + LIMB_NUM).y =
          vertices_of_supporting_leg_triangle(1, i);
        supporting_leg_triangle.points.at(i + LIMB_NUM).z =
          vertices_of_supporting_leg_triangle(2, i);

        supporting_leg_triangle_points.end_effector_position.at(i + LIMB_NUM).x =
          vertices_of_supporting_leg_triangle(0, i);
        supporting_leg_triangle_points.end_effector_position.at(i + LIMB_NUM).y =
          vertices_of_supporting_leg_triangle(1, i);
        supporting_leg_triangle_points.end_effector_position.at(i + LIMB_NUM).z =
          vertices_of_supporting_leg_triangle(2, i);

        supporting_leg_triangle.points.at(i + LIMB_NUM * 2).x =
          vertices_of_supporting_leg_triangle(0, i);
        supporting_leg_triangle.points.at(i + LIMB_NUM * 2).y =
          vertices_of_supporting_leg_triangle(1, i);
        supporting_leg_triangle.points.at(i + LIMB_NUM * 2).z =
          vertices_of_supporting_leg_triangle(2, i);

        supporting_leg_triangle_points.end_effector_position.at(i + LIMB_NUM * 2).x =
          vertices_of_supporting_leg_triangle(0, i);
        supporting_leg_triangle_points.end_effector_position.at(i + LIMB_NUM * 2).y =
          vertices_of_supporting_leg_triangle(1, i);
        supporting_leg_triangle_points.end_effector_position.at(i + LIMB_NUM * 2).z =
          vertices_of_supporting_leg_triangle(2, i);
      } else {
        supporting_leg_triangle.points.at(i).x = vertices_of_supporting_leg_triangle(0, i);
        supporting_leg_triangle.points.at(i).y = vertices_of_supporting_leg_triangle(1, i);
        supporting_leg_triangle.points.at(i).z = vertices_of_supporting_leg_triangle(2, i);

        supporting_leg_triangle_points.end_effector_position.at(i).x =
          vertices_of_supporting_leg_triangle(0, i);
        supporting_leg_triangle_points.end_effector_position.at(i).y =
          vertices_of_supporting_leg_triangle(1, i);
        supporting_leg_triangle_points.end_effector_position.at(i).z =
          vertices_of_supporting_leg_triangle(2, i);
      }
    }
#if DEBUG_ENABLED
    std::cout << "vertices_of_supporting_leg_triangle = " << vertices_of_supporting_leg_triangle <<
      std::endl;
#endif  // DEBUG_ENABLED
    supporting_leg_triangle.action = visualization_msgs::msg::Marker::MODIFY;
  } else {
    supporting_leg_triangle.action = visualization_msgs::msg::Marker::DELETE;
  }
  supporting_leg_triangle_pub_->publish(supporting_leg_triangle);
  supporting_leg_triangle_points_pub_->publish(supporting_leg_triangle_points);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StateEstimator>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
