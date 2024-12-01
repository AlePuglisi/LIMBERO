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

#include "lbr_low_level_controller/lbr_low_level_controller.hpp"

#define DEBUG_ENABLED false

LowLevelController::LowLevelController()
: Node("lbr_low_level_controller")
{
  std::cout << "LowLevelController class is established." << std::endl;

  // Get node name.
  std::string topic_prefix = "/" + std::string(this->get_name());
#if DEBUG_ENABLED
  std::cout << "node_name = " << this->get_name() << std::endl;
  std::cout << "topic_prefix = " << topic_prefix << std::endl;
#endif  // DEBUG_ENABLED

  // Publisher
  motion_finish_signal_pub_ = this->create_publisher<std_msgs::msg::String>(
    topic_prefix + "/motion_finish_signal", 1);

  EE_pose_pub_LF_ = this->create_publisher<lbr_msgs::msg::EndEffectorPoseFourDof>(
    "LF" + topic_prefix + "/EE_pose", 1);
  EE_pose_pub_LH_ = this->create_publisher<lbr_msgs::msg::EndEffectorPoseFourDof>(
    "LH" + topic_prefix + "/EE_pose", 1);
  EE_pose_pub_RH_ = this->create_publisher<lbr_msgs::msg::EndEffectorPoseFourDof>(
    "RH" + topic_prefix + "/EE_pose", 1);
  EE_pose_pub_RF_ = this->create_publisher<lbr_msgs::msg::EndEffectorPoseFourDof>(
    "RF" + topic_prefix + "/EE_pose", 1);

  gripper_state_pub_LF_ = this->create_publisher<std_msgs::msg::Bool>(
    "LF" + topic_prefix + "/gripper_command", 1);
  gripper_state_pub_LH_ = this->create_publisher<std_msgs::msg::Bool>(
    "LH" + topic_prefix + "/gripper_command", 1);
  gripper_state_pub_RH_ = this->create_publisher<std_msgs::msg::Bool>(
    "RH" + topic_prefix + "/gripper_command", 1);
  gripper_state_pub_RF_ = this->create_publisher<std_msgs::msg::Bool>(
    "RF" + topic_prefix + "/gripper_command", 1);

  grieel_state_pub_LF_ = this->create_publisher<std_msgs::msg::String>(
    "LF" + topic_prefix + "/grieel_command", 1);
  grieel_state_pub_LH_ = this->create_publisher<std_msgs::msg::String>(
    "LH" + topic_prefix + "/grieel_command", 1);
  grieel_state_pub_RH_ = this->create_publisher<std_msgs::msg::String>(
    "RH" + topic_prefix + "/grieel_command", 1);
  grieel_state_pub_RF_ = this->create_publisher<std_msgs::msg::String>(
    "RF" + topic_prefix + "/grieel_command", 1);

  grieel_driving_mode_pub_LF_ = this->create_publisher<std_msgs::msg::Int64>(
    "LF" + topic_prefix + "/grieel_driving_command", 1);
  grieel_driving_mode_pub_LH_ = this->create_publisher<std_msgs::msg::Int64>(
    "LH" + topic_prefix + "/grieel_driving_command", 1);
  grieel_driving_mode_pub_RH_ = this->create_publisher<std_msgs::msg::Int64>(
    "RH" + topic_prefix + "/grieel_driving_command", 1);
  grieel_driving_mode_pub_RF_ = this->create_publisher<std_msgs::msg::Int64>(
    "RF" + topic_prefix + "/grieel_driving_command", 1);

  // Subscriber
  command_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/lbr_command_interface/command/low_level_controller", 3,
    std::bind(&LowLevelController::commandCallback, this, std::placeholders::_1));

  EE_pose_sub_LF_ = this->create_subscription<lbr_msgs::msg::EndEffectorPoseFourDof>(
    "/LF/lbr_limb_controller/estimated_EE_pose", 1,
    std::bind(&LowLevelController::endEffectorPoseCallback, this, std::placeholders::_1));
  EE_pose_sub_LH_ = this->create_subscription<lbr_msgs::msg::EndEffectorPoseFourDof>(
    "/LH/lbr_limb_controller/estimated_EE_pose", 1,
    std::bind(&LowLevelController::endEffectorPoseCallback, this, std::placeholders::_1));
  EE_pose_sub_RH_ = this->create_subscription<lbr_msgs::msg::EndEffectorPoseFourDof>(
    "/RH/lbr_limb_controller/estimated_EE_pose", 1,
    std::bind(&LowLevelController::endEffectorPoseCallback, this, std::placeholders::_1));
  EE_pose_sub_RF_ = this->create_subscription<lbr_msgs::msg::EndEffectorPoseFourDof>(
    "/RF/lbr_limb_controller/estimated_EE_pose", 1,
    std::bind(&LowLevelController::endEffectorPoseCallback, this, std::placeholders::_1));

  limb_motion_sub_ = this->create_subscription<lbr_msgs::msg::LimbMotionTask>(
    "/lbr_high_level_controller/motion_package/limb_motion_task", 5,
    std::bind(&LowLevelController::limbMotionCallback, this, std::placeholders::_1));
  limb_motion_command_sub_ = this->create_subscription<lbr_msgs::msg::LimbMotionTask>(
    "/lbr_command_interface/motion_package/limb_motion_task", 5,
    std::bind(&LowLevelController::limbMotionCallback, this, std::placeholders::_1));
  base_motion_sub_ = this->create_subscription<lbr_msgs::msg::BaseMotionTask>(
    "/lbr_high_level_controller/motion_package/base_motion_task", 5,
    std::bind(&LowLevelController::baseMotionCallback, this, std::placeholders::_1));
  base_motion_command_sub_ = this->create_subscription<lbr_msgs::msg::BaseMotionTask>(
    "/lbr_command_interface/motion_package/base_motion_task", 5,
    std::bind(&LowLevelController::baseMotionCallback, this, std::placeholders::_1));
  motion_package_sub_ = this->create_subscription<lbr_msgs::msg::MotionPackage>(
    "/lbr_high_level_controller/motion_package", 10,
    std::bind(&LowLevelController::motionPackageCallback, this, std::placeholders::_1));
  grieel_command_sub_ = this->create_subscription<lbr_msgs::msg::GrieelModeChange>(
    "/lbr_high_level_controller/grieel_mode_change_config", 5,
    std::bind(&LowLevelController::GrieelModeCallback, this, std::placeholders::_1));
  grieel_driving_configuration_sub_ = this->create_subscription<std_msgs::msg::Int64>(
    "/lbr_high_level_controller/grieel_driving_configuration", 5,
    std::bind(&LowLevelController::grieelDrivingConfigurationCallback, this, std::placeholders::_1));

  current_EE_pose.resize(LIMB_NUM);
  initial_EE_pose.resize(LIMB_NUM);
  folding_EE_pose.resize(LIMB_NUM);

  // Set EE pose as initial limb position.
  for (int i = 0; i < LIMB_NUM; i++) {
#if 1  // Use grippers for climbing
    initial_EE_pose.at(i).end_effector_position.x = 0.18;
    initial_EE_pose.at(i).end_effector_position.y = 0.0;
    initial_EE_pose.at(i).end_effector_position.z = -0.045;
    initial_EE_pose.at(i).end_effector_pitch_angle = M_PI_2;
#else
    // Another initial pose for ball foot.
    initial_EE_pose.at(i).end_effector_position.x = 0.185;
    initial_EE_pose.at(i).end_effector_position.y = 0.0;
    initial_EE_pose.at(i).end_effector_position.z = -0.27;
    initial_EE_pose.at(i).end_effector_pitch_angle = M_PI_2;
#endif

    folding_EE_pose.at(i).end_effector_position.x = 0.062;
    folding_EE_pose.at(i).end_effector_position.y = 0.0;
    folding_EE_pose.at(i).end_effector_position.z = -0.065;
    folding_EE_pose.at(i).end_effector_pitch_angle = M_PI_2;
  }
}

void LowLevelController::endEffectorPoseCallback(
  const lbr_msgs::msg::EndEffectorPoseFourDof & EE_pose)
{
  updateEEPose(EE_pose);

#if DEBUG_ENABLED
  std::cout << "LLC::endEffectorPoseCallback" << std::endl;

  std::cout << "EE_pose.limb_id = " <<
    EE_pose.limb_id << std::endl;
  std::cout << "EE_pose.end_effector_position.x = " <<
    EE_pose.end_effector_position.x << std::endl;
  std::cout << "EE_pose.end_effector_position.y = " <<
    EE_pose.end_effector_position.y << std::endl;
  std::cout << "EE_pose.end_effector_position.z = " <<
    EE_pose.end_effector_position.z << std::endl;
  std::cout << "EE_pose.end_effector_pitch_angle = " <<
    EE_pose.end_effector_pitch_angle << std::endl;
#endif  // DEBUG_ENABLED
}

void LowLevelController::updateEEPose(const lbr_msgs::msg::EndEffectorPoseFourDof & next_EE_pose)
{
  const double position_threshold = 0.03;  // [m]
  const double angle_threshold = 0.3;  // [rad]

  double error_x = current_EE_pose[next_EE_pose.limb_id].end_effector_position.x -
    next_EE_pose.end_effector_position.x;
  double error_y = current_EE_pose[next_EE_pose.limb_id].end_effector_position.y -
    next_EE_pose.end_effector_position.y;
  double error_z = current_EE_pose[next_EE_pose.limb_id].end_effector_position.z -
    next_EE_pose.end_effector_position.z;
  double error_pitch = current_EE_pose[next_EE_pose.limb_id].end_effector_pitch_angle -
    next_EE_pose.end_effector_pitch_angle;

  // If EE pose does not change so much, do not update the value.
  if (abs(error_x) > position_threshold) {
    current_EE_pose[next_EE_pose.limb_id].end_effector_position.x =
      next_EE_pose.end_effector_position.x;
  }
  if (abs(error_y) > position_threshold) {
    current_EE_pose[next_EE_pose.limb_id].end_effector_position.y =
      next_EE_pose.end_effector_position.y;
  }
  if (abs(error_z) > position_threshold) {
    current_EE_pose[next_EE_pose.limb_id].end_effector_position.z =
      next_EE_pose.end_effector_position.z;
  }
  if (abs(error_pitch) > angle_threshold) {
    current_EE_pose[next_EE_pose.limb_id].end_effector_pitch_angle =
      next_EE_pose.end_effector_pitch_angle;
  }
}

Eigen::Vector3d LowLevelController::transformRotationBaseToLimb(
  const int & limb_id, const Eigen::Vector3d vector_in_base_coordinate)
{
  // Angle in Base to Limb coordinate [rad]
  double angle_Base_to_Limb = M_PI_4 + (M_PI_2 * limb_id);

  Eigen::Matrix3d rotation_matrix;
  rotation_matrix <<
    cos(angle_Base_to_Limb), sin(angle_Base_to_Limb), 0,
    -sin(angle_Base_to_Limb), cos(angle_Base_to_Limb), 0,
    0, 0, 1;

  // Transform Base to Limb
  Eigen::Vector3d vector_in_limb_coordinate = rotation_matrix * vector_in_base_coordinate;

#if DEBUG_ENABLED
  std::cout << "transformRotationBaseToLimb" << std::endl;
  std::cout << "rotation_matrix = " << rotation_matrix << std::endl;
  std::cout << "vector_in_base_coordinate = " << vector_in_base_coordinate << std::endl;
  std::cout << "vector_in_limb_coordinate = " << vector_in_limb_coordinate << std::endl;
#endif  // DEBUG_ENABLED

  return vector_in_limb_coordinate;
}

void LowLevelController::publishNextEndEffectorPose(
  const lbr_msgs::msg::EndEffectorPoseFourDof & EE_displacement)
{
#if DEBUG_ENABLED
  std::cout << "LLC::publishNextEndEffectorPose" << std::endl;
  std::cout << "EE_displacement.limb_id = " << EE_displacement.limb_id << std::endl;
  std::cout << "EE_displacement.end_effector_position.x = " <<
    EE_displacement.end_effector_position.x << std::endl;
  std::cout << "EE_displacement.end_effector_position.y = " <<
    EE_displacement.end_effector_position.y << std::endl;
  std::cout << "EE_displacement.end_effector_position.z = " <<
    EE_displacement.end_effector_position.z << std::endl;
  std::cout << "EE_displacement.end_effector_pitch_angle = " <<
    EE_displacement.end_effector_pitch_angle << std::endl;
#endif  // DEBUG_ENABLED

  // Limb ID
  current_EE_pose.at(EE_displacement.limb_id).limb_id = EE_displacement.limb_id;
  // x, y, z
  current_EE_pose.at(EE_displacement.limb_id).end_effector_position.x +=
    EE_displacement.end_effector_position.x;
  current_EE_pose.at(EE_displacement.limb_id).end_effector_position.y +=
    EE_displacement.end_effector_position.y;
  current_EE_pose.at(EE_displacement.limb_id).end_effector_position.z +=
    EE_displacement.end_effector_position.z;
  // Pitch
  current_EE_pose.at(EE_displacement.limb_id).end_effector_pitch_angle +=
    EE_displacement.end_effector_pitch_angle;

#if DEBUG_ENABLED
  std::cout << "current_EE_pose.at(" << EE_displacement.limb_id << ").limb_id = " <<
    current_EE_pose.at(EE_displacement.limb_id).limb_id << std::endl;
  std::cout << "current_EE_pose.at(" << EE_displacement.limb_id <<
    ").end_effector_position.x = " <<
    current_EE_pose.at(EE_displacement.limb_id).end_effector_position.x << std::endl;
  std::cout << "current_EE_pose.at(" << EE_displacement.limb_id <<
    ").end_effector_position.y = " <<
    current_EE_pose.at(EE_displacement.limb_id).end_effector_position.y << std::endl;
  std::cout << "current_EE_pose.at(" << EE_displacement.limb_id <<
    ").end_effector_position.z = " <<
    current_EE_pose.at(EE_displacement.limb_id).end_effector_position.z << std::endl;
  std::cout << "current_EE_pose.at(" << EE_displacement.limb_id <<
    ").end_effector_pitch_angle = " <<
    current_EE_pose.at(EE_displacement.limb_id).end_effector_pitch_angle << std::endl;
#endif  // DEBUG_ENABLED

  // Publish desired EE pose to each limb.
  current_EE_pose.at(EE_displacement.limb_id).header.stamp = this->now();
  if (EE_displacement.limb_id == 0) {
    EE_pose_pub_LF_->publish(current_EE_pose.at(EE_displacement.limb_id));
  } else if (EE_displacement.limb_id == 1) {
    EE_pose_pub_LH_->publish(current_EE_pose.at(EE_displacement.limb_id));
  } else if (EE_displacement.limb_id == 2) {
    EE_pose_pub_RH_->publish(current_EE_pose.at(EE_displacement.limb_id));
  } else if (EE_displacement.limb_id == 3) {
    EE_pose_pub_RF_->publish(current_EE_pose.at(EE_displacement.limb_id));
  }
}

void LowLevelController::commandCallback(const std_msgs::msg::String & command)
{
  if (command.data == "initialize") {
    setDefaultEndEffectorPose(initial_EE_pose);
  } else if (command.data == "gripper_open") {
    for (int limb_id = 0; limb_id < LIMB_NUM; limb_id++) {
      publishGripperCommand(limb_id, false);  // Open gripper
    }
  } else if (command.data == "gripper_close") {
    for (int limb_id = 0; limb_id < LIMB_NUM; limb_id++) {
      publishGripperCommand(limb_id, true);  // Close gripper
    }
  } else {
    std::cout << "INVALID COMMAND" << std::endl;
  }
}

void LowLevelController::publishGripperCommand(const int & limb_id, const bool & close_gripper)
{
  std_msgs::msg::Bool execute_grasping;
  execute_grasping.data = close_gripper;

  if (limb_id == 0) {
    gripper_state_pub_LF_->publish(execute_grasping);
  } else if (limb_id == 1) {
    gripper_state_pub_LH_->publish(execute_grasping);
  } else if (limb_id == 2) {
    gripper_state_pub_RH_->publish(execute_grasping);
  } else if (limb_id == 3) {
    gripper_state_pub_RF_->publish(execute_grasping);
  }
}

void LowLevelController::limbMotionCallback(const lbr_msgs::msg::LimbMotionTask & limb_motion)
{
  execLimbMotion(limb_motion);
}

void LowLevelController::execLimbMotion(const lbr_msgs::msg::LimbMotionTask & limb_motion)
{
#if DEBUG_ENABLED
  std::cout << "LLC::limbMotionCallback" << std::endl;

  std::cout << "limb_motion.limb_id = " << limb_motion.limb_id << std::endl;
  std::cout << "limb_motion.end_effector_displacement.x = " <<
    limb_motion.end_effector_displacement.x << std::endl;
  std::cout << "limb_motion.end_effector_displacement.y = " <<
    limb_motion.end_effector_displacement.y << std::endl;
  std::cout << "limb_motion.end_effector_displacement.z = " <<
    limb_motion.end_effector_displacement.z << std::endl;
#endif  // DEBUG_ENABLED
  const int step_num = 100;
  const int step_time = limb_motion.swing_duration / step_num;  // [ms]
  if (step_time == 0) {
    std::cerr << "INVALID VALUE: step_time =" << step_time << std::endl;
    std::abort();
  }

  lbr_msgs::msg::EndEffectorPoseFourDof d_EE_displacement;
  d_EE_displacement.limb_id = limb_motion.limb_id;

  // Change pitch angle
  d_EE_displacement.end_effector_pitch_angle =
    limb_motion.pitch_angle_displacement / (step_num - 1);
  Eigen::Vector3d displacement_in_base_coordinate(
    limb_motion.end_effector_displacement.x,
    limb_motion.end_effector_displacement.y,
    limb_motion.end_effector_displacement.z);

  Eigen::Vector3d displacement_in_limb_coordinate =
    transformRotationBaseToLimb(limb_motion.limb_id, displacement_in_base_coordinate);

  // Initialize mid position of swing
  Eigen::Vector3d mid_position;
  mid_position(0) = limb_motion.swing_mid_point.x;  // BUG(MI): Move different direction not x
  mid_position(1) = limb_motion.swing_mid_point.y;  // BUG(MI): Move different direction not y
  mid_position(2) = limb_motion.swing_mid_point.z;
  // Initialize coefficients for spline curve
  Eigen::Matrix<double, 8, 3> spline_coefficient = Eigen::MatrixXd::Zero(8, 3);

  // Calculate coefficients for spline curve
  for (int i = 0; i < displacement_in_base_coordinate.size(); i++) {
    mid_position(i) = mid_position(i) + (displacement_in_limb_coordinate(i) / 2);
    spline_coefficient.col(i) = calculate7orderSplineTrajectory(
      displacement_in_limb_coordinate(i), mid_position(i), limb_motion.swing_duration);
  }

  // Elapsed time from initial time [ms]
  double elapsed_time = 0.0;
  Eigen::Vector3d EE_displacement_from_initial_position;
  Eigen::Vector3d EE_displacement_from_initial_position_in_previous_step;

  // Update displacement of end-effector position
  for (int step = 0; step < step_num; step++) {
    for (int i = 0; i < displacement_in_base_coordinate.size(); i++) {
      EE_displacement_from_initial_position(i) =
        spline_coefficient(0, i) +
        spline_coefficient(1, i) * elapsed_time +
        spline_coefficient(2, i) * pow(elapsed_time, 2) +
        spline_coefficient(3, i) * pow(elapsed_time, 3) +
        spline_coefficient(4, i) * pow(elapsed_time, 4) +
        spline_coefficient(5, i) * pow(elapsed_time, 5) +
        spline_coefficient(6, i) * pow(elapsed_time, 6) +
        spline_coefficient(7, i) * pow(elapsed_time, 7);
    }
    // Update displacement
    d_EE_displacement.end_effector_position.x =
      EE_displacement_from_initial_position(0) -
      EE_displacement_from_initial_position_in_previous_step(0);
    d_EE_displacement.end_effector_position.y =
      EE_displacement_from_initial_position(1) -
      EE_displacement_from_initial_position_in_previous_step(1);
    d_EE_displacement.end_effector_position.z =
      EE_displacement_from_initial_position(2) -
      EE_displacement_from_initial_position_in_previous_step(2);
    // Store end-effector position from initial position
    EE_displacement_from_initial_position_in_previous_step =
      EE_displacement_from_initial_position;
    // Update elapsed time
    elapsed_time = elapsed_time + step_time;
#if DEBUG_ENABLED
    std::cout << "Swing limb: limb_id = " << limb_motion.limb_id << std::endl;
    std::cout << "Duration: " << step_time * step << " ms/" <<
      limb_motion.swing_duration << " ms" << std::endl;

    std::cout << "d_EE_displacement.end_effector_position.x = " <<
      d_EE_displacement.end_effector_position.x << std::endl;
    std::cout << "d_EE_displacement.end_effector_position.y = " <<
      d_EE_displacement.end_effector_position.y << std::endl;
    std::cout << "d_EE_displacement.end_effector_position.z = " <<
      d_EE_displacement.end_effector_position.z << std::endl;
#endif  // DEBUG_ENABLED
    publishNextEndEffectorPose(d_EE_displacement);
    std::this_thread::sleep_for(std::chrono::milliseconds(step_time));
  }
}

// TODO(KT): execLimbMotionArray and execLimbMotion have to be merged in the future.
void LowLevelController::execLimbMotionArray(
  const std::vector<lbr_msgs::msg::LimbMotionTask> & limb_motion_task_array)
{
#if DEBUG_ENABLED
  std::cout << "LLC::execLimbMotionArray" << std::endl;
#endif  // DEBUG_ENABLED
  const int lm_num = limb_motion_task_array.size();  // lm stands for limb motion

  // Define step time
  const int step_num = 100;
  std::vector<int> step_time(lm_num);
  for (int i = 0; i < lm_num; i++) {
    step_time.at(i) = limb_motion_task_array.at(i).swing_duration / step_num;  // [ms]
    if (step_time.at(i) == 0) {
      std::cerr << "INVALID VALUE: step_time.at(" << i << ") =" << step_time.at(i) << std::endl;
      std::abort();
    }
  }

  // Define EE displacement
  std::vector<lbr_msgs::msg::EndEffectorPoseFourDof> d_EE_displacement(lm_num);
  // Eigen with std::vector may cause some bugs, so using aligned_allocator.
  std::vector<Eigen::Vector3d,
    Eigen::aligned_allocator<Eigen::Vector3d>> displacement_in_base_coordinate(lm_num);
  std::vector<Eigen::Vector3d,
    Eigen::aligned_allocator<Eigen::Vector3d>> displacement_in_limb_coordinate(lm_num);
  std::vector<Eigen::Vector3d,
    Eigen::aligned_allocator<Eigen::Vector3d>> mid_position(lm_num);
  std::vector<Eigen::Matrix<double, 8, 3>,
    Eigen::aligned_allocator<Eigen::Matrix<double, 8, 3>>> spline_coefficient(lm_num);
  std::vector<double> elapsed_time(lm_num);

  for (int i = 0; i < lm_num; i++) {
    d_EE_displacement.at(i).limb_id = limb_motion_task_array.at(i).limb_id;

    // pitch angle displacement
    d_EE_displacement.at(i).end_effector_pitch_angle =
      limb_motion_task_array.at(i).pitch_angle_displacement / (step_num - 1);

    displacement_in_base_coordinate.at(i)(0) =
      limb_motion_task_array.at(i).end_effector_displacement.x;
    displacement_in_base_coordinate.at(i)(1) =
      limb_motion_task_array.at(i).end_effector_displacement.y;
    displacement_in_base_coordinate.at(i)(2) =
      limb_motion_task_array.at(i).end_effector_displacement.z;

    displacement_in_limb_coordinate.at(i) = transformRotationBaseToLimb(
      limb_motion_task_array.at(i).limb_id, displacement_in_base_coordinate.at(i));

    // Initialize mid position of swing
    mid_position.at(i)(0) =
      limb_motion_task_array.at(i).swing_mid_point.x;  // BUG(MI): Move different direction not x
    mid_position.at(i)(1) =
      limb_motion_task_array.at(i).swing_mid_point.y;  // BUG(MI): Move different direction not y
    mid_position.at(i)(2) =
      limb_motion_task_array.at(i).swing_mid_point.z;
    // Initialize coefficients for spline curve
    spline_coefficient.at(i) = Eigen::MatrixXd::Zero(8, 3);

    // Calculate coefficients for spline curve
    for (int j = 0; j < displacement_in_base_coordinate.at(i).size(); j++) {
      mid_position.at(i)(j) = mid_position.at(i)(j) +
        (displacement_in_limb_coordinate.at(i)(j) / 2);
      spline_coefficient.at(i).col(j) = calculate7orderSplineTrajectory(
        displacement_in_limb_coordinate.at(i)(j),
        mid_position.at(i)(j),
        limb_motion_task_array.at(i).swing_duration);
    }

    // Elapsed time from initial time [ms]
    elapsed_time.at(i) = 0.0;
  }
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
  EE_displacement_from_initial_position(lm_num);
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
  EE_displacement_from_initial_position_in_previous_step(lm_num);

  // Update displacement of end-effector position
  for (int step = 0; step < step_num; step++) {
#if DEBUG_ENABLED
    std::cout << "step = " << step << std::endl;
#endif  // DEBUG_ENABLED
    for (int l = 0; l < lm_num; l++) {
      if (step == 0) {  // Initialization
        EE_displacement_from_initial_position.at(l) = Eigen::Vector3d::Zero();
        EE_displacement_from_initial_position_in_previous_step.at(l) = Eigen::Vector3d::Zero();
      }
#if DEBUG_ENABLED
      std::cout << "l = " << l << std::endl;
#endif  // DEBUG_ENABLED
      for (int m = 0; m < displacement_in_base_coordinate.at(l).size(); m++) {
        EE_displacement_from_initial_position.at(l)(m) =
          spline_coefficient.at(l)(0, m) +
          spline_coefficient.at(l)(1, m) * elapsed_time.at(l) +
          spline_coefficient.at(l)(2, m) * pow(elapsed_time.at(l), 2) +
          spline_coefficient.at(l)(3, m) * pow(elapsed_time.at(l), 3) +
          spline_coefficient.at(l)(4, m) * pow(elapsed_time.at(l), 4) +
          spline_coefficient.at(l)(5, m) * pow(elapsed_time.at(l), 5) +
          spline_coefficient.at(l)(6, m) * pow(elapsed_time.at(l), 6) +
          spline_coefficient.at(l)(7, m) * pow(elapsed_time.at(l), 7);
      }
      // Update displacement
      d_EE_displacement.at(l).end_effector_position.x =
        EE_displacement_from_initial_position.at(l)(0) -
        EE_displacement_from_initial_position_in_previous_step.at(l)(0);
      d_EE_displacement.at(l).end_effector_position.y =
        EE_displacement_from_initial_position.at(l)(1) -
        EE_displacement_from_initial_position_in_previous_step.at(l)(1);
      d_EE_displacement.at(l).end_effector_position.z =
        EE_displacement_from_initial_position.at(l)(2) -
        EE_displacement_from_initial_position_in_previous_step.at(l)(2);

      // Store end-effector position from initial position
      EE_displacement_from_initial_position_in_previous_step.at(l) =
        EE_displacement_from_initial_position.at(l);
      // Update elapsed time
      elapsed_time.at(l) = elapsed_time.at(l) + step_time.at(l);

      publishNextEndEffectorPose(d_EE_displacement.at(l));
      std::this_thread::sleep_for(std::chrono::milliseconds(step_time.at(l)));
    }
  }
}

Eigen::Vector<double, 8> LowLevelController::calculate7orderSplineTrajectory(
  const double & final_position,
  const double & mid_position,
  const int & swing_duration)
{
  const double initial_time = 0.0;
  const double final_time = swing_duration;  // [ms]
  double mid_time = (final_time + initial_time) / 2;  // [ms]
  const double initial_position = 0.0;  // [m]
  const double initial_velocity = 0.0;
  // ?(MI): 5/3 is from ClimbLab code.
  double mid_velocity = 5 / 3 * (final_position - initial_position) / (final_time - initial_time);
  const double final_velocity = 0.0;
  const double initial_acceleration = 0.0;
  const double final_acceleration = 0.0;

  // Condition for position and velocity
  Eigen::Vector<double, 8> condition_vector;
  condition_vector <<
    initial_position,
    mid_position,
    final_position,
    initial_velocity,
    mid_velocity,
    final_velocity,
    initial_acceleration,
    final_acceleration;

  Eigen::Matrix<double, 8, 8> time_matrix;
  time_matrix.row(0) <<
    1, initial_time, pow(initial_time, 2), pow(initial_time, 3),
    pow(initial_time, 4), pow(initial_time, 5), pow(initial_time, 6), pow(initial_time, 7);
  time_matrix.row(1) <<
    1, mid_time, pow(mid_time, 2), pow(mid_time, 3),
    pow(mid_time, 4), pow(mid_time, 5), pow(mid_time, 6), pow(mid_time, 7);
  time_matrix.row(2) <<
    1, final_time, pow(final_time, 2), pow(final_time, 3),
    pow(final_time, 4), pow(final_time, 5), pow(final_time, 6), pow(final_time, 7);
  time_matrix.row(3) <<
    0, 1, 2 * initial_time, 3 * pow(initial_time, 2), 4 * pow(initial_time, 3),
    5 * pow(initial_time, 4), 6 * pow(initial_time, 5), 7 * pow(initial_time, 6);
  time_matrix.row(4) <<
    0, 1, 2 * mid_time, 3 * pow(mid_time, 2), 4 * pow(mid_time, 3),
    5 * pow(mid_time, 4), 6 * pow(mid_time, 5), 7 * pow(mid_time, 6);
  time_matrix.row(5) <<
    0, 1, 2 * final_time, 3 * pow(final_time, 2), 4 * pow(final_time, 3),
    5 * pow(final_time, 4), 6 * pow(final_time, 5), 7 * pow(final_time, 6);
  time_matrix.row(6) <<
    0, 0, 2, 6 * initial_time, 12 * pow(initial_time, 2),
    20 * pow(initial_time, 3), 30 * pow(initial_time, 4), 42 * pow(initial_time, 5);
  time_matrix.row(7) <<
    0, 0, 2, 6 * final_time, 12 * pow(final_time, 2),
    20 * pow(final_time, 3), 30 * pow(final_time, 4), 42 * pow(final_time, 5);

  // Calculate coefficients for spline curve
  Eigen::Vector<double, 8> spline_coefficient;
  spline_coefficient = time_matrix.inverse() * condition_vector;

  return spline_coefficient;
}

void LowLevelController::baseMotionCallback(const lbr_msgs::msg::BaseMotionTask & base_motion)
{
  execBaseMotion(base_motion);
}

void LowLevelController::execBaseMotion(const lbr_msgs::msg::BaseMotionTask & base_motion)
{
  // TODO(KT): Implement feature to stop limb motion when one of limbs reaches limit.
  const int step_num = 100;
  const int step_time = base_motion.motion_duration / step_num;
  if (step_time == 0) {
    std::cerr << "INVALID VALUE: step_time =" << step_time << std::endl;
    std::abort();
  }

  Eigen::Vector3d translation_in_base_coordinate(
    base_motion.base_displacement.x,
    base_motion.base_displacement.y,
    base_motion.base_displacement.z);
  Eigen::Vector3d rotation_in_base_coordinate(
    base_motion.base_angle_displacement.x,
    base_motion.base_angle_displacement.y,
    base_motion.base_angle_displacement.z);

  Eigen::Vector3d d_translation_in_base_coordinate =
    translation_in_base_coordinate / (step_num - 1);
  Eigen::Vector3d d_rotation_in_base_coordinate =
    rotation_in_base_coordinate / (step_num - 1);

  for (int step = 0; step < step_num; step++) {
    for (int limb = 0; limb < LIMB_NUM; limb++) {
      lbr_msgs::msg::EndEffectorPoseFourDof d_EE_displacement;
      d_EE_displacement.limb_id = limb;
      d_EE_displacement.end_effector_position.x = 0.0;
      d_EE_displacement.end_effector_position.y = 0.0;
      d_EE_displacement.end_effector_position.z = 0.0;

      // Base translation motion
      Eigen::Vector3d d_translation_in_limb_coordinate =
        transformRotationBaseToLimb(limb, d_translation_in_base_coordinate);

      // Convert base translation motion to limb EE motion.
      d_EE_displacement.end_effector_position.x += -d_translation_in_limb_coordinate(0);
      d_EE_displacement.end_effector_position.y += -d_translation_in_limb_coordinate(1);
      d_EE_displacement.end_effector_position.z += -d_translation_in_limb_coordinate(2);

      // Base rotation motion
      Eigen::Vector3d d_EE_motion_in_base_coordinate =
        baseMotionToEndEffectorMotion(limb, d_rotation_in_base_coordinate);

      Eigen::Vector3d d_EE_motion_in_limb_coordinate =
        transformRotationBaseToLimb(limb, d_EE_motion_in_base_coordinate);

      d_EE_displacement.end_effector_position.x += d_EE_motion_in_limb_coordinate(0);
      d_EE_displacement.end_effector_position.y += d_EE_motion_in_limb_coordinate(1);
      d_EE_displacement.end_effector_position.z += d_EE_motion_in_limb_coordinate(2);

      publishNextEndEffectorPose(d_EE_displacement);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(step_time));
#if DEBUG_ENABLED
    std::cout << "Base motion: " << step_time * step << " ms/" <<
      base_motion.motion_duration << " ms" << std::endl;
#endif  // DEBUG_ENABLED
  }
}

Eigen::Vector3d LowLevelController::baseMotionToEndEffectorMotion(
  const int & limb_id, const Eigen::Vector3d & base_rotation)
{
  const double base_length = 0.27;  // length between LF and LH, and so on. [m]
  // TODO(KT): Set accurate value for base_length. THIS IS TENTATIVE.
  const double base_diagonal_length = base_length * sqrt(2);
  const double half_base_length = base_length / 2;
  const double half_base_diagonal_length = base_diagonal_length / 2;

  double roll = base_rotation(0);
  double pitch = base_rotation(1);
  double yaw = base_rotation(2);

  Eigen::Vector3d EE_motion;
  EE_motion(0) = 0.0;
  EE_motion(1) = 0.0;
  EE_motion(2) = 0.0;

  // TODO(KT): Roll and Pitch should also consider EE position.

  // Roll
  if (limb_id == 0 || limb_id == 1) {
    EE_motion(1) += half_base_length * (1 - cos(roll));
    EE_motion(2) += -half_base_length * sin(roll);
  } else if (limb_id == 2 || limb_id == 3) {
    EE_motion(1) += -half_base_length * (1 - cos(roll));
    EE_motion(2) += half_base_length * sin(roll);
  }

  // Pitch
  if (limb_id == 0 || limb_id == 3) {
    EE_motion(0) += half_base_length * (1 - cos(pitch));
    EE_motion(2) += -half_base_length * sin(pitch);
  } else if (limb_id == 1 || limb_id == 2) {
    EE_motion(0) += -half_base_length * (1 - cos(pitch));
    EE_motion(2) += half_base_length * sin(pitch);
  }

  // Yaw
  // The center of yaw motion corresponds to the center of limbs for this code.
  double EE_dist_xy_from_limb_coordinate =
    sqrt(
    pow(current_EE_pose.at(limb_id).end_effector_position.x, 2) +
    pow(current_EE_pose.at(limb_id).end_effector_position.y, 2));
  double theta_1 = atan2(
    current_EE_pose.at(limb_id).end_effector_position.y,
    current_EE_pose.at(limb_id).end_effector_position.x);
  double EE_dist_xy_from_base_coordinate =
    sqrt(
    pow(EE_dist_xy_from_limb_coordinate, 2) +
    pow(half_base_diagonal_length, 2) -
    2 * EE_dist_xy_from_limb_coordinate * half_base_diagonal_length * cos(M_PI - theta_1));
  double cos_base_yaw_angle = (pow(half_base_diagonal_length, 2) +
    pow(EE_dist_xy_from_base_coordinate, 2) - pow(EE_dist_xy_from_limb_coordinate, 2)) /
    (2 * half_base_diagonal_length * EE_dist_xy_from_base_coordinate);
  double sin_base_yaw_angle = sqrt(1 - pow(cos_base_yaw_angle, 2));
  double base_yaw_angle;
  if (theta_1 >= 0.0) {
    base_yaw_angle = atan2(sin_base_yaw_angle, cos_base_yaw_angle);
  } else if (theta_1 < 0.0) {  // When theta_1 is negative, sin is also negative.
    base_yaw_angle = atan2(-sin_base_yaw_angle, cos_base_yaw_angle);
  }
  base_yaw_angle += M_PI_4 + limb_id * M_PI_2;
#if DEBUG_ENABLED
  std::cout << "LLC::baseMotionToEndEffectorMotion" << std::endl;
  std::cout << "limb_id = " << limb_id << std::endl;
  std::cout << "theta_1 = " << theta_1 << std::endl;
  std::cout << "EE_dist_xy_from_base_coordinate = " << EE_dist_xy_from_base_coordinate << std::endl;
  std::cout << "cos_base_yaw_angle = " << cos_base_yaw_angle << std::endl;
  std::cout << "sin_base_yaw_angle = " << sin_base_yaw_angle << std::endl;
  std::cout << "base_yaw_angle = " << base_yaw_angle << std::endl;
#endif

  double d_x = EE_dist_xy_from_base_coordinate * (1 - cos(yaw));
  double d_y = -EE_dist_xy_from_base_coordinate * sin(yaw);

  EE_motion(0) += d_x * cos(base_yaw_angle) - d_y * sin(base_yaw_angle);
  EE_motion(1) += -d_x * sin(base_yaw_angle) + d_y * cos(base_yaw_angle);

  return EE_motion;
}

void LowLevelController::setDefaultEndEffectorPose(
  const std::vector<lbr_msgs::msg::EndEffectorPoseFourDof> & default_EE_pose)
{
  // Publish EE pose to be initial pose.
  std::vector<geometry_msgs::msg::Vector3> moving_vector(LIMB_NUM);
  std::vector<double> pitch_difference(LIMB_NUM);

  const int initialize_duration = 10000;  // [ms]
  const int step_num = 100;
  const int step_time = initialize_duration / step_num;

  std::vector<lbr_msgs::msg::EndEffectorPoseFourDof> d_EE_displacement(LIMB_NUM);
  for (int limb = 0; limb < LIMB_NUM; limb++) {
    moving_vector.at(limb).x =
      default_EE_pose.at(limb).end_effector_position.x -
      current_EE_pose.at(limb).end_effector_position.x;
    moving_vector.at(limb).y =
      default_EE_pose.at(limb).end_effector_position.y -
      current_EE_pose.at(limb).end_effector_position.y;
    moving_vector.at(limb).z =
      default_EE_pose.at(limb).end_effector_position.z -
      current_EE_pose.at(limb).end_effector_position.z;
    pitch_difference.at(limb) =
      default_EE_pose.at(limb).end_effector_pitch_angle -
      current_EE_pose.at(limb).end_effector_pitch_angle;

    d_EE_displacement.at(limb).limb_id = limb;
    d_EE_displacement.at(limb).end_effector_position.x = moving_vector.at(limb).x / (step_num - 1);
    d_EE_displacement.at(limb).end_effector_position.y = moving_vector.at(limb).y / (step_num - 1);
    d_EE_displacement.at(limb).end_effector_position.z = moving_vector.at(limb).z / (step_num - 1);
    d_EE_displacement.at(limb).end_effector_pitch_angle =
      pitch_difference.at(limb) / (step_num - 1);
  }

  for (int step = 0; step < step_num; step++) {
    for (int limb = 0; limb < LIMB_NUM; limb++) {
      publishNextEndEffectorPose(d_EE_displacement.at(limb));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(step_time));
#if DEBUG_ENABLED
    float progress_percent = static_cast<float>(step) / static_cast<float>(step_num);
    std::cout << "Initialize: " << progress_percent * 100 << "%" << std::endl;
#endif  // DEBUG_ENABLED
  }
  RCLCPP_INFO(this->get_logger(), "Finish initialization.");
}

void LowLevelController::motionPackageCallback(const lbr_msgs::msg::MotionPackage & motion_package)
{
  int task_num = motion_package.motion_task_array.size();
  for (int i = 0; i < task_num; i++) {
#if DEBUG_ENABLED
    std::cout << "LLC::motionPackageCallback: i = " << i << std::endl;
#endif  // DEBUG_ENABLED
    int task_id = motion_package.motion_task_array.at(i).task_id_array.at(i);
    if (task_id == 0) {  // ID 0: Limb motion
      execLimbMotion(motion_package.motion_task_array.at(i).limb_motion_task);
      //std::cout << "Executing limb motion = " << i << "\n" << std::endl;
    } else if (task_id == 1) {  // ID 1: Base motion
      execBaseMotion(motion_package.motion_task_array.at(i).base_motion_task);
      //std::cout << "Executing Base motion = " << i << "\n" << std::endl;
    } else if (task_id == 2) {  // ID 2: Limb motion array
      execLimbMotionArray(motion_package.motion_task_array.at(i).limb_motion_task_array);
    } else {
      RCLCPP_ERROR(this->get_logger(), "INVALID TASK ID IS FOUND.");
    }
  }
  std_msgs::msg::String signal;
  signal.data = "fin";
  motion_finish_signal_pub_->publish(signal);
}

void LowLevelController::GrieelModeCallback(const lbr_msgs::msg::GrieelModeChange & grieel_mode)
{
  std_msgs::msg::String new_grieel_mode;
  new_grieel_mode.data = grieel_mode.mode;

  if(grieel_mode.limb_id == 0){
    grieel_state_pub_LF_->publish(new_grieel_mode);
  } else if(grieel_mode.limb_id == 1){
    grieel_state_pub_LH_->publish(new_grieel_mode);
  }else if(grieel_mode.limb_id == 2){
    grieel_state_pub_RH_->publish(new_grieel_mode);
  }else if(grieel_mode.limb_id == 3){
    grieel_state_pub_RF_->publish(new_grieel_mode);
  }
}

void LowLevelController::grieelDrivingConfigurationCallback(const std_msgs::msg::Int64 & driving_configuration)
{
  std_msgs::msg::Int64 new_driving_configuration;
  int new_config = driving_configuration.data;
  if (new_config == 0 || new_config == 1 || new_config == 2 || new_config == 3){
    new_driving_configuration.data = 0;
  } else if (new_config == 10 || new_config == 11 || new_config == 12 || new_config == 13){
    new_driving_configuration.data = 1;
  }

  if ((driving_configuration.data == 100) || (driving_configuration.data == -1)){
    new_driving_configuration.data = driving_configuration.data;
    grieel_driving_mode_pub_LF_->publish(new_driving_configuration);
    grieel_driving_mode_pub_LH_->publish(new_driving_configuration);
    grieel_driving_mode_pub_RH_->publish(new_driving_configuration);
    grieel_driving_mode_pub_RF_->publish(new_driving_configuration);
  } else if ((driving_configuration.data == 0) || (driving_configuration.data == 10)){
    grieel_driving_mode_pub_LF_->publish(new_driving_configuration);
  } else if ((driving_configuration.data == 1) || (driving_configuration.data == 11)){
    grieel_driving_mode_pub_LH_->publish(new_driving_configuration);
  } else if ((driving_configuration.data == 2) || (driving_configuration.data == 12)){
    grieel_driving_mode_pub_RH_->publish(new_driving_configuration);
  } else if ((driving_configuration.data == 3) || (driving_configuration.data == 13)){
    grieel_driving_mode_pub_RF_->publish(new_driving_configuration);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LowLevelController>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
