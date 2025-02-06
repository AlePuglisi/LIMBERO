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

#include "lbr_limb_controller/lbr_limb_controller.hpp"

#define DEBUG_ENABLED false
#define SIMULATION false

LimbController::LimbController()
: Node("lbr_limb_controller")
{
  std::cout << "LimbController class is established." << std::endl;

  // Set ROS 2 parameters
  this->declare_parameter("rviz_debug", false);
  enable_rviz_debug = this->get_parameter("rviz_debug").as_bool();

  // Get namespace and node name.
  std::string topic_prefix = std::string(this->get_namespace()) +
    "/" + std::string(this->get_name());
  std::string name_space = std::string(this->get_namespace());
  driving = false;
  stop = true;
  direction = 0; 

  this->declare_parameter("wheel_mode", true);

  bool wheel_state;
  this->get_parameter("wheel_mode", wheel_state);

  if(wheel_state == true){
    grieel_mode = "wheel";
  } else if(wheel_state == false){
    grieel_mode = "gripper";
  }

  std::cout << name_space << " (LC) : wheel_mode:= " << wheel_state << std::endl; 

  // Publisher
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    topic_prefix + "/joint_state", 1);
  rviz_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    topic_prefix + "/rviz_joint_state", 1);
  gripping_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    topic_prefix + "/gripper/gripper_command", 1);
  gripper_state_text_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    topic_prefix + "/gripper/gripper_state_text", 1);
  estimated_EE_pose_pub_ = this->create_publisher<lbr_msgs::msg::EndEffectorPoseFourDof>(
    topic_prefix + "/estimated_EE_pose", 1);
  target_position_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    topic_prefix + "/target_EE_position", 10);
  trajectory_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    topic_prefix + "/EE_trajectory_array", 10);
  grieel_joint_finish_pub_ = this->create_publisher<std_msgs::msg::String>(
    topic_prefix + "/grieel_joint_finish", 10);

  // Subscriber
  EE_pose_sub_ = this->create_subscription<lbr_msgs::msg::EndEffectorPoseFourDof>(
    std::string(this->get_namespace()) + "/lbr_low_level_controller/EE_pose", 10,
    std::bind(&LimbController::endEffectorPoseCallback, this, std::placeholders::_1));
  gripper_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    std::string(this->get_namespace()) + "/lbr_low_level_controller/gripper_command", 1,
    std::bind(&LimbController::gripperCommandCallback, this, std::placeholders::_1));
  grieel_state_sub_ = this->create_subscription<std_msgs::msg::String>(
    std::string(this->get_namespace()) + "/lbr_low_level_controller/grieel_command", 1,
    std::bind(&LimbController::grieelChangeCallback, this, std::placeholders::_1));
  driving_mode_sub_ = this->create_subscription<std_msgs::msg::Int64>(
    std::string(this->get_namespace()) + "/lbr_low_level_controller/grieel_driving_command", 1,
    std::bind(&LimbController::drivingModeCallback, this, std::placeholders::_1));

  // TODO(KT): Change encoder FB part for smooth motion
  encoder_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    std::string(this->get_namespace()) + "/lbr_dynamixel_controller/encoder_joint_state", 1,
    std::bind(&LimbController::encoderJointStateCallback, this, std::placeholders::_1));

  std::string joint_prefix;
  if (name_space == "/LF") {
    LIMB_ID = 0;
    joint_prefix = "LF";
  } else if (name_space == "/LH") {
    LIMB_ID = 1;
    joint_prefix = "LH";
  } else if (name_space == "/RH") {
    LIMB_ID = 2;
    joint_prefix = "RH";
  } else if (name_space == "/RF") {
    LIMB_ID = 3;
    joint_prefix = "RF";
  }

  // Define joint limits.
  LOWER_LIMIT.resize(JOINT_NUM);
  LOWER_LIMIT.at(0) = JOINT1_LOWER;
  LOWER_LIMIT.at(1) = JOINT2_LOWER;
  LOWER_LIMIT.at(2) = JOINT3_LOWER;
  LOWER_LIMIT.at(3) = JOINT4_LOWER;
  LOWER_LIMIT.at(4) = JOINT5_LOWER;
  LOWER_LIMIT.at(5) = JOINT6_LOWER;
  LOWER_LIMIT.at(6) = JOINT7_LOWER;

  UPPER_LIMIT.resize(JOINT_NUM);
  UPPER_LIMIT.at(0) = JOINT1_UPPER;
  UPPER_LIMIT.at(1) = JOINT2_UPPER;
  UPPER_LIMIT.at(2) = JOINT3_UPPER;
  UPPER_LIMIT.at(3) = JOINT4_UPPER;
  UPPER_LIMIT.at(4) = JOINT5_UPPER;
  UPPER_LIMIT.at(5) = JOINT6_UPPER;
  UPPER_LIMIT.at(6) = JOINT7_UPPER;

  // Error handling
  if (LOWER_LIMIT.size() != JOINT_NUM || UPPER_LIMIT.size() != JOINT_NUM) {
    std::cout << "Joint limits are not defined correctly." << std::endl;
    std::exit(0);
  }

  current_joint_state.position.resize(JOINT_NUM);
  current_joint_state.velocity.resize(JOINT_NUM);
  current_joint_state.name.resize(JOINT_NUM);
  current_joint_state.name.at(0) = joint_prefix + "_B2C";
  current_joint_state.name.at(1) = joint_prefix + "_C2F";
  current_joint_state.name.at(2) = joint_prefix + "_F2T";
  current_joint_state.name.at(3) = joint_prefix + "_T2E";
  current_joint_state.name.at(4) = joint_prefix + "_wristH";
  current_joint_state.name.at(5) = joint_prefix + "_wristV";
  current_joint_state.name.at(6) = joint_prefix + "_driving";
  current_joint_state.header.frame_id = this->get_namespace();

  current_joint_state.position.at(0) = 0.0;
  current_joint_state.position.at(1) = 0.0;
  current_joint_state.position.at(2) = 0.0;
  current_joint_state.position.at(3) = 0.0;
  if(grieel_mode == "gripper"){
    current_joint_state.position.at(4) = 0.0;
    current_joint_state.position.at(5) = 0.0;
  } else if(grieel_mode == "wheel"){
    current_joint_state.position.at(4) = M_PI;
    current_joint_state.position.at(5) = M_PI_2;   
  }
  current_joint_state.position.at(6) = 0.0;

  // TODO(KT): Separate visualization to different class for clarity.
  target_EE_position.header.frame_id = joint_prefix + "_limb_root";
  target_EE_position.ns = "end_effector_target_position";
  target_EE_position.id = 0;
  target_EE_position.type = visualization_msgs::msg::Marker::SPHERE;
  target_EE_position.action = visualization_msgs::msg::Marker::ADD;
  target_EE_position.scale.x = 0.03;
  target_EE_position.scale.y = 0.03;
  target_EE_position.scale.z = 0.03;
  target_EE_position.color.a = 0.5;
  target_EE_position.color.r = 1.0;
  target_EE_position.color.g = 0.0;
  target_EE_position.color.b = 0.0;

  EE_trajectory_array.markers.resize(2);
  EE_trajectory_array.markers[0].header.frame_id = joint_prefix + "_foot";
  EE_trajectory_array.markers[0].ns = "end_effector_trajectory_points";
  EE_trajectory_array.markers[0].id = 0;
  EE_trajectory_array.markers[0].type = visualization_msgs::msg::Marker::SPHERE;
  EE_trajectory_array.markers[0].action = visualization_msgs::msg::Marker::ADD;
  EE_trajectory_array.markers[0].pose.position.x = 0.0;
  EE_trajectory_array.markers[0].pose.position.y = 0.0;
  EE_trajectory_array.markers[0].pose.position.z = 0.0;
  EE_trajectory_array.markers[0].scale.x = 0.005;
  EE_trajectory_array.markers[0].scale.y = 0.005;
  EE_trajectory_array.markers[0].scale.z = 0.005;
  EE_trajectory_array.markers[0].color.a = 0.2;
  EE_trajectory_array.markers[0].color.r = 0.0;
  EE_trajectory_array.markers[0].color.g = 1.0;
  EE_trajectory_array.markers[0].color.b = 1.0;
  // lifetime decides how long markers visualized.
  EE_trajectory_array.markers[0].lifetime = rclcpp::Duration::from_seconds(10);

  EE_trajectory_array.markers[1].header.frame_id = joint_prefix + "_foot";
  EE_trajectory_array.markers[1].ns = "end_effector_trajectory_line";
  EE_trajectory_array.markers[1].id = 0;
  EE_trajectory_array.markers[1].type = visualization_msgs::msg::Marker::LINE_STRIP;
  EE_trajectory_array.markers[1].action = visualization_msgs::msg::Marker::ADD;
  EE_trajectory_array.markers[1].scale.x = 0.001;
  EE_trajectory_array.markers[1].color.a = 0.1;
  EE_trajectory_array.markers[1].color.r = 0.0;
  EE_trajectory_array.markers[1].color.g = 1.0;
  EE_trajectory_array.markers[1].color.b = 1.0;
  EE_trajectory_array.markers[1].lifetime = rclcpp::Duration::from_seconds(10);

#if DEBUG_ENABLED
  std::cout << "namespace = " << name_space << std::endl;
  std::cout << "node_name = " << this->get_name() << std::endl;
  std::cout << "topic_prefix = " << topic_prefix << std::endl;

  std::cout << "LIMB_ID = " << LIMB_ID << std::endl;
#endif  // DEBUG_ENABLED
}

void LimbController::publishEndEffectorTrajectory()
{
  geometry_msgs::msg::Point p;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  EE_trajectory_array.markers[1].points.push_back(p);
  p.x = 0.02;
  EE_trajectory_array.markers[1].points.push_back(p);

  trajectory_array_pub_->publish(EE_trajectory_array);
  EE_trajectory_array.markers[0].id += 1;
  EE_trajectory_array.markers[1].id += 1;
}

void LimbController::gripperCommandCallback(const std_msgs::msg::Bool & execute_grasping)
{
  // Send command to Dynamixel controller.
  gripping_pub_->publish(execute_grasping);

  // Send text data to RViz node.
  visualization_msgs::msg::Marker gripper_state_text;
  if (execute_grasping.data) {
    gripper_state_text.text = "CLOSE";
  } else if (!execute_grasping.data) {
    gripper_state_text.text = "OPEN";
  }
  gripper_state_text_pub_->publish(gripper_state_text);
}

std::array<float, 6> LimbController::computeFifthOrderTraj(float q0, float Dq, float T)
{
  std::array<float, 6>  coefficients;
  coefficients[0] = q0;
  coefficients[1] = 0.0; 
  coefficients[2] = 0.0; 
  coefficients[3] = 20 * Dq / (2 * std::pow(T,3));
  coefficients[4] = -30 * Dq / (2 * std::pow(T,4));
  coefficients[5] = 12 * Dq / (2 * std::pow(T,5));

  return coefficients;
}


void LimbController::grieelChangeCallback(const std_msgs::msg::String & new_grieel_mode)
{
  grieel_mode = new_grieel_mode.data;
  driving = false;
  double wristH;
  double wristV;
  // define joint configuration for desired new Grieel mode
  if (new_grieel_mode.data == "wheel"){
    if ((LIMB_ID == 0) || (LIMB_ID == 2)){
      wristH =  M_PI;
    }
    if ((LIMB_ID == 1) || (LIMB_ID == 3)){
      wristH =  M_PI;
    }
    wristV = M_PI_2;
  } else if(new_grieel_mode.data == "gripper"){
    wristH = 0.0;
    wristV = 0.0;
  }
  
  std::string name_space = std::string(this->get_namespace());
  std::cout << name_space << " Transforming into " << new_grieel_mode.data << " mode" << std::endl;

  sensor_msgs::msg::JointState temp_joint_state;
  temp_joint_state.position.resize(JOINT_NUM);
  temp_joint_state.velocity.resize(JOINT_NUM);
  temp_joint_state.position = current_joint_state.position;

  double delta_wristH = wristH - temp_joint_state.position.at(4);
  double delta_wristV = wristV - temp_joint_state.position.at(5);
  double delta_T_wristH = abs(delta_wristH / MAX_WRIST_VEL)*std::pow(10,9);  
  double delta_T_wristV = abs(delta_wristV / MAX_WRIST_VEL)*std::pow(10,9);    
  double initial_time = this->now().nanoseconds();
  double current_time = this->now().nanoseconds();
  std::array<float, 6>  a_wristH = computeFifthOrderTraj(temp_joint_state.position.at(4),delta_wristH, delta_T_wristH);
  std::array<float, 6>  a_wristV = computeFifthOrderTraj(temp_joint_state.position.at(5),delta_wristV, delta_T_wristV);
  double dt = 0.0; 

  std::cout << name_space << " Going from WristH: " << temp_joint_state.position.at(4) << "to final wristH:" << wristH << "in" << delta_T_wristH << "[ns]" << std::endl;
  std::cout << name_space << " Going from WristV: " << temp_joint_state.position.at(5) << "to final wristV:" << wristV << "in" << delta_T_wristV << "[ns]" << std::endl;
  // send grieel joint wristH, wristV trajectory as 100 step from current to desired one
  rclcpp::Rate rate(1000.0 / 10.0);  // Convert ms to Hz
  bool finish_wristH = false; 
  bool finish_wristV = false; 
  while(!finish_wristH || !finish_wristV){
    current_time = this->now().nanoseconds();
    dt = current_time - initial_time;
    if(current_time < initial_time + delta_T_wristH){
      current_joint_state.position.at(4) = a_wristH[0] + a_wristH[1]*dt + a_wristH[2]*std::pow(dt,2) + a_wristH[3]*std::pow(dt,3) + a_wristH[4]*std::pow(dt,4) + a_wristH[5]*std::pow(dt,5);
    } else{ finish_wristH = true; 
    }
    if(current_time < initial_time + delta_T_wristV){
        current_joint_state.position.at(5) = a_wristV[0] + a_wristV[1]*dt + a_wristV[2]*std::pow(dt,2)+ a_wristV[3]*std::pow(dt,3) + a_wristV[4]*std::pow(dt,4) + a_wristV[5]*std::pow(dt,5);
    }
    else{ finish_wristV = true; 
    }
    current_joint_state.velocity.at(6) = 0.0;
    // keep other joints position in the current state
    current_joint_state.header.stamp = this->now();
    joint_state_pub_->publish(current_joint_state);
    //std::this_thread::sleep_for(std::chrono::milliseconds(10)); // small sleep to avoid step like trajectory signal
    rate.sleep();
  }
  std::cout << name_space << " Finish " << new_grieel_mode.data << " mode Transformation" << std::endl;
  //std::this_thread::sleep_for(std::chrono::seconds(3));
  std_msgs::msg::String grieel_joint_finish;
  grieel_joint_finish.data = grieel_mode;
  grieel_joint_finish_pub_->publish(grieel_joint_finish);
}

void LimbController::drivingModeCallback(const std_msgs::msg::Int64 & driving_mode)//msg type)
{
  double wristH;
  // define joint configuration for desired new Grieel mode
  if((driving_mode.data == 0)&&(grieel_mode == "wheel")){
    driving = true;
    std::cout << "Going to DRIVING MODE: " << driving << "with stop = " << stop << std::endl;   
    if ((LIMB_ID == 0) || (LIMB_ID == 2)){
      wristH = 3*M_PI_4;
    }
    if ((LIMB_ID == 1) || (LIMB_ID == 3)){
      wristH = -3*M_PI_4;
    }
    sensor_msgs::msg::JointState temp_joint_state;
    temp_joint_state.position.resize(JOINT_NUM);
    temp_joint_state.velocity.resize(JOINT_NUM);
    temp_joint_state.position = current_joint_state.position;
    double delta_wristH = wristH - temp_joint_state.position.at(4);
    double delta_T_wristH = abs(delta_wristH / MAX_WRIST_VEL);  
    double initial_time = this->now().seconds();
    double current_time = this->now().seconds();
    std::array<float, 6>  a_wristH = computeFifthOrderTraj(temp_joint_state.position.at(4), delta_wristH, delta_T_wristH);
    double dt = 0.0; 
    // send grieel joint wristH, wristV trajectory as 100 step from current to desired one
    while(!(current_time > initial_time + delta_T_wristH)){
      current_time = this->now().seconds();
      dt = current_time - initial_time;
      current_joint_state.position.at(4) = a_wristH[0] + a_wristH[1]*dt + a_wristH[2]*std::pow(dt,2) + a_wristH[3]*std::pow(dt,3) + a_wristH[4]*std::pow(dt,4) + a_wristH[5]*std::pow(dt,5);
      current_joint_state.position.at(5) = M_PI_2;
      current_joint_state.velocity.at(6) = 0.0;
      // keep other joints position in the current state
      current_joint_state.header.stamp = this->now();
      joint_state_pub_->publish(current_joint_state);
      std::this_thread::sleep_for(std::chrono::milliseconds(10)); // small sleep to avoid step like trajectory signal
    }
  }
  if((driving_mode.data == 1) && (grieel_mode == "wheel")){
    std::cout << "Going to STANDARD MODE: " << stop << std::endl;   
    driving = false;
    if ((LIMB_ID == 0) || (LIMB_ID == 2)){
      wristH =  M_PI;
    }
    if ((LIMB_ID == 1) || (LIMB_ID == 3)){
      wristH =  M_PI;
    }
    sensor_msgs::msg::JointState temp_joint_state;
    temp_joint_state.position.resize(JOINT_NUM);
    temp_joint_state.velocity.resize(JOINT_NUM);
    temp_joint_state.position = current_joint_state.position;
    double delta_wristH = wristH - temp_joint_state.position.at(4);
    // send grieel joint wristH, wristV trajectory as 100 step from current to desired one
    double delta_T_wristH = abs(delta_wristH / MAX_WRIST_VEL);  
    double initial_time = this->now().seconds();
    double current_time = this->now().seconds();
    std::array<float, 6>  a_wristH = computeFifthOrderTraj(temp_joint_state.position.at(4), delta_wristH, delta_T_wristH);
    double dt = 0.0; 
    // send grieel joint wristH, wristV trajectory as 100 step from current to desired one
    while(!(current_time > initial_time + delta_T_wristH)){
      current_time = this->now().seconds();
      dt = current_time - initial_time;
      current_joint_state.position.at(4) = a_wristH[0] + a_wristH[1]*dt + a_wristH[2]*std::pow(dt,2) + a_wristH[3]*std::pow(dt,3) + a_wristH[4]*std::pow(dt,4) + a_wristH[5]*std::pow(dt,5);
      current_joint_state.position.at(5) = M_PI_2;
      current_joint_state.velocity.at(6) = 0.0;
      // keep other joints position in the current state
      current_joint_state.header.stamp = this->now();
      joint_state_pub_->publish(current_joint_state);
      std::this_thread::sleep_for(std::chrono::milliseconds(10)); // small sleep to avoid step like trajectory signal
    }
  }

  if (driving_mode.data == 100){
    stop = false;
    double driving_speed;
    direction = 1; 
    if ((LIMB_ID == 0) || (LIMB_ID == 1)){
      driving_speed = 1;
    }
    if ((LIMB_ID == 2) || (LIMB_ID == 3)){
      driving_speed = -1;
    }
    std::thread drive_thread([this, driving_speed](){
        // sensor_msgs::msg::JointState temp_joint_state;
        // temp_joint_state.position.resize(JOINT_NUM);
        // temp_joint_state.velocity.resize(JOINT_NUM);
        // temp_joint_state.position = current_joint_state.position;
        // send grieel joint wristH, wristV trajectory as 100 step from current to desired one
        while(!stop){
          current_joint_state.velocity.at(6) = 3.125*driving_speed;
          // keep other joints position in the current state
          current_joint_state.header.stamp = this->now();
          joint_state_pub_->publish(current_joint_state);
          std::this_thread::sleep_for(std::chrono::milliseconds(15));
          //std::cout << "Current STOP state: " << stop << std::endl; 
        }
      });

    drive_thread.detach();

  //std::this_thread::sleep_for(std::chrono::seconds(3));
  //PUBLISH A FEEDBACK
  }
  if (driving_mode.data == -100){
    stop = false;
    double driving_speed;
    direction = -1; 
    if ((LIMB_ID == 0) || (LIMB_ID == 1)){
      driving_speed = 1;
    }
    if ((LIMB_ID == 2) || (LIMB_ID == 3)){
      driving_speed = -1;
    }
    std::thread drive_thread([this, driving_speed](){
        // sensor_msgs::msg::JointState temp_joint_state;
        // temp_joint_state.position.resize(JOINT_NUM);
        // temp_joint_state.velocity.resize(JOINT_NUM);
        // temp_joint_state.position = current_joint_state.position;
        // send grieel joint wristH, wristV trajectory as 100 step from current to desired one
        while(!stop){
          current_joint_state.velocity.at(6) = direction*3.125*driving_speed;
          // keep other joints position in the current state
          current_joint_state.header.stamp = this->now();
          joint_state_pub_->publish(current_joint_state);
          std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }
      });

    drive_thread.detach();

  //std::this_thread::sleep_for(std::chrono::seconds(3));
  //PUBLISH A FEEDBACK
  }
  if(driving_mode.data == -1){
    stop = true;
    //std::cout << "Current STOP state: " << stop << std::endl; 
    current_joint_state.velocity.at(6) = 0.0;
    current_joint_state.header.stamp = this->now();
    joint_state_pub_->publish(current_joint_state);
  }
}


void LimbController::encoderJointStateCallback(
  const sensor_msgs::msg::JointState & encoder_joint_state)
{
#if DEBUG_ENABLED
  std::cout << "encoderJointStateCallback" << std::endl;
  std::cout << "updateJointState(encoder_joint_state) = " <<
    updateJointState(encoder_joint_state) << std::endl;
#endif  // DEBUG_ENABLED
  if (updateJointState(encoder_joint_state)) {
    current_joint_state.header.stamp = this->now();
    joint_state_pub_->publish(current_joint_state);

    // Update EE pose based on encoder feedback.
    current_EE_pose = forwardKinematics(current_joint_state);
    current_EE_pose.limb_id = LIMB_ID;
    current_EE_pose.header.stamp = this->now();
    estimated_EE_pose_pub_->publish(current_EE_pose);
  }
}

bool LimbController::updateJointState(const sensor_msgs::msg::JointState & next_joint_state)
{
  const double angle_threshold = 0.005;  // [rad]
  bool update_joint_state = false;

  for (int i = 0; i < JOINT_NUM; i++) {
    double joint_state_error = current_joint_state.position.at(i) - next_joint_state.position.at(i);
    if (abs(joint_state_error) > angle_threshold) {
      current_joint_state.position.at(i) = next_joint_state.position.at(i);
      update_joint_state = true;
    }
  }
#if DEBUG_ENABLED
  std::cout << "update_joint_state = " << update_joint_state << std::endl;
#endif  // DEBUG_ENABLED
  return update_joint_state;
}

lbr_msgs::msg::EndEffectorPoseFourDof LimbController::forwardKinematics(
  const sensor_msgs::msg::JointState & joint_state)
{
  const double theta1 = joint_state.position.at(0);
  const double theta2 = joint_state.position.at(1);
  const double theta3 = joint_state.position.at(2);
  const double theta4 = joint_state.position.at(3);

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

sensor_msgs::msg::JointState LimbController::inverseKinematics(
  const lbr_msgs::msg::EndEffectorPoseFourDof & EE_pose)
{
  // p is EE position.
  const double p_x = EE_pose.end_effector_position.x;
  const double p_y = EE_pose.end_effector_position.y;
  const double p_z = EE_pose.end_effector_position.z;
  const double pitch = EE_pose.end_effector_pitch_angle;

  sensor_msgs::msg::JointState joint_state;
  joint_state.position.resize(JOINT_NUM);
  joint_state.velocity.resize(JOINT_NUM);

  // Solve Inverse Kinematics
  double theta1, theta2, theta3, theta4;

  // theta1
  theta1 = atan2(p_y, p_x);

  // theta3
  // q is position of Joint 4.
  double q_x = p_x - (L4 * cos(pitch)) * cos(theta1);
  double q_y = p_y - (L4 * cos(pitch)) * sin(theta1);
  double q_z = p_z + L4 * sin(pitch);

#if DEBUG_ENABLED
  std::cout << "q_x = " << q_x << std::endl;
  std::cout << "q_y = " << q_y << std::endl;
  std::cout << "q_z = " << q_z << std::endl;
#endif  // DEBUG_ENABLED

  double square_of_distance_C2F_to_T2G;
  if (sqrt(pow(q_x, 2) + pow(q_y, 2)) > L1) {
    square_of_distance_C2F_to_T2G =
      pow(q_x - L1 * cos(theta1), 2) + pow(q_y - L1 * sin(theta1), 2) + pow(q_z, 2);
  } else if (sqrt(pow(q_x, 2) + pow(q_y, 2)) <= L1) {
    square_of_distance_C2F_to_T2G =
      pow(-q_x + L1 * cos(theta1), 2) + pow(-q_y + L1 * sin(theta1), 2) + pow(q_z, 2);
  }

  double cosine_pi_minus_theta3 =
    (pow(L2, 2) + pow(L3, 2) - square_of_distance_C2F_to_T2G) / (2 * L2 * L3);
  double cosine_theta3 = -cosine_pi_minus_theta3;
  // This is elbow up position
  theta3 = atan2(sqrt(1 - pow(cosine_theta3, 2)), cosine_theta3);

  // theta2
  double alpha;
  if (sqrt(pow(q_x, 2) + pow(q_y, 2)) > L1) {
    alpha = atan2(-q_z, sqrt(pow(q_x - L1 * cos(theta1), 2) + pow(q_y - L1 * sin(theta1), 2)));
  } else if (sqrt(pow(q_x, 2) + pow(q_y, 2)) <= L1) {
    alpha = atan2(-q_z, -sqrt(pow(-q_x + L1 * cos(theta1), 2) + pow(-q_y + L1 * sin(theta1), 2)));
  }
  double beta = atan2(L3 * sin(theta3), L2 + L3 * cos(theta3));
  theta2 = alpha - beta;
#if DEBUG_ENABLED
  std::cout << "alpha = " << alpha << std::endl;
  std::cout << "beta = " << beta << std::endl;
#endif  // DEBUG_ENABLED

  // theta4
  theta4 = pitch - theta2 - theta3;

  joint_state.position.at(0) = theta1;
  joint_state.position.at(1) = theta2;
  joint_state.position.at(2) = theta3;
  joint_state.position.at(3) = theta4;

  // keep grieel joint fixed on mode configuration for now
  if(!driving){
    if(grieel_mode == "gripper"){
      joint_state.position.at(4) = 0.0;
      joint_state.position.at(5) = 0.0;
      joint_state.velocity.at(6) = 0.0;
    }
    if(grieel_mode == "wheel"){
      if ((LIMB_ID == 0) || (LIMB_ID == 2)){
        joint_state.position.at(4) =  M_PI;
      }
      if ((LIMB_ID == 1) || (LIMB_ID == 3)){
        joint_state.position.at(4) =  M_PI;
      }
      joint_state.position.at(5) =  M_PI_2;
      joint_state.velocity.at(6) = 0.0;
    }
  }
  if((driving) && (grieel_mode == "wheel")){
    if(!stop){
      std::cout << "Sending IK DRIVING condition" << std::endl;
      if ((LIMB_ID == 0) || (LIMB_ID == 2)){
        joint_state.position.at(4) = 3*M_PI_4;
        joint_state.position.at(5) =  M_PI_2;
        joint_state.velocity.at(6) = direction * 3.125;
      }
      if ((LIMB_ID == 1) || (LIMB_ID == 3)){
        joint_state.position.at(4) = - 3*M_PI_4;
        joint_state.position.at(5) =  M_PI_2;
        joint_state.velocity.at(6) = direction * -3.125;
      }
    }
    if(stop){
      std::cout << "Sending IK STOP condition" << std::endl;
      if ((LIMB_ID == 0) || (LIMB_ID == 2)){
        joint_state.position.at(4) =  3*M_PI_4;
        joint_state.position.at(5) =  M_PI_2;
        joint_state.velocity.at(6) = 0.0;
      }
      if ((LIMB_ID == 1) || (LIMB_ID == 3)){
        joint_state.position.at(4) = - 3*M_PI_4;
        joint_state.position.at(5) =  M_PI_2;
        joint_state.velocity.at(6) = 0.0;
      }
    }
  }




#if DEBUG_ENABLED
  std::cout << "theta1 = " << theta1 << std::endl;
  std::cout << "theta2 = " << theta2 << std::endl;
  std::cout << "theta3 = " << theta3 << std::endl;
  std::cout << "theta4 = " << theta4 << std::endl;
#endif  // DEBUG_ENABLED

  return joint_state;
}

bool LimbController::checkInverseKinematicsSolution(
  const sensor_msgs::msg::JointState & joint_state)
{
  for (int i = 0; i < JOINT_NUM; i++) {
    if (isnan(joint_state.position.at(i))) {
      std::cout << "Joint" << i + 1 << " has NaN." << std::endl;
      return false;
    }
  }
  return true;
}

bool LimbController::checkJointLimit(
  const sensor_msgs::msg::JointState & joint_state)
{
  // Check joint limits
  for (int i = 0; i < JOINT_NUM; i++) {
    if (joint_state.position.at(i) < LOWER_LIMIT.at(i)) {
      std::cout << "Joint" << i + 1 << " reaches its lower joint limit." << std::endl;
      return false;
    } else if (joint_state.position.at(i) > UPPER_LIMIT.at(i)) {
      std::cout << "Joint" << i + 1 << " reaches its upper joint limit." << std::endl;
      return false;
    }
  }

  return true;
}

void LimbController::endEffectorPoseCallback(
  const lbr_msgs::msg::EndEffectorPoseFourDof & EE_pose_msg)
{
  sensor_msgs::msg::JointState temp_joint_state = inverseKinematics(EE_pose_msg);

  // Check if IK solution is valid or not.
  bool joint_state_is_valid = checkInverseKinematicsSolution(temp_joint_state);
  if (joint_state_is_valid) {
    bool joint_limit_is_ok = checkJointLimit(temp_joint_state);
    if (joint_limit_is_ok) {
      // Update joint state
      for (int i = 0; i < JOINT_NUM; i++) {
        current_joint_state.position.at(i) = temp_joint_state.position.at(i);
      }
      current_joint_state.velocity.at(6) = temp_joint_state.velocity.at(6);
      // Update EE pose
      current_EE_pose = EE_pose_msg;
    } else {
      std::cout <<
        "joint_state is not updated because one of joints reaches the limit." << std::endl;
    }
  } else {
    std::cout << "joint_state is not updated because IK solution has NaN value." << std::endl;
  }

  // Publish joint state to Dynamixel controller.
  current_joint_state.header.stamp = this->now();
  joint_state_pub_->publish(current_joint_state);

  // Publish visualization related topics.
  enable_rviz_debug = this->get_parameter("rviz_debug").as_bool();
  if (enable_rviz_debug) {
    rviz_joint_state_pub_->publish(current_joint_state);
  }
  // publishEndEffectorTrajectory();  // !: This code causes memory leak. Need to modify.
  target_EE_position.pose.position.x = current_EE_pose.end_effector_position.x;
  target_EE_position.pose.position.y = current_EE_pose.end_effector_position.y;
  target_EE_position.pose.position.z = current_EE_pose.end_effector_position.z;
  target_position_pub_->publish(target_EE_position);
#if DEBUG_ENABLED
  std::cout << "LC::endEffectorPoseCallback" << std::endl;

  std::cout << "current_EE_pose.limb_id = " << current_EE_pose.limb_id << std::endl;
  std::cout << "current_EE_pose.end_effector_position.x = " <<
    current_EE_pose.end_effector_position.x << std::endl;
  std::cout << "current_EE_pose.end_effector_position.y = " <<
    current_EE_pose.end_effector_position.y << std::endl;
  std::cout << "current_EE_pose.end_effector_position.z = " <<
    current_EE_pose.end_effector_position.z << std::endl;
  std::cout << "current_EE_pose.end_effector_pitch_angle = " <<
    current_EE_pose.end_effector_pitch_angle << std::endl;
#endif  // DEBUG_ENABLED
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LimbController>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
