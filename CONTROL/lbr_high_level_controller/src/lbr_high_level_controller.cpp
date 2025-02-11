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

#include "lbr_high_level_controller/lbr_high_level_controller.hpp"

#define DEBUG_ENABLED false
#define SIMULATION true

HighLevelController::HighLevelController()
: Node("lbr_high_level_controller")
{
  std::cout << "HighLevelController class is established." << std::endl;

  // Get node name.
  std::string topic_prefix = "/" + std::string(this->get_name());
#if DEBUG_ENABLED
  std::cout << "node_name = " << this->get_name() << std::endl;
  std::cout << "topic_prefix = " << topic_prefix << std::endl;
#endif  // DEBUG_ENABLED

  // Publisher
  base_motion_pub_ = this->create_publisher<lbr_msgs::msg::BaseMotionTask>(
    topic_prefix + "/motion_package/base_motion_task", 1);
  limb_motion_pub_ = this->create_publisher<lbr_msgs::msg::LimbMotionTask>(
    topic_prefix + "/motion_package/limb_motion_task", 1);
  motion_package_pub_ = this->create_publisher<lbr_msgs::msg::MotionPackage>(
    topic_prefix + "/motion_package", 1);
  grasping_detect_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/lbr_low_level_controller/grasping_detect", 1);
  grasping_trace_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/lbr_low_level_controller/grasping_trace", 1);
  grasping_detect_pitch_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/lbr_low_level_controller/grasping_detect_pitch", 1);
  grieel_mode_pub_ = this->create_publisher<lbr_msgs::msg::GrieelModeChange>(
    "/lbr_high_level_controller/grieel_mode_change_config", 1);
  grieel_driving_configuration_pub_ = this->create_publisher<std_msgs::msg::Int64>(
    "/lbr_high_level_controller/grieel_driving_configuration", 1);
  msg_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/user_command", 1);

  fake_end_effector_contact_state_pub_ =
    this->create_publisher<lbr_msgs::msg::EndEffectorContactState>(
    topic_prefix + "/contact_state", 10);

  // Subscriber
  user_command_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/user_command", 5,
    std::bind(&HighLevelController::userCommandCallback, this, std::placeholders::_1));
  motion_finish_signal_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/lbr_low_level_controller/motion_finish_signal", 3,
    std::bind(&HighLevelController::motionFinishSignalCallback, this, std::placeholders::_1));
  supporting_leg_triangle_sub_ = this->create_subscription<lbr_msgs::msg::EndEffectorContactState>(
    "/lbr_state_estimator/supporting_leg_triangle_points", 1,
    std::bind(&HighLevelController::endEffectorContactStateCallback, this, std::placeholders::_1));
  command_interface_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/lbr_command_interface/command/high_level_controller", 5,
    std::bind(&HighLevelController::interfaceCommandCallback, this, std::placeholders::_1));

  grieel_transform_end_sub_LF_ = this->create_subscription<std_msgs::msg::String>(
    "/LF/lbr_dynamixel_controller/grieel_transform_end",1,
    std::bind(&HighLevelController::grieelFinishCallback, this, std::placeholders::_1));
  grieel_transform_end_sub_LH_ = this->create_subscription<std_msgs::msg::String>(
    "/LH/lbr_dynamixel_controller/grieel_transform_end",1,
    std::bind(&HighLevelController::grieelFinishCallback, this, std::placeholders::_1));
  grieel_transform_end_sub_RH_ = this->create_subscription<std_msgs::msg::String>(
    "/RH/lbr_dynamixel_controller/grieel_transform_end",1,
    std::bind(&HighLevelController::grieelFinishCallback, this, std::placeholders::_1));
  grieel_transform_end_sub_RF_ = this->create_subscription<std_msgs::msg::String>(
    "/RF/lbr_dynamixel_controller/grieel_transform_end",1,
    std::bind(&HighLevelController::grieelFinishCallback, this, std::placeholders::_1));

  grieel_end_sub_LF_ = this->create_subscription<std_msgs::msg::String>(
      "/LF/lbr_limb_controller/grieel_joint_finish", 1,
      std::bind(&HighLevelController::updateGrieelState, this, std::placeholders::_1));
  grieel_end_sub_LH_ = this->create_subscription<std_msgs::msg::String>(
      "/LH/lbr_limb_controller/grieel_joint_finish", 1,
      std::bind(&HighLevelController::updateGrieelState, this, std::placeholders::_1));
  grieel_end_sub_RH_ = this->create_subscription<std_msgs::msg::String>(
      "/RH/lbr_limb_controller/grieel_joint_finish", 1,
      std::bind(&HighLevelController::updateGrieelState, this, std::placeholders::_1));
  grieel_end_sub_RF_ = this->create_subscription<std_msgs::msg::String>(
      "/RF/lbr_limb_controller/grieel_joint_finish", 1,
      std::bind(&HighLevelController::updateGrieelState, this, std::placeholders::_1));
  


  trot_gait_is_executing = false;
  motion_package_end = false;
  grieel_transformation_ = false;
  single_transform_end_ = false;
  grieel_transform_end = false; 

  move = false;
  crawl_gate = false;

  this->declare_parameter("wheel_mode", true);

  bool wheel_mode;
  this->get_parameter("wheel_mode", wheel_mode);

  std::cout << "HLC: wheel_mode:= " << wheel_mode << std::endl; 
  if(wheel_mode == true){
    grieel_state_ = "wheel";
    for(int i=0; i<LIMB_NUM; i++){
      grieel_state_list_[i] = true; 
    }
  } else if(wheel_mode == false){
    grieel_state_ = "gripper";
    for(int i=0; i<LIMB_NUM; i++){
      grieel_state_list_[i] = false; 
    }
  }

  fake_contact.is_contact.resize(LIMB_NUM);
}

void HighLevelController::userCommandCallback(const std_msgs::msg::String & msg)
{
#if DEBUG_ENABLED
  std::cout << "subscribed msg = " << msg.data << std::endl;
#endif  // DEBUG_ENABLED
  if (msg.data == "limb_motion_debug") {
    lbr_msgs::msg::LimbMotionTask limb_motion_task;
    limb_motion_task.limb_id = 0;
    limb_motion_task.swing_duration = 5000;  // [ms]
    limb_motion_task.end_effector_displacement.x = 0.1;  // [m]
    limb_motion_task.end_effector_displacement.y = 0.0;
    limb_motion_task.end_effector_displacement.z = 0.0;
    limb_motion_task.header.stamp = this->now();

    limb_motion_pub_->publish(limb_motion_task);
  }

  if (msg.data == "base_motion_debug") {
    lbr_msgs::msg::BaseMotionTask base_motion;
    base_motion.motion_duration = 5000;
    base_motion.base_displacement.x = -0.1;
    base_motion.base_displacement.y = 0.0;
    base_motion.base_displacement.z = 0.0;
    base_motion.base_angle_displacement.x = 0.0;
    base_motion.base_angle_displacement.y = 0.0;
    base_motion.base_angle_displacement.z = 0.0;
    base_motion.header.stamp = this->now();

    base_motion_pub_->publish(base_motion);
  }

  if (msg.data == "Crawl_Gait_debug") {
    // geometry_msgs::msg::Twist mv;
    // mv.linear.x = 0.05;
    // mv.linear.y = 0.05;
    // generateCrawlGait(mv);
    stabilizeBaseInSupportingLegsTriangle(supporting_leg_polygon);
  }

  if (msg.data == "trot_gait_debug") {
    geometry_msgs::msg::Twist mv;
    mv.linear.x = 0.05;
    mv.linear.y = 0.05;
    generateTrotGait(mv);
  }

  std_msgs::msg::Bool grasp_trace;
  lbr_msgs::msg::LimbMotionTask guarantee_limb_motion_task;
  guarantee_limb_motion_task.limb_id = 0;
  guarantee_limb_motion_task.swing_duration = 2000;  // [ms]
  guarantee_limb_motion_task.end_effector_displacement.x = 0.0;  // [m]
  guarantee_limb_motion_task.end_effector_displacement.y = 0.0;
  guarantee_limb_motion_task.end_effector_displacement.z = 0.0;
  guarantee_limb_motion_task.pitch_angle_displacement = 0.0;  // [rad]
  guarantee_limb_motion_task.header.stamp = this->now();
  if (msg.data == "tracing_grasp_plus_x") {
    guarantee_limb_motion_task.end_effector_displacement.x = 0.01;  // [m]

    limb_motion_pub_->publish(guarantee_limb_motion_task);
    // TODO(KT): change sleep() to avoid bug.
    usleep(2500000);  // [micro seconds]
    grasp_trace.data = true;
    grasping_trace_pub_->publish(grasp_trace);  // Start detect tracing
  }
  if (msg.data == "tracing_grasp_minus_x") {
    guarantee_limb_motion_task.end_effector_displacement.x = -0.01;  // [m]

    limb_motion_pub_->publish(guarantee_limb_motion_task);
    usleep(2500000);
    grasp_trace.data = true;
    grasping_trace_pub_->publish(grasp_trace);  // Start detect tracing
  }
  if (msg.data == "tracing_grasp_plus_y") {
    guarantee_limb_motion_task.end_effector_displacement.y = 0.01;  // [m]

    limb_motion_pub_->publish(guarantee_limb_motion_task);
    usleep(2500000);
    grasp_trace.data = true;
    grasping_trace_pub_->publish(grasp_trace);  // Start detect tracing
  }
  if (msg.data == "tracing_grasp_minus_y") {
    guarantee_limb_motion_task.end_effector_displacement.y = -0.01;  // [m]

    limb_motion_pub_->publish(guarantee_limb_motion_task);
    usleep(2500000);
    grasp_trace.data = true;
    grasping_trace_pub_->publish(grasp_trace);  // Start detect tracing
  }
  if (msg.data == "grasp_detect") {
    guarantee_limb_motion_task.end_effector_displacement.z = 0.02;  // [m]

    limb_motion_pub_->publish(guarantee_limb_motion_task);
    usleep(2500000);
    std_msgs::msg::Bool grasp_detect;
    grasp_detect.data = true;
    grasping_detect_pub_->publish(grasp_detect);  // Start grasping determination
  }
  if (msg.data == "grasp_detect_return") {
    guarantee_limb_motion_task.end_effector_displacement.z = -0.02;  // [m]

    limb_motion_pub_->publish(guarantee_limb_motion_task);
    usleep(2500000);
    std_msgs::msg::String motion_msg;
    motion_msg.data = "grasp_detect_pitch";
    msg_pub_->publish(motion_msg);  // to high level controller
  }
  if (msg.data == "grasp_detect_pitch") {
    guarantee_limb_motion_task.pitch_angle_displacement = 0.262;  // [rad], 15deg

    limb_motion_pub_->publish(guarantee_limb_motion_task);
    usleep(2500000);
    std_msgs::msg::Bool grasp_detect_pitch;
    grasp_detect_pitch.data = true;
    // Start grasping determination of pitch
    grasping_detect_pitch_pub_->publish(grasp_detect_pitch);
  }
}

void HighLevelController::interfaceCommandCallback(const std_msgs::msg::String & msg)
{
  if(msg.data == "grieel_mode_transform"){
    //uncomment during experiments until contact sensors available.
    fake_contact.is_contact.at(0) = true;
    fake_contact.is_contact.at(1) = true;
    fake_contact.is_contact.at(2) = true;
    fake_contact.is_contact.at(3) = true;
    fake_contact.header.stamp = this->now();
    fake_end_effector_contact_state_pub_->publish(fake_contact);
    std::cout << "pub fake contact" << std::endl;

    if((supporting_leg_polygon.end_effector_position.size() > 3)&&!(grieel_transformation_)){
      grieel_transformation_ = true;
      std::this_thread::sleep_for(std::chrono::milliseconds(100)); // small delay to avoid multiple request
      std::cout << "Grieel Transformation task start" << std::endl;
      std::thread(&HighLevelController::grieelTransformation, this).detach();
    }
    else if(supporting_leg_polygon.end_effector_position.size() <= 3){
      std::cout << "Not all legs in contact..." << std::endl;
    }
  }

  if(msg.data == "center_base"){
    if((supporting_leg_polygon.end_effector_position.size() > 3)&&!(grieel_transformation_)){
      std::cout << "centering base in the middle of support polygon" << std::endl;
      std::thread(&HighLevelController::centerBase,this, supporting_leg_polygon).detach();
    }
    else if(supporting_leg_polygon.end_effector_position.size() <= 3){
      std::cout << "Not 4 legs in contact" << std::endl;
    }
    else if(grieel_transformation_){
      std::cout << "currently transforming." << std::endl;
    }
    else{
      std::cout << "support_leg.size = " << supporting_leg_polygon.end_effector_position.size() << "grieel_transformation = " << grieel_transformation_ << std::endl;
    }
  }
  if(msg.data == "move_forward"){
    move = true;
    if(!crawl_gate){
      crawl_gate = true;
      std::thread(&HighLevelController::crawlGate,this).detach();
    }
  }
  if(msg.data == "stop_move"){
    crawl_gate = false;
    move = false;
  }

  if(msg.data == "driving_forward_config_LF"){
    std_msgs::msg::Int64 driving_configuration;
    driving_configuration.data = 0;
    grieel_driving_configuration_pub_->publish(driving_configuration);
  }
  if(msg.data == "driving_forward_config_LH"){
    std_msgs::msg::Int64 driving_configuration;
    driving_configuration.data = 1;
    grieel_driving_configuration_pub_->publish(driving_configuration);
  }
  if(msg.data == "driving_forward_config_RH"){
    std_msgs::msg::Int64 driving_configuration;
    driving_configuration.data = 2;
    grieel_driving_configuration_pub_->publish(driving_configuration);
  }
  if(msg.data == "driving_forward_config_RF"){
    std_msgs::msg::Int64 driving_configuration;
    driving_configuration.data = 3;
    grieel_driving_configuration_pub_->publish(driving_configuration);
  }
  if(msg.data == "driving_forward_config_ALL"){
    std_msgs::msg::Int64 driving_configuration;
    driving_configuration.data = 4;
    grieel_driving_configuration_pub_->publish(driving_configuration);
  }


  if(msg.data == "standard_config_LF"){
    std_msgs::msg::Int64 driving_configuration;
    driving_configuration.data = 10;
    grieel_driving_configuration_pub_->publish(driving_configuration);
  }
  if(msg.data == "standard_config_LH"){
    std_msgs::msg::Int64 driving_configuration;
    driving_configuration.data = 11;
    grieel_driving_configuration_pub_->publish(driving_configuration);
  }
  if(msg.data == "standard_config_RH"){
    std_msgs::msg::Int64 driving_configuration;
    driving_configuration.data = 12;
    grieel_driving_configuration_pub_->publish(driving_configuration);
  }
  if(msg.data == "standard_config_RF"){
    std_msgs::msg::Int64 driving_configuration;
    driving_configuration.data = 13;
    grieel_driving_configuration_pub_->publish(driving_configuration);
  }
  if(msg.data == "standard_config_ALL"){
    std_msgs::msg::Int64 driving_configuration;
    driving_configuration.data = 14;
    grieel_driving_configuration_pub_->publish(driving_configuration);
  }

  if(msg.data == "driving_forward"){
    std_msgs::msg::Int64 driving_configuration;
    driving_configuration.data = 100;
    grieel_driving_configuration_pub_->publish(driving_configuration);
  }
  if(msg.data == "stop_driving"){
    std_msgs::msg::Int64 driving_configuration;
    driving_configuration.data = -1;
    grieel_driving_configuration_pub_->publish(driving_configuration);
  }
  if(msg.data == "driving_backward"){
    std_msgs::msg::Int64 driving_configuration;
    driving_configuration.data = -100;
    grieel_driving_configuration_pub_->publish(driving_configuration);
  }
}

void::HighLevelController::grieelTransformation()
{
  bool wheel_mode;
  this->get_parameter("wheel_mode", wheel_mode);

  if(wheel_mode == true){
    grieel_state_ = "wheel";
  } else if(wheel_mode == false){
    grieel_state_ = "gripper";
  }

  int i= 0;
  bool first_time = true;
  while(i < LIMB_NUM){
#if SIMULATION
    if((motion_package_end || first_time)&&(supporting_leg_polygon.end_effector_position.size() > 3)&&(grieel_transform_end || first_time)){
#endif //SIMULATION
#if !SIMULATION
    if((single_transform_end_ || first_time)&&(supporting_leg_polygon.end_effector_position.size() > 3)){
#endif //NOT IN SIMULATIONsingle_transform_end_
      for(int j=0; (j<LIMB_NUM) ; j++ ){
        if(j==i){fake_contact.is_contact.at(i) = false;}
        else{
          fake_contact.is_contact.at(j) = true;
        }
      }
      fake_contact.header.stamp = this->now();
      fake_end_effector_contact_state_pub_->publish(fake_contact);

      motion_package_end = false;
      single_transform_end_ = false;
      first_time = false;
      grieel_transform_end = false; 
      singleLegGrieelTransformation(supporting_leg_polygon, i);

      i = i+1;
    }
  grieel_transformation_ = false;
  // update overall grieel state
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

  while(!motion_package_end){
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // wait for final task to end
  }
  std::cout << "Grieel Transform task end" << std::endl;

  while(supporting_leg_polygon.end_effector_position.size() <= 4){
    // wait for final limb to lay on the ground
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if(supporting_leg_polygon.end_effector_position.size() == 4){
      centerBase(supporting_leg_polygon);
    std::cout << "Centering Base" << std::endl;
    }
  }

  if(grieel_state_ == "wheel"){
    grieel_state_ = "gripper";
  } else if(grieel_state_ == "gripper"){
    grieel_state_ = "wheel";
  }

 }


void HighLevelController::singleLegGrieelTransformation(
  const lbr_msgs::msg::EndEffectorContactState & supporting_leg_polygon, int limb_id)
{
  std::cout << "Leg " << limb_id << " transformation start"<< std::endl;
  lbr_msgs::msg::EndEffectorContactState next_support_polygon;
  next_support_polygon.end_effector_position.resize(LIMB_NUM -1);
  for (int j = 0; j < LIMB_NUM ; j++ ){
    if(j!=limb_id){
      if(j>limb_id){
        next_support_polygon.end_effector_position.at(j-1) = supporting_leg_polygon.end_effector_position.at(j);
      }
      else{
        next_support_polygon.end_effector_position.at(j) = supporting_leg_polygon.end_effector_position.at(j);
      }
    }
  }

  double vecX_other;
  double vecY_other;
  if (limb_id == 0 || limb_id == 2){
    vecX_other = (supporting_leg_polygon.end_effector_position.at(1).x +
                  supporting_leg_polygon.end_effector_position.at(3).x) / 2;
    vecY_other = (supporting_leg_polygon.end_effector_position.at(1).y +
                  supporting_leg_polygon.end_effector_position.at(3).y) / 2;
  }
  else{
    vecX_other = (supporting_leg_polygon.end_effector_position.at(0).x +
                supporting_leg_polygon.end_effector_position.at(2).x) / 2;
    vecY_other = (supporting_leg_polygon.end_effector_position.at(0).y +
                  supporting_leg_polygon.end_effector_position.at(2).y) / 2;
  }

#if !SIMULATION
  const int task_num = 2;
#endif
#if SIMULATION
  const int task_num = 2;
#endif
  lbr_msgs::msg::MotionPackage basic_transfrom_sequence;
  lbr_msgs::msg::MotionTask motion_task;

  motion_task.task_id_array.resize(task_num);
  basic_transfrom_sequence.motion_task_array.resize(task_num);

  Eigen::Vector3d p1 = {next_support_polygon.end_effector_position.at(0).x,
      next_support_polygon.end_effector_position.at(0).y,
      next_support_polygon.end_effector_position.at(0).z};
  Eigen::Vector3d p2 = {next_support_polygon.end_effector_position.at(1).x,
      next_support_polygon.end_effector_position.at(1).y,
      next_support_polygon.end_effector_position.at(1).z};
  Eigen::Vector3d p3 = {next_support_polygon.end_effector_position.at(2).x,
      next_support_polygon.end_effector_position.at(2).y,
      next_support_polygon.end_effector_position.at(2).z};

  Eigen::Vector3d base_link = {0.0, 0.0, 0.0};  // Base link x, y position

  lbr_msgs::msg::BaseMotionTask base_motion;
    // if (isInsideTriangle(base_link, p1, p2, p3)) {
    //   std::cout << "Base link is inside the triangle." << std::endl;
    //   double vecX = 0.0;
    //   double vecY = 0.0;
    //   base_motion.motion_duration = 1000;
    //   base_motion.base_displacement.x = vecX;
    //   base_motion.base_displacement.y = vecY;
    // } else {
    //  std::cout << "Base link is outside the triangle." << std::endl;

  double vecX = ((p1(0) + p2(0) + p3(0)) / 3) - base_link(0);
  double vecY = ((p1(1) + p2(1) + p3(1)) / 3) - base_link(1);

  double vecX_final = vecX_other + (vecX - vecX_other) / 3;
  double vecY_final = vecY_other + (vecY - vecY_other) / 3;

  base_motion.motion_duration = 3000;
  base_motion.base_displacement.x = vecX_final;
  base_motion.base_displacement.y = vecY_final;
    //}

  base_motion.header.stamp = this->now();
  //base_motion_pub_->publish(base_motion);
  motion_task.base_motion_task = base_motion;
  motion_task.task_id_array.at(0) = 1;
  basic_transfrom_sequence.motion_task_array.at(0) = motion_task;

  lbr_msgs::msg::LimbMotionTask limb_motion_task;
  limb_motion_task.limb_id = limb_id;
  limb_motion_task.swing_duration = 3500;  // [ms]
  limb_motion_task.end_effector_displacement.x = 0.0;  // [m]
  limb_motion_task.end_effector_displacement.y = 0.0;
  limb_motion_task.end_effector_displacement.z = LIMB_TRANSFORM_HEIGHT;
// #if !SIMULATION
//   if(grieel_state_ == "wheel"){
//     limb_motion_task.end_effector_displacement.z = 2*LIMB_TRANSFORM_HEIGHT;
//    }else if(grieel_state_ == "gripper"){
//     limb_motion_task.end_effector_displacement.z = LIMB_TRANSFORM_HEIGHT;
//   }
// #endif
// #if SIMULATION
//   limb_motion_task.end_effector_displacement.z = LIMB_TRANSFORM_HEIGHT;
// #endif

  limb_motion_task.pitch_angle_displacement = 0.0;  // [rad]
  limb_motion_task.header.stamp = this->now();

  motion_task.limb_motion_task = limb_motion_task;
  motion_task.task_id_array.at(1) = 0;
  basic_transfrom_sequence.motion_task_array.at(1) = motion_task;

  motion_package_pub_->publish(basic_transfrom_sequence);

  while(!motion_package_end){
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

#if SIMULATION
  lbr_msgs::msg::GrieelModeChange new_grieel_state;
  new_grieel_state.limb_id = limb_id;
  if(grieel_state_list_[limb_id] == true){
    grieel_state_list_[limb_id] = false;

    new_grieel_state.mode = "gripper";
    grieel_mode_pub_->publish(new_grieel_state);
  } else if(grieel_state_list_[limb_id] == false){
    grieel_state_list_[limb_id] = true;

    new_grieel_state.mode = "wheel";
    grieel_mode_pub_->publish(new_grieel_state);
  }

  std::thread wait_thread([this, limb_id]() {
    bool wait = true;
    while(wait){
      if(grieel_transform_end){
        wait = false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }  
    std::cout << " sending lay down command" << std::endl;
    const int task_num_final = 1;  

    lbr_msgs::msg::MotionPackage basic_transfrom_sequence;
    lbr_msgs::msg::MotionTask motion_task;
    lbr_msgs::msg::LimbMotionTask limb_motion_task;

    motion_task.task_id_array.resize(task_num_final);
    basic_transfrom_sequence.motion_task_array.resize(task_num_final);
    limb_motion_task.limb_id = limb_id;
    limb_motion_task.swing_duration = 3500;  // [ms]
    limb_motion_task.end_effector_displacement.x = 0.0;  // [m]
    limb_motion_task.end_effector_displacement.y = 0.0;
    limb_motion_task.end_effector_displacement.z = -LIMB_TRANSFORM_HEIGHT;
    limb_motion_task.pitch_angle_displacement = 0.0;  // [rad]
    limb_motion_task.header.stamp = this->now();
  
    motion_task.limb_motion_task = limb_motion_task;
    motion_task.task_id_array.at(0) = 0;
  
    basic_transfrom_sequence.motion_task_array.at(0) = motion_task;
    motion_package_pub_->publish(basic_transfrom_sequence);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));});
  wait_thread.detach();


  // SEND COMMAND TO GRIEEL FOR TRANSITION MODE
  // limb_motion_task.limb_id = limb_id;
  // limb_motion_task.swing_duration = 6000;  // [ms]
  // limb_motion_task.end_effector_displacement.x = 0.0;  // [m]
  // limb_motion_task.end_effector_displacement.y = 0.0;
  // limb_motion_task.end_effector_displacement.z = 0.0;
  // limb_motion_task.pitch_angle_displacement = 0.0;  // [rad]
  // limb_motion_task.header.stamp = this->now();

  // motion_task.limb_motion_task = limb_motion_task;
  // motion_task.task_id_array.at(2) = 0;
  // basic_transfrom_sequence.motion_task_array.at(2) = motion_task;

  // // HERE CHECK that grieel finish the transition, and put back leg after received feedback from grieel!
  // // the leg back on the ground will not be part of this motion package, but executed as limb motion later.
  // limb_motion_task.limb_id = limb_id;
  // limb_motion_task.swing_duration = 3500;  // [ms]
  // limb_motion_task.end_effector_displacement.x = 0.0;  // [m]
  // limb_motion_task.end_effector_displacement.y = 0.0;
  // limb_motion_task.end_effector_displacement.z = - LIMB_TRANSFORM_HEIGHT;
  // limb_motion_task.pitch_angle_displacement = 0.0;  // [rad]
  // limb_motion_task.header.stamp = this->now();

  // motion_task.limb_motion_task = limb_motion_task;
  // motion_task.task_id_array.at(3) = 0;
  // basic_transfrom_sequence.motion_task_array.at(3) = motion_task;
#endif //SIMULATION

  // motion_package_pub_->publish(basic_transfrom_sequence);

  // SEND COMMAND TO GRIEEL FOR TRANSITION MODE
#if !SIMULATION
  std::thread wait_thread([this, limb_id]() {
  bool wait = true;
  while(wait){
    if(motion_package_end){
      // as soon as motion mackage end is received
      lbr_msgs::msg::GrieelModeChange new_grieel_state;
      new_grieel_state.limb_id = limb_id;
      if(grieel_state_ == "wheel"){
        new_grieel_state.mode = "gripper";
        grieel_mode_pub_->publish(new_grieel_state);
      } else if(grieel_state_ == "gripper"){
        new_grieel_state.mode = "wheel";
        grieel_mode_pub_->publish(new_grieel_state);
      }
      wait = false;
    }}});
  wait_thread.detach();
#endif //NOT IN SIMULATION
    fake_contact.is_contact.at(0) = true;
    fake_contact.is_contact.at(1) = true;
    fake_contact.is_contact.at(2) = true;
    fake_contact.is_contact.at(3) = true;
    fake_contact.header.stamp = this->now();
    fake_end_effector_contact_state_pub_->publish(fake_contact);
}

void HighLevelController::grieelFinishCallback(const std_msgs::msg::String &grieel_finish_string){
  // used only when dynamixel controller is running
  std::cout <<"put limb down" << std::endl;
  int limb_id;
  if(grieel_finish_string.data == "/LF"){
    limb_id = 0;
  } else if(grieel_finish_string.data == "/LH"){
    limb_id = 1;
  } else if(grieel_finish_string.data == "/RH"){
    limb_id = 2;
  } else if(grieel_finish_string.data == "/RF"){
    limb_id = 3;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::thread lay_down([this, limb_id]() {
    lbr_msgs::msg::LimbMotionTask limb_motion_task;
    limb_motion_task.limb_id = limb_id;
    limb_motion_task.swing_duration = 3500;  // [ms]
    limb_motion_task.end_effector_displacement.x = 0.0;  // [m]
    limb_motion_task.end_effector_displacement.y = 0.0;
    // while(supporting_leg_polygon.end_effector_position.size() < 3 ){
    //   std::cout <<"pub next limb task" << std::endl;
    //   limb_motion_task.end_effector_displacement.z = -0.02;
    //   limb_motion_task.pitch_angle_displacement = 0.0;  // [rad]
    //   limb_motion_task.header.stamp = this->now();
    //   limb_motion_pub_->publish(limb_motion_task);
    //   single_transform_end_ = true;
    //   std::this_thread::sleep_for(std::chrono::milliseconds(50));

    limb_motion_task.end_effector_displacement.z = -LIMB_TRANSFORM_HEIGHT + 0.05;
    limb_motion_task.pitch_angle_displacement = 0.0;  // [rad]
    limb_motion_task.header.stamp = this->now();
    limb_motion_pub_->publish(limb_motion_task);


    fake_contact.is_contact.at(0) = true;
    fake_contact.is_contact.at(1) = true;
    fake_contact.is_contact.at(2) = true;
    fake_contact.is_contact.at(3) = true;
    fake_contact.header.stamp = this->now();
    fake_end_effector_contact_state_pub_->publish(fake_contact);

    std::this_thread::sleep_for(std::chrono::milliseconds(3500));
    single_transform_end_ = true;

    });

  lay_down.detach();
  std::cout <<"start lay down limb task" << std::endl;


}

void HighLevelController::updateGrieelState(const std_msgs::msg::String &grieel_finish_string){
  // used only when dynamixel controller is running
  //int limb_id;
  if(grieel_finish_string.data == "wheel"){
    grieel_transform_end = true;
  } else if(grieel_finish_string.data == "gripper"){
    grieel_transform_end = true;
  } 
}



void HighLevelController::crawlGate(){
  // std::thread crawl_step([this]() {
  //   int task_num = 5;  // One sequence consists of 4 limb motion and 1 base motion.
  //   lbr_msgs::msg::MotionPackage crawl_gait_motion_package;
  //   crawl_gait_motion_package.motion_task_array.resize(task_num);
  //   lbr_msgs::msg::MotionTask motion_task;
  //   motion_task.task_id_array.resize(task_num);

  //   lbr_msgs::msg::LimbMotionTask limb_forward_swing;
  //   limb_forward_swing.end_effector_displacement.x = 0.20;
  //   limb_forward_swing.end_effector_displacement.y = 0.0;
  //   limb_forward_swing.end_effector_displacement.z = 0.0;
  //   limb_forward_swing.swing_mid_point.x = 0.20/2;
  //   limb_forward_swing.swing_mid_point.y = 0.0;
  //   limb_forward_swing.swing_mid_point.z = 0.20;
  //   limb_forward_swing.swing_duration = 3000;
  //   limb_forward_swing.pitch_angle_displacement = 0.0;  // [rad]

  //   std::vector<int> gait_numbers = {0, 2, 1, 3};
  //   for (int i = 0; i < 2; i++) {
  //     limb_forward_swing.limb_id = gait_numbers.at(i);
  //     limb_forward_swing.header.stamp = this->now();
  //     motion_task.limb_motion_task = limb_forward_swing;
  //     motion_task.task_id_array.at(i) = 0;  // ID of limb motion task is 0.
  //     crawl_gait_motion_package.motion_task_array.at(i) = motion_task;
  //   }
  //   lbr_msgs::msg::BaseMotionTask base_motion;
  //   base_motion.motion_duration = 3000;
  //   base_motion.base_displacement.x = 0.15;
  //   base_motion.base_displacement.y = 0.0;
  //   base_motion.base_displacement.z = 0.0;
  //   base_motion.base_angle_displacement.x = 0.0;
  //   base_motion.base_angle_displacement.y = 0.0;
  //   base_motion.base_angle_displacement.z = 0.0;
  //   base_motion.header.stamp = this->now();
  //   motion_task.base_motion_task = base_motion;
  //   motion_task.task_id_array.at(2) = 1;  // ID of base motion task is 1.
  //   crawl_gait_motion_package.motion_task_array.at(2) = motion_task;

  //   for (int i = 2; i < 4; i++) {
  //     limb_forward_swing.limb_id = gait_numbers.at(i);
  //     limb_forward_swing.header.stamp = this->now();
  //     motion_task.limb_motion_task = limb_forward_swing;
  //     motion_task.task_id_array.at(i+1) = 0;  // ID of limb motion task is 0.
  //     crawl_gait_motion_package.motion_task_array.at(i+1) = motion_task;
  //   }

  //   crawl_gait_motion_package.header.stamp = this->now();

  //   motion_package_pub_->publish(crawl_gait_motion_package);

  //   // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  // });
  std::thread crawl_step([this]() {
    bool first_time = true;
    float i = 0;
    while(move){
      if(motion_package_end || first_time){
        std::cout << "Crawl gate step: " << i << " start." << std::endl;
        i = i+1;
        int task_num = 4*3+1;  // One sequence consists of 4 limb motion and 1 base motion.
        lbr_msgs::msg::MotionPackage crawl_gait_motion_package;
        crawl_gait_motion_package.motion_task_array.resize(task_num);
        lbr_msgs::msg::MotionTask motion_task;
        motion_task.task_id_array.resize(task_num);

        lbr_msgs::msg::LimbMotionTask limb_up;
        limb_up.end_effector_displacement.x = 0.0;
        limb_up.end_effector_displacement.y = 0.0;
        limb_up.end_effector_displacement.z = 0.2;
        limb_up.swing_duration = 1500;
        limb_up.pitch_angle_displacement = 0.0;  // [rad]

        lbr_msgs::msg::LimbMotionTask limb_forward;
        limb_forward.end_effector_displacement.x = 0.15;
        limb_forward.end_effector_displacement.y = 0.0;
        limb_forward.end_effector_displacement.z = 0.0;
        limb_forward.swing_duration = 1500;
        limb_forward.pitch_angle_displacement = 0.0;  // [rad]

        lbr_msgs::msg::LimbMotionTask limb_down;
        limb_down.end_effector_displacement.x = 0.0;
        limb_down.end_effector_displacement.y = 0.0;
        limb_down.end_effector_displacement.z = -0.2;
        limb_down.swing_duration = 1500;
        limb_down.pitch_angle_displacement = 0.0;  // [rad]


        std::vector<int> gait_numbers = {0, 2, 1, 3};
        for (int i = 0; i < 2; i++) {
          limb_up.limb_id = gait_numbers.at(0+i*3);
          limb_up.header.stamp = this->now();
          motion_task.limb_motion_task = limb_up;
          motion_task.task_id_array.at(0+i*3) = 0;  // ID of limb motion task is 0.
          crawl_gait_motion_package.motion_task_array.at(0+i*3) = motion_task;
          
          limb_forward.limb_id = gait_numbers.at(i);
          limb_forward.header.stamp = this->now();
          motion_task.limb_motion_task = limb_forward;
          motion_task.task_id_array.at(1+i*3) = 0;  // ID of limb motion task is 0.
          crawl_gait_motion_package.motion_task_array.at(1+i*3) = motion_task;
          
          limb_down.limb_id = gait_numbers.at(i);
          limb_down.header.stamp = this->now();
          motion_task.limb_motion_task = limb_down;
          motion_task.task_id_array.at(2+i*3) = 0;  // ID of limb motion task is 0.
          crawl_gait_motion_package.motion_task_array.at(2+i*3) = motion_task;
        }

        lbr_msgs::msg::BaseMotionTask base_motion;
        base_motion.motion_duration = 3000;
        base_motion.base_displacement.x = 0.15;
        base_motion.base_displacement.y = 0.0;
        base_motion.base_displacement.z = 0.0;
        base_motion.base_angle_displacement.x = 0.0;
        base_motion.base_angle_displacement.y = 0.0;
        base_motion.base_angle_displacement.z = 0.0;
        base_motion.header.stamp = this->now();
        motion_task.base_motion_task = base_motion;
        motion_task.task_id_array.at(6) = 1;  // ID of base motion task is 1.
        crawl_gait_motion_package.motion_task_array.at(6) = motion_task;

        for (int i = 2; i < 4; i++) {
          limb_up.limb_id = gait_numbers.at(i);
          limb_up.header.stamp = this->now();
          motion_task.limb_motion_task = limb_up;
          motion_task.task_id_array.at(1+i*3) = 0;  // ID of limb motion task is 0.
          crawl_gait_motion_package.motion_task_array.at(1+i*3) = motion_task;
          
          limb_forward.limb_id = gait_numbers.at(i);
          limb_forward.header.stamp = this->now();
          motion_task.limb_motion_task = limb_forward;
          motion_task.task_id_array.at(2+i*3) = 0;  // ID of limb motion task is 0.
          crawl_gait_motion_package.motion_task_array.at(2+i*3) = motion_task;
          
          limb_down.limb_id = gait_numbers.at(i);
          limb_down.header.stamp = this->now();
          motion_task.limb_motion_task = limb_down;
          motion_task.task_id_array.at(3+i*3) = 0;  // ID of limb motion task is 0.
          crawl_gait_motion_package.motion_task_array.at(3+i*3) = motion_task;
        }

        crawl_gait_motion_package.header.stamp = this->now();

        motion_package_pub_->publish(crawl_gait_motion_package);
        motion_package_end = false;
        std::this_thread::sleep_for(std::chrono::seconds(20));
      }
      if(first_time == true){
        first_time = false;
      }
    }
    

  });
  crawl_step.detach();
}

// TODO(RN): Implement gait generator
// [-] publish limb motion tasks to realize crawl gait
// []
// [-] implement trot gait
// [] add joy input for above feature
// RF(3)→LH(1)→LF(0)→RH(2)
void HighLevelController::generateCrawlGait(const geometry_msgs::msg::Twist & move_direction)
{
  int task_num = 9;  // One sequence consists of 4 limb motion and 1 base motion.
  lbr_msgs::msg::MotionPackage crawl_gait_motion_package;
  crawl_gait_motion_package.motion_task_array.resize(task_num);
  lbr_msgs::msg::MotionTask motion_task;
  motion_task.task_id_array.resize(task_num);

  lbr_msgs::msg::LimbMotionTask limb_forward_swing, limb_backward_swing;
  limb_forward_swing.end_effector_displacement.x = move_direction.linear.x;
  limb_forward_swing.end_effector_displacement.y = move_direction.linear.y;
  limb_forward_swing.swing_mid_point.z =
    (sqrt(pow(move_direction.linear.x, 2) + pow(move_direction.linear.y, 2))) / 2;
  limb_forward_swing.swing_duration = 3000;

  limb_backward_swing.end_effector_displacement.x = -move_direction.linear.x;
  limb_backward_swing.end_effector_displacement.y = -move_direction.linear.y;
  limb_backward_swing.swing_mid_point.z = -0.005;
  limb_backward_swing.swing_duration = 3000;


  std::vector<int> gait_numbers = {3, 1, 0, 2};
  std::vector<lbr_msgs::msg::LimbMotionTask> limb_motion_package(4);
  for (int i = 0; i < 8; i++) {
    limb_forward_swing.limb_id = gait_numbers.at(i / 2);
    limb_backward_swing.limb_id = gait_numbers.at(i / 2);
    motion_task.limb_motion_task = limb_forward_swing;
    std::cout << "limb_motion_package.at(" << i << ").limb_id = " <<
      limb_forward_swing.limb_id << std::endl;
    motion_task.task_id_array.at(i) = 0;  // ID of limb motion task is 0.
    crawl_gait_motion_package.motion_task_array.at(i) = motion_task;
    i = i + 1;
    motion_task.limb_motion_task = limb_backward_swing;
    limb_motion_package.at(i / 2).header.stamp = this->now();
    crawl_gait_motion_package.motion_task_array.at(i) = motion_task;
  }

  lbr_msgs::msg::BaseMotionTask base_motion;
  base_motion.motion_duration = 1000;
  base_motion.base_displacement.x = move_direction.linear.x;
  base_motion.base_displacement.y = move_direction.linear.y;
  base_motion.base_displacement.z = 0.0;
  base_motion.base_angle_displacement.x = 0.0;
  base_motion.base_angle_displacement.y = 0.0;
  base_motion.base_angle_displacement.z = 0.0;
  base_motion.header.stamp = this->now();

  std::cout << "base_motion" << std::endl;

  motion_task.base_motion_task = base_motion;
  motion_task.task_id_array.at(8) = 1;  // ID of base motion task is 1.
  crawl_gait_motion_package.motion_task_array.at(8) = motion_task;

  // Publish motion package of crawl gait to LLC.
  crawl_gait_motion_package.header.stamp = this->now();
  motion_package_pub_->publish(crawl_gait_motion_package);

  std::cout << "generateCrawlGait" << std::endl;
}

void HighLevelController::generateTrotGait(const geometry_msgs::msg::Twist & move_direction)
{
#if DEBUG_ENABLED
  std::cout << "HLC::generateTrotGait" << std::endl;
#endif  // DEBUG_ENABLED

  const int task_num = 4;  // Forward and backward swings with diagonal limbs
  lbr_msgs::msg::MotionPackage trot_gait_motion_package;
  lbr_msgs::msg::MotionTask motion_task;
  lbr_msgs::msg::LimbMotionTask limb_forward_swing, limb_backward_swing;

  motion_task.task_id_array.resize(task_num);
  trot_gait_motion_package.motion_task_array.resize(task_num);

  limb_forward_swing.end_effector_displacement.x = move_direction.linear.x;
  limb_forward_swing.end_effector_displacement.y = move_direction.linear.y;
  limb_forward_swing.swing_mid_point.z =
    (sqrt(pow(move_direction.linear.x, 2) + pow(move_direction.linear.y, 2))) / 2;
  limb_forward_swing.swing_duration = 2500;

  limb_backward_swing.end_effector_displacement.x = -move_direction.linear.x;
  limb_backward_swing.end_effector_displacement.y = -move_direction.linear.y;
  limb_backward_swing.swing_mid_point.z = -0.005;
  limb_backward_swing.swing_duration = 2500;

  // Forward swing with LF and RH
  motion_task.limb_motion_task_array.resize(2);
  limb_forward_swing.limb_id = 0;  // LF
  motion_task.limb_motion_task_array.at(0) = limb_forward_swing;
  limb_forward_swing.limb_id = 2;  // RH
  motion_task.limb_motion_task_array.at(1) = limb_forward_swing;

  motion_task.task_id_array.at(0) = 2;  // ID 2: limb motion task array
  trot_gait_motion_package.motion_task_array.at(0) = motion_task;

  // Backward swing with LF and RH
  limb_backward_swing.limb_id = 0;  // LF
  motion_task.limb_motion_task_array.at(0) = limb_backward_swing;
  limb_backward_swing.limb_id = 2;  // RH
  motion_task.limb_motion_task_array.at(1) = limb_backward_swing;

  motion_task.task_id_array.at(1) = 2;
  trot_gait_motion_package.motion_task_array.at(1) = motion_task;

  // Forward swing with LH and RF
  motion_task.limb_motion_task_array.resize(2);
  limb_forward_swing.limb_id = 1;  // LH
  motion_task.limb_motion_task_array.at(0) = limb_forward_swing;
  limb_forward_swing.limb_id = 3;  // RF
  motion_task.limb_motion_task_array.at(1) = limb_forward_swing;

  motion_task.task_id_array.at(2) = 2;
  trot_gait_motion_package.motion_task_array.at(2) = motion_task;

  // Backward swing with LH and RF
  limb_backward_swing.limb_id = 1;  // LH
  motion_task.limb_motion_task_array.at(0) = limb_backward_swing;
  limb_backward_swing.limb_id = 3;  // RF
  motion_task.limb_motion_task_array.at(1) = limb_backward_swing;

  motion_task.task_id_array.at(3) = 2;
  trot_gait_motion_package.motion_task_array.at(3) = motion_task;

  trot_gait_motion_package.header.stamp = this->now();
  motion_package_pub_->publish(trot_gait_motion_package);
}

void HighLevelController::motionFinishSignalCallback(const std_msgs::msg::String & signal)
{
#if DEBUG_ENABLED
  std::cout << "HCL::motionFinishSignalCallback" << std::endl;
  std::cout << "trot_gait_is_executing = " << trot_gait_is_executing << std::endl;
#endif  // DEBUG_ENABLED

  if (signal.data == "fin") {
    //trot_gait_is_executing = false;
    motion_package_end = true;
    std::cout << " motion package finish "  << std::endl;
  }
}

bool HighLevelController::isInsideTriangle(
  const Eigen::Vector3d & base_link,
  const Eigen::Vector3d & p1, const Eigen::Vector3d & p2, const Eigen::Vector3d & p3)
{
  Eigen::Vector3d normal = (p2 - p1).cross(p3 - p1);
  Eigen::Vector3d link_to_point1 = p1 - base_link;
  Eigen::Vector3d link_to_point2 = p2 - base_link;
  Eigen::Vector3d link_to_point3 = p3 - base_link;

  // Determines if the triangle is contained within a triangle
  return normal.dot(link_to_point1.cross(link_to_point2)) >= 0 &&
         normal.dot(link_to_point2.cross(link_to_point3)) >= 0 &&
         normal.dot(link_to_point3.cross(link_to_point1)) >= 0;
}

void HighLevelController::endEffectorContactStateCallback(
  const lbr_msgs::msg::EndEffectorContactState & msg)
{
  supporting_leg_polygon = msg;
}

void HighLevelController::stabilizeBaseInSupportingLegsTriangle(
  const lbr_msgs::msg::EndEffectorContactState & supporting_leg_polygon)
{
#if DEBUG_ENABLED
  std::cout << "Number of contact points is " <<
    supporting_leg_polygon.end_effector_position.size() << std::endl;
#endif  // DEBUG_ENABLED

  if (supporting_leg_polygon.end_effector_position.size() == 3) {
    Eigen::Vector3d p1 = {supporting_leg_polygon.end_effector_position.at(0).x,
      supporting_leg_polygon.end_effector_position.at(0).y,
      supporting_leg_polygon.end_effector_position.at(0).z};
    Eigen::Vector3d p2 = {supporting_leg_polygon.end_effector_position.at(1).x,
      supporting_leg_polygon.end_effector_position.at(1).y,
      supporting_leg_polygon.end_effector_position.at(1).z};
    Eigen::Vector3d p3 = {supporting_leg_polygon.end_effector_position.at(2).x,
      supporting_leg_polygon.end_effector_position.at(2).y,
      supporting_leg_polygon.end_effector_position.at(2).z};

    Eigen::Vector3d base_link = {0.0, 0.0, 0.0};  // Base link x, y position

    if (isInsideTriangle(base_link, p1, p2, p3)) {
      std::cout << "Base link is inside the triangle." << std::endl;
    } else {
      std::cout << "Base link is outside the triangle." << std::endl;

      double vecX = (p1(0) + p2(0) + p3(0)) / 3 - base_link(0);
      double vecY = (p1(1) + p2(1) + p3(1)) / 3 - base_link(1);

      lbr_msgs::msg::BaseMotionTask base_motion;
      base_motion.motion_duration = 3000;
      base_motion.base_displacement.x = vecX;
      base_motion.base_displacement.y = vecY;
      base_motion.header.stamp = this->now();
      base_motion_pub_->publish(base_motion);
    }
  }
}

void HighLevelController::centerBase(
  const lbr_msgs::msg::EndEffectorContactState & supporting_leg_polygon)
{
#if DEBUG_ENABLED
  std::cout << "Number of contact points is " <<
    supporting_leg_polygon.end_effector_position.size() << std::endl;
#endif  // DEBUG_ENABLED

 
  if (supporting_leg_polygon.end_effector_position.size() > 3) {
     std::cout << "Centering Base" << std::endl;

    Eigen::Vector3d p1 = {supporting_leg_polygon.end_effector_position.at(0).x,
      supporting_leg_polygon.end_effector_position.at(0).y,
      supporting_leg_polygon.end_effector_position.at(0).z};
    Eigen::Vector3d p2 = {supporting_leg_polygon.end_effector_position.at(1).x,
      supporting_leg_polygon.end_effector_position.at(1).y,
      supporting_leg_polygon.end_effector_position.at(1).z};
    Eigen::Vector3d p3 = {supporting_leg_polygon.end_effector_position.at(2).x,
      supporting_leg_polygon.end_effector_position.at(2).y,
      supporting_leg_polygon.end_effector_position.at(2).z};
    Eigen::Vector3d p4 = {supporting_leg_polygon.end_effector_position.at(3).x,
      supporting_leg_polygon.end_effector_position.at(3).y,
      supporting_leg_polygon.end_effector_position.at(3).z};


    Eigen::Vector3d base_link = {0.0, 0.0, 0.0};  // Base link x, y position

    double vecX = (p1(0) + p2(0) + p3(0) + p4(0)) / 4 - base_link(0);
    double vecY = (p1(1) + p2(1) + p3(1) + p4(1)) / 4 - base_link(1);

      lbr_msgs::msg::BaseMotionTask base_motion;
      base_motion.motion_duration = 3000;
      base_motion.base_displacement.x = vecX;
      base_motion.base_displacement.y = vecY;
      base_motion.header.stamp = this->now();
      base_motion_pub_->publish(base_motion);
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HighLevelController>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}