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

#include "lbr_dynamixel_controller/lbr_dynamixel_controller.hpp"

#define DEBUG_ENABLED false

DynamixelController::DynamixelController()
: Node("lbr_dynamixel_controller"),
  DYNAMIXEL_OFFSET_F2T(1.364075222), DYNAMIXEL_OFFSET_T2G(0.2067211047)
{
  std::cout << "DynamixelController class is established." << std::endl;

  // Get namespace and node name.
  limb_namespace = std::string(this->get_namespace());
  std::string topic_prefix = std::string(this->get_namespace()) +
    "/" + std::string(this->get_name());
#if DEBUG_ENABLED
  std::cout << "namespace = " << this->get_namespace() << std::endl;
  std::cout << "node_name = " << this->get_name() << std::endl;
  std::cout << "topic_prefix = " << topic_prefix << std::endl;
#endif  // DEBUG_ENABLED

  const char * c_device_name;

  if (limb_namespace == "/LF") {
    device_name = "/dev/ttyUSB0";
    INITIAL_DYNAMIXEL_ID = 1;  // LF uses ID 1~4 (5~8 LF GRIEEL).
    limb_id = 0;
  } else if (limb_namespace == "/LH") {
    device_name = "/dev/ttyUSB1";
    INITIAL_DYNAMIXEL_ID = 9;  // LH uses ID 9~12 (13~16 LH GRIEEL).
    limb_id = 1;
  } else if (limb_namespace == "/RH") {
    device_name = "/dev/ttyUSB2";
    INITIAL_DYNAMIXEL_ID = 17;  // RH uses ID 17~20 (21~24 RH GRIEEL).
    limb_id = 2;
  } else if (limb_namespace == "/RF") {
    device_name = "/dev/ttyUSB3";
    INITIAL_DYNAMIXEL_ID = 25;  // RF uses ID 25~28 (29~32 RF GRIEEL).
    limb_id = 3;
  }

  c_device_name = device_name.c_str();

  // Define port handler and packet handler
  portHandler = dynamixel::PortHandler::getPortHandler(c_device_name);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  RCLCPP_INFO(this->get_logger(), "[Dynamixel] Protocol Version : %.1f", PROTOCOL_VERSION);
  RCLCPP_INFO(this->get_logger(), "[Dynamixel] Baudrate : %d", BAUDRATE);
  // Open port
  if (!portHandler->openPort()) {
    RCLCPP_ERROR(this->get_logger(), "[Dynamixel] Failed to open the port.");
  } else {
    RCLCPP_INFO(this->get_logger(), "[Dynamixel] Success to open the port.");
  }
  // Set baudrate
  if (!portHandler->setBaudRate(BAUDRATE)) {
    RCLCPP_ERROR(this->get_logger(), "[Dynamixel] Failed to set the baudrate.");
  } else {
    RCLCPP_INFO(this->get_logger(), "[Dynamixel] Success to set the baudrate.");
  }
  // Turn on LED
  enableLED(true);
  // Enable torque
  enableTorque(true);
  // Set velocity limit for initialization
  setVelocityLimit(0);
  // initialize PID gains
  setPIDGains();

  // Publisher
  encoder_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    topic_prefix + "/encoder_joint_state", 10);
  dynamixel_temperature_pub_ = this->create_publisher<std_msgs::msg::Int64MultiArray>(
    topic_prefix + "/temperature", 10);
  grieel_transform_end_pub_ = this->create_publisher<std_msgs::msg::String>(
    topic_prefix + "/grieel_transform_end",10);
  contact_state_pub_ = this->create_publisher<lbr_msgs::msg::SingleEndEffectorContactState>(
    topic_prefix + "/contact_state", 10);

  // Subscriber
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    limb_namespace + "/lbr_limb_controller/joint_state", 10,
    std::bind(&DynamixelController::setPositionCallback, this, std::placeholders::_1));
  gripping_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    limb_namespace + "/lbr_limb_controller/gripper/gripper_command", 1,
    std::bind(&DynamixelController::grippingCallback, this, std::placeholders::_1));
  grieel_sub_ = this->create_subscription<std_msgs::msg::String>(
    limb_namespace + "/lbr_limb_controller/grieel_joint_finish", 1,
    std::bind(&DynamixelController::setVelocityCallback, this, std::placeholders::_1));

  int timer_frequency = 10;  // [Hz]
  int loop_rate = 1000 / timer_frequency;  // [ms]
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(loop_rate), std::bind(&DynamixelController::timerCallback, this));
  second_timer_ = this->create_wall_timer(
    std::chrono::seconds(10), std::bind(&DynamixelController::secondTimerCallback, this));
  contact_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&DynamixelController::contactCallback, this));

  dynamixel_position.resize(JOINT_NUM + GRIPPER_MOTOR_NUM);
  dynamixel_velocity.resize(JOINT_NUM + GRIPPER_MOTOR_NUM);
  dynamixel_current.resize(JOINT_NUM + GRIPPER_MOTOR_NUM);
  current.resize(JOINT_NUM + GRIPPER_MOTOR_NUM);

  encoder_joint_state.position.resize(JOINT_NUM + GRIPPER_MOTOR_NUM);
  encoder_joint_state.velocity.resize(JOINT_NUM + GRIPPER_MOTOR_NUM);
  encoder_joint_state.effort.resize(JOINT_NUM + GRIPPER_MOTOR_NUM);
  encoder_joint_state.name.resize(JOINT_NUM + GRIPPER_MOTOR_NUM);
  encoder_joint_state.name.at(0) = limb_namespace + "/Joint1";
  encoder_joint_state.name.at(1) = limb_namespace + "/Joint2";
  encoder_joint_state.name.at(2) = limb_namespace + "/Joint3";
  encoder_joint_state.name.at(3) = limb_namespace + "/Joint4";
  encoder_joint_state.name.at(4) = limb_namespace + "/Joint5";
  encoder_joint_state.name.at(5) = limb_namespace + "/Joint6";
  encoder_joint_state.name.at(6) = limb_namespace + "/Joint7";
  encoder_joint_state.name.at(7) = limb_namespace + "/Joint8";

  encoder_joint_state.header.frame_id = limb_namespace;

  dynamixel_temperature.data.resize(JOINT_NUM + GRIPPER_MOTOR_NUM);

  RCLCPP_INFO(this->get_logger(), "Program is running.");
}

DynamixelController::~DynamixelController()
{
  std::cout << "DynamixelController class is destructed." << std::endl;

  enableLED(false);  // Turn off LED
  enableTorque(false);  // Disable torque
}

void DynamixelController::checkAngleLimit(const sensor_msgs::msg::JointState & joint_state)
{
  const double delta = 0.001;  // [rad]
  std::vector<double> lower_limit = {JOINT1_LOWER, JOINT2_LOWER, JOINT3_LOWER, JOINT4_LOWER, JOINT5_LOWER,JOINT6_LOWER, JOINT7_LOWER};
  std::vector<double> upper_limit = {JOINT1_UPPER, JOINT2_UPPER, JOINT3_UPPER, JOINT4_UPPER, JOINT5_UPPER, JOINT6_UPPER, JOINT7_UPPER};
  // Set limits slightly larger than limits in LC.
  for (int i = 0; i < JOINT_NUM; i++) {
    lower_limit.at(i) -= delta;
    upper_limit.at(i) += delta;
  }

  for (int i = 0; i < JOINT_NUM; i++) {
    // Check position limit.
    if (joint_state.position.at(i) < lower_limit.at(i)) {
      std::cerr << "Joint" << i + 1 << " exceeds lower limit." << std::endl;
      //std::exit(0);
    } else if (joint_state.position.at(i) > upper_limit.at(i)) {
      std::cerr << "Joint" << i + 1 << " exceeds upper limit." << std::endl;
      //std::exit(0);
    }
  }
}

void DynamixelController::timerCallback()
{
  int end_dynamixel_id = INITIAL_DYNAMIXEL_ID + JOINT_NUM + GRIPPER_MOTOR_NUM;
  bool success_read_position = readPosition(dynamixel_position);
  if (success_read_position == true) {
    for (int id = INITIAL_DYNAMIXEL_ID; id < end_dynamixel_id; id++) {
      encoder_joint_state.position[id - INITIAL_DYNAMIXEL_ID] = dynamixelPositionToAngle(
        id, dynamixel_position[id - INITIAL_DYNAMIXEL_ID]);
    }
    checkAngleLimit(encoder_joint_state);
  }

  bool success_read_velocity = readVelocity(dynamixel_velocity);
  if (success_read_velocity == true) {
    for (int id = INITIAL_DYNAMIXEL_ID; id < end_dynamixel_id; id++) {
      encoder_joint_state.velocity[id - INITIAL_DYNAMIXEL_ID] = dynamixelVelocityToAngularVelocity(
        id, dynamixel_velocity[id - INITIAL_DYNAMIXEL_ID]);
    }
  }
#if DEBUG_ENABLED
  // if(success_read_velocity){
  //   std::cout <<"(driving) dynamixel speed: " << dynamixel_velocity[INITIAL_DYNAMIXEL_ID + JOINT_NUM-1] << ", angular speed: " <<
  //   dynamixelVelocityToAngularVelocity(INITIAL_DYNAMIXEL_ID + JOINT_NUM-1, dynamixel_velocity[INITIAL_DYNAMIXEL_ID + JOINT_NUM-1]) << std::endl;
  // }
  //if(success_read_position){
    //int locking_id;
    //locking_id = INITIAL_DYNAMIXEL_ID + JOINT_NUM;
    //std::cout <<"(locking) dynamixel position: " << dynamixel_position[locking_id-INITIAL_DYNAMIXEL_ID] << ", angular postion: " <<
    //dynamixelPositionToAngle(locking_id, dynamixel_position[locking_id-INITIAL_DYNAMIXEL_ID]) << std::endl;
  //}
#endif

  readCurrent(dynamixel_current, current);

  // if (success_read_current == true) {
  //   for (int id = INITIAL_DYNAMIXEL_ID; id < end_dynamixel_id; id++) {
  //     //TODO(KeT): Need to implement convert function current ->, torque.
  //     encoder_joint_state.effort[id - INITIAL_DYNAMIXEL_ID] = convertDynamixelCurrent(
  //       id, dynamixel_current[id - INITIAL_DYNAMIXEL_ID]);
  //     std::cout << "current red!" << std::endl;
  //   }
  // }

  bool success_read = success_read_position ;//&& success_read_velocity && success_read_current;
  if (success_read == true) {
    encoder_joint_state.header.stamp = this->now();
    encoder_joint_state_pub_->publish(encoder_joint_state);
  }
}

void DynamixelController::secondTimerCallback()
{
#if DEBUG_ENABLED
  std::cout << limb_namespace + ": secondTimerCallback" << std::endl;
#endif  // DEBUG_ENABLED
  bool success_read_temperature = readTemperature(dynamixel_temperature);
  if (success_read_temperature == true) {
    dynamixel_temperature_pub_->publish(dynamixel_temperature);
  }
}

void DynamixelController::contactCallback()
{
  lbr_msgs::msg::SingleEndEffectorContactState contact_state;
  contact_state.header.frame_id = limb_namespace.substr(1);
  bool success_read_current = readCurrent(dynamixel_current, current);
  int C2F_id = INITIAL_DYNAMIXEL_ID + 1;
  //int F2T_id = INITIAL_DYNAMIXEL_ID + 2;
  if (success_read_current == true) {
    if((dynamixel_current.at(C2F_id-INITIAL_DYNAMIXEL_ID) > 0)){ //&& (dynamixel_current.at(F2T_id-INITIAL_DYNAMIXEL_ID) > 0)){
      std::cout << limb_namespace << "IS IN CONTACT." << std::endl;
      contact_state.is_contact = true;
    }
    else{
      contact_state.is_contact = false;
    }
    contact_state_pub_->publish(contact_state);
  }
}

void DynamixelController::grippingCallback(const std_msgs::msg::Bool & execute_grasping)
{
  const int gripper_open_current = 350;  // [mA]
  const int gripper_close_current = -10;

  const int gripper_dynamixel_id = INITIAL_DYNAMIXEL_ID + JOINT_NUM;
  int current_input;

  if (execute_grasping.data == true) {
    current_input = gripper_close_current;
  } else if (execute_grasping.data == false) {
    current_input = gripper_open_current;
#if DEBUG_ENABLED
    std::cout << "gripper open for " << limb_namespace << std::endl;
#endif  // DEBUG_ENABLED
  }

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  uint16_t current = (unsigned int)current_input;  // Convert int16 -> uint16
  dxl_comm_result = packetHandler->write2ByteTxRx(
    portHandler, (uint8_t)gripper_dynamixel_id, ADDR_GOAL_CURRENT, current, &dxl_error);

  if (dxl_comm_result == COMM_SUCCESS) {
#if DEBUG_ENABLED
    RCLCPP_INFO(
      this->get_logger(),
      "grippingCallback: [ID:%d] [PROFILE:%d]", (gripper_dynamixel_id), current);
#endif  // DEBUG_ENABLED
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to set current! Result: %d", dxl_comm_result);
  }
}

void DynamixelController::setVelocityLimit(int velocity_limit_input)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  int end_dynamixel_id = INITIAL_DYNAMIXEL_ID + JOINT_NUM + GRIPPER_MOTOR_NUM;
  for (int id = INITIAL_DYNAMIXEL_ID; id < end_dynamixel_id; id++) {
    uint32_t velocity_limit = (unsigned int)velocity_limit_input;  // Convert int32 -> uint32
    // Write Velocity Limit (length : 4 bytes)
    dxl_comm_result = packetHandler->write4ByteTxRx(
      portHandler, (uint8_t)(id), ADDR_PROFILE_VELOCITY, velocity_limit, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
#if DEBUG_ENABLED
      RCLCPP_INFO(
        this->get_logger(), "setVelocityProfile: [ID:%d] [PROFILE:%d]",
        id, velocity_limit);
#endif  // DEBUG_ENABLED
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to set profile velocity of [ID:%d]. Result: %d", id,
        dxl_comm_result);
    }
  }
}

bool DynamixelController::readPosition(std::vector<int> & dynamixel_position)
{
  auto start = std::chrono::high_resolution_clock::now();  // DEBUG
  // Below is based on sync_read_write_node.cpp in dynamixel_sdk_examples
  int data_length = 4;  // byte
  dynamixel::GroupSyncRead groupSyncRead(
    portHandler, packetHandler, ADDR_PRESENT_POSITION, data_length);
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;
  std::vector<uint32_t> position;
  position.resize(JOINT_NUM + GRIPPER_MOTOR_NUM);

  auto instance = std::chrono::high_resolution_clock::now();  // DEBUG

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  int end_dynamixel_id = INITIAL_DYNAMIXEL_ID + JOINT_NUM + GRIPPER_MOTOR_NUM;
  for (int id = INITIAL_DYNAMIXEL_ID; id < end_dynamixel_id; id++) {
    dxl_addparam_result = groupSyncRead.addParam((uint8_t)(id));
    if (dxl_addparam_result != true) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to addparam to groupSyncRead for Dynamixel ID %d", id);
      return false;
    }
  }

  auto add_param = std::chrono::high_resolution_clock::now();  // DEBUG

  dxl_comm_result = groupSyncRead.txRxPacket();

  auto sync_read = std::chrono::high_resolution_clock::now();  // DEBUG

  if (dxl_comm_result == COMM_SUCCESS) {
    for (int id = INITIAL_DYNAMIXEL_ID; id < end_dynamixel_id; id++) {
      position.at(id - INITIAL_DYNAMIXEL_ID) = groupSyncRead.getData(
        (uint8_t)(id), ADDR_PRESENT_POSITION, data_length);
      dynamixel_position[id - INITIAL_DYNAMIXEL_ID] = position.at(id - INITIAL_DYNAMIXEL_ID);
#if DEBUG_ENABLED
      RCLCPP_INFO(
        this->get_logger(), "getPosition : [ID:%d] [POSITION:%d]", id,
        position.at(id - INITIAL_DYNAMIXEL_ID));
#endif
    }
    groupSyncRead.clearParam();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to get position! Result: %d", dxl_comm_result);
    groupSyncRead.clearParam();
    return false;
  }
  auto finish = std::chrono::high_resolution_clock::now();  // DEBUG

  std::chrono::duration<double> elapsed = finish - start;
  std::chrono::duration<double> inst = instance - start;
  std::chrono::duration<double> param = add_param - instance;
  std::chrono::duration<double> sync = sync_read - add_param;
#if DEBUG_ENABLED
  std::cout << "readPosition" << std::endl;
  std::cout << "elapsed = " << 1000 * elapsed.count() << " ms" << std::endl;
  std::cout << "instance = " << 1000 * inst.count() << " ms" << std::endl;
  std::cout << "param = " << 1000 * param.count() << " ms" << std::endl;
  std::cout << "sync = " << 1000 * sync.count() << " ms" << std::endl;
#endif  // DEBUG_ENABLED
  return true;
}

bool DynamixelController::readVelocity(std::vector<int> & dynamixel_velocity)
{
  const int vel_data_length = 4;
  dynamixel::GroupSyncRead groupSyncVelRead(
    portHandler, packetHandler, ADDR_PRESENT_VELOCITY, vel_data_length);
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;
  std::vector<uint32_t> velocity;
  velocity.resize(JOINT_NUM + GRIPPER_MOTOR_NUM);

  // Read Present Velocity (length : 4 bytes) and Convert uint32 -> int32
  int end_dynamixel_id = INITIAL_DYNAMIXEL_ID + JOINT_NUM + GRIPPER_MOTOR_NUM;
  for (int id = INITIAL_DYNAMIXEL_ID; id < end_dynamixel_id; id++) {
    dxl_addparam_result = groupSyncVelRead.addParam((uint8_t)(id));
    if (dxl_addparam_result != true) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to addparam to groupSyncVelRead for Dynamixel ID %d", id);
      return false;
    }
  }

  dxl_comm_result = groupSyncVelRead.txRxPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    for (int id = INITIAL_DYNAMIXEL_ID; id < end_dynamixel_id; id++) {
      velocity.at(id - INITIAL_DYNAMIXEL_ID) = groupSyncVelRead.getData(
        (uint8_t)(id), ADDR_PRESENT_VELOCITY, vel_data_length);
      dynamixel_velocity[id - INITIAL_DYNAMIXEL_ID] = velocity.at(id - INITIAL_DYNAMIXEL_ID);
#if DEBUG_ENABLED
      RCLCPP_INFO(
        this->get_logger(), "getVelocity : [ID:%d] [VELOCITY:%d]", id,
        velocity.at(id - INITIAL_DYNAMIXEL_ID));
#endif
    }
    groupSyncVelRead.clearParam();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to get velocity! Result: %d", dxl_comm_result);
    groupSyncVelRead.clearParam();
    return false;
  }
  return true;
}

bool DynamixelController::readTemperature(std_msgs::msg::Int64MultiArray & dynamixel_temperature)
{
  // Initialize GroupBulkRead instance
  dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);

  bool dxl_addparam_result = false;  // addParam result
  bool dxl_getdata_result = false;  // GetParam result

  std::vector<uint32_t> temperature;
  temperature.resize(JOINT_NUM + GRIPPER_MOTOR_NUM);

  int end_dynamixel_id = INITIAL_DYNAMIXEL_ID + JOINT_NUM + GRIPPER_MOTOR_NUM;
  for (int id = INITIAL_DYNAMIXEL_ID; id < end_dynamixel_id; id++) {
    dxl_addparam_result = groupBulkRead.addParam(
      (uint8_t)(id), ADDR_PRESENT_TEMPERATURE, LEN_PRESENT_TEMPERATURE);
    if (dxl_addparam_result != true) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to addparam to groupBulkRead for Dynamixel ID %d", id);
      return false;
    }
  }

  // Bulkread present temperature status
  groupBulkRead.txRxPacket();

  // Check if groupbulkread data is available
  for (int id = INITIAL_DYNAMIXEL_ID; id < end_dynamixel_id; id++) {
    dxl_getdata_result = groupBulkRead.isAvailable(
      (uint8_t)(id), ADDR_PRESENT_TEMPERATURE, LEN_PRESENT_TEMPERATURE);
    if (dxl_getdata_result != true) {
      RCLCPP_ERROR(
        this->get_logger(), "[ID:%d] groupBulkRead getdata failed", id);
      return false;
    }
  }

  // Get present temperature value
  for (int id = INITIAL_DYNAMIXEL_ID; id < end_dynamixel_id; id++) {
    temperature.at(id - INITIAL_DYNAMIXEL_ID) = groupBulkRead.getData(
      (uint8_t)(id), ADDR_PRESENT_TEMPERATURE, LEN_PRESENT_TEMPERATURE);
    dynamixel_temperature.data[id - INITIAL_DYNAMIXEL_ID] =
      temperature.at(id - INITIAL_DYNAMIXEL_ID);
#if DEBUG_ENABLED
    std::cout << "temperature[" << id - INITIAL_DYNAMIXEL_ID << "] = " <<
      temperature.at(id - INITIAL_DYNAMIXEL_ID) << std::endl;
#endif
  }
  groupBulkRead.clearParam();
  return true;
}

bool DynamixelController::readCurrent(std::vector<int> & dynamixel_current, std::vector<int> & current)
{
  auto start = std::chrono::high_resolution_clock::now();  // DEBUG
  // Initialize GroupBulkRead instance
  dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);

  bool dxl_addparam_result = false;  // addParam result
  bool dxl_getdata_result = false;  // GetParam result


  int end_dynamixel_id = INITIAL_DYNAMIXEL_ID + JOINT_NUM + GRIPPER_MOTOR_NUM;
  for (int id = INITIAL_DYNAMIXEL_ID; id < end_dynamixel_id; id++) {
    dxl_addparam_result = groupBulkRead.addParam(
      (uint8_t)(id), ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
    if (dxl_addparam_result != true) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to addparam to groupBulkRead for Dynamixel ID %d", id);
      return false;
    }
  }
  auto instance = std::chrono::high_resolution_clock::now();  // DEBUG

  // Bulkread present current status
  groupBulkRead.txRxPacket();
  auto bulk_read = std::chrono::high_resolution_clock::now();  // DEBUG

  // Check if groupbulkread data is available
  for (int id = INITIAL_DYNAMIXEL_ID; id < end_dynamixel_id; id++) {
    dxl_getdata_result = groupBulkRead.isAvailable(
      (uint8_t)(id), ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
    if (dxl_getdata_result != true) {
      RCLCPP_ERROR(
        this->get_logger(), "[ID:%d] groupBulkRead getdata failed", id);
      return false;
    }
  }
  auto check_data = std::chrono::high_resolution_clock::now();  // DEBUG

  // Get present current value
  for (int id = INITIAL_DYNAMIXEL_ID; id < end_dynamixel_id; id++) {
    current.at(id - INITIAL_DYNAMIXEL_ID) = groupBulkRead.getData(
      (uint8_t)(id), ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
    // HACK(KT): Because current data is 2 byte (not 4 byte) data
    // (Ref: https://forum.robotis.com/t/negative-value-of-current-rpm-and-position-are-wrongly-read/2093)
    if (current.at(id - INITIAL_DYNAMIXEL_ID) > 32767) {
      dynamixel_current[id - INITIAL_DYNAMIXEL_ID] =
        -(65536 - current.at(id - INITIAL_DYNAMIXEL_ID));
    } else {
      dynamixel_current[id - INITIAL_DYNAMIXEL_ID] =
        current.at(id - INITIAL_DYNAMIXEL_ID);
    }
#if DEBUG_ENABLED
    std::cout << "LIMB" << limb_id << "| current[" << id - INITIAL_DYNAMIXEL_ID << "] = " <<
      current.at(id - INITIAL_DYNAMIXEL_ID) << std::endl;
    std::cout << "dynamixel_current[" << id - INITIAL_DYNAMIXEL_ID << "] = " <<
      dynamixel_current[id - INITIAL_DYNAMIXEL_ID] << std::endl;
#endif
  }
  groupBulkRead.clearParam();
  auto finish = std::chrono::high_resolution_clock::now();  // DEBUG

  std::chrono::duration<double> elapsed = finish - start;
  std::chrono::duration<double> inst = instance - start;
  std::chrono::duration<double> read = bulk_read - instance;
  std::chrono::duration<double> check = check_data - bulk_read;
#if DEBUG_ENABLED
  std::cout << "readCurrent" << std::endl;
  std::cout << "elapsed = " << 1000 * elapsed.count() << " ms" << std::endl;
  std::cout << "instance = " << 1000 * inst.count() << " ms" << std::endl;
  std::cout << "read = " << 1000 * read.count() << " ms" << std::endl;
  std::cout << "check = " << 1000 * check.count() << " ms" << std::endl;
#endif //DEBUG_ENABLED
  return true;
}

void DynamixelController::enableLED(bool enable_LED)
{
#if DEBUG_ENABLED
  std::cout << "enableLED" << std::endl;
#endif  // DEBUG_ENABLED
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int end_dynamixel_id = INITIAL_DYNAMIXEL_ID + JOINT_NUM + GRIPPER_MOTOR_NUM;
  for (int id = INITIAL_DYNAMIXEL_ID; id < end_dynamixel_id; id++) {
    dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->write1ByteTxRx(
      portHandler, id, ADDR_LED, enable_LED, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(), "[Dynamixel] Failed to control LED for Dynamixel ID %d", id);
    } else {
#if DEBUG_ENABLED
      RCLCPP_INFO(
        this->get_logger(), "[Dynamixel] Success to control LED for Dynamixel ID %d", id);
#endif  // DEBUG_ENABLED
    }
  }
}

void DynamixelController::setPIDGains(){
  std::vector<PIDGains> pid_gains_list;

  std::ifstream file("/home/ale/lbr_ws/src/LIMBERO/CONTROL/lbr_dynamixel_controller/config/PIDGains.csv");
  std::string line;
  std::getline(file, line);

  while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string joint;
      int kp, ki, kd;

      std::getline(ss, joint, ',');
      ss >> kp;
      ss.ignore(); // Ignore the comma
      ss >> ki;
      ss.ignore(); // Ignore the comma
      ss >> kd;

      pid_gains_list.push_back({joint, kp, ki, kd});
  }

  file.close();

  uint8_t dxl_error = 0;
  int end_dynamixel_id = INITIAL_DYNAMIXEL_ID + JOINT_NUM;
  // set PID gains from B2C to driving
  for (int id = INITIAL_DYNAMIXEL_ID; id < end_dynamixel_id; id++) {
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_POSITION_P_GAIN, pid_gains_list.at(id-INITIAL_DYNAMIXEL_ID).kp, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_POSITION_D_GAIN, pid_gains_list.at(id-INITIAL_DYNAMIXEL_ID).kd, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_POSITION_I_GAIN, pid_gains_list.at(id-INITIAL_DYNAMIXEL_ID).ki, &dxl_error);
    RCLCPP_INFO(this->get_logger(), "Motor ID: %d, initialized PID, P: %d, I: %d, D: %d",
    id-INITIAL_DYNAMIXEL_ID, pid_gains_list.at(id-INITIAL_DYNAMIXEL_ID).kp, pid_gains_list.at(id-INITIAL_DYNAMIXEL_ID).ki, pid_gains_list.at(id-INITIAL_DYNAMIXEL_ID).kd);
  }
  //than set PID of locking as Velocity gains.
}


void DynamixelController::enableTorque(bool enable_torque)
{
#if DEBUG_ENABLED
  std::cout << "enableTorque" << std::endl;
#endif  // DEBUG_ENABLED
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int end_dynamixel_id = INITIAL_DYNAMIXEL_ID + JOINT_NUM + GRIPPER_MOTOR_NUM;
  for (int id = INITIAL_DYNAMIXEL_ID; id < end_dynamixel_id; id++) {
    dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->write1ByteTxRx(
      portHandler, id, ADDR_TORQUE_ENABLE, enable_torque, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(), "[Dynamixel] Failed to control torque for Dynamixel ID %d", id);
    } else {
#if DEBUG_ENABLED
      RCLCPP_INFO(
        this->get_logger(), "[Dynamixel] Success to control torque for Dynamixel ID %d", id);
#endif  // DEBUG_ENABLED
    }
  }
}

int DynamixelController::angleToDynamixelPositionValue(float angle, int joint_id)
{
  int dynamixel_position;
  if (joint_id % (JOINT_NUM + GRIPPER_MOTOR_NUM) == 1) {
    dynamixel_position = floor(angle * 2048 / M_PI) + 2048;
  } else if (joint_id % (JOINT_NUM + GRIPPER_MOTOR_NUM) == 2) {
    dynamixel_position = floor(angle * 2048 / M_PI) + 2048;
  } else if (joint_id % (JOINT_NUM + GRIPPER_MOTOR_NUM) == 3) {
    dynamixel_position = floor((angle - DYNAMIXEL_OFFSET_F2T) * 2048 / M_PI) + 2048;
  } else if (joint_id % (JOINT_NUM + GRIPPER_MOTOR_NUM) == 4) {
    dynamixel_position = floor(-(angle - DYNAMIXEL_OFFSET_T2G) * 2048 / M_PI) + 2048;
  } else if (joint_id % (JOINT_NUM + GRIPPER_MOTOR_NUM) == 5) {
    dynamixel_position = floor(angle * 2048 / M_PI);
  } else if (joint_id % (JOINT_NUM + GRIPPER_MOTOR_NUM) == 6) {
    dynamixel_position = floor(angle * 2048 / M_PI) + 2048*1.5;
  } else if (joint_id % (JOINT_NUM + GRIPPER_MOTOR_NUM) == 7) {
    dynamixel_position = floor(angle * 2048 / M_PI) + 2048*0.75;
  }
#if DEBUG_ENABLED
  std::cout << "joint_id = " << joint_id << std::endl;
  std::cout << "angle = " << angle << std::endl;
  std::cout << "dynamixel_position = " << dynamixel_position << std::endl;
#endif  // DEBUG_ENABLED
  return dynamixel_position;
}

double DynamixelController::dynamixelPositionToAngle(
  const int & joint_id, const int & dynamixel_position)
{
  float angle;
  if (joint_id % (JOINT_NUM + GRIPPER_MOTOR_NUM) == 1) {
    angle = (dynamixel_position - 2048) * M_PI / 2048;
  } else if (joint_id % (JOINT_NUM + GRIPPER_MOTOR_NUM) == 2) {
    angle = (dynamixel_position - 2048) * M_PI / 2048;
  } else if (joint_id % (JOINT_NUM + GRIPPER_MOTOR_NUM) == 3) {
    angle = (dynamixel_position - 2048) * M_PI / 2048 + DYNAMIXEL_OFFSET_F2T;
  } else if (joint_id % (JOINT_NUM + GRIPPER_MOTOR_NUM) == 4) {
    angle = -((dynamixel_position - 2048) * M_PI / 2048) + DYNAMIXEL_OFFSET_T2G;
  } else if (joint_id % (JOINT_NUM + GRIPPER_MOTOR_NUM) == 5) {
    angle = (dynamixel_position ) * M_PI / 2048;
  } else if (joint_id % (JOINT_NUM + GRIPPER_MOTOR_NUM) == 6) {
    angle = (dynamixel_position - 2048*1.5) * M_PI / 2048;
  } else if (joint_id % (JOINT_NUM + GRIPPER_MOTOR_NUM) == 7) {
    angle = (dynamixel_position - 2048*0.75) * M_PI / 2048;
  } else if (joint_id % (JOINT_NUM + GRIPPER_MOTOR_NUM) == 0) {
    angle = (dynamixel_position - 2048) * M_PI / 2048;
  }  else {
    abort();
  }
  return angle;
}

double DynamixelController::dynamixelVelocityToAngularVelocity(
  const int & joint_id, const int & dynamixel_velocity)
{
  const double velocity_unit = 0.229;  // [rev/min]
  const double rev2rad = 2 * M_PI;  // [rad/rev]
  const double min2sec = 1.0 / 60.0;  // [min/sec]

  double angular_velocity;
  if (joint_id % 5 == 1) {
    angular_velocity = (velocity_unit * static_cast<double>(dynamixel_velocity)) *
      rev2rad * min2sec;
  } else if (joint_id % 5 == 2) {
    angular_velocity = (velocity_unit * static_cast<double>(dynamixel_velocity)) *
      rev2rad * min2sec;
  } else if (joint_id % 5 == 3) {
    angular_velocity = (velocity_unit * static_cast<double>(dynamixel_velocity)) *
      rev2rad * min2sec;
  } else if (joint_id % 5 == 4) {
    angular_velocity = -((velocity_unit * static_cast<double>(dynamixel_velocity)) *
      rev2rad * min2sec);
  } else if (joint_id % 5 == 0) {
    angular_velocity = (velocity_unit * static_cast<double>(dynamixel_velocity)) *
      rev2rad * min2sec;
  } else {
    abort();
  }
  return angular_velocity;
}

double DynamixelController::convertDynamixelCurrent(
  const int & joint_id, const int & dynamixel_current)
{
  const double current_unit = 2.69;  // [mA]

  double current;
  if (joint_id % 5 == 1) {
    current = (current_unit * static_cast<double>(dynamixel_current));
  } else if (joint_id % 5 == 2) {
    current = (current_unit * static_cast<double>(dynamixel_current));
  } else if (joint_id % 5 == 3) {
    current = (current_unit * static_cast<double>(dynamixel_current));
  } else if (joint_id % 5 == 4) {
    current = -(current_unit * static_cast<double>(dynamixel_current));
  } else if (joint_id % 5 == 0) {
    current = (current_unit * static_cast<double>(dynamixel_current));
  } else {
    abort();
  }
  return current;
}

void DynamixelController::setPositionCallback(
  const sensor_msgs::msg::JointState & input_joint_state)
{
#if DEBUG_ENABLED
  std::cout << "setPositionCallback" << std::endl;
#endif  // DEBUG_ENABLED
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;
  uint8_t param_goal_position[JOINT_NUM-1][4];
  std::vector<uint32_t> position;
  position.resize(JOINT_NUM-1);
  const int pos_data_length = 4;
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler,
    packetHandler, ADDR_GOAL_POSITION, pos_data_length);

  for (int id = INITIAL_DYNAMIXEL_ID; id < INITIAL_DYNAMIXEL_ID + JOINT_NUM-1; id++) {
    int index = id - INITIAL_DYNAMIXEL_ID;
    int input_position = angleToDynamixelPositionValue(input_joint_state.position[index], id);

#if DEBUG_ENABLED
    if(index == 4 || index == 5 || index == 0){
      std::cout << "angletoDynamixel id " << index << " position : " << input_position << std::endl;
    }
# endif

    position.at(index) = (unsigned int)input_position;  // Convert int32 -> uint32
    param_goal_position[index][0] = DXL_LOBYTE(DXL_LOWORD(position.at(index)));
    param_goal_position[index][1] = DXL_HIBYTE(DXL_LOWORD(position.at(index)));
    param_goal_position[index][2] = DXL_LOBYTE(DXL_HIWORD(position.at(index)));
    param_goal_position[index][3] = DXL_HIBYTE(DXL_HIWORD(position.at(index)));
  }
  // Write Goal Position (length : 4 bytes)
  for (int id = INITIAL_DYNAMIXEL_ID; id < INITIAL_DYNAMIXEL_ID + JOINT_NUM-1; id++) {
    int index = id - INITIAL_DYNAMIXEL_ID;
    dxl_addparam_result = groupSyncWrite.addParam((uint8_t)id, param_goal_position[index]);
    if (dxl_addparam_result != true) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to addparam to groupSyncWrite for Dynamixel ID %d", id);
    }
  }
  // Send to Dynamixel
  dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    for (int id = INITIAL_DYNAMIXEL_ID; id < INITIAL_DYNAMIXEL_ID + JOINT_NUM-1; id++) {
#if DEBUG_ENABLED
      int index = id - INITIAL_DYNAMIXEL_ID;
      RCLCPP_INFO(this->get_logger(), "setPosition : [ID:%d] [POSITION:%d]", id, position[index]);
#endif  // DEBUG_ENABLED
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to set position! Result: %d", dxl_comm_result);
  }
  groupSyncWrite.clearParam();

  int driving_id = JOINT_NUM + INITIAL_DYNAMIXEL_ID - 1;
  float driving_vel = input_joint_state.velocity.at(driving_id-INITIAL_DYNAMIXEL_ID);
  // bring rad/s in rev/min and than in dynamixel resolution
  float dynamixel_driving_vel = driving_vel*30/M_PI*1/0.229;
  setGoalVelocity(dynamixel_driving_vel, driving_id);

}

void DynamixelController::setVelocityCallback(
  const std_msgs::msg::String & grieel_mode_change)
{
  // send velocity command to open("gripper") or close("wheel") grieel

  std::this_thread::sleep_for(std::chrono::seconds(1));
  int id = INITIAL_DYNAMIXEL_ID+JOINT_NUM;
  bool set_vel = false;
  bool read_position_ok = false;
  while(!read_position_ok){
    read_position_ok = readPosition(dynamixel_position);
  }
  int initial_grieel_dynamixel = dynamixel_position[id-INITIAL_DYNAMIXEL_ID];

#if DEBUG_ENABLED
  std::cout << "initial grieel dynamixel angle: " << initial_grieel_dynamixel << std::endl;
#endif //DEBUG ENBLED

  if(grieel_mode_change.data == "wheel"){
    set_vel = setGoalVelocity(-265,id);
    while(set_vel == false){
      set_vel = setGoalVelocity(-265,id);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }else if(grieel_mode_change.data == "gripper"){
    set_vel = setGoalVelocity(265,id);
    while(set_vel == false){
      set_vel = setGoalVelocity(265,id);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  std::string new_grieel_mode = grieel_mode_change.data;
  // Start a new thread for sleeping and sending the stop command
  std::thread sleep_thread([this, new_grieel_mode, id, initial_grieel_dynamixel]() {
    // full transition takes 40 secs more or less

    int delta_grieel_dynamixel = 0;
    if(limb_id == 0){
      delta_grieel_dynamixel = 138000;
    } else if(limb_id == 1){
      delta_grieel_dynamixel = 123000;
    }else if(limb_id == 2){
       delta_grieel_dynamixel = 140000;
    }else if(limb_id == 3){
       delta_grieel_dynamixel = 105000;
    }

    if(new_grieel_mode == "wheel"){
      int goal_dynamixel_angle = initial_grieel_dynamixel - delta_grieel_dynamixel;
      while(dynamixel_position[id-INITIAL_DYNAMIXEL_ID] > goal_dynamixel_angle){
          if(dynamixel_position[id-INITIAL_DYNAMIXEL_ID] <= goal_dynamixel_angle){
              setGoalVelocity(0,id);
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }

    if(new_grieel_mode == "gripper"){
      int goal_dynamixel_angle = initial_grieel_dynamixel  + delta_grieel_dynamixel;
      while(dynamixel_position[id-INITIAL_DYNAMIXEL_ID] < goal_dynamixel_angle){
          if(dynamixel_position[id-INITIAL_DYNAMIXEL_ID] >= goal_dynamixel_angle){
              setGoalVelocity(0, id);
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }

    bool set_vel = setGoalVelocity(0,id);
    while(set_vel == false){
      set_vel = setGoalVelocity(0,id);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std_msgs::msg::String grieel_finish;
    grieel_finish.data = std::string(this->get_namespace());
    std::cout <<"send grieel finish data" << std::endl;
    grieel_transform_end_pub_->publish(grieel_finish);});
  // Detach the thread so it can run independently
  sleep_thread.detach();
}

bool DynamixelController::setGoalVelocity(int goal_velocity_input, int id)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Write Goal Velocity (length : 4 bytes)
  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler, (uint8_t)(id), ADDR_GOAL_VELOCITY, goal_velocity_input, &dxl_error);

  if (dxl_comm_result == COMM_SUCCESS) {
    return true;
  } else {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to set goal velocity of [ID:%d]. Result: %d", id,
      dxl_comm_result);
      return false;
  }
}

void DynamixelController::closePort()
{
  portHandler->closePort();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynamixelController>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
