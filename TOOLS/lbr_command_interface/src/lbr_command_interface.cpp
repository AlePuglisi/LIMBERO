// Copyright (c) 2024 Tohoku Univ. Space Robotics Lab.
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

#include "lbr_command_interface/lbr_command_interface.hpp"

#define DEBUG_ENABLED false

CommandInterface::CommandInterface()
: Node("lbr_command_interface")
{
  std::cout << "CommandInterface class is established." << std::endl;

  // Get namespace and node name.
  std::string topic_prefix = "/" + std::string(this->get_name());

  // Publisher
  base_motion_task_pub_ = this->create_publisher<lbr_msgs::msg::BaseMotionTask>(
    topic_prefix + "/motion_package/base_motion_task", 1);
  command_to_LLC_pub_ = this->create_publisher<std_msgs::msg::String>(
    topic_prefix + "/command/low_level_controller", 1);
  command_to_HLC_pub_ = this->create_publisher<std_msgs::msg::String>(
    topic_prefix + "/command/high_level_controller", 1);
  command_to_state_estimator_pub_ = this->create_publisher<std_msgs::msg::String>(
    topic_prefix + "/command/state_estimator", 1);

  gripper_state_pub_LF_ = this->create_publisher<std_msgs::msg::Bool>(
    "LF" + topic_prefix + "/gripper_command", 1);
  gripper_state_pub_LH_ = this->create_publisher<std_msgs::msg::Bool>(
    "LH" + topic_prefix + "/gripper_command", 1);
  gripper_state_pub_RH_ = this->create_publisher<std_msgs::msg::Bool>(
    "RH" + topic_prefix + "/gripper_command", 1);
  gripper_state_pub_RF_ = this->create_publisher<std_msgs::msg::Bool>(
    "RF" + topic_prefix + "/gripper_command", 1);

  limb_motion_task_pub_ = this->create_publisher<lbr_msgs::msg::LimbMotionTask>(
    topic_prefix + "/motion_package/limb_motion_task", 1);

  // Subscriber
  string_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/string", 1, std::bind(&CommandInterface::stringCallback, this, std::placeholders::_1));
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 1, std::bind(&CommandInterface::joyCallback, this, std::placeholders::_1));

  // Set timer callback
  int timer_frequency = 20;  // [Hz]
  timer_loop_rate = 1000 / timer_frequency;  // [ms]
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(timer_loop_rate),
    std::bind(&CommandInterface::timerCallback, this));

  // Initialize input_joy_cmd to execute timerCallback.
  input_joy_cmd.header.stamp = rclcpp::Clock().now();
  input_joy_cmd.axes = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  input_joy_cmd.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  showCommandList();
}

void CommandInterface::stringCallback(const std_msgs::msg::String & string)
{
  execInputCommand(string.data);
}

void CommandInterface::timerCallback()
{
#if DEBUG_ENABLED
  std::cout << "timerCallback" << std::endl;
#endif  // DEBUG_ENABLED
  bool square_button_pressed = input_joy_cmd.buttons.at(3) == 1;
  bool X_button_pressed = input_joy_cmd.buttons.at(0) == 1;
  bool circle_button_pressed = input_joy_cmd.buttons.at(1) == 1;
  bool triangle_button_pressed = input_joy_cmd.buttons.at(2) == 1;
  bool L1_pressed = input_joy_cmd.buttons.at(4) == 1;
  bool L2_pressed = input_joy_cmd.buttons.at(6) == 1;
  bool R1_pressed = input_joy_cmd.buttons.at(5) == 1;
  bool R2_pressed = input_joy_cmd.buttons.at(7) == 1;
  bool option_button_pressed = input_joy_cmd.buttons.at(9) == 1;
  bool share_button_pressed = input_joy_cmd.buttons.at(8) == 1;

  bool left_joystick_input = input_joy_cmd.axes.at(0) != 0 || input_joy_cmd.axes.at(1) != 0;
  bool right_joystick_input = input_joy_cmd.axes.at(3) != 0 || input_joy_cmd.axes.at(4) != 0;
  bool L1_R1_are_pressed = input_joy_cmd.buttons.at(4) == 1 && input_joy_cmd.buttons.at(5) == 1;
  bool L1_or_L2_pressed = L1_pressed || L2_pressed;
  bool R1_or_R2_pressed = R1_pressed || R2_pressed;
  bool L2_R2_are_pressed = input_joy_cmd.buttons.at(6) == 1 && input_joy_cmd.buttons.at(7) == 1;
  bool axes_are_pressed = input_joy_cmd.axes.at(6) != 0 || input_joy_cmd.axes.at(7) != 0;
  bool stick_is_pressed = input_joy_cmd.buttons.at(11) == 1 || input_joy_cmd.buttons.at(12) == 1;
  bool left_joy_stick_used = input_joy_cmd.axes.at(0) != 0.0 || input_joy_cmd.axes.at(1) != 0.0;

  bool circletriangle = circle_button_pressed && triangle_button_pressed;
  bool xsquare =  X_button_pressed && square_button_pressed;

  bool xy_base_motion = (left_joystick_input && right_joystick_input);
  bool z_base_motion = (L1_R1_are_pressed || L2_R2_are_pressed);
  bool rpy_base_motion = (axes_are_pressed || stick_is_pressed);

  if (xsquare){
    std_msgs::msg::String msg;
    msg.data = "stop_move";
    command_to_HLC_pub_->publish(msg);
    std::cout << "Input command is<" << "\033[32m stop move\033[m" << ">." << std::endl;
  }

  if (circletriangle){
    std_msgs::msg::String msg;
    msg.data = "move_forward";
    command_to_HLC_pub_->publish(msg);
    std::cout << "Input command is<" << "\033[32m move forward\033[m" << ">." << std::endl;
  }
  
  if (square_button_pressed) {
    if (left_joy_stick_used || L1_or_L2_pressed) {
      joyLimbMotionTask(0, input_joy_cmd);  // Limb control
    } else if (R1_or_R2_pressed) {
      if (R1_pressed) {
        publishGripperCommand(0, false);  // Open gripper
      } else if (R2_pressed) {
        publishGripperCommand(0, true);  // Close gripper
      }
    }
  } else if (X_button_pressed) {
    if (left_joy_stick_used || L1_or_L2_pressed) {
      joyLimbMotionTask(1, input_joy_cmd);
    } else if (R1_or_R2_pressed) {
      if (R1_pressed) {
        publishGripperCommand(1, false);
      } else if (R2_pressed) {
        publishGripperCommand(1, true);
      }
    }
  } else if (circle_button_pressed) {
    if (left_joy_stick_used || L1_or_L2_pressed) {
      joyLimbMotionTask(2, input_joy_cmd);
    } else if (R1_or_R2_pressed) {
      if (R1_pressed) {
        publishGripperCommand(2, false);
      } else if (R2_pressed) {
        publishGripperCommand(2, true);
      }
    }
  } else if (triangle_button_pressed) {
    if (left_joy_stick_used || L1_or_L2_pressed) {
      joyLimbMotionTask(3, input_joy_cmd);
    } else if (R1_or_R2_pressed) {
      if (R1_pressed) {
        publishGripperCommand(3, false);
      } else if (R2_pressed) {
        publishGripperCommand(3, true);
      }
    }
  } else if (xy_base_motion || z_base_motion || rpy_base_motion) {
    joyBaseMotionTask(input_joy_cmd);
  } else if (option_button_pressed) {
    std_msgs::msg::String msg;
    msg.data = "initialize";
    command_to_LLC_pub_->publish(msg);
    std::cout << "Input command is<" << "\033[32m initialize\033[m" << ">." << std::endl;
  } else if (share_button_pressed) {
    std_msgs::msg::String msg;
    msg.data = "grieel_mode_transform";
    command_to_HLC_pub_->publish(msg);
    std::cout << "Input command is<" << "\033[32m grieel mode transform\033[m" << ">." << std::endl;
  }
}

void CommandInterface::showCommandList()
{
  using std::cout;
  using std::endl;

  cout << endl << "------------------------ Command List ------------------------" << endl;
// low level commands that can be comunicated directly to low level controller
  cout << "COMMANDS TO LLC:" << endl;
  cout << "" << endl;
  cout << "<initialize>: Set robot pose as initial pose" << endl;
  cout << "<gripper_open>: Open all gripper" << endl;
  cout << "<gripper_close>: Close all gripper" << endl;
  cout << "" << endl;
// high level commands that require high level controller functionality to be executed
  cout << "COMMANDS TO HLC:" << endl;
  cout << "" << endl;
  cout << "<grieel_mode_transform>: Start all Grieel Trasformation automatic algorithm" << endl;
  cout << "<center_base>: Base motion to Support Polygon center" << endl;
  cout << "<driving_forward_config_LF>: Move LF Grieel joints in driving configuration" << endl;
  cout << "<driving_forward_config_LH>: Move LH Grieel joints in driving configuration" << endl;
  cout << "<driving_forward_config_RH>: Move RH Grieel joints in driving configuration" << endl;
  cout << "<driving_forward_config_RF>: Move RF Grieel joints in driving configuration" << endl;
  cout << "<standard_config_LF>: Move LF Grieel joints in walking/climbing configuration" << endl;
  cout << "<standard_config_LH>: Move LH Grieel joints in walking/climbing configuration" << endl;
  cout << "<standard_config_RH>: Move RH Grieel joints in walking/climbing configuration" << endl;
  cout << "<standard_config_RF>: Move RF Grieel joints in walking/climbing configuration" << endl;
  cout << "<driving_forward>: Move driving joint of ALL Grieel modules" << endl;
  cout << "<stop_driving_forward>: STOP driving joint motion of ALL Grieel modules" << endl;
  cout << "" << endl;
  cout << endl << "<help>: Show command list" << endl;

  cout << "------------------------ End ------------------------" << endl << endl;
  cout << "" << endl;

  cout << "Please open another terminal and send a command." << endl;
  cout << "\033[32m ros2 topic pub --once /string std_msgs/msg/String '{data: YOUR_COMMAND}'\033[m";
  cout << "" << endl;

  cout << endl << endl << "Waiting command..." << endl << endl;
}

void CommandInterface::execInputCommand(const std::string command)
{
  std::cout << "Input command is <" << "\033[32m" << command << "\033[m" << ">." << std::endl;
  std_msgs::msg::String msg;
  msg.data = command;

  if (command == "initialize") {
    command_to_LLC_pub_->publish(msg);
    std::cout << "\033[32m INPUT ACCEPTED \033[m" << std::endl;
  } else if (command == "gripper_open") {
    command_to_LLC_pub_->publish(msg);
    std::cout << "\033[32m INPUT ACCEPTED \033[m" << std::endl;
  } else if (command == "gripper_close") {
    command_to_LLC_pub_->publish(msg);
    std::cout << "\033[32m INPUT ACCEPTED \033[m" << std::endl;
  } else if (command == "grieel_mode_transform") {
    command_to_HLC_pub_->publish(msg);
    std::cout << "\033[32m INPUT ACCEPTED \033[m" << std::endl;
  } else if (command == "center_base") {
    command_to_HLC_pub_->publish(msg);
    std::cout << "\033[32m INPUT ACCEPTED \033[m" << std::endl;
  } else if (command.find("offset_condition_") != std::string::npos) {
    // This part is tentative. Need to be deleted in the future.
    command_to_state_estimator_pub_->publish(msg);
  } else if (command == "driving_forward_config_LF") {
    command_to_HLC_pub_->publish(msg);
    std::cout << "\033[32m INPUT ACCEPTED \033[m" << std::endl;
  } else if (command == "driving_forward_config_LH") {
    command_to_HLC_pub_->publish(msg);
    std::cout << "\033[32m INPUT ACCEPTED \033[m" << std::endl;
  }  else if (command == "driving_forward_config_RH") {
    command_to_HLC_pub_->publish(msg);
    std::cout << "\033[32m INPUT ACCEPTED \033[m" << std::endl;
  }  else if (command == "driving_forward_config_RF") {
    command_to_HLC_pub_->publish(msg);
    std::cout << "\033[32m INPUT ACCEPTED \033[m" << std::endl;
  }  else if (command == "standard_config_LF") {
    command_to_HLC_pub_->publish(msg);
    std::cout << "\033[32m INPUT ACCEPTED \033[m" << std::endl;
  }  else if (command == "standard_config_LH") {
    command_to_HLC_pub_->publish(msg);
    std::cout << "\033[32m INPUT ACCEPTED \033[m" << std::endl;
  }  else if (command == "standard_config_RH") {
    command_to_HLC_pub_->publish(msg);
    std::cout << "\033[32m INPUT ACCEPTED \033[m" << std::endl;
  }  else if (command == "standard_config_RF") {
    command_to_HLC_pub_->publish(msg);
    std::cout << "\033[32m INPUT ACCEPTED \033[m" << std::endl;
  }    else if (command == "driving_forward") {
    command_to_HLC_pub_->publish(msg);
    std::cout << "\033[32m INPUT ACCEPTED \033[m" << std::endl;
  } else if (command == "stop_driving_forward") {
    command_to_HLC_pub_->publish(msg);
    std::cout << "\033[32m INPUT ACCEPTED \033[m" << std::endl;
  } else if (command == "") {
    command_to_HLC_pub_->publish(msg);
    std::cout << "\033[32m INPUT ACCEPTED \033[m" << std::endl;
  } else if (command == "help") {
    showCommandList();
  } else {
    std::cout << "\033[31m INVALID COMMAND INPUT \033[m" << std::endl;
  }

  std::cout << std::endl << "Waiting command... " << std::endl;
}

void CommandInterface::joyBaseMotionTask(const sensor_msgs::msg::Joy & joy)
{
  lbr_msgs::msg::BaseMotionTask base_motion_task;
  const double xyz_gain = 0.005;
  const double rpy_gain = 0.03;

  base_motion_task.motion_duration = 100;  // [ms]

  // Translation motion
  base_motion_task.base_displacement.x =
    xyz_gain * (joy.axes.at(1) + joy.axes.at(4));
  base_motion_task.base_displacement.y =
    xyz_gain * (joy.axes.at(0) + joy.axes.at(3));
  base_motion_task.base_displacement.z =
    xyz_gain * (joy.buttons.at(4) + joy.buttons.at(5) -
    joy.buttons.at(6) - joy.buttons.at(7));

  // Rotation motion
  base_motion_task.base_angle_displacement.x =
    rpy_gain * -joy.axes.at(6);
  base_motion_task.base_angle_displacement.y =
    rpy_gain * -joy.axes.at(7);
  base_motion_task.base_angle_displacement.z =
    rpy_gain * (joy.buttons.at(11) - joy.buttons.at(12));

  base_motion_task_pub_->publish(base_motion_task);

#if DEBUG_ENABLED
  std::cout << "base_motion_task.base_displacement.x = " <<
    base_motion_task.base_displacement.x << std::endl;
  std::cout << "base_motion_task.base_displacement.y = " <<
    base_motion_task.base_displacement.y << std::endl;
  std::cout << "base_motion_task.base_displacement.z = " <<
    base_motion_task.base_displacement.z << std::endl;

  std::cout << "base_motion_task.base_angle_displacement.x = " <<
    base_motion_task.base_angle_displacement.x << std::endl;
  std::cout << "base_motion_task.base_angle_displacement.y = " <<
    base_motion_task.base_angle_displacement.y << std::endl;
  std::cout << "base_motion_task.base_angle_displacement.z = " <<
    base_motion_task.base_angle_displacement.z << std::endl;
#endif  // DEBUG_ENABLED
}

void CommandInterface::joyCallback(const sensor_msgs::msg::Joy & joy_cmd)
{
#if DEBUG_ENABLED
  std::cout << "joyCallback" << std::endl;
#endif  // DEBUG_ENABLED
  input_joy_cmd = joy_cmd;  // To subscribe topic "/joy" in a certain rate.
}

void CommandInterface::joyLimbMotionTask(
  const int & limb_id, const sensor_msgs::msg::Joy & joy)
{
  lbr_msgs::msg::LimbMotionTask limb_motion_task;
  const double xyz_gain = 0.01;
  const double pitch_gain = 0.05;

  limb_motion_task.limb_id = limb_id;
  limb_motion_task.swing_duration = 100;  // [ms]

  // x
  limb_motion_task.end_effector_displacement.x =
    xyz_gain * joy.axes.at(1);  // [m]
  // y
  limb_motion_task.end_effector_displacement.y =
    xyz_gain * joy.axes.at(0);
  // z
  if (joy.buttons.at(4) == 1) {
    limb_motion_task.end_effector_displacement.z =
      xyz_gain * joy.buttons.at(4);
  } else if (joy.buttons.at(6) == 1.0) {
    limb_motion_task.end_effector_displacement.z =
      -xyz_gain * joy.buttons.at(6);
  } else {
    limb_motion_task.end_effector_displacement.z = 0.0;
  }
#if DEBUG_ENABLED
  std::cout << "limb_motion_task.end_effector_displacement.x = " <<
    limb_motion_task.end_effector_displacement.x << std::endl;
  std::cout << "limb_motion_task.end_effector_displacement.y = " <<
    limb_motion_task.end_effector_displacement.y << std::endl;
  std::cout << "limb_motion_task.end_effector_displacement.z = " <<
    limb_motion_task.end_effector_displacement.z << std::endl;
#endif  // DEBUG_ENABLED
  // pitch
  limb_motion_task.pitch_angle_displacement =
    pitch_gain * -joy.axes.at(7);  // [rad]

  limb_motion_task.header.stamp = this->now();
  limb_motion_task_pub_->publish(limb_motion_task);
}

void CommandInterface::publishGripperCommand(const int & limb_id, const bool & close_gripper)
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CommandInterface>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}