#include "lbr_data_record/lbr_data_record.hpp"

DataRecord::DataRecord()
: Node("lbr_data_record")
{

  std::cout << "DataRecord class is established." << std::endl;

  std::string package_share_directory = ament_index_cpp::get_package_share_directory("lbr_data_record");
  file_out_LF.open("/home/ale/lbr_ws/src/LIMBERO/TOOLS/lbr_data_record/script/data_record_LF.csv",std::ofstream::out | std::ofstream::trunc); 
  file_out_LH.open("/home/ale/lbr_ws/src/LIMBERO/TOOLS/lbr_data_record/script/data_record_LH.csv",std::ofstream::out | std::ofstream::trunc); 
  file_out_RH.open("/home/ale/lbr_ws/src/LIMBERO/TOOLS/lbr_data_record/script/data_record_RH.csv",std::ofstream::out | std::ofstream::trunc); 
  file_out_RF.open("/home/ale/lbr_ws/src/LIMBERO/TOOLS/lbr_data_record/script/data_record_RF.csv",std::ofstream::out | std::ofstream::trunc); 

  for(int i=0; i < LIMB_NUM; i++){
    if(i == 0){
      file_out_list[i] = &file_out_LF; 
    }else if (i==1){
      file_out_list[i] = &file_out_LH; 
    }else if (i==2){
      file_out_list[i] = &file_out_RH; 
    }else if (i==3){
      file_out_list[i] = &file_out_RF; 
    }
    
    *file_out_list[i] << "time,B2C_joint_pub,B2C_reference_position,B2C_feedback_position,B2C_position_error,B2C_reference_velocity,B2C_feedback_velocity,B2C_velocity_error,B2C_torque,"
                     <<      "C2F_joint_pub,C2F_reference_position,C2F_feedback_position,C2F_position_error,C2F_reference_velocity,C2F_feedback_velocity,C2F_velocity_error,C2F_torque,"
                     <<      "F2T_joint_pub,F2T_reference_position,F2T_feedback_position,F2T_position_error,F2T_reference_velocity,F2T_feedback_velocity,F2T_velocity_error,F2T_torque,"
                     <<      "T2E_joint_pub,T2E_reference_position,T2E_feedback_position,T2E_position_error,T2E_reference_velocity,T2E_feedback_velocity,T2E_velocity_error,T2E_torque,"
                     <<      "wristH_joint_pub,wristH_reference_position,wristH_feedback_position,wristH_position_error,wristH_reference_velocity,wristH_feedback_velocity,wristH_velocity_error,wristH_torque,"
                     <<      "wristV_joint_pub,wristV_reference_position,wristV_feedback_position,wristV_position_error,wristV_reference_velocity,wristV_feedback_velocity,wristV_velocity_error,wristV_torque,"
                     <<      "driving_joint_pub,driving_reference_position,driving_feedback_position,driving_position_error,driving_reference_velocity,driving_feedback_velocity,driving_velocity_error,driving_torque,\n";
  }

  // initialize
  for(int j=0; j < LIMB_NUM; j++){
    for(int i=0; i < JOINT_NUM; i++){   
      joint_trajectory_points[j*JOINT_NUM + i] = 0.0;
    }
  }

  // initialize Subscirbers
  controller_state_sub_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
    "custom_joint_trajectory_controller/controller_state", 1,
    std::bind(&DataRecord::controllerStateCallback, this, std::placeholders::_1));
  
  joint_trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "/custom_joint_trajectory_controller/joint_trajectory", 1,
    std::bind(&DataRecord::jointTrajectoryCallback, this, std::placeholders::_1));
  
  start_control = false; 
}

void DataRecord::controllerStateCallback(
  const control_msgs::msg::JointTrajectoryControllerState & controller_state)
{
  if(start_control == false){
    start_control = true;
    initial_time = this->get_clock()->now().seconds();
  }

  for(int j=0; j < LIMB_NUM; j++){
    *file_out_list[j] << this->get_clock()->now().seconds() - initial_time <<",";
    for(int i=0; i < JOINT_NUM; i++){  
      *file_out_list[j] << joint_trajectory_points[j*JOINT_NUM + i] << ","; 
      *file_out_list[j] << controller_state.reference.positions[j*JOINT_NUM + i] << ",";
      *file_out_list[j] << controller_state.feedback.positions[j*JOINT_NUM + i] << ",";
      *file_out_list[j] << controller_state.error.positions[j*JOINT_NUM + i] << ",";
      *file_out_list[j] << controller_state.reference.velocities[j*JOINT_NUM + i] << ",";
      *file_out_list[j] << controller_state.feedback.velocities[j*JOINT_NUM + i] << ",";
      *file_out_list[j] << controller_state.error.velocities[j*JOINT_NUM + i] << ",";
      *file_out_list[j] << controller_state.output.effort[j*JOINT_NUM + i] << ",";
    }
    *file_out_list[j] <<"\n";
  }
}

void DataRecord::jointTrajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory & joint_trajectory_state)
{

  for(int j=0; j < LIMB_NUM; j++){
    for(int i=0; i < JOINT_NUM; i++){   
      joint_trajectory_points[j*JOINT_NUM + i] = joint_trajectory_state.points.at(0).positions.at(j*JOINT_NUM+i);
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DataRecord>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  for(int i=0; i<LIMB_NUM; i++){
    node->file_out_list[i]->close();
  }

  return 0;
}