#include "lbr_data_csv/lbr_data_csv.hpp"


DataCSV::DataCSV()
: Node("lbr_data_csv")
{
  Ts = 20; //[ms]
  start_sampler = false; 
  initial_time = 0; 

  limb_prefix = std::string(this->get_namespace());
  std::string topic_prefix = std::string(this->get_namespace()) +
    "/" + std::string(this->get_name());
  
    std::cout << "[Limb " + limb_prefix + " ] DataCSV class is established." << std::endl;

    dynamixel_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      std::string(this->get_namespace()) + "/lbr_dynamixel_controller/encoder_joint_state", 1,
      std::bind(&DataCSV::dynamixelEncoderCallback, this, std::placeholders::_1));
    limb_controller_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      std::string(this->get_namespace()) + "/lbr_limb_controller/joint_state", 1,
      std::bind(&DataCSV::limbControllerCallback, this, std::placeholders::_1));


      // Open the csv file to save control results and tracking
    file_out.open("/home/ale/lbr_ws/src/LIMBERO/TOOLS/lbr_data_csv/" + limb_prefix + "_joint_data.csv",std::ofstream::out | std::ofstream::trunc);
    file_out << "time,B2C_reference,B2C_state,C2F_reference,C2F_state,F2T_reference,F2T_state,T2E_reference,T2E_state,wristH_reference,wristH_state,wristV_reference,wristV_state,driving_reference,driving_state,\n";

    current_dynamixel_state.position.resize(JOINT_NUM);
    current_dynamixel_state.name.resize(JOINT_NUM);
    current_dynamixel_state.name.at(0) = limb_prefix + "_B2C";
    current_dynamixel_state.name.at(1) = limb_prefix + "_C2F";
    current_dynamixel_state.name.at(2) = limb_prefix + "_F2T";
    current_dynamixel_state.name.at(3) = limb_prefix + "_T2E";
    current_dynamixel_state.name.at(4) = limb_prefix + "_wristH";
    current_dynamixel_state.name.at(5) = limb_prefix + "_wristV";
    current_dynamixel_state.name.at(6) = limb_prefix + "_driving"; 
    current_dynamixel_state.header.frame_id = this->get_namespace();
    current_dynamixel_state.position.at(0) = 0.0;
    current_dynamixel_state.position.at(1) = 0.0;
    current_dynamixel_state.position.at(2) = 0.0;
    current_dynamixel_state.position.at(3) = 0.0;
    current_dynamixel_state.position.at(4) = 0.0;
    current_dynamixel_state.position.at(5) = 0.0;
    current_dynamixel_state.position.at(6) = 0.0;

    reference_joint_state.position.resize(JOINT_NUM);
    reference_joint_state.name.resize(JOINT_NUM);
    reference_joint_state.name.at(0) = limb_prefix + "_B2C";
    reference_joint_state.name.at(1) = limb_prefix + "_C2F";
    reference_joint_state.name.at(2) = limb_prefix + "_F2T";
    reference_joint_state.name.at(3) = limb_prefix + "_T2E";
    reference_joint_state.name.at(4) = limb_prefix + "_wristH";
    reference_joint_state.name.at(5) = limb_prefix + "_wristV";
    reference_joint_state.name.at(6) = limb_prefix + "_driving";
    reference_joint_state.header.frame_id = this->get_namespace();
    reference_joint_state.position.at(0) = 0.0;
    reference_joint_state.position.at(1) = 0.0;
    reference_joint_state.position.at(2) = 0.0;
    reference_joint_state.position.at(3) = 0.0;
    reference_joint_state.position.at(4) = 0.0;
    reference_joint_state.position.at(5) = 0.0;
    reference_joint_state.position.at(6) = 0.0;

}

void DataCSV::samplerCallback()
{
  if(initial_time == 0){
    initial_time = this->get_clock()->now().nanoseconds()/1e6;
  }
  double current_time = this->get_clock()->now().nanoseconds()/1e6 - initial_time;
  file_out << current_time << ",";

  for(int i=0; i<JOINT_NUM; i++){

    file_out << reference_joint_state.position.at(i) << ","
             << current_dynamixel_state.position.at(i) << ",";
  }
  file_out << "\n";
}

void DataCSV::dynamixelEncoderCallback(const sensor_msgs::msg::JointState & encoder_joint_state)
{
    current_dynamixel_state.header.stamp = encoder_joint_state.header.stamp;
    for (int i=0; i < JOINT_NUM; i++){
    current_dynamixel_state.position.at(i) = encoder_joint_state.position.at(i);
    }
    if(start_sampler == false){
      data_sampler_timer = this->create_wall_timer(
        std::chrono::milliseconds(Ts), std::bind(&DataCSV::samplerCallback, this));
      start_sampler = true; 
    }
}


void DataCSV::limbControllerCallback(const sensor_msgs::msg::JointState & limb_controller_joint_state)
{
  reference_joint_state.header.stamp = limb_controller_joint_state.header.stamp;
  for (int i=0; i < JOINT_NUM; i++){
    reference_joint_state.position.at(i) = limb_controller_joint_state.position.at(i);
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DataCSV>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  node->file_out.close();

  return 0;
}