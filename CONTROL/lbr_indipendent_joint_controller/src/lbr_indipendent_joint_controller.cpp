#include "lbr_indipendent_joint_controller/lbr_indipendent_joint_controller.hpp"

IndipendentJointController::IndipendentJointController()
: Node("lbr_indipendent_joint_controller")
{
  std::cout << "IndipendentJointController class is established." << std::endl;

  // initialize variables and resize
  std::string name_space = std::string(this->get_namespace());
  current_joint_state.position.resize(JOINT_NUM + 1);
  current_joint_state.velocity.resize(JOINT_NUM + 1);
  reference_joint_state.position.resize(JOINT_NUM);
  reference_joint_state.velocity.resize(JOINT_NUM);
  torque_control_output.data.resize(JOINT_NUM*LIMB_NUM);


  Ts =TS; // Samplin time [ms]
  Tf = TF; // 10Hz cut off of LP filter
  Tdf = TDF; // 50 Hz of LP filtering in derivative
  Tc = TC; // anti-wind up gain

  // initialize contact information for controller state managment
  in_contact = false;
  start_control = false;
  initial_time = 0;
  limb_in_contact = 4; // assume initially 4 limbs in contact
  swing_counter = 0;

  // Initialize controller gains and
  gravity_compensation = GRAVITY_COMPENSATION;
  control_grieel = CONTROL_GRIEEL; 

  this->declare_parameter("wheel_mode", true);

  bool wheel_mode;
  this->get_parameter("wheel_mode", wheel_mode);

  std::cout << name_space << " (IJC) : wheel_mode:= " << wheel_mode << std::endl; 
  
  if(wheel_mode == true){
    grieel_state_ = "wheel";
  } else if(wheel_mode == false){
    grieel_state_ = "gripper";
  }

  for (int i=0; i < JOINT_NUM; i++){
    Kpp[i] = KPP[i];
    Kpv[i] = KPV[i];
    Tiv[i] = TIV[i];
    N[i] = GEARBOX_RATIO[i];
    torque_limit[i] = SATURATION[i];
  }
  if(grieel_state_ == "wheel"){
    Kpv[6] = 20.0;
    Tiv[6] = 0.1;
    reference_grieel_state = M_PI/6;
  }else if(grieel_state_ == "gripper"){
    // Kpv[6] = 20.0;
    // Tiv[6] = 0.1;
    reference_grieel_state = 0.0;
  }

  if(PID == 1){  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    end_effector_contact_state_sub_LH_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    end_effector_contact_state_sub_RH_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    end_effector_contact_state_sub_RF_;

    for (int i=0; i < JOINT_NUM-1; i++){
      KP[i] = Kpv[i]*(Kpp[i] + 1/Tiv[i]);
      TI[i] = KP[i]*Tiv[i]/(Kpp[i]*Kpv[i]);
      TD[i] = Kpv[i]/KP[i];
    }
    KP[JOINT_NUM-1] = 0.0; 
    TD[JOINT_NUM-1] = 1.0; 
    TI[JOINT_NUM-1] = 1.0;  
  }

  Kp_GRIEEL = KP_GRIEEL;

  // initialize all controller related signals and reference joint state (only F2T as pi/2 initial desired position)
  for (int i=0; i < JOINT_NUM; i++){
    if(i==2){
      reference_joint_state.position.at(i) = M_PI_2;
      reference_joint_state.velocity.at(i) = 0.0;
    } else if(grieel_state_ == "wheel" && i==4){
      reference_joint_state.position.at(i) = M_PI;
      reference_joint_state.velocity.at(i) = 0.0;
    } else if(grieel_state_ == "wheel" && i==5){
      reference_joint_state.position.at(i) = M_PI_2;
      reference_joint_state.velocity.at(i) = 0.0;  
    }
    else{
      reference_joint_state.position.at(i) = 0.0;
      reference_joint_state.velocity.at(i) = 0.0;
    }

    integral[i] = 0.0; // initialize integral error cumulation
    anti_wind_up[i] = 0.0; // anti wind-up action
    previous_reference_joint_position[i] = 0.0;
    reference_joint_position_filtered[i] = 0.0;
    previous_reference_joint_position_filtered[i] = 0.0;
    previous_velocity_error[i] = 0.0;
    previous_saturated_torque[i] = 0.0;
    previous_u1[i] = 0.0;
    previous_u2[i] = 0.0;
    joint_velocity_feed_forward[i] = 0.0;
    gravitational_torque[i] = 0.0;

    previous_joint_position[i] = 0.0; 
    estimated_joint_velocity[i] = 0.0; 

    if(PID == 1){
      derivative_torque[i] = 0.0;
      previous_position_error[i] = 0.0; 
    }
  }
  // Identification of namespace for limb controller set-up
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

  // Open the csv file to save control results and tracking
  file_out.open("/home/ale/lbr_ws/src/LIMBERO/CONTROL/lbr_indipendent_joint_controller/" + joint_prefix + "_data_record.csv",std::ofstream::out | std::ofstream::trunc);
  if(PID == 0){
    file_out << "time,limb_in_contact,B2C_reference_position,B2C_reference_position_filtered,B2C_current_position,B2C_position_error,B2C_reference_velocity,B2C_current_velocity,B2C_feed_forward_velocity,B2C_velocity_error,B2C_integral_action,B2C_control_torque,B2C_gravity_torque,"
            << "C2F_reference_position,C2F_reference_position_filtered,C2F_current_position,C2F_position_error,C2F_reference_velocity,C2F_current_velocity,C2F_feed_forward_velocity,C2F_velocity_error,C2F_integral_action,C2F_control_torque,C2F_gravity_torque,"
            << "F2T_reference_position,F2T_reference_position_filtered,F2T_current_position,F2T_position_error,F2T_reference_velocity,F2T_current_velocity,F2T_feed_forward_velocity,F2T_velocity_error,F2T_integral_action,F2T_control_torque,F2T_gravity_torque,"
            << "T2E_reference_position,T2E_reference_position_filtered,T2E_current_position,T2E_position_error,T2E_reference_velocity,T2E_current_velocity,T2E_feed_forward_velocity,T2E_velocity_error,T2E_integral_action,T2E_control_torque,T2E_gravity_torque,"
            << "wristH_reference_position,wristH_reference_position_filtered,wristH_current_position,wristH_position_error,wristH_reference_velocity,wristH_current_velocity,wristH_feed_forward_velocity,wristH_velocity_error,wristH_integral_action,wristH_control_torque,wristH_gravity_torque,"
            << "wristV_reference_position,wristV_reference_position_filtered,wristV_current_position,wristV_position_error,wristV_reference_velocity,wristV_current_velocity,wristV_feed_forward_velocity,wristV_velocity_error,wristV_integral_action,wristV_control_torque,wristV_gravity_torque,"
            << "driving_reference_position,driving_reference_position_filtered,driving_current_position,driving_position_error,driving_reference_velocity,driving_current_velocity,driving_feed_forward_velocity,driving_velocity_error,driving_integral_action,driving_control_torque,driving_gravity_torque,\n";
  } else if(PID == 1){
    file_out << "time,limb_in_contact,B2C_reference_position,B2C_reference_position_filtered,B2C_current_position,B2C_position_error,B2C_integral_action,B2C_control_torque,B2C_gravity_torque,"
            << "C2F_reference_position,C2F_reference_position_filtered,C2F_current_position,C2F_position_error,C2F_integral_action,C2F_control_torque,C2F_gravity_torque,"
            << "F2T_reference_position,F2T_reference_position_filtered,F2T_current_position,F2T_position_error,F2T_integral_action,F2T_control_torque,F2T_gravity_torque,"
            << "T2E_reference_position,T2E_reference_position_filtered,T2E_current_position,T2E_position_error,T2E_integral_action,T2E_control_torque,T2E_gravity_torque,"
            << "wristH_reference_position,wristH_reference_position_filtered,wristH_current_position,wristH_position_error,wristH_integral_action,wristH_control_torque,wristH_gravity_torque,"
            << "wristV_reference_position,wristV_reference_position_filtered,wristV_current_position,wristV_position_error,wristV_integral_action,wristV_control_torque,wristV_gravity_torque,"
            << "driving_reference_position,driving_reference_position_filtered,driving_current_position,driving_position_error,driving_integral_action,driving_control_torque,driving_gravity_torque,\n";
  }

  // initialize Publisher and Subscirbers
  current_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    name_space + "/joint_states", 1,
    std::bind(&IndipendentJointController::currentJointStateCallback, this, std::placeholders::_1));
  joint_state_reference_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    name_space + "/lbr_limb_controller/joint_state", 1,
    std::bind(&IndipendentJointController::referenceJointStateCallback, this, std::placeholders::_1));
  end_effector_contact_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    name_space + "/contact", 10,
    std::bind(&IndipendentJointController::endEffectorContactStateCallback, this, std::placeholders::_1));
  whole_contact_state_sub_ =
    this->create_subscription<lbr_msgs::msg::EndEffectorContactState>(
    "/lbr_sim/contact_state", 1,
    std::bind(&IndipendentJointController::allEndEffectorContactStateCallback, this, std::placeholders::_1));

  // torque_control_pub_B2C_ = this-> create_publisher<std_msgs::msg::Float64>(
  //   name_space + "_B2C/joint_torques", 10);
  // torque_control_pub_C2F_ = this-> create_publisher<std_msgs::msg::Float64>(
  //   name_space + "_C2F/joint_torques", 10);
  // torque_control_pub_F2T_ = this-> create_publisher<std_msgs::msg::Float64>(
  //   name_space + "_F2T/joint_torques", 10);
  // torque_control_pub_T2E_ = this-> create_publisher<std_msgs::msg::Float64>(
  //   name_space + "_T2E/joint_torques", 10);
  // torque_control_pub_wristH_ = this-> create_publisher<std_msgs::msg::Float64>(
  //   name_space + "_wristH/joint_torques", 10);
  // torque_control_pub_wristV_ = this-> create_publisher<std_msgs::msg::Float64>(
  //   name_space + "_wristV/joint_torques", 10);
  // torque_control_pub_driving_ = this-> create_publisher<std_msgs::msg::Float64>(
  //   name_space + "_driving/joint_torques", 10);
  
  std::array<std::string, JOINT_NUM> joint_names = {"_B2C", "_C2F", "_F2T", "_T2E", "_wristH", "_wristV", "_driving"};
  for(int i=0; i<JOINT_NUM; i++){
    torque_control_publishers_[i] = this-> create_publisher<std_msgs::msg::Float64>(
    name_space + joint_names[i] + "/joint_torques", 10);
  }
  grieel_torque_publisher_ = this-> create_publisher<std_msgs::msg::Float64>(
    name_space + "_F1_H2P1" + "/joint_torques", 10);

  // Set up control loop rate
  if(PID == 0){
    control_loop_timer_ = this->create_wall_timer(
              std::chrono::milliseconds(Ts), std::bind(&IndipendentJointController::controlLoopPPI, this));
  } else if(PID == 1){
    control_loop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(Ts), std::bind(&IndipendentJointController::controlLoopPID, this));
  }
}

void IndipendentJointController::currentJointStateCallback(
    const sensor_msgs::msg::JointState & joint_state)
{
  // update current joint state with Gazebo feedback
  for (int i=0; i < JOINT_NUM + 1; i++){
    current_joint_state.position.at(i) = joint_state.position.at(i);
    current_joint_state.velocity.at(i) = joint_state.velocity.at(i);
  }
}

void IndipendentJointController::referenceJointStateCallback(
    const sensor_msgs::msg::JointState & desired_joint_state)
{
  // update reference joint state (set point) with LC control signals
  for (int i=0; i < JOINT_NUM; i++){
    reference_joint_state.position.at(i) = desired_joint_state.position.at(i);
    reference_joint_state.velocity.at(i) = desired_joint_state.velocity.at(i);
  }
}

void IndipendentJointController::endEffectorContactStateCallback(
  const std_msgs::msg::Bool & contact_state)
{
    if(contact_state.data == false){
      swing_counter ++;
      if(swing_counter >= 4 ){
        in_contact = false;
        swing_counter = 0;
      }
    }
    else if (contact_state.data == true) {
      in_contact = true;
      swing_counter = 0;
      if(start_control == false){
        start_control = true;
      }
    }
}


void IndipendentJointController::allEndEffectorContactStateCallback(
  const lbr_msgs::msg::EndEffectorContactState & contact_state)
{
  limb_in_contact = 0;
  for (int limb_id = 0; limb_id < LIMB_NUM; limb_id++) {
    if (contact_state.is_contact.at(limb_id) == true) {
      limb_in_contact++;
    }
  }
  if(limb_in_contact == 4){
    start_control = true;
  }
}

std::array<float, JOINT_NUM> IndipendentJointController::GravityCompensation(){
  // define and initialize matrices and vector used for Tau_g computation
  Eigen::Matrix<double, 4, 4> A_beforei_i[JOINT_NUM];
  Eigen::Matrix<double, 4, 4> A_0_i[JOINT_NUM];
  Eigen::Matrix<double, 1, 4> p_cm_tilde[JOINT_NUM];
  Eigen::Matrix<double, 1, 4> r_cm_tilde[JOINT_NUM];
  Eigen::Vector3d p_cm[JOINT_NUM];
  Eigen::Vector3d p_o[JOINT_NUM];
  Eigen::Vector3d z[JOINT_NUM];
  Eigen::Vector3d g0;
  Eigen::Matrix<double, 3, JOINT_NUM> Jp_l[JOINT_NUM];

  bool contact = in_contact;

  // properly select the model
  double a[JOINT_NUM];
  double d[JOINT_NUM];
  double alpha[JOINT_NUM];
  double offset[JOINT_NUM];
  double m[JOINT_NUM];

  if(contact){
    for(int i=0; i<JOINT_NUM; i++){
      a[i] = A_CONTACT[i];
      d[i] = D_CONTACT[i];
      alpha[i] = ALPHA_CONTACT[i];
      offset[i] = OFFSET_CONTACT[i];
      r_cm_tilde[i] << CoM_LINK_FRAME_CONTACT[i][0], CoM_LINK_FRAME_CONTACT[i][1], CoM_LINK_FRAME_CONTACT[i][2],1;
      m[i] = MASS_CONTACT[i];
      g0 << 0.0,0.0,GRAVITY;
    }
    if(limb_in_contact!=0){ // avoid numerical problem
      m[JOINT_NUM-1] += BODY_MASS/limb_in_contact;
    }
  }else if(!contact){
    for(int i=0; i<JOINT_NUM; i++){
      a[i] = A[i];
      d[i] = D[i];
      alpha[i] = ALPHA[i];
      offset[i] = OFFSET[i];
      r_cm_tilde[i] << CoM_LINK_FRAME[i][0], CoM_LINK_FRAME[i][1], CoM_LINK_FRAME[i][2],1;
      m[i] = MASS[i];
      g0 << 0.0,0.0,-GRAVITY;
    }
  }

  // compute the homogeneus transformation matrices and CoM, z-axis vectors, in base frame
  float q[JOINT_NUM];
  for(int i=0; i<JOINT_NUM; i++){
    q[i] = current_joint_state.position.at(i);
    if(contact){
      q[i] = current_joint_state.position.at(JOINT_NUM - i - 1);
      if(i == 4){
        q[i] = current_joint_state.position.at(JOINT_NUM - i - 1) - M_PI_2;
      }
    }
    A_beforei_i[i].row(0) << std::cos(q[i]+offset[i]), -std::sin(q[i]+offset[i])*std::cos(alpha[i]), std::sin(q[i]+offset[i])*std::sin(alpha[i]), a[i]*std::cos(q[i]+offset[i]);
    A_beforei_i[i].row(1) << std::sin(q[i]+offset[i]),  std::cos(q[i]+offset[i])*std::cos(alpha[i]), -std::cos(q[i]+offset[i])*std::sin(alpha[i]), a[i]*std::sin(q[i]+offset[i]);
    A_beforei_i[i].row(2) <<    0 , std::sin(alpha[i]), std::cos(alpha[i]), d[i];
    A_beforei_i[i].row(3) <<    0,        0,                 0,               1;
    if(i==0){
       A_0_i[0] = A_beforei_i[0];
    } else if(i>0){
        A_0_i[i] =  A_0_i[i-1]*A_beforei_i[i];
    }
    p_cm_tilde[i] = A_0_i[i]*r_cm_tilde[i].transpose();
    p_cm[i] << p_cm_tilde[i](0), p_cm_tilde[i](1), p_cm_tilde[i](2);
    p_o[i]  << A_0_i[i](0,3),A_0_i[i](1,3),A_0_i[i](2,3);
    z[i]    << A_0_i[i](0,2),A_0_i[i](1,2),A_0_i[i](2,2);
  }

  for(int j=0; j<JOINT_NUM; j++){
    for(int i=0; i<JOINT_NUM; i++){
      if(i==0){
        Eigen::Vector3d z0(0.0, 0.0, 1.0);
        Eigen::Vector3d p_o0(0.0, 0.0, 0.0);
        Jp_l[j].col(i) << z0.cross(p_cm[j] - p_o0);
      }
      else if(i>0){
        if(i>j){
          Jp_l[j].col(i) << 0.0, 0.0, 0.0;
        } else{
          Jp_l[j].col(i) = z[i-1].cross(p_cm[j]-p_o[i-1]);
        }
      }
    }
  }

  std::array<float, JOINT_NUM>  Tau_g;
  for(int i=0; i<JOINT_NUM; i++){
    Tau_g[i] = 0.0;
    for(int j=0; j<JOINT_NUM; j++){
      Tau_g[i] = Tau_g[i] - m[j]*g0.transpose()*Jp_l[j].col(i);
    }
  }

  std::array<float, JOINT_NUM>  Tau_g_contact;
  if(contact){
    for(int i=0; i<JOINT_NUM; i++){
      Tau_g_contact[i] = Tau_g[JOINT_NUM-i-1];
    }
    Tau_g = Tau_g_contact;
    Tau_g[0] = 0.0; // maybe B2C horizontal revolute joint can be affected by undesired disturbances..
  }

  return Tau_g;
}

void IndipendentJointController::controlLoopPPI()
{
  // compute control action
  // definition of support variables
  float joint_position_error[JOINT_NUM];
  float joint_velocity_error[JOINT_NUM];
  float joint_velocity_reference[JOINT_NUM];
  float u1[JOINT_NUM];
  float u2[JOINT_NUM];



  if(start_control){
      if(initial_time == 0){
          initial_time = this->get_clock()->now().seconds();
      }
      double current_time = this->get_clock()->now().seconds() - initial_time;
      file_out << current_time << ",";
      if(in_contact){
        file_out << 1.0 << ",";
      } else{
        file_out << 0.0 <<",";
      }
      
      gravitational_torque = GravityCompensation();

    // computation of all control signal components
    for (int i=0; i < JOINT_NUM; i++){
      // dq_d(t) = kpp*(q_d(t) - q(t)) + dq_ff(t), Proportional position controller + feedforwaed velocity

      reference_joint_position_filtered[i] = reference_joint_position_filtered[i]*(Tf-(Ts*1e-3))/Tf + (Ts*1e-3)/Tf * previous_reference_joint_position[i];
      joint_position_error[i] = (reference_joint_position_filtered[i] - current_joint_state.position.at(i));

      joint_velocity_feed_forward[i] = joint_velocity_feed_forward[i]*(1-(Ts*1e-3)*(1/Tdf)) + (reference_joint_position_filtered[i] - previous_reference_joint_position_filtered[i])*(1/Tdf) ;
      
      if (joint_velocity_feed_forward[i] > velocity_limit[i]){
        joint_velocity_feed_forward[i] = velocity_limit[i];
      } else if(joint_velocity_feed_forward[i] < -velocity_limit[i] ){
        joint_velocity_feed_forward[i] = -velocity_limit[i];
      }

      joint_velocity_reference[i] = joint_position_error[i] * Kpp[i] + Vff*joint_velocity_feed_forward[i];
      if (joint_velocity_reference[i] > velocity_limit[i]){
        joint_velocity_reference[i] = velocity_limit[i];
      } else if(joint_velocity_reference[i] < -velocity_limit[i] ){
        joint_velocity_reference[i] = -velocity_limit[i];
      }

      estimated_joint_velocity[i] = (1-(1/Tdf)*Ts*1e-3)*estimated_joint_velocity[i] + (1/Tdf)*(current_joint_state.position.at(i) - previous_joint_position[i]);
     
     if((i == 6) && (grieel_state_ == "wheel")){
        joint_velocity_reference[i] = reference_joint_state.velocity.at(i);
        //estimated_joint_velocity[6] = current_joint_state.velocity.at(6);
      }

      // tau(t) = kpv*(dq_d(t) - dq(t)) + Ts*(dq_d(t) - dq(t))*kpv/Tiv, Propodtional Integral controller + gravity compensation (to estimate)
      
      //joint_velocity_error[i] = joint_velocity_reference[i] - current_joint_state.velocity.at(i);
      joint_velocity_error[i] = joint_velocity_reference[i] - estimated_joint_velocity[i];

      if(ANTI_WINDUP_METHOD == 1){ // back-calculation
        integral[i] += (Ts*1e-3)*(Kpv[i]/Tiv[i]*joint_velocity_error[i] + anti_wind_up[i]);


        torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) = Kpv[i] * joint_velocity_error[i] + integral[i] + gravity_compensation*gravitational_torque[i];

        // ANTI-WIND-UP computation and Torque SATURATION
        if(torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) > torque_limit[i]){
          anti_wind_up[i] = (torque_limit[i] - torque_control_output.data.at(LIMB_ID*JOINT_NUM + i))*1/Tc;
          torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) = torque_limit[i];
        } else if(torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) < - torque_limit[i]){
          anti_wind_up[i] = (-torque_limit[i] - torque_control_output.data.at(LIMB_ID*JOINT_NUM + i))*1/Tc;
          torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) = - torque_limit[i];
        } else {
          anti_wind_up[i] = 0.0;
        }

      } else if(ANTI_WINDUP_METHOD == 2){ // de-saturation
          float T = Ts*1e-3;
          u1[i] = (Tiv[i]- L*T)*previous_u1[i]/Tiv[i] + Kpv[i]*joint_velocity_error[i] + Kpv[i]*(T-Tiv[i])*previous_velocity_error[i]/Tiv[i];
          u2[i] = (Tiv[i]-L*T)*previous_u2[i]/Tiv[i] + L*T*previous_saturated_torque[i]/Tiv[i];

          torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) = u1[i] + u2[i] + gravity_compensation*gravitational_torque[i];

          if(torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) > torque_limit[i]){
          torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) = torque_limit[i];
          } else if(torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) < - torque_limit[i]){
          torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) = - torque_limit[i];
          }

          previous_saturated_torque[i] = torque_control_output.data.at(LIMB_ID*JOINT_NUM + i);
          previous_velocity_error[i] = joint_velocity_error[i];
          previous_u1[i] = u1[i];
          previous_u2[i] = u2[i];

      } else if(ANTI_WINDUP_METHOD == 3){ // conditional integration (clamping)
        torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) = Kpv[i]*joint_velocity_error[i] + Kpv[i]/Tiv[i] * integral[i] + gravity_compensation*gravitational_torque[i];;
        if(torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) >= torque_limit[i] && joint_velocity_error[i] >= 0){
          // don't update integral
          torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) = torque_limit[i];
        } else if(torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) <= -torque_limit[i] && joint_velocity_error[i] <= 0){
          // don't update integral
          torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) = -torque_limit[i];
        } else{
          integral[i] += Ts*1e-3*joint_velocity_error[i];
        }
      }

      previous_reference_joint_position[i] = reference_joint_state.position.at(i); // store reference joint state for next iteration
      previous_reference_joint_position_filtered[i] = reference_joint_position_filtered[i];
      previous_joint_position[i] = current_joint_state.position.at(i);

      torque_control_data.data = torque_control_output.data.at(LIMB_ID*JOINT_NUM + i);
      torque_control_publishers_[i]->publish(torque_control_data);
      if(ANTI_WINDUP_METHOD == 1){
          file_out << reference_joint_state.position.at(i) << ","
                  << reference_joint_position_filtered[i] << ","
                  << current_joint_state.position.at(i)   << ","
                  << joint_position_error[i]              << ","
                  << joint_velocity_reference[i]          << ","
                  //<< current_joint_state.velocity.at(i)   << ","
                  << estimated_joint_velocity[i]          << ","
                  << joint_velocity_feed_forward[i]       << ","
                  << joint_velocity_error[i]              << ","
                  << integral[i]                          << ","
                  << torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) <<","
                  << gravitational_torque[i] <<",";
      }else if(ANTI_WINDUP_METHOD == 2){
          file_out << reference_joint_state.position.at(i) << ","
                  << reference_joint_position_filtered[i] << ","
                  << current_joint_state.position.at(i)   << ","
                  << joint_position_error[i]              << ","
                  << joint_velocity_reference[i]          << ","
                  << current_joint_state.velocity.at(i)   << ","
                  << joint_velocity_feed_forward[i]       << ","
                  << joint_velocity_error[i]              << ","
                  <<  u2[i]                          << ","
                  << torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) <<","
                  << gravitational_torque[i] <<",";
      }else if(ANTI_WINDUP_METHOD == 3){
          file_out << reference_joint_state.position.at(i) << ","
                  << reference_joint_position_filtered[i] << ","
                  << current_joint_state.position.at(i)   << ","
                  << joint_position_error[i]              << ","
                  << joint_velocity_reference[i]          << ","
                  // << current_joint_state.velocity.at(i)   << ","
                  << estimated_joint_velocity[i]          << ","
                  << joint_velocity_feed_forward[i]       << ","
                  << joint_velocity_error[i]              << ","
                  << integral[i]                          << ","
                  << torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) <<","
                  << gravitational_torque[i] <<",";
      }
    }
    if(control_grieel){
      float grieel_torque; 
      float grieel_error = reference_grieel_state - current_joint_state.position.at(JOINT_NUM);
      if(grieel_error > 0.1){
        grieel_torque = (grieel_error)*Kp_GRIEEL;
        std_msgs::msg::Float64 grieel_msg;
        grieel_msg.data = grieel_torque;
        grieel_torque_publisher_->publish(grieel_msg);
      }
    }
    // Publish controller Torque output
    file_out << "\n";
    // torque_control_pub_->publish(torque_control_output);
  }
}

void IndipendentJointController::controlLoopPID()
{
  // compute control action
  // definition of support variables
  float joint_position_error[JOINT_NUM];


  if(start_control){
    if(initial_time == 0){
        initial_time = this->get_clock()->now().seconds();
    }
    double current_time = this->get_clock()->now().seconds() - initial_time;
    file_out << current_time << ",";
    if(in_contact){
      file_out << 1.0 << ",";
    } else{
      file_out << 0.0 <<",";
    }
    gravitational_torque = GravityCompensation();
  

    // computation of all control signal components
    for (int i=0; i < JOINT_NUM; i++){
      // dq_d(t) = kpp*(q_d(t) - q(t)) + dq_ff(t), Proportional position controller + feedforwaed velocity

      reference_joint_position_filtered[i] = reference_joint_position_filtered[i]*(Tf-(Ts*1e-3))/Tf + (Ts*1e-3)/Tf * previous_reference_joint_position[i];
      
      joint_position_error[i] = (reference_joint_position_filtered[i] - current_joint_state.position.at(i));

      derivative_torque[i] = derivative_torque[i]*(TD[i]-N_PID*Ts*1e-3)/TD[i] + (joint_position_error[i] - previous_position_error[i])*KP[i]/TD[i];

      if(ANTI_WINDUP_METHOD == 1){ // back-calculation
        integral[i] += (Ts*1e-3)*(KP[i]/TI[i]*previous_position_error[i] + anti_wind_up[i]);


        torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) = KP[i]*joint_position_error[i] + derivative_torque[i] + integral[i] + gravity_compensation*gravitational_torque[i];

        // ANTI-WIND-UP computation and Torque SATURATION
        if(torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) > torque_limit[i]){
          anti_wind_up[i] = (torque_limit[i] - torque_control_output.data.at(LIMB_ID*JOINT_NUM + i))*1/Tc;
          torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) = torque_limit[i];
        } else if(torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) < - torque_limit[i]){
          anti_wind_up[i] = (-torque_limit[i] - torque_control_output.data.at(LIMB_ID*JOINT_NUM + i))*1/Tc;
          torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) = - torque_limit[i];
        } else {
          anti_wind_up[i] = 0.0;
        }

      } else if(ANTI_WINDUP_METHOD == 2){ // de-saturation NOT WORKING FOR PID!!!
          // float T = Ts*1e-3;
          // u1[i] = (Tiv[i]- L*T)*previous_u1[i]/Tiv[i] + Kpv[i]*joint_velocity_error[i] + Kpv[i]*(T-Tiv[i])*previous_velocity_error[i]/Tiv[i];
          // u2[i] = (Tiv[i]-L*T)*previous_u2[i]/Tiv[i] + L*T*previous_saturated_torque[i]/Tiv[i];

          // torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) = u1[i] + u2[i] + gravity_compensation*gravitational_torque[i];

          // if(torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) > torque_limit[i]){
          // torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) = torque_limit[i];
          // } else if(torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) < - torque_limit[i]){
          // torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) = - torque_limit[i];
          // }

          // previous_saturated_torque[i] = torque_control_output.data.at(LIMB_ID*JOINT_NUM + i);
          // previous_velocity_error[i] = joint_velocity_error[i];
          // previous_u1[i] = u1[i];
          // previous_u2[i] = u2[i];

      } else if(ANTI_WINDUP_METHOD == 3){ // conditional integration (clamping)
        torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) = KP[i]*joint_position_error[i] + derivative_torque[i] + KP[i]/TI[i]*integral[i] + gravity_compensation*gravitational_torque[i];
        if(torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) >= torque_limit[i] && joint_position_error[i] >= 0){
          // don't update integral
          torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) = torque_limit[i];
        } else if(torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) <= -torque_limit[i] && joint_position_error[i] <= 0){
          // don't update integral
          torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) = -torque_limit[i];
        } else{
          integral[i] += Ts*1e-3*joint_position_error[i];
        }
      }

      previous_reference_joint_position[i] = reference_joint_state.position.at(i); // store reference joint state for next iteration
      previous_position_error[i] = joint_position_error[i];

      previous_reference_joint_position_filtered[i] = reference_joint_position_filtered[i];

      torque_control_data.data = torque_control_output.data.at(LIMB_ID*JOINT_NUM + i);
      torque_control_publishers_[i]->publish(torque_control_data);

      if(ANTI_WINDUP_METHOD == 1){
          file_out << reference_joint_state.position.at(i) << ","
                  << reference_joint_position_filtered[i] << ","
                  << current_joint_state.position.at(i)   << ","
                  << joint_position_error[i]              << ","
                  << integral[i]                          << ","
                  << torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) <<","
                  << gravitational_torque[i] <<",";
      }else if(ANTI_WINDUP_METHOD == 2){
          // file_out << reference_joint_state.position.at(i) << ","
          //         << reference_joint_position_filtered[i] << ","
          //         << current_joint_state.position.at(i)   << ","
          //         << joint_position_error[i]              << ","
          //         << joint_velocity_reference[i]          << ","
          //         << current_joint_state.velocity.at(i)   << ","
          //         << joint_velocity_feed_forward[i]       << ","
          //         << joint_velocity_error[i]              << ","
          //         <<  u2[i]                          << ","
          //         << torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) <<","
          //         << gravitational_torque[i] <<",";
      }else if(ANTI_WINDUP_METHOD == 3){
          file_out << reference_joint_state.position.at(i) << ","
                  << reference_joint_position_filtered[i] << ","
                  << current_joint_state.position.at(i)   << ","
                  << joint_position_error[i]              << ","
                  << integral[i]                          << ","
                  << torque_control_output.data.at(LIMB_ID*JOINT_NUM + i) <<","
                  << gravitational_torque[i] <<",";
      }
    }
    // Publish controller Torque output

    file_out << "\n";
    // torque_control_pub_->publish(torque_control_output);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IndipendentJointController>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  node->file_out.close();

  return 0;
}
