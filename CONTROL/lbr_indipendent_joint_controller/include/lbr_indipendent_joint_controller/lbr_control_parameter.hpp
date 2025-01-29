#include <math.h>
#include <lbr_parameter.hpp>

// CONTROLLER PARAMETER:

constexpr float TS = 1; // sampling time of discrete time controller [ms]

// P+PI gains, tuned in MATLAB
constexpr float KPP[JOINT_NUM] =  {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 0.0};  
constexpr float KP_GRIEEL =  10.0;           // Position P gains
// constexpr float KPV[JOINT_NUM] =  {1090.0, 970.0, 680.0, 820.0, 680.0, 970.0, 0.0};   // Velocity P gains
// constexpr float TIV[JOINT_NUM] = {0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 1.0};           // Velocity I gains
constexpr float KPV[JOINT_NUM] =  {40.0, 80.0, 60.0, 40.0, 60.0, 50.0, 0.0};       // Velocity P gains
constexpr float TIV[JOINT_NUM] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};                    // Velocity I gains

// Saturation on control action, for anti-wind up implementation
// constexpr float SATURATION[JOINT_NUM] = {15.0, 15.0, 15.0, 10.0, 8.0, 10.0, 6.0};

constexpr float SATURATION[JOINT_NUM] = {16.0, 16.0, 10.0, 8.0, 12.0, 12.0, 8.0};
constexpr float velocity_limit[JOINT_NUM] = {3.0, 5.0, 5.0, 4.0, 4.0, 4.0, 4.0};

// additional control paramater
constexpr float WCP_F = 2.0; // [Hz] cut off frequency of reference position LP filter
constexpr float WCV_F = 10.0; // [Hz] cut off frequency of velocity derivation (feed forward action) LP filter, for REALIZABILITY of derivative

constexpr float TF = 1.0/(WCP_F*2*M_PI);  // position refrence smoothed, LP filter time constant
constexpr float TDF = 1.0/(WCV_F*2*M_PI); // velocity derivative feed forward, LP filter time constant
constexpr float TC = TIV[0]*0.1; // anti-wind up feedback gain 

constexpr float L = 20.0; // additional paramter for the de-saturation dynamic

constexpr float N_PID = 10.0; // realizability gain for derivative PID action

constexpr bool CONTROL_GRIEEL = true; 
constexpr int GRAVITY_COMPENSATION = 0; // flag variable to know if use or not gravity compensation action
constexpr double Vff = 0.1; //flag variable for velocity feed forward action (between 0 and 1 to modulate the effect)
constexpr int ANTI_WINDUP_METHOD = 1; // 1 = back-calculation; 2 = de saturation; 3 = conditional integration (Clamping)
constexpr int PID = 0; // decide if using 1 = PID or 0 = (cascade) PPI 

// SYSTEM PARAMETER:

// environemntal gravity vector magnitude
constexpr float GRAVITY = 9.81; // earth gravity

// motor characteristic
constexpr float GEARBOX_RATIO[JOINT_NUM] = {272.5, 272.5, 353.5, 353.5, 257.4, 257.4, 353.5}; // reduction ratio of motors

