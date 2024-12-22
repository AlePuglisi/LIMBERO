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

#ifndef LBR_DESCRIPTION__LBR_DESCRIPTION_HPP_
#define LBR_DESCRIPTION__LBR_DESCRIPTION_HPP_

#include <math.h>

// Limb,Joint and Gripper numbers of LIMBERO+GRIEEL
constexpr int LIMB_NUM = 4;
constexpr int JOINT_NUM = 7;
constexpr int GRIPPER_MOTOR_NUM = 1;

// Link lengths (Unit: [m])
constexpr double L1 = 0.031;   // Coxa
constexpr double L2 = 0.2;     // Femur
constexpr double L3 = 0.2;     // Tibia
constexpr double L4 = 0.11475; // Limb end

// Limb Joint Limits (Unit: [rad])
constexpr double JOINT1_UPPER = 2.10;
constexpr double JOINT1_LOWER = -JOINT1_UPPER;
constexpr double JOINT2_UPPER = 1.30;
constexpr double JOINT2_LOWER = -1.95;
constexpr double JOINT3_UPPER = 2.95;
constexpr double JOINT3_LOWER = -0.40;
constexpr double JOINT4_UPPER = M_PI_2;
constexpr double JOINT4_LOWER = -JOINT4_UPPER;
// Grieel Joint Limits
constexpr double JOINT5_UPPER = M_PI;
constexpr double JOINT5_LOWER = -M_PI;
constexpr double JOINT6_UPPER = 0.5*M_PI;
constexpr double JOINT6_LOWER = -0.25*M_PI;
constexpr double JOINT7_UPPER = INFINITY;
constexpr double JOINT7_LOWER = -INFINITY;


// dynamic model parameter and DH parameter, all based on URDF2DH
constexpr double BODY_MASS = 3.1152;

// limb in swing phase (joint from coxa to driving)
// link masses
constexpr double MASS[JOINT_NUM] = {0.0911, 0.1282, 0.0618, 0.0820, 0.1103, 0.1327, 0.4158};
// CoM vector in the link frame (DH frames)
constexpr double CoM_LINK_FRAME[JOINT_NUM][3] = {{-0.0151, 0.0015, 0.0006},
                                                 {-0.0992, 0.0000, 0.0001},
                                                 {-0.0970, -0.0192, 0.0000},
                                                 {-0.0000, -0.0004, -0.0116},
                                                 {-0.0127, 0.0005, 0.0005},
                                                 {-0.0009, -0.0043, 0.0001},
                                                 {-0.0022, -0.0037, -0.0290}};
// DH parameters (a,d,alpha,offset)
constexpr double A[JOINT_NUM] = {0.0310, 0.2, 0.2, 0, 0.024, 0.0458, 0.0};
constexpr double D[JOINT_NUM] = {0, 0, 0, 0, 0.0468, 0, 0.1131};
constexpr double ALPHA[JOINT_NUM] = {-M_PI_2, 0, 0, -M_PI_2, M_PI_2, -M_PI_2, 0};
constexpr double OFFSET[JOINT_NUM] = {0, 0, 0, -M_PI_2, 0, 0, 0};

// limb in contact phase (joint from driving to coxa)
// link masses
constexpr double MASS_CONTACT[JOINT_NUM] = {0.4158, 0.1327, 0.0121, 0.0902, 0.0618, 0.1282, 0.0911};
// CoM vector in the link frame (DH frames)
constexpr double CoM_LINK_FRAME_CONTACT[JOINT_NUM][3] = {{-0.0435, -0.0841, 0.0037},
                                                         {-0.0689, 0.0043, 0.0001},
                                                         {-0.0113, -0.0473, 0.0005},
                                                         {-0.1884, 0.0000, 0.0004},
                                                         {-0.2192, -0.1030, 0.0000},
                                                         {-0.1318, -0.0001, 0.0000},
                                                         {-0.0159, -0.0006, 0.0185}};
// DH parameters (a,d,alpha,offset)
constexpr double A_CONTACT[JOINT_NUM] = {0.0458, 0.0240, 0, 0.2, 0.2, 0.0310, 0};
constexpr double D_CONTACT[JOINT_NUM] = {0, 0, -0.0468, 0, 0, 0, -0.02};
constexpr double ALPHA_CONTACT[JOINT_NUM] = {-M_PI_2, M_PI_2, -M_PI_2, 0, 0, -M_PI_2, 0};
constexpr double OFFSET_CONTACT[JOINT_NUM] = {0, 0, 0, M_PI_2, -M_PI_2, 0, 0};

#endif  // LBR_DESCRIPTION__LBR_DESCRIPTION_HPP_
