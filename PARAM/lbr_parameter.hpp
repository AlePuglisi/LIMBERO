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

constexpr int LIMB_NUM = 4;
constexpr int JOINT_NUM = 7;
constexpr int GRIPPER_MOTOR_NUM = 1;

// Link lengths (Unit: [m])
constexpr double L1 = 0.031;
constexpr double L2 = 0.2;
constexpr double L3 = 0.2;
constexpr double L4 = 0.11475;

// Joint Limits (Unit: [rad])
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
constexpr double JOINT6_LOWER = 0;
constexpr double JOINT7_UPPER = INFINITY;
constexpr double JOINT7_LOWER = -INFINITY;



#endif  // LBR_DESCRIPTION__LBR_DESCRIPTION_HPP_
