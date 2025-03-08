#ifndef LBR_DATA_CSV__LBR_DATA_CSV_HPP_
#define LBR_DATA_CSV__LBR_DATA_CSV_HPP_

#include <chrono>
#include <cstdio>
#include <iostream>
#include <fstream>

#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <lbr_parameter.hpp>

class DataCSV : public rclcpp::Node
{
    public: 
        DataCSV();

        std::fstream file_out;

    private:
        void samplerCallback();
        void dynamixelEncoderCallback(
            const sensor_msgs::msg::JointState & encoder_joint_state);
        void limbControllerCallback(
                const sensor_msgs::msg::JointState & encoder_joint_state);

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr dynamixel_joint_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr limb_controller_joint_sub_;
        
        rclcpp::TimerBase::SharedPtr data_sampler_timer; 

        int Ts; 
        bool start_sampler; 
        float initial_time; 

        sensor_msgs::msg::JointState current_dynamixel_state;
        sensor_msgs::msg::JointState reference_joint_state;

        std::string limb_prefix;
};

#endif  // LBR_DATA_CSV__LBR_DATA_CSV_HPP_
