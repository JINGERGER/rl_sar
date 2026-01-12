/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 * 
 * B2+Z1 specific simulation controller
 * Interfaces with separate leg_controller (B2) and arm_controller (Z1)
 */

#ifndef RL_SIM_B2Z1_HPP
#define RL_SIM_B2Z1_HPP

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_all.hpp"

#include <csignal>
#include <vector>
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#if defined(USE_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/empty.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#endif

class RL_Sim_B2Z1 : public RL
{
public:
    RL_Sim_B2Z1(int argc, char **argv);
    ~RL_Sim_B2Z1();

#if defined(USE_ROS2)
    std::shared_ptr<rclcpp::Node> ros2_node;
#endif

private:
    // rl functions
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();

    // loop
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;

    // plot
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<float>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();

    // ros interface
    std::string ros_namespace;

#if defined(USE_ROS2)
    sensor_msgs::msg::Imu gazebo_imu;
    geometry_msgs::msg::Twist cmd_vel;
    sensor_msgs::msg::Joy joy_msg;
    
    // Joint state tracking
    sensor_msgs::msg::JointState joint_states;
    
    // Publishers for separate controllers
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr leg_controller_publisher;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_controller_publisher;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gazebo_imu_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;
    
    // Gazebo services
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr gazebo_pause_physics_client;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr gazebo_unpause_physics_client;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr gazebo_reset_world_client;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr param_client;
    
    // Callbacks
    void GazeboImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
#endif

    // others
    std::string gazebo_model_name;
    
    // B2+Z1 joint configuration
    // B2 has 12 leg joints, Z1 has 6 arm joints
    std::vector<std::string> leg_joint_names;  // 12 B2 leg joints
    std::vector<std::string> arm_joint_names;  // 6 Z1 arm joints
    
    // Joint state storage (by joint name)
    std::map<std::string, float> joint_positions;
    std::map<std::string, float> joint_velocities;
    std::map<std::string, float> joint_efforts;
};

#endif // RL_SIM_B2Z1_HPP
