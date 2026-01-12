/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 * 
 * B2+Z1 specific simulation controller
 * Uses RL control for B2 legs, position control for Z1 arm
 */

#include "rl_sim_b2z1.hpp"

RL_Sim_B2Z1::RL_Sim_B2Z1(int argc, char **argv)
{
#if defined(USE_ROS2)
    ros2_node = std::make_shared<rclcpp::Node>("rl_sim_b2z1_node");
    this->ang_vel_axis = "body";
    this->ros_namespace = ros2_node->get_namespace();
    
    // Get params from param_node
    param_client = ros2_node->create_client<rcl_interfaces::srv::GetParameters>("/param_node/get_parameters");
    while (!param_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok()) {
            std::cout << LOGGER::ERROR << "Interrupted while waiting for param_node service. Exiting." << std::endl;
            return;
        }
        std::cout << LOGGER::WARNING << "Waiting for param_node service to be available..." << std::endl;
    }
    
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {"robot_name", "gazebo_model_name"};
    auto future = param_client->async_send_request(request);
    auto status = rclcpp::spin_until_future_complete(ros2_node->get_node_base_interface(), future, std::chrono::seconds(5));
    
    if (status == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = future.get();
        if (result->values.size() < 2)
        {
            std::cout << LOGGER::ERROR << "Failed to get all parameters from param_node" << std::endl;
        }
        else
        {
            this->robot_name = result->values[0].string_value;
            this->gazebo_model_name = result->values[1].string_value;
            std::cout << LOGGER::INFO << "Get param robot_name: " << this->robot_name << std::endl;
            std::cout << LOGGER::INFO << "Get param gazebo_model_name: " << this->gazebo_model_name << std::endl;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "Failed to call param_node service" << std::endl;
    }
#endif

    // Read params from yaml - for B2+Z1 we use B2's params for RL control
    this->ReadYaml("b2", "base.yaml");

    // Auto load FSM for B2 (the leg controller)
    if (FSMManager::GetInstance().IsTypeSupported("b2"))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM("b2", this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: b2" << std::endl;
    }

    // Init robot with 12 B2 leg joints (RL control)
    this->InitJointNum(12);
    this->InitOutputs();
    this->InitControl();

    // B2 leg joint names (12 joints)
    leg_joint_names = {
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
    };
    
    // Z1 arm joint names (6 joints)
    arm_joint_names = {
        "z1_joint1", "z1_joint2", "z1_joint3",
        "z1_joint4", "z1_joint5", "z1_joint6"
    };

#if defined(USE_ROS2)
    // Publishers for B2 legs (effort control via trajectory) and Z1 arm (position control via trajectory)
    leg_controller_publisher = ros2_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/leg_controller/joint_trajectory", rclcpp::SystemDefaultsQoS());
    
    arm_controller_publisher = ros2_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/arm_controller/joint_trajectory", rclcpp::SystemDefaultsQoS());

    // Subscribers
    cmd_vel_subscriber = ros2_node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this] (const geometry_msgs::msg::Twist::SharedPtr msg) {this->CmdvelCallback(msg);}
    );
    
    joy_subscriber = ros2_node->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", rclcpp::SystemDefaultsQoS(),
        [this] (const sensor_msgs::msg::Joy::SharedPtr msg) {this->JoyCallback(msg);}
    );
    
    gazebo_imu_subscriber = ros2_node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SystemDefaultsQoS(), 
        [this] (const sensor_msgs::msg::Imu::SharedPtr msg) {this->GazeboImuCallback(msg);}
    );
    
    joint_state_subscriber = ros2_node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SystemDefaultsQoS(),
        [this] (const sensor_msgs::msg::JointState::SharedPtr msg) {this->JointStateCallback(msg);}
    );

    // Services
    gazebo_pause_physics_client = ros2_node->create_client<std_srvs::srv::Empty>("/pause_physics");
    gazebo_unpause_physics_client = ros2_node->create_client<std_srvs::srv::Empty>("/unpause_physics");
    gazebo_reset_world_client = ros2_node->create_client<std_srvs::srv::Empty>("/reset_world");

    auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result = this->gazebo_reset_world_client->async_send_request(empty_request);
#endif

    // Initialize joint state maps
    for (const auto& name : leg_joint_names) {
        joint_positions[name] = 0.0f;
        joint_velocities[name] = 0.0f;
        joint_efforts[name] = 0.0f;
    }
    for (const auto& name : arm_joint_names) {
        joint_positions[name] = 0.0f;
        joint_velocities[name] = 0.0f;
        joint_efforts[name] = 0.0f;
    }

    // Loop control
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), 
        std::bind(&RL_Sim_B2Z1::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), 
        std::bind(&RL_Sim_B2Z1::RunModel, this));
    this->loop_control->start();
    this->loop_rl->start();

    // Keyboard
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, 
        std::bind(&RL_Sim_B2Z1::KeyboardInterface, this));
    this->loop_keyboard->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(12);
    this->plot_target_joint_pos.resize(12);
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.001, std::bind(&RL_Sim_B2Z1::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit("b2_z1");
#endif

    std::cout << LOGGER::INFO << "RL_Sim_B2Z1 start - B2 legs with RL control, Z1 arm at default position" << std::endl;
}

RL_Sim_B2Z1::~RL_Sim_B2Z1()
{
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Sim_B2Z1 exit" << std::endl;
}

void RL_Sim_B2Z1::GetState(RobotState<float> *state)
{
#if defined(USE_ROS2)
    const auto &orientation = this->gazebo_imu.orientation;
    const auto &angular_velocity = this->gazebo_imu.angular_velocity;
#endif

    state->imu.quaternion[0] = orientation.w;
    state->imu.quaternion[1] = orientation.x;
    state->imu.quaternion[2] = orientation.y;
    state->imu.quaternion[3] = orientation.z;

    state->imu.gyroscope[0] = angular_velocity.x;
    state->imu.gyroscope[1] = angular_velocity.y;
    state->imu.gyroscope[2] = angular_velocity.z;

    // Get B2 leg joint states (12 joints)
    for (int i = 0; i < 12; ++i)
    {
        state->motor_state.q[i] = this->joint_positions[leg_joint_names[i]];
        state->motor_state.dq[i] = this->joint_velocities[leg_joint_names[i]];
        state->motor_state.tau_est[i] = this->joint_efforts[leg_joint_names[i]];
    }
}

void RL_Sim_B2Z1::SetCommand(const RobotCommand<float> *command)
{
#if defined(USE_ROS2)
    // Publish B2 leg commands using RL output
    trajectory_msgs::msg::JointTrajectory leg_traj;
    leg_traj.header.stamp = ros2_node->now();
    leg_traj.joint_names = leg_joint_names;
    
    trajectory_msgs::msg::JointTrajectoryPoint leg_point;
    leg_point.positions.resize(12);
    leg_point.velocities.resize(12);
    leg_point.effort.resize(12);
    leg_point.time_from_start = rclcpp::Duration::from_seconds(0.001);
    
    for (int i = 0; i < 12; ++i)
    {
        leg_point.positions[i] = command->motor_command.q[i];
        leg_point.velocities[i] = command->motor_command.dq[i];
        leg_point.effort[i] = command->motor_command.tau[i];
    }
    
    leg_traj.points.push_back(leg_point);
    leg_controller_publisher->publish(leg_traj);
    
    // Publish Z1 arm commands - keep at default/home position
    trajectory_msgs::msg::JointTrajectory arm_traj;
    arm_traj.header.stamp = ros2_node->now();
    arm_traj.joint_names = arm_joint_names;
    
    trajectory_msgs::msg::JointTrajectoryPoint arm_point;
    arm_point.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // Home position
    arm_point.time_from_start = rclcpp::Duration::from_seconds(0.001);
    
    arm_traj.points.push_back(arm_point);
    arm_controller_publisher->publish(arm_traj);
#endif
}

void RL_Sim_B2Z1::RobotControl()
{
    this->GetState(&this->robot_state);

    this->StateController(&this->robot_state, &this->robot_command);

    if (this->control.current_keyboard == Input::Keyboard::R || this->control.current_gamepad == Input::Gamepad::RB_Y)
    {
#if defined(USE_ROS2)
        auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = this->gazebo_reset_world_client->async_send_request(empty_request);
#endif
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::Enter || this->control.current_gamepad == Input::Gamepad::RB_X)
    {
        if (simulation_running)
        {
#if defined(USE_ROS2)
            auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = this->gazebo_pause_physics_client->async_send_request(empty_request);
#endif
            std::cout << std::endl << LOGGER::INFO << "Simulation Stop" << std::endl;
        }
        else
        {
#if defined(USE_ROS2)
            auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = this->gazebo_unpause_physics_client->async_send_request(empty_request);
#endif
            std::cout << std::endl << LOGGER::INFO << "Simulation Start" << std::endl;
        }
        simulation_running = !simulation_running;
        this->control.current_keyboard = this->control.last_keyboard;
    }

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);
}

#if defined(USE_ROS2)
void RL_Sim_B2Z1::GazeboImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    this->gazebo_imu = *msg;
}

void RL_Sim_B2Z1::JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        const std::string& joint_name = msg->name[i];
        
        if (i < msg->position.size())
            this->joint_positions[joint_name] = msg->position[i];
        if (i < msg->velocity.size())
            this->joint_velocities[joint_name] = msg->velocity[i];
        if (i < msg->effort.size())
            this->joint_efforts[joint_name] = msg->effort[i];
    }
}
#endif

void RL_Sim_B2Z1::CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->cmd_vel = *msg;
}

void RL_Sim_B2Z1::JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    this->joy_msg = *msg;

    // Joystick control (same as original rl_sim)
    if (this->joy_msg.buttons[0]) this->control.SetGamepad(Input::Gamepad::A);
    if (this->joy_msg.buttons[1]) this->control.SetGamepad(Input::Gamepad::B);
    if (this->joy_msg.buttons[2]) this->control.SetGamepad(Input::Gamepad::X);
    if (this->joy_msg.buttons[3]) this->control.SetGamepad(Input::Gamepad::Y);
    if (this->joy_msg.buttons[4]) this->control.SetGamepad(Input::Gamepad::LB);
    if (this->joy_msg.buttons[5]) this->control.SetGamepad(Input::Gamepad::RB);
    if (this->joy_msg.buttons[9]) this->control.SetGamepad(Input::Gamepad::LStick);
    if (this->joy_msg.buttons[10]) this->control.SetGamepad(Input::Gamepad::RStick);
    if (this->joy_msg.axes[7] > 0) this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (this->joy_msg.axes[7] < 0) this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (this->joy_msg.axes[6] < 0) this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (this->joy_msg.axes[6] > 0) this->control.SetGamepad(Input::Gamepad::DPadRight);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[0]) this->control.SetGamepad(Input::Gamepad::LB_A);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[1]) this->control.SetGamepad(Input::Gamepad::LB_B);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[2]) this->control.SetGamepad(Input::Gamepad::LB_X);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[3]) this->control.SetGamepad(Input::Gamepad::LB_Y);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[9]) this->control.SetGamepad(Input::Gamepad::LB_LStick);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[10]) this->control.SetGamepad(Input::Gamepad::LB_RStick);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[7] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[7] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[6] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[6] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[0]) this->control.SetGamepad(Input::Gamepad::RB_A);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[1]) this->control.SetGamepad(Input::Gamepad::RB_B);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[2]) this->control.SetGamepad(Input::Gamepad::RB_X);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[3]) this->control.SetGamepad(Input::Gamepad::RB_Y);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[9]) this->control.SetGamepad(Input::Gamepad::RB_LStick);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[10]) this->control.SetGamepad(Input::Gamepad::RB_RStick);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[7] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[7] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[6] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[6] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[5]) this->control.SetGamepad(Input::Gamepad::LB_RB);

    this->control.x = this->joy_msg.axes[1]; // LY
    this->control.y = this->joy_msg.axes[0]; // LX
    this->control.yaw = this->joy_msg.axes[3]; // RX
}

std::vector<float> RL_Sim_B2Z1::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (this->params.Get<std::vector<int>>("observations_history").size() != 0)
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}

void RL_Sim_B2Z1::RunModel()
{
    this->Forward();
}

void RL_Sim_B2Z1::Plot()
{
#ifdef PLOT
    static int count = 0;
    this->plot_t[count % this->plot_size] = count;
    for (int i = 0; i < 12; ++i)
    {
        this->plot_real_joint_pos[i][count % this->plot_size] = this->robot_state.motor_state.q[i];
        this->plot_target_joint_pos[i][count % this->plot_size] = this->robot_command.motor_command.q[i];
    }
    count++;

    if (count % this->plot_size != this->plot_size - 1)
    {
        return;
    }

    plt::clf();
    for (int i = 0; i < 12; ++i)
    {
        plt::subplot(3, 4, i + 1);
        plt::named_plot("real", this->plot_t, this->plot_real_joint_pos[i], "b");
        plt::named_plot("target", this->plot_t, this->plot_target_joint_pos[i], "r");
        plt::legend();
    }
    plt::pause(0.001);
#endif
}

#if defined(USE_ROS2)
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto rl_sim_b2z1 = std::make_shared<RL_Sim_B2Z1>(argc, argv);

    rclcpp::spin(rl_sim_b2z1->ros2_node);

    rclcpp::shutdown();
    return 0;
}
#endif
