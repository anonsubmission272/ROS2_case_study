// Copyright 2023 ros2_control Development Team
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

#ifndef ROS2_CONTROL_DEMO_EXAMPLE_7__R6BOT_CONTROLLER_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_7__R6BOT_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace ros2_control_demo_example_7
{

//DEBUG

typedef struct {
  size_t positions_length;
  double positions[100]; // Assuming a maximum of 100 positions

  size_t velocities_length;
  double velocities[100]; // Assuming a maximum of 100 velocities

  size_t accelerations_length;
  double accelerations[100]; // Assuming a maximum of 100 accelerations

  size_t effort_length;
  double effort[100]; // Assuming a maximum of 100 effort values

  int32_t time_from_start_sec; // seconds part of the duration
  uint32_t time_from_start_nsec; // nanoseconds part of the duration
} MappedJointTrajectoryPoint;

struct MappedJointTrajectory {
    size_t joint_names_length;
    char joint_names[10][256]; // Assuming a maximum of 10 joint names, each with a maximum length of 256

    size_t points_length;
    MappedJointTrajectoryPoint points[256]; // Assuming a maximum of 100 points
};

typedef struct {
  int idx;
  MappedJointTrajectoryPoint value; // u, (dx, da) <- not yet
} Vote;

typedef struct {
  int idx;
  MappedJointTrajectory value; // x,a,t temp(dx, da)
  int32_t cur_time_sec;
  //uint32_t cur_time_nsec;
} State_vote;

class RobotController : public controller_interface::ControllerInterface
{
public:

  //DEBUG
  // inline bool exists_test3 (const std::string& name);

  void setup_mapped_mem();

  void serialize_joint_trajectory(const std::shared_ptr<trajectory_msgs::msg::JointTrajectory>& src, State_vote* dest);

  RobotController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:

  //DEBUG
  int fd0;
  State_vote* state_vote;
  int fd1;
  int fd2;
  int fd3;
  int fd4;
  Vote* actuation;
  int myIdx;
  // Vote *tmp_vote;
  // State_vote *tmp_state;
  // bool have_actuation;
  Vote* data0;
  Vote* data1;
  Vote* data2;


  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_subscriber_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectory>>
    traj_msg_external_point_ptr_;
  bool new_msg_ = false;
  rclcpp::Time start_time_;
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg_;
  trajectory_msgs::msg::JointTrajectoryPoint point_interp_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_position_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_velocity_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_position_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_velocity_state_interface_;

  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
    command_interface_map_ = {
      {"position", &joint_position_command_interface_},
      {"velocity", &joint_velocity_command_interface_}};

  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
    state_interface_map_ = {
      {"position", &joint_position_state_interface_},
      {"velocity", &joint_velocity_state_interface_}};
};

}  // namespace ros2_control_demo_example_7

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_7__R6BOT_CONTROLLER_HPP_
