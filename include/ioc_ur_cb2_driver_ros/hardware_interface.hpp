// Copyright 2019 FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \author  Andy Zelenak zelenak@picknik.ai
 * \author  Marvin Große Besselmann grosse@fzi.de
 * \date    2019-04-11
 *
 */
//----------------------------------------------------------------------

#ifndef UR_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_
#define UR_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_

// System
#include <memory>
#include <string>
#include <vector>
#include <limits>
// #include <thread>
// #include <bitset>

// ros2_control hardware_interface
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
// #include "hardware_interface/handle.hpp"

// UR stuff
#include "ioc_ur_cb2_driver/ur_driver.h"

// ROS
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
// #include "geometry_msgs/msg/transform_stamped.hpp"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ioc_ur_cb2_driver_ros
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

enum StoppingInterface
{
  NONE,
  STOP_POSITION,
  STOP_VELOCITY
};

/*!
 * \brief The HardwareInterface class handles the interface between the ROS system and the main
 * driver. It contains the read and write methods of the main control loop and registers various ROS
 * topics and services.
 */
class URPositionHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(URPositionHardwareInterface);
  // virtual ~URPositionHardwareInterface();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) final;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) final;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) final;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) final;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) final;

  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) final;

  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) final;
                                                              
  static constexpr double NO_NEW_CMD_ = std::numeric_limits<double>::quiet_NaN();

protected:

  // void extractToolPose();

  std::array<double, 6> ur_position_commands_;
  std::array<double, 6> ur_position_commands_old_;
  std::array<double, 6> ur_velocity_commands_;
  std::array<double, 6> ur_joint_positions_;
  std::array<double, 6> ur_joint_velocities_;
  std::array<double, 6> ur_joint_efforts_;
  std::array<double, 6> ur_ft_sensor_measurements_;
  std::array<double, 6> ur_tcp_pose_;
  
  // transform stuff
  // tf2::Vector3 tcp_force_;
  // tf2::Vector3 tcp_torque_;
  // geometry_msgs::msg::TransformStamped tcp_transform_;

  bool packet_read_;

  // robot states
  double robot_mode_;
  // UniversalRobot::RobotModeData robot_mode_data_;
  // UniversalRobot::RobotMode robot_mode_;
  bool controllers_initialized_;

  // asynchronous commands
  double target_speed_fraction_;
  double target_speed_fraction_cmd_;
  double resend_robot_program_cmd_;
  double resend_robot_program_async_success_;
  bool first_pass_;
  bool initialized_;
  double system_interface_initialized_;

  // resources switching aux vars
  std::vector<uint> stop_modes_;
  std::vector<std::string> start_modes_;
  bool position_controller_running_;
  bool velocity_controller_running_;

  std::unique_ptr<UrDriver> ur_driver_;

  // timer to limit sending cmd frequency should be <= 125 Hz
  rclcpp::Time time_last_cmd_send_;
  rclcpp::Time time_now_;
};

}  // namespace ioc_ur_cb2_driver_ros

#endif  // UR_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_

