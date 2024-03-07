// Copyright 2019, FZI Forschungszentrum Informatik
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
 * \date    2020-11-9
 *
 */
//----------------------------------------------------------------------
#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
// #include <limits>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "ur_cb2_driver_ros/hardware_interface.hpp"


namespace ur_cb2_driver_ros {

// URPositionHardwareInterface::~URPositionHardwareInterface()
// {
//   // If the controller manager is shutdown via Ctrl + C the on_deactivate methods won't be called.
//   // We therefore need to make sure to actually deactivate the communication
//   on_deactivate(rclcpp_lifecycle::State());
// }

hardware_interface::CallbackReturn URPositionHardwareInterface::on_init(const hardware_interface::HardwareInfo & system_info)
{
    if (hardware_interface::SystemInterface::on_init(system_info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = system_info;

    // state interface
    ur_joint_positions_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
    ur_joint_velocities_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
    ur_joint_efforts_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
    ur_tcp_pose_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
    ur_ft_sensor_measurements_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

    // command interface
    ur_position_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
    ur_position_commands_old_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
    ur_velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

    // other hardware parameters
    position_controller_running_ = false;
    velocity_controller_running_ = false;
    controllers_initialized_ = false;
    first_pass_ = true;
    initialized_ = false;
    system_interface_initialized_ = 0.0;
    time_last_cmd_send_ = rclcpp::Clock().now();

    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
        if (joint.command_interfaces.size() != 2) {
            RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                        "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
                        joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                        "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                        joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                        "Joint '%s' have %s command interfaces found as second command interface. '%s' expected.",
                        joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 3) {
            RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"), "Joint '%s' has %zu state interface. 3 expected.",
                        joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                    "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                    "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
            RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                    "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> URPositionHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &ur_joint_positions_[i]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &ur_joint_velocities_[i]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &ur_joint_efforts_[i]));
    }

    // Obtain the tf_prefix from the urdf so that we can have the general interface multiple times
    // NOTE using the tf_prefix at this point is some kind of workaround. One should actually go through the list of gpio
    // state interface in info_ and match them accordingly
    const std::string tf_prefix = info_.hardware_parameters.at("tf_prefix");

    state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "system_interface", "initialized", &system_interface_initialized_));

    for (auto& sensor : info_.sensors) {
      for (uint j = 0; j < sensor.state_interfaces.size(); ++j) {
        RCLCPP_WARN(rclcpp::get_logger("UrRobotHW"), "sensor name %s",sensor.name.c_str());
        state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, sensor.state_interfaces[j].name,
                                                                        &ur_ft_sensor_measurements_[j]));
      }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> URPositionHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &ur_position_commands_[i]));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &ur_velocity_commands_[i]));
    }

    // Obtain the tf_prefix from the urdf so that we can have the general interface multiple times
    // NOTE using the tf_prefix at this point is some kind of workaround. One should actually go through the list of gpio
    // command interface in info_ and match them accordingly
    const std::string tf_prefix = info_.hardware_parameters.at("tf_prefix");

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        tf_prefix + "speed_scaling", "target_speed_fraction_cmd", &target_speed_fraction_cmd_));
    
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        tf_prefix + "resend_robot_program", "resend_robot_program_cmd", &resend_robot_program_cmd_));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        tf_prefix + "resend_robot_program", "resend_robot_program_async_success", &resend_robot_program_async_success_));

    return command_interfaces;
}

hardware_interface::CallbackReturn URPositionHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void) (previous_state);
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Starting ...please wait...");

  // The robot's name.
  std::string robot_name = info_.hardware_parameters["name"];
  // The robot's type.
  std::string ur_type = info_.hardware_parameters["ur_type"];
  // The robot's IP address.
  std::string robot_ip = info_.hardware_parameters["robot_ip"];
  // Port that will be opened to communicate between the driver and the robot controller.
  // Note: Must be different if multiples ur_drivers are launched from the same machine!
  int reverse_port = stoi(info_.hardware_parameters["reverse_port"]);
  // Path to the urscript code that will be sent to the robot.
  std::string script_filename = info_.hardware_parameters["script_filename"];

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Initializing driver...");

  ur_driver_ = std::make_unique<UrDriver>(rt_msg_cond_, msg_cond_, robot_ip, reverse_port, 0.03, 300);
  if (!ur_driver_->start()) {
    RCLCPP_ERROR(rclcpp::get_logger("URPositionHardwareInterface"), "Could not connect to Robot!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "System successfully started!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn URPositionHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void) (previous_state);
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Stopping ...please wait...");

  // Anything to clean up?
  ur_driver_->halt();

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "System successfully stopped!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type URPositionHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void) (time);
  (void) (period);
	std::vector<double> pos, vel, current, tcp_force, tcp_pose;
	pos = ur_driver_->rt_interface_->robot_state_->getQActual();
	vel = ur_driver_->rt_interface_->robot_state_->getQdActual();
	current = ur_driver_->rt_interface_->robot_state_->getIActual();
	tcp_force = ur_driver_->rt_interface_->robot_state_->getTcpForce();
	tcp_pose = ur_driver_->rt_interface_->robot_state_->getToolVectorActual();

  // printf("getTcpForce: ");
	// for (size_t i = 0; i < tcp_pose.size(); ++i) {
  //   printf("%.2f ", tcp_pose[i]);
  // }
  // printf("\n");

	for (std::size_t i = 0; i < info_.joints.size(); i++) {
		ur_joint_positions_[i] = pos[i];
		ur_joint_velocities_[i] = vel[i];
		ur_joint_efforts_[i] = current[i];
    ur_ft_sensor_measurements_[i] = tcp_force[i];
    ur_tcp_pose_[i] = tcp_pose[i];
	}
	// for (std::size_t i = 0; i < 3; ++i) {
	// 	robot_force_[i] = tcp[i];
	// 	robot_torque_[i] = tcp[i + 3];
	// }

  
  robot_mode_ = ur_driver_->rt_interface_->robot_state_->getRobotMode();
  // printf("%.2f \n", robot_mode_);

  // ROBOT MODE STATES:
  // 0 : OK (READY TO BE USED)
  // 1 : TEACH (BREAK REALASE || CONTROLLED FROM TEACH)
  // 2 :
  // 3 : INITIALIZING
  // 4 : SECURITY STOPPED (SAFE STOP DUE COALITION)
  // 5 : EMERGENCY STOPPED (RED BUTTON PRESSED)
  // 6 : 
  // 7 : ROBOT POWER OFF

  // ur_tcp_pose_[0] = ur_interface.unionValue(ur_interface.cartesianInfo.info.x);
  // ur_tcp_pose_[1] = ur_interface.unionValue(ur_interface.cartesianInfo.info.y);
  // ur_tcp_pose_[2] = ur_interface.unionValue(ur_interface.cartesianInfo.info.z);
  // ur_tcp_pose_[3] = ur_interface.unionValue(ur_interface.cartesianInfo.info.Rx);
  // ur_tcp_pose_[4] = ur_interface.unionValue(ur_interface.cartesianInfo.info.Ry);
  // ur_tcp_pose_[5] = ur_interface.unionValue(ur_interface.cartesianInfo.info.Rz);
  // extractToolPose();

  // // other robot states data
  // robot_mode_data_ = ur_interface.robotMode.rmd;
  // robot_mode_ = static_cast<UniversalRobot::RobotMode>(robot_mode_data_.state.robotMode);
  // target_speed_fraction_ = ur_interface.unionValue(robot_mode_data_.targetSpeedFraction);
  
  packet_read_ = true;

  if (first_pass_ && !initialized_)
  {
    // initialize commands
    ur_position_commands_ = ur_position_commands_old_ = ur_joint_positions_;
    ur_velocity_commands_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    target_speed_fraction_cmd_ = NO_NEW_CMD_;
    resend_robot_program_cmd_ = NO_NEW_CMD_;
    initialized_ = true;
    system_interface_initialized_ = 1.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type URPositionHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void) (time);
  (void) (period);
  // If there is no interpreting program running on the robot, we do not want to send anything.
  time_now_ = rclcpp::Clock().now();
  rclcpp::Duration time_since_last_send_ = time_now_ - time_last_cmd_send_;
  
  if(packet_read_ && time_since_last_send_ >= rclcpp::Duration(0, 8000000)) {
    if (position_controller_running_ && (ur_position_commands_ != ur_position_commands_old_)) {
      ur_position_commands_old_ = ur_position_commands_;
      ur_driver_->setServo(ur_position_commands_[0], 
                            ur_position_commands_[1], 
                            ur_position_commands_[2], 
                            ur_position_commands_[3], 
                            ur_position_commands_[4], 
                            ur_position_commands_[5], 5.); // Time [seconds]
    } else if (velocity_controller_running_) {
      ur_driver_->setSpeed(ur_velocity_commands_[0],
                            ur_velocity_commands_[1],
                            ur_velocity_commands_[2],
                            ur_velocity_commands_[3],
                            ur_velocity_commands_[4],
                            ur_velocity_commands_[5], 5.);  // Joint acceleration [rad/s²]
    } else {
      // Do something to keep it alive
    }
    time_last_cmd_send_ = time_now_;
    packet_read_ = false;
  }
  return hardware_interface::return_type::OK;
}

// void URPositionHardwareInterface::extractToolPose()
// {
//   // imported from ROS1 driver hardware_interface.cpp#L911-L928
//   double tcp_angle =
//       std::sqrt(std::pow(ur_tcp_pose_[3], 2) + std::pow(ur_tcp_pose_[4], 2) + std::pow(ur_tcp_pose_[5], 2));

//   tf2::Vector3 rotation_vec(ur_tcp_pose_[3], ur_tcp_pose_[4], ur_tcp_pose_[5]);
//   tf2::Quaternion rotation;
//   if (tcp_angle > 1e-16) {
//     rotation.setRotation(rotation_vec.normalized(), tcp_angle);
//   } else {
//     rotation.setValue(0.0, 0.0, 0.0, 1.0);  // default Quaternion is 0,0,0,0 which is invalid
//   }
//   tcp_transform_.transform.translation.x = ur_tcp_pose_[0];
//   tcp_transform_.transform.translation.y = ur_tcp_pose_[1];
//   tcp_transform_.transform.translation.z = ur_tcp_pose_[2];

//   tcp_transform_.transform.rotation = tf2::toMsg(rotation);
// }

hardware_interface::return_type URPositionHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  start_modes_.clear();
  stop_modes_.clear();

  // Starting interfaces
  // add start interface per joint in tmp var for later check
  for (const auto& key : start_interfaces) {
    for (auto i = 0u; i < info_.joints.size(); i++) {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        start_modes_.push_back(hardware_interface::HW_IF_POSITION);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        start_modes_.push_back(hardware_interface::HW_IF_VELOCITY);
      }
    }
  }
  // set new mode to all interfaces at the same time
  if (start_modes_.size() != 0 && start_modes_.size() != 6) {
    ret_val = hardware_interface::return_type::ERROR;
  }

  // all start interfaces must be the same - can't mix position and velocity control
  if (start_modes_.size() != 0 && !std::equal(start_modes_.begin() + 1, start_modes_.end(), start_modes_.begin())) {
    ret_val = hardware_interface::return_type::ERROR;
  }

  // Stopping interfaces
  // add stop interface per joint in tmp var for later check
  for (const auto& key : stop_interfaces) {
    for (auto i = 0u; i < info_.joints.size(); i++) {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        stop_modes_.push_back(StoppingInterface::STOP_POSITION);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        stop_modes_.push_back(StoppingInterface::STOP_VELOCITY);
      }
    }
  }
  // stop all interfaces at the same time
  if (stop_modes_.size() != 0 &&
      (stop_modes_.size() != 6 || !std::equal(stop_modes_.begin() + 1, stop_modes_.end(), stop_modes_.begin()))) {
    ret_val = hardware_interface::return_type::ERROR;
  }

  controllers_initialized_ = true;
  return ret_val;
}

hardware_interface::return_type URPositionHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
{
  (void) (start_interfaces);
  (void) (stop_interfaces);
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  if (stop_modes_.size() != 0 &&
      std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_POSITION) != stop_modes_.end()) {
    position_controller_running_ = false;
    ur_position_commands_ = ur_position_commands_old_ = ur_joint_positions_;

  } else if (stop_modes_.size() != 0 &&
             std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_VELOCITY) != stop_modes_.end()) {
    velocity_controller_running_ = false;
    ur_velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  }

  if (start_modes_.size() != 0 &&
      std::find(start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_POSITION) != start_modes_.end()) {
    velocity_controller_running_ = false;
    ur_position_commands_ = ur_position_commands_old_ = ur_joint_positions_;
    position_controller_running_ = true;

  } else if (start_modes_.size() != 0 && std::find(start_modes_.begin(), start_modes_.end(),
                                                   hardware_interface::HW_IF_VELOCITY) != start_modes_.end()) {
    position_controller_running_ = false;
    ur_velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
    velocity_controller_running_ = true;
  }

  start_modes_.clear();
  stop_modes_.clear();

  return ret_val;
}
}  // namespace ur_cb2_driver_ros

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ur_cb2_driver_ros::URPositionHardwareInterface, hardware_interface::SystemInterface)
