#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "ddsm115_driver/DDSM115Communicator.h"

namespace ddsm115_hardware {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using hardware_interface::return_type;

class DDSM115HardwareInterface : public hardware_interface::SystemInterface {
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::shared_ptr<ddsm115::Communicator> comm_;
    // Joint state and command buffers
    std::vector<double> position_;
    std::vector<double> velocity_;
    std::vector<double> effort_;
    std::vector<double> command_;
  
    // Configuration
    std::vector<uint8_t> motor_ids_;
    std::vector<ddsm115::Mode> motor_modes_;
    std::string port;
};

CallbackReturn DDSM115HardwareInterface::on_init(const hardware_interface::HardwareInfo & info) {
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("DDSM115HW"),
    ">>> hardware_parameters has %zu entries:", info.hardware_parameters.size());
  for (auto &kv : info.hardware_parameters) {
    RCLCPP_INFO(
      rclcpp::get_logger("DDSM115HW"),
      "    '%s' -> '%s'",
      kv.first.c_str(), kv.second.c_str());
  }
  
  // Log joint names from URDF
  RCLCPP_INFO(rclcpp::get_logger("DDSM115HW"), "Found %zu joints in URDF:", info.joints.size());
  for (const auto & joint : info.joints) {
    RCLCPP_INFO(rclcpp::get_logger("DDSM115HW"), "  - %s", joint.name.c_str());
  }

  // std::stringstream ss(info.hardware_parameters.at("motor_ids"));
  // std::string token;
  // while (std::getline(ss, token, ',')) {
  //   motor_ids_.push_back(static_cast<uint8_t>(std::stoi(token)));
  // }

  // std::stringstream ss1(info.hardware_parameters.at("motor_modes"));
  // std::string token1;
  // while (std::getline(ss1, token1, ',')) {
  //   if (token1 == "VELOCITY_LOOP") {
  //     motor_modes_.push_back(ddsm115::Mode::VELOCITY_LOOP);
  //   } else if (token1 == "POSITION_LOOP") {
  //     motor_modes_.push_back(ddsm115::Mode::POSITION_LOOP);
  //   } else if (token1 == "CURRENT_LOOP") {
  //     motor_modes_.push_back(ddsm115::Mode::CURRENT_LOOP);
  //   } else {
  //     RCLCPP_ERROR(rclcpp::get_logger("DDSM115HW"), "Unknown motor mode: %s", token1.c_str());
  //     return CallbackReturn::ERROR;
  //   }
  // }
  // std::vector<ddsm115::Mode> joint_modes_; 
  // For each joint, read its motor_id and mode parameters
  for (const auto & joint : info.joints) {
    // motor_id
    auto it_id = joint.parameters.find("motor_id");
    if (it_id == joint.parameters.end() || it_id->second.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("DDSM115HW"), "Joint '%s' missing motor_id param", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    motor_ids_.push_back(static_cast<uint8_t>(std::stoi(it_id->second)));

    // mode
    auto it_mode = joint.parameters.find("mode");
    std::string m = it_mode->second ;
    if (m == "position") 
      motor_modes_.push_back(ddsm115::Mode::POSITION_LOOP);
    else                  
      motor_modes_.push_back(ddsm115::Mode::VELOCITY_LOOP);

    RCLCPP_INFO(rclcpp::get_logger("DDSM115HW"), "Joint '%s': motor_id=%d, mode=%s",
                joint.name.c_str(), motor_ids_.back(),m.c_str());
  }
  

  // Read 'port' from <ros2_control> <param> in URDF
  port = "/dev/ttyUSB0";
  auto it = info.hardware_parameters.find("serialport");
  if (it != info.hardware_parameters.end()) {
    port = it->second;
  }

  // Allocate buffers
  size_t n = motor_ids_.size();
  position_.assign(n, 0.0);
  velocity_.assign(n, 0.0);
  effort_.assign(n, 0.0);
  command_.assign(n, 0.0);
  
  RCLCPP_INFO(rclcpp::get_logger("DDSM115HW"), "Opening serial port: %s", port.c_str());

  // Initialize the communicator
  comm_ = std::make_unique<ddsm115::Communicator>(port);
  if (comm_->getState() != ddsm115::State::NORMAL) {
    RCLCPP_ERROR(rclcpp::get_logger("DDSM115HW"), "Failed to open port %s", port.c_str());
    return CallbackReturn::ERROR;
  }else {
    RCLCPP_INFO(rclcpp::get_logger("DDSM115HW"), "Opened port %s successfully", port.c_str());
  }

  RCLCPP_INFO(rclcpp::get_logger("DDSM115HW"), "Motor IDs and modes");
  for (int i = 0; i < motor_ids_.size(); ++i) {
    RCLCPP_INFO(rclcpp::get_logger("DDSM115HW"), "ID: %u, Mode: %s", motor_ids_[i], motor_modes_[i] == ddsm115::Mode::VELOCITY_LOOP ? "VELOCITY_LOOP" : (motor_modes_[i] == ddsm115::Mode::POSITION_LOOP ? "POSITION_LOOP" : "CURRENT_LOOP"));
    comm_->switchMode(motor_ids_[i], motor_modes_[i]);
  }

  // for (auto &mc : motor_ids_) {
  //   comm_mc, ddsm115::Mode::VELOCITY_LOOP);
  // }


  return CallbackReturn::SUCCESS;
}

CallbackReturn DDSM115HardwareInterface::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("DDSM115HW"), "Activating hardware interface");
  std::fill(command_.begin(), command_.end(), 0.0);
  return CallbackReturn::SUCCESS;
}

CallbackReturn DDSM115HardwareInterface::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("DDSM115HW"), "Deactivating hardware interface");
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    comm_->driveMotor(motor_ids_[i], 0, /*acc=*/10, /*brake=*/0);
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DDSM115HardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    auto &j = info_.joints[i];
    interfaces.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &position_[i]);
    interfaces.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &velocity_[i]);
    interfaces.emplace_back(j.name, hardware_interface::HW_IF_EFFORT,   &effort_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
DDSM115HardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    auto &j = info_.joints[i];
    // Expose position or velocity command based on mode
    if (motor_modes_[i] == ddsm115::Mode::POSITION_LOOP)
      interfaces.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &command_[i]);
    else if (motor_modes_[i] == ddsm115::Mode::VELOCITY_LOOP)
      interfaces.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &command_[i]);
    else if (motor_modes_[i] == ddsm115::Mode::CURRENT_LOOP)
      interfaces.emplace_back(j.name, hardware_interface::HW_IF_EFFORT, &command_[i]);
  }
  return interfaces;
}

return_type DDSM115HardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &) {
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
      auto fb = comm_->getAdditionalFeedback(motor_ids_[i]);
      if (fb.status != ddsm115::State::NORMAL) {
        RCLCPP_WARN(rclcpp::get_logger("DDSM115HW"), "Read error on motor %u", motor_ids_[i]);
        continue;
      }
      // std::cout << "fb,position: " << fb.position << std::endl;
      // std::cout << "fb,velocity: " << fb.velocity << std::endl;
      // std::cout << "fb,current: " << fb.current << std::endl;

      position_[i] = (fb.position * 3.14 ) / 180.0; // Convert to radians
      velocity_[i] = (2*3.14*0.5035*fb.velocity)/60.0;
      effort_[i]   = fb.current;
    }
    return return_type::OK;
}

return_type DDSM115HardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &) {
    // RCLCPP_INFO(rclcpp::get_logger("DDSM115HW"), "writing from DDSM115...");
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
      // Ensure mode before sending command
      int16_t val = static_cast<int16_t>(command_[i]);
      comm_->driveMotor(motor_ids_[i], val, /*acc=*/10, /*brake=*/0);
    }
    return return_type::OK;
}


// Register the plugin with pluginlib
PLUGINLIB_EXPORT_CLASS(
  ddsm115_hardware::DDSM115HardwareInterface,
  hardware_interface::SystemInterface
)

} // namespace ddsm115_hardware
