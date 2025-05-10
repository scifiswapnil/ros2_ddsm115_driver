#include "ddsm115_hardware_interface.hpp"

namespace ddsm115_hardware {

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

  
  // For each joint, read its motor_id and mode parameters
  for (const auto & joint : info.joints) {
    uint8_t motor_id = 0;
    ddsm115::Mode motor_mode = ddsm115::Mode::POSITION_LOOP;
    
    // check motor_id
    auto it_id = joint.parameters.find("motor_id");
    if (it_id == joint.parameters.end() || it_id->second.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("DDSM115HW"), "Joint '%s' missing motor_id param", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    motor_id = static_cast<uint8_t>(std::stoi(it_id->second));

    // check motor mode
    auto it_mode = joint.parameters.find("mode");
    std::string m = it_mode->second ;
    if (m == "POSITION_LOOP") 
      motor_mode =  ddsm115::Mode::POSITION_LOOP;
    else if (m == "VELOCITY_LOOP")                  
      motor_mode = ddsm115::Mode::VELOCITY_LOOP;
    else if (m == "EFFORT_LOOP")
      motor_mode = ddsm115::Mode::CURRENT_LOOP;
    else {
      RCLCPP_ERROR(rclcpp::get_logger("DDSM115HW"), "Joint '%s' has invalid mode '%s'", joint.name.c_str(), m.c_str());
      return CallbackReturn::ERROR;
    }
    

    RCLCPP_INFO(rclcpp::get_logger("DDSM115HW"), "Joint '%s': motor_id=%d, mode=%s",
                joint.name.c_str(), motor_id, m.c_str());
    motor_info_.emplace_back(MotorInfo{motor_id, motor_mode});
  }
  
  // Read 'port' from <ros2_control> <param> in URDF
  port = "/dev/ttyACM0";
  auto it = info.hardware_parameters.find("serialport");
  if (it != info.hardware_parameters.end()) {
    port = it->second;
  }

  // Allocate buffers
  size_t n = motor_info_.size();
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

  // Set motor modes 
  RCLCPP_INFO(rclcpp::get_logger("DDSM115HW"), "Motor IDs and modes");
  for (int i = 0; i < motor_info_.size(); ++i) {
    RCLCPP_INFO(rclcpp::get_logger("DDSM115HW"), "ID: %u, Mode: %s", motor_info_[i].id, motor_info_[i].mode == ddsm115::Mode::VELOCITY_LOOP ? "VELOCITY_LOOP" : (motor_info_[i].mode == ddsm115::Mode::POSITION_LOOP ? "POSITION_LOOP" : "EFFORT_LOOP"));
    comm_->switchMode(motor_info_[i].id, motor_info_[i].mode);
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn DDSM115HardwareInterface::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("DDSM115HW"), "Activating hardware interface");
  std::fill(command_.begin(), command_.end(), 0.0);
  return CallbackReturn::SUCCESS;
}

CallbackReturn DDSM115HardwareInterface::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("DDSM115HW"), "Deactivating hardware interface");
  
  for (size_t i = 0; i < motor_info_.size(); ++i) {
    comm_->switchMode(motor_info_[i].id, ddsm115::Mode::VELOCITY_LOOP);
    comm_->driveMotor(motor_info_[i].id, 0, /*acc=*/10, /*brake=*/0);
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DDSM115HardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < motor_info_.size(); ++i) {
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
  for (size_t i = 0; i < motor_info_.size(); ++i) {
    auto &j = info_.joints[i];
    if (motor_info_[i].mode == ddsm115::Mode::POSITION_LOOP)
      interfaces.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &command_[i]);
    else if (motor_info_[i].mode == ddsm115::Mode::VELOCITY_LOOP)
      interfaces.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &command_[i]);
    else if (motor_info_[i].mode == ddsm115::Mode::CURRENT_LOOP)
      interfaces.emplace_back(j.name, hardware_interface::HW_IF_EFFORT, &command_[i]);
  }
  return interfaces;
}

return_type DDSM115HardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &) {
    for (size_t i = 0; i < motor_info_.size(); ++i) {
      if (motor_info_[i].feedback.status != ddsm115::State::NORMAL){
        RCLCPP_WARN(rclcpp::get_logger("DDSM115HW"), "Read error on motor with ID : %u", motor_info_[i].id);
        continue;
      }
      position_[i] = (motor_info_[i].feedback.position * M_PI ) / 180.0; // Convert to radians
      velocity_[i] = 2 * M_PI * motor_radius_ * (motor_info_[i].feedback.velocity/60.0);
      effort_[i]   = motor_torque_constant_ * (motor_info_[i].feedback.current);
    }
    return return_type::OK;
}

return_type DDSM115HardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &) {    
    for (size_t i = 0; i < motor_info_.size(); ++i){
      double cmd_value_ = static_cast<double>(command_[i]);
      switch (motor_info_[i].mode) {
        case ddsm115::Mode::POSITION_LOOP:
          cmd_value_ = ( cmd_value_ * 180.0 / M_PI);
          cmd_value_ = cmd_value_ < 0.0 ? 360.0 + cmd_value_ : cmd_value_;
          cmd_value_ = std::clamp(std::fmod(cmd_value_, 360.0) * (motor_readings_value_ / 360.0), 0.0, motor_readings_value_);
          motor_info_[i].feedback = comm_->driveMotor(motor_info_[i].id, static_cast<int16_t>(std::clamp(cmd_value_, 0.0, motor_readings_value_)), /*acc=*/10, /*brake=*/0);
          break;
        case ddsm115::Mode::VELOCITY_LOOP:
          cmd_value_ = ((cmd_value_ * 60) / (2 * M_PI * motor_radius_));
          cmd_value_ = static_cast<int16_t>(std::clamp(cmd_value_, -motor_max_rpm_, motor_max_rpm_)); 
          motor_info_[i].feedback = comm_->driveMotor(motor_info_[i].id, cmd_value_, /*acc=*/10, /*brake=*/0);
          break;
        case ddsm115::Mode::CURRENT_LOOP:
          cmd_value_ = (cmd_value_ / motor_torque_constant_) ;
          cmd_value_ = static_cast<int16_t>(std::clamp(cmd_value_ * (motor_readings_value_ / 8.0), -motor_readings_value_, motor_readings_value_));
          motor_info_[i].feedback = comm_->driveMotor(motor_info_[i].id, cmd_value_, /*acc=*/10, /*brake=*/0);
          break;
      }
    }
    return return_type::OK;
}

// Register the plugin with pluginlib
PLUGINLIB_EXPORT_CLASS(
  ddsm115_hardware::DDSM115HardwareInterface,
  hardware_interface::SystemInterface
)

} // namespace ddsm115_hardware
