#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "ddsm115_driver/DDSM115Communicator.h"

namespace ddsm115_hardware {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using hardware_interface::return_type;

class DDSM115HardwareInterface : public hardware_interface::SystemInterface {
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override { return {}; }
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override { return {}; }
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

  // Read 'port' from <ros2_control> <param> in URDF
  port = "/dev/ttyUSB0";
  auto it = info.hardware_parameters.find("serialport ");
  if (it != info.hardware_parameters.end()) {
    port = it->second;
  }

  std::stringstream ss(info.hardware_parameters.at("motor_ids"));
  std::string token;
  while (std::getline(ss, token, ',')) {
    motor_ids_.push_back(static_cast<uint8_t>(std::stoi(token)));
  }

  // Allocate buffers
  size_t n = motor_ids_.size();
  position_.assign(n, 0.0);
  velocity_.assign(n, 0.0);
  effort_.assign(n, 0.0);
  command_.assign(n, 0.0);
  
  RCLCPP_INFO(rclcpp::get_logger("DDSM115HW"), "Motor IDs: ");
  for (const auto & id : motor_ids_) {
    RCLCPP_INFO(rclcpp::get_logger("DDSM115HW"), "  - %u", id);
  }
  RCLCPP_INFO(rclcpp::get_logger("DDSM115HW"), "Opening serial port: %s", port.c_str());

  // Initialize the communicator
  comm_ = std::make_unique<ddsm115::Communicator>(port);
  if (comm_->getState() != ddsm115::State::NORMAL) {
    RCLCPP_ERROR(rclcpp::get_logger("DDSM115HW"), "Failed to open port %s", port.c_str());
    return CallbackReturn::ERROR;
  }else {
    RCLCPP_INFO(rclcpp::get_logger("DDSM115HW"), "Opened port %s successfully", port.c_str());
  }

  for (auto &mc : motor_ids_) {
    comm_->switchMode(mc, ddsm115::Mode::VELOCITY_LOOP);
  }


  return CallbackReturn::SUCCESS;
}

return_type DDSM115HardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &) {
  // RCLCPP_INFO(rclcpp::get_logger("DDSM115HW"), "Reading from DDSM115...");
    // for (size_t i = 0; i < motor_ids_.size(); ++i) {
    //   auto fb = comm_->getAdditionalFeedback(motor_ids_[i]);
    //   if (fb.status != ddsm115::State::NORMAL) {
    //     RCLCPP_WARN(rclcpp::get_logger("DDSM115HW"), "Read error on motor %u", motor_ids_[i]);
    //     continue;
    //   }
    // }
    return return_type::OK;
}

return_type DDSM115HardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &) {
    // RCLCPP_INFO(rclcpp::get_logger("DDSM115HW"), "writing from DDSM115...");
    return return_type::OK;
}


// Register the plugin with pluginlib
PLUGINLIB_EXPORT_CLASS(
  ddsm115_hardware::DDSM115HardwareInterface,
  hardware_interface::SystemInterface
)

} // namespace ddsm115_hardware
