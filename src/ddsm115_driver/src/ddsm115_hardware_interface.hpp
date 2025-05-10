// ddsm115_hardware_interface.hpp

#ifndef DDSM115_HARDWARE_INTERFACE_HPP_
#define DDSM115_HARDWARE_INTERFACE_HPP_

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
  struct MotorInfo {
    uint8_t id;
    ddsm115::Mode mode;
    ddsm115::Feedback feedback;
  };
  double motor_radius_ = 0.05035; // radius of motor in meters
  double motor_torque_constant_ = 0.75; // torque constant of motor in Nm/A
  double motor_max_rpm_ = 330.0;
  double motor_readings_value_ = 32767.0;

  std::shared_ptr<ddsm115::Communicator> comm_;
  // Joint state and command buffers
  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> effort_;
  std::vector<double> command_;

  // Configuration
  std::vector<MotorInfo> motor_info_;
  std::string port;
};
}

#endif  // DDSM115_HARDWARE_INTERFACE_HPP_
