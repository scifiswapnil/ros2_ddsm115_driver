// ddsm115_hardware_interface.hpp

#ifndef DDSM115_HARDWARE_INTERFACE_HPP_
#define DDSM115_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"  // for rclcpp_lifecycle::State
#include "hardware_interface/types/hardware_interface_type_values.hpp"  // for HW_IF_*

#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
// #include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "ddsm115_driver/DDSM115Communicator.h"

namespace ddsm115_hardware
{

using hardware_interface::return_type;
// using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class DDSM115HardwareInterface : public hardware_interface::SystemInterface
{
public:
  // Lifecycle callbacks
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
//   CallbackReturn on_activate(const rclcpp::Time & /*time*/) override;
//   CallbackReturn on_deactivate(const rclcpp::Time & /*time*/) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override;


  // Read from and write to the hardware
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Underlying serial driver
  std::unique_ptr<ddsm115::Communicator> comm_;

  // Joint data buffers
  std::vector<double> position_, velocity_, effort_, command_;

  // Per-joint parameters
  std::vector<uint8_t>  motor_ids_;
  uint8_t               default_acc_time_{10};
  uint8_t               default_brake_{0};

  // Serial port name, pulled from <ros2_control> spec in URDF
  std::string port_name_;
};

}  // namespace ddsm115_hardware

#endif  // DDSM115_HARDWARE_INTERFACE_HPP_
